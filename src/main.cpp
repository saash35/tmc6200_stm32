// =============================================================================
//  48 V PMSM  -  SimpleFOC on Nucleo-F401RE with a TMC6200 gate driver (SPI)
//
//  Architecture
//    UART (USART2 / ST-Link VCP)  --> Commander  --> speed target (rad/s, signed)
//    speed target --> slew limiter --> SimpleFOC velocity loop --> (current loops)
//                 --> SVPWM --> TIM1 6-PWM --> TMC6200 --> MOSFET bridge --> motor
//    TMC6200 SPI:   configuration at boot, status polling, fault decoding
//    TMC6200 FAULT: interrupt, drops DRV_EN immediately
//    TMC6200 CURx:  ADC1 --> phase currents --> Clarke/Park --> Id/Iq
//
//  All hardware-specific values live in include/board_config.h
// =============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

#include "board_config.h"
#include "tmc6200.h"

#define USE_MOTOR        (CONTROL_STAGE >= 1)
#define USE_SENSOR       (CONTROL_STAGE >= 2)
#define USE_FOC_CURRENT  (CONTROL_STAGE >= 3)

static const float RPM_PER_RAD_S = 9.5492966f;

// ----------------------------------------------------------------------------
//  Objects
// ----------------------------------------------------------------------------
tmc6200::Driver tmc(SPI, PIN_TMC_CSN, TMC_SPI_HZ);
tmc6200::Config tmcCfg;

Commander command = Commander(Serial);

#if USE_MOTOR
BLDCMotor      motor  = BLDCMotor(MOTOR_POLE_PAIRS);
// DRV_EN is given to SimpleFOC as the enable pin: motor.enable()/disable()
// drive it, so "disabled" always means the TMC6200 outputs are floating.
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL,
                                       PIN_TMC_DRV_EN);
// TMC6200 amplifiers measure in-line (phase) shunts -> InlineCurrentSense
InlineCurrentSense currentSense = InlineCurrentSense(SHUNT_RESISTANCE, (float)TMC_CS_GAIN,
                                                     PIN_CUR_U, PIN_CUR_V, PIN_CUR_W);
static bool currentSenseOk = false;
#endif

#if USE_SENSOR
#if SENSOR_TYPE == SENSOR_HALL
HallSensor sensor = HallSensor(PIN_SENS_A, PIN_SENS_B, PIN_SENS_C, MOTOR_POLE_PAIRS);
void sensorIsrA() { sensor.handleA(); }
void sensorIsrB() { sensor.handleB(); }
void sensorIsrC() { sensor.handleC(); }
#elif SENSOR_TYPE == SENSOR_ENCODER
Encoder sensor = Encoder(PIN_SENS_A, PIN_SENS_B, ENCODER_PPR);
void sensorIsrA() { sensor.handleA(); }
void sensorIsrB() { sensor.handleB(); }
#endif
#endif

// ----------------------------------------------------------------------------
//  State
// ----------------------------------------------------------------------------
static float targetRequested = 0.0f;   // [rad/s] what the user asked for
static float targetRamped    = 0.0f;   // [rad/s] what the controller gets
static volatile bool faultIrq = false; // set in the FAULT pin interrupt
static bool tripped = false;           // latched: stays off until 'C'

// ----------------------------------------------------------------------------
//  Safety
// ----------------------------------------------------------------------------
// FAULT rising edge: kill the power stage first, ask questions later.
void faultIsr() {
  digitalWrite(PIN_TMC_DRV_EN, LOW);
  faultIrq = true;
}

static void powerStageOff() {
#if USE_MOTOR
  motor.disable();                      // PWM to zero + DRV_EN low
#endif
  digitalWrite(PIN_TMC_DRV_EN, LOW);
}

static void trip(const char* reason) {
  powerStageOff();
  targetRequested = 0.0f;
  targetRamped = 0.0f;
  if (tripped) return;                  // report only once
  tripped = true;
  Serial.print("\n*** TRIP: ");
  Serial.println(reason);
  uint32_t gstat = tmc.readStatus();
  Serial.println("TMC6200 GSTAT:");
  tmc6200::Driver::printGstat(Serial, gstat);
  Serial.println("Power stage is OFF. Fix the cause, then send C to clear.");
}

static void fatal(const char* msg) {
  digitalWrite(PIN_TMC_DRV_EN, LOW);
  while (true) {
    Serial.print("FATAL: ");
    Serial.println(msg);
    delay(2000);
  }
}

static bool tmcSetup() {
  tmcCfg.singleLine  = false;           // 6-PWM. The chip's reset default is 3-PWM!
  tmcCfg.faultDirect = false;           // FAULT latched until DRV_EN goes low
  tmcCfg.csGain      = TMC_CS_GAIN;
  tmcCfg.drvStrength = TMC_DRV_STRENGTH;
  tmcCfg.s2vsLevel   = TMC_S2VS_LEVEL;
  tmcCfg.s2gLevel    = TMC_S2G_LEVEL;
  bool ok = tmc.configure(tmcCfg);
  tmc.clearStatus();                    // clears the power-on RESET flag
  return ok;
}

// Runs every loop: FAULT interrupt flag, e-stop button, periodic SPI status poll.
static void supervise() {
  if (faultIrq) {
    faultIrq = false;
    trip("TMC6200 FAULT pin went high");
  }
  if (digitalRead(PIN_ESTOP_BTN) == LOW) {
    trip("emergency stop button");
  }

  static uint32_t lastPoll = 0;
  uint32_t now = millis();
  if (!tripped && (now - lastPoll >= 200)) {
    lastPoll = now;
    uint32_t gstat = tmc.readStatus();
    if (gstat & tmc6200::GSTAT_RESET) {
      // VS dipped: registers are back at defaults (3-PWM mode!). Must not run.
      trip("TMC6200 reset detected (supply brown-out?)");
    } else if (gstat & tmc6200::GSTAT_FAULT_MASK) {
      trip("TMC6200 status error");
    } else if (gstat & tmc6200::GSTAT_DRV_OTPW) {
      Serial.println("warning: TMC6200 over-temperature pre-warning");
      tmc.clearStatus(tmc6200::GSTAT_DRV_OTPW);
    }
  }
}

// ----------------------------------------------------------------------------
//  Speed command handling
// ----------------------------------------------------------------------------
static void setTarget(float radPerSec) {
  if (tripped) {
    Serial.println("tripped - send C first");
    return;
  }
  targetRequested = constrain(radPerSec, -MAX_SPEED_RAD_S, MAX_SPEED_RAD_S);
  Serial.print("target ");
  Serial.print(targetRequested, 2);
  Serial.print(" rad/s  (");
  Serial.print(targetRequested * RPM_PER_RAD_S, 1);
  Serial.println(" rpm)");
}

// Slew-rate limit the target so a step command cannot demand a huge torque
// (and so the open-loop stage does not lose synchronisation).
static void updateRamp() {
  static uint32_t last = micros();
  uint32_t now = micros();
  float dt = (uint32_t)(now - last) * 1e-6f;
  last = now;
  if (dt > 0.01f) dt = 0.01f;
  float step = ACCEL_LIMIT_RAD_S2 * dt;
  float err = targetRequested - targetRamped;
  targetRamped += constrain(err, -step, step);
}

// ----------------------------------------------------------------------------
//  UART commands (terminate each with a newline)
//    T<x>   target speed in rad/s, sign = direction      T30   T-30
//    R<x>   target speed in rpm,   sign = direction      R500  R-500
//    D      reverse direction (same speed)
//    S      stop (ramp to zero)
//    E0/E1  disable / enable the power stage
//    C      clear a trip: re-configure TMC6200, clear GSTAT, re-enable
//    G      dump TMC6200 registers
//    I      print phase currents, speed, target
//    M...   full SimpleFOC motor interface (PID tuning, limits, monitoring);
//           works with SimpleFOCStudio. Use T/R for the speed, not M.
//    ?      list commands
// ----------------------------------------------------------------------------
void cmdTarget(char* cmd) {
  float v = targetRequested;
  command.scalar(&v, cmd);
  setTarget(v);
}

void cmdRpm(char* cmd) {
  float rpm = targetRequested * RPM_PER_RAD_S;
  command.scalar(&rpm, cmd);
  setTarget(rpm / RPM_PER_RAD_S);
}

void cmdReverse(char*) { setTarget(-targetRequested); }

void cmdStop(char*) { setTarget(0.0f); }

void cmdEnable(char* cmd) {
  if (cmd[0] == '1') {
    if (tripped) { Serial.println("tripped - send C first"); return; }
    targetRequested = 0.0f;
    targetRamped = 0.0f;
#if USE_MOTOR
    motor.enable();
#endif
    Serial.println("enabled");
  } else {
    powerStageOff();
    targetRequested = 0.0f;
    targetRamped = 0.0f;
    Serial.println("disabled");
  }
}

void cmdClear(char*) {
  powerStageOff();                      // DRV_EN low also releases the FAULT latch
  delay(5);
  if (!tmc.isConnected()) { Serial.println("TMC6200 not answering on SPI"); return; }
  if (!tmcSetup())        { Serial.println("TMC6200 config readback mismatch"); return; }
  delay(5);
  uint32_t gstat = tmc.readStatus();
  if ((gstat & tmc6200::GSTAT_FAULT_MASK) || digitalRead(PIN_TMC_FAULT) == HIGH) {
    Serial.println("fault still present:");
    tmc6200::Driver::printGstat(Serial, gstat);
    return;
  }
  faultIrq = false;
  tripped = false;
  targetRequested = 0.0f;
  targetRamped = 0.0f;
#if USE_MOTOR
  motor.enable();
#endif
  Serial.println("cleared, power stage enabled, target 0");
}

void cmdTmc(char*) { tmc.printStatus(Serial); }

void cmdInfo(char*) {
  Serial.print("stage ");
  Serial.print(CONTROL_STAGE);
  Serial.print(tripped ? "  TRIPPED" : "  ok");
  Serial.print("  target ");
  Serial.print(targetRamped, 2);
  Serial.print(" rad/s");
#if USE_MOTOR
  Serial.print("  speed ");
  Serial.print(motor.shaft_velocity, 2);
  Serial.print(" rad/s (");
  Serial.print(motor.shaft_velocity * RPM_PER_RAD_S, 1);
  Serial.print(" rpm)  Uq ");
  Serial.print(motor.voltage.q, 2);
  Serial.print(" V");
  if (currentSenseOk) {
    PhaseCurrent_s i = currentSense.getPhaseCurrents();
    Serial.print("  Iu/Iv/Iw ");
    Serial.print(i.a, 2); Serial.print(" / ");
    Serial.print(i.b, 2); Serial.print(" / ");
    Serial.print(i.c, 2); Serial.print(" A");
  }
#endif
  Serial.println();
}

#if USE_MOTOR
void cmdMotor(char* cmd) { command.motor(&motor, cmd); }
#endif

// ----------------------------------------------------------------------------
//  Setup
// ----------------------------------------------------------------------------
void setup() {
  // --- 1. Safe pin states before anything else
  digitalWrite(PIN_TMC_DRV_EN, LOW);
  pinMode(PIN_TMC_DRV_EN, OUTPUT);
  digitalWrite(PIN_TMC_DRV_EN, LOW);          // power stage off

  if (PIN_TMC_SPE >= 0) {
    pinMode(PIN_TMC_SPE, OUTPUT);
    digitalWrite(PIN_TMC_SPE, HIGH);          // select SPI mode
  }
  pinMode(PIN_TMC_FAULT, INPUT_PULLDOWN);
  pinMode(PIN_ESTOP_BTN, INPUT_PULLUP);

  // --- 2. UART
  Serial.begin(UART_BAUD);
  delay(300);
  Serial.println("\n=== PMSM FOC / TMC6200 / Nucleo-F401RE ===");
  Serial.print("control stage ");
  Serial.println(CONTROL_STAGE);
  SimpleFOCDebug::enable(&Serial);

  // --- 3. SPI + TMC6200
  SPI.setMOSI(PIN_TMC_MOSI);
  SPI.setMISO(PIN_TMC_MISO);
  SPI.setSCLK(PIN_TMC_SCK);
  SPI.begin();
  tmc.begin();
  delay(20);

  if (!tmc.isConnected()) {
    Serial.print("IOIN = 0x");
    Serial.println(tmc.readInputs(), HEX);
    fatal("TMC6200 not found on SPI. Check: VS (motor supply) is ON, SPE high, "
          "VCC_IO = 3V3, common GND, SCK/SDI/SDO/CSN wiring. "
          "0x00000000 or 0xFFFFFFFF means no answer at all.");
  }
  Serial.println("TMC6200 found (version 0x10)");

  uint32_t bootStatus = tmc.readStatus();
  Serial.println("GSTAT at boot (RESET is normal after power-up):");
  tmc6200::Driver::printGstat(Serial, bootStatus);

  if (!tmcSetup()) fatal("TMC6200 register readback mismatch (noisy SPI?)");
  delay(5);
  tmc.printStatus(Serial);

  uint32_t gstat = tmc.readStatus();
  if (gstat & tmc6200::GSTAT_FAULT_MASK) {
    tmc6200::Driver::printGstat(Serial, gstat);
    fatal("TMC6200 reports a fault before the power stage was even enabled");
  }

  attachInterrupt(digitalPinToInterrupt(PIN_TMC_FAULT), faultIsr, RISING);

#if USE_MOTOR
  // --- 4. Rotor sensor
#if USE_SENSOR
  sensor.pullup = Pullup::USE_INTERN;
  sensor.init();
#if SENSOR_TYPE == SENSOR_HALL
  sensor.enableInterrupts(sensorIsrA, sensorIsrB, sensorIsrC);
#else
  sensor.enableInterrupts(sensorIsrA, sensorIsrB);
#endif
  motor.linkSensor(&sensor);
#endif

  // --- 5. PWM driver (TIM1, 6-PWM, centre-aligned, hardware dead time)
  driver.pwm_frequency        = PWM_FREQUENCY_HZ;
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit        = SUPPLY_VOLTAGE;
  driver.dead_zone            = PWM_DEAD_ZONE;
  if (!driver.init()) fatal("PWM driver init failed (pins must be TIM1 CHx / CHxN pairs)");
  motor.linkDriver(&driver);

  // --- 6. Current sense (offsets are calibrated here, with the stage disabled)
  currentSense.linkDriver(&driver);
  currentSenseOk = (currentSense.init() == 1);
  Serial.println(currentSenseOk ? "current sense ready" : "current sense init FAILED");

  // --- 7. Motor + control configuration
  motor.foc_modulation       = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.velocity_limit       = MAX_SPEED_RAD_S;

#if CONTROL_STAGE == 1
  motor.controller    = MotionControlType::velocity_openloop;
  motor.voltage_limit = OPENLOOP_VOLTAGE;
#else
  motor.controller      = MotionControlType::velocity;
  motor.voltage_limit   = CL_VOLTAGE_LIMIT;
  motor.current_limit   = CL_CURRENT_LIMIT;
  motor.PID_velocity.P  = VEL_PID_P;
  motor.PID_velocity.I  = VEL_PID_I;
  motor.PID_velocity.D  = 0.0f;
  motor.LPF_velocity.Tf = VEL_LPF_TF;
#if USE_FOC_CURRENT
  if (!currentSenseOk) fatal("stage 3 needs working current sense");
  motor.linkCurrentSense(&currentSense);
  motor.torque_controller = TorqueControlType::foc_current;
  motor.PID_current_q.P = CUR_PID_P;
  motor.PID_current_q.I = CUR_PID_I;
  motor.PID_current_d.P = CUR_PID_P;
  motor.PID_current_d.I = CUR_PID_I;
  motor.LPF_current_q.Tf = CUR_LPF_TF;
  motor.LPF_current_d.Tf = CUR_LPF_TF;
#else
  motor.torque_controller = TorqueControlType::voltage;
#endif
#endif

  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0;         // off; enable from the M interface if wanted

  // --- 8. Start. init() enables the power stage (DRV_EN high).
  if (!motor.init()) fatal("motor init failed");
#if USE_SENSOR
  // Aligns sensor <-> electrical angle (the rotor will jerk and turn slowly),
  // and in stage 3 also checks current sense phase order and sign.
  if (!motor.initFOC()) {
    powerStageOff();
    fatal("initFOC failed: check sensor wiring, pole pairs, ALIGN_VOLTAGE");
  }
#endif
  command.add('M', cmdMotor, "SimpleFOC motor interface");
#else
  Serial.println("Stage 0: SPI check only. DRV_EN stays low, no PWM.");
#endif

  // --- 9. Commands
  command.add('T', cmdTarget,  "target speed [rad/s], sign = direction");
  command.add('R', cmdRpm,     "target speed [rpm], sign = direction");
  command.add('D', cmdReverse, "reverse direction");
  command.add('S', cmdStop,    "stop");
  command.add('E', cmdEnable,  "E1 enable / E0 disable power stage");
  command.add('C', cmdClear,   "clear trip and re-enable");
  command.add('G', cmdTmc,     "dump TMC6200 registers");
  command.add('I', cmdInfo,    "currents / speed / state");

  Serial.println("Ready. Send ? for the command list. Blue button = emergency stop.");
}

// ----------------------------------------------------------------------------
//  Main loop - runs as fast as possible, no delays allowed in here
// ----------------------------------------------------------------------------
void loop() {
#if USE_MOTOR
  motor.loopFOC();                      // angle -> Park/Clarke -> current PI -> SVPWM
  updateRamp();
  motor.move(targetRamped);             // velocity loop (or open-loop angle generator)
#endif
  supervise();
  command.run();
}
