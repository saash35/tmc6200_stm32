// =============================================================================
//  board_config.h  -  THE ONLY FILE YOU SHOULD NEED TO EDIT
//
//  Nucleo-F401RE  <->  TMC6200 (SPI mode, 6-PWM)  <->  48 V PMSM
//
//  Everything marked  <<< SET ME  is a placeholder. The firmware will build
//  with the placeholders, but the motor will not run correctly (or safely)
//  until they match your hardware.
// =============================================================================
#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
//  1. BRING-UP STAGE  - move up one step at a time
// -----------------------------------------------------------------------------
//  0 = TMC6200 only: SPI link check + register dump. No PWM, DRV_EN stays low.
//  1 = Open-loop velocity. No rotor sensor needed. Proves PWM, gate drive,
//      phase wiring and the UART commands.
//  2 = Closed-loop velocity with rotor sensor, voltage-mode torque.
//  3 = Closed-loop velocity with rotor sensor + d/q current loops
//      (the same cascade as a textbook / Simulink FOC model).
#define CONTROL_STAGE 1

// -----------------------------------------------------------------------------
//  2. ROTOR POSITION SENSOR  (needed for stage 2 and 3)
// -----------------------------------------------------------------------------
#define SENSOR_NONE     0
#define SENSOR_HALL     1    // 3 hall sensors, 120 deg
#define SENSOR_ENCODER  2    // incremental A/B (index not used)
#define SENSOR_TYPE     SENSOR_NONE          // <<< SET ME
#define ENCODER_PPR     1024                 // <<< SET ME (pulses per rev, before x4)

// -----------------------------------------------------------------------------
//  3. MOTOR + SUPPLY
// -----------------------------------------------------------------------------
#define MOTOR_POLE_PAIRS      4       // <<< SET ME (number of poles / 2)
#define SUPPLY_VOLTAGE        48.0f   // [V] DC bus

// Open-loop (stage 1) and sensor-alignment voltage, applied as a Uq vector.
// Phase current is roughly  V / R_phase , so with a 0.2 ohm phase 1 V is ~5 A.
// START LOW. Raise only if the motor will not turn.
#define OPENLOOP_VOLTAGE      1.0f    // [V]  <<< SET ME (= I_wanted * R_phase)
#define ALIGN_VOLTAGE         1.0f    // [V]  <<< SET ME

// Closed-loop limits
#define CL_VOLTAGE_LIMIT      12.0f   // [V] max Uq/Ud magnitude in stage 2/3
#define CL_CURRENT_LIMIT      5.0f    // [A] max Iq in stage 3      <<< SET ME

// Speed command limits (mechanical rad/s;  1 rad/s = 9.549 rpm)
#define MAX_SPEED_RAD_S       100.0f  // [rad/s] clamp for commands  <<< SET ME
#define ACCEL_LIMIT_RAD_S2    20.0f   // [rad/s^2] slew rate of the speed target

// PWM
#define PWM_FREQUENCY_HZ      20000   // centre-aligned, TIM1
// Dead time as a fraction of the PWM period, inserted by TIM1.
// 0.02 @ 20 kHz = 1.0 us. Must exceed your MOSFET turn-off time.
// (TMC6200's own BBMCLKS setting is NOT active in 6-PWM mode.)
#define PWM_DEAD_ZONE         0.02f   // <<< SET ME for your MOSFETs

// Controller gains. Copy these from your Simulink model once it runs.
//   PID_velocity:  output is Uq [V] in stage 2, Iq [A] in stage 3
#define VEL_PID_P             0.2f
#define VEL_PID_I             2.0f
#define VEL_LPF_TF            0.02f   // [s] velocity low-pass (hall: 0.02-0.05)
//   Current PI (stage 3):  P ~ L*wc [V/A],  I ~ R*wc [V/(A*s)]
#define CUR_PID_P             1.0f    // <<< SET ME
#define CUR_PID_I             100.0f  // <<< SET ME
#define CUR_LPF_TF            0.005f  // [s]

// -----------------------------------------------------------------------------
//  4. CURRENT SENSING (TMC6200 internal amplifiers, in-line phase shunts)
// -----------------------------------------------------------------------------
//  CURx = 5VOUT/3 (~1.67 V) + gain * I * Rshunt.
//  Linear differential input range with the default offset:
//     gain 5: +-200 mV    gain 10: +-100 mV    gain 20: +-40 mV
//  e.g. 5 mOhm shunt, gain 10  ->  +-20 A full scale, 50 mV/A at the ADC.
#define SHUNT_RESISTANCE      0.005f  // [ohm]  <<< SET ME
#define TMC_CS_GAIN           10      // 5, 10 or 20 (written to GCONF)

// -----------------------------------------------------------------------------
//  5. TMC6200 GATE DRIVER SETTINGS  (written over SPI at boot)
// -----------------------------------------------------------------------------
#define TMC_DRV_STRENGTH      2       // 0 weak, 1 weak+TC, 2 medium (reset default), 3 strong
#define TMC_S2VS_LEVEL        6       // short-to-supply sensitivity (reset default 6)
#define TMC_S2G_LEVEL         6       // short-to-ground sensitivity (reset default 6)
#define TMC_SPI_HZ            1000000 // max 3.5 MHz with the internal oscillator

// =============================================================================
//  6. PIN MAP - every TMC6200 pin that touches the MCU
// =============================================================================
//
//  TMC6200 pin        MCU pin   Nucleo header     Peripheral       Direction
//  -----------------  --------  ----------------  ---------------  ---------
//  UH   (17)          PA8       D7  / CN10-23     TIM1_CH1         MCU -> TMC
//  UL   (18)          PB13      CN10-30 (morpho)  TIM1_CH1N        MCU -> TMC
//  VH   (20)          PA9       D8  / CN10-21     TIM1_CH2         MCU -> TMC
//  VL   (21)          PB14      CN10-28 (morpho)  TIM1_CH2N        MCU -> TMC
//  WH   (22)          PA10      D2  / CN10-33     TIM1_CH3         MCU -> TMC
//  WL   (23)          PB15      CN10-26 (morpho)  TIM1_CH3N        MCU -> TMC
//  SCK_IDRV1   (14)   PA5       D13 / CN10-11     SPI1_SCK         MCU -> TMC
//  SDO_SINGLE  (16)   PA6       D12 / CN10-13     SPI1_MISO        TMC -> MCU
//  SDI_AMPLx10 (15)   PA7       D11 / CN10-15     SPI1_MOSI        MCU -> TMC
//  CSN_IDRV0   (13)   PB6       D10 / CN10-17     GPIO out         MCU -> TMC
//  SPE  (25)          PB5       D4  / CN10-29     GPIO out (high)  MCU -> TMC
//  DRV_EN (27)        PB10      D6  / CN10-25     GPIO out         MCU -> TMC
//  FAULT (26)         PB4       D5  / CN10-27     GPIO in, EXTI4   TMC -> MCU
//  CURU (9)           PA0       A0  / CN7-28      ADC1_IN0         TMC -> MCU
//  CURV (10)          PA1       A1  / CN7-30      ADC1_IN1         TMC -> MCU
//  CURW (11)          PA4       A2  / CN7-32      ADC1_IN4         TMC -> MCU
//  VCC_IO (19)        3V3       CN7-16            -                supply
//  GND                GND       CN7-20 / CN10-9   -                common ground
//  CLK (24)           GND       -                 -                tie low = internal 24 MHz clock
//  VOFS/TEST (12)     open      -                 -                open = 5VOUT/3 amplifier offset
//
//  Not MCU pins (power stage side, must already be right on your board):
//  VS, VSA, 5VOUT, 12VOUT, VCP, CPI, CPO, GNDA, CU/CV/CW (bootstrap),
//  HSU/HSV/HSW, LSU/LSV/LSW (gates), U/V/W (bridge centres),
//  USENSE/VSENSE/WSENSE (shunt sense inputs).
//
//  Used on the Nucleo itself:
//  PA2/PA3   USART2 TX/RX -> ST-Link virtual COM port (the UART command port)
//  PA13/PA14 SWD (flash/debug)       PC13 blue user button = emergency stop
//  NOTE: PA5 also drives the green LD2 LED. Leave it; do not use LED_BUILTIN.
// =============================================================================

// --- 6-PWM gate inputs (all on TIM1, complementary pairs -> hardware dead time)
#define PIN_UH          PA8
#define PIN_UL          PB13
#define PIN_VH          PA9
#define PIN_VL          PB14
#define PIN_WH          PA10
#define PIN_WL          PB15

// --- SPI1 to TMC6200
#define PIN_TMC_SCK     PA5
#define PIN_TMC_MISO    PA6     // <- TMC SDO
#define PIN_TMC_MOSI    PA7     // -> TMC SDI
#define PIN_TMC_CSN     PB6

// --- TMC6200 control / status
#define PIN_TMC_SPE     PB5     // high = SPI mode. Set to -1 if SPE is hard-wired to VCC_IO
#define PIN_TMC_DRV_EN  PB10    // high = power stage enabled
#define PIN_TMC_FAULT   PB4     // high = driver error

// --- Current sense amplifier outputs (ADC1)
#define PIN_CUR_U       PA0
#define PIN_CUR_V       PA1
#define PIN_CUR_W       PA4

// --- Rotor sensor inputs (5 V tolerant pins, internal pull-ups enabled)
//     Hall: A/B/C = H1/H2/H3.   Encoder: A/B, C unused.
#define PIN_SENS_A      PB8     // D15 / CN10-3
#define PIN_SENS_B      PB9     // D14 / CN10-5
#define PIN_SENS_C      PC7     // D9  / CN10-19

// --- Emergency stop: blue user button on the Nucleo (active low)
#define PIN_ESTOP_BTN   PC13

// --- UART command port (USART2 via ST-Link USB)
#define UART_BAUD       115200

// -----------------------------------------------------------------------------
//  Sanity checks
// -----------------------------------------------------------------------------
#if (CONTROL_STAGE >= 2) && (SENSOR_TYPE == SENSOR_NONE)
#error "CONTROL_STAGE 2 and 3 need a rotor sensor: set SENSOR_TYPE in board_config.h"
#endif
#if (TMC_CS_GAIN != 5) && (TMC_CS_GAIN != 10) && (TMC_CS_GAIN != 20)
#error "TMC_CS_GAIN must be 5, 10 or 20"
#endif
