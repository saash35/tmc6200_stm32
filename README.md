# PMSM FOC firmware: Nucleo-F401RE + TMC6200 (SPI) + SimpleFOC

```
foc_tmc6200/
├── platformio.ini            build / flash / monitor settings
├── include/board_config.h    EVERY pin and every motor/driver number (edit this)
├── lib/TMC6200/              register-level SPI driver for the TMC6200
└── src/main.cpp              SimpleFOC setup, safety supervisor, UART commands
```

## 1. Toolchain (about 15 minutes, mostly downloads)

1. Install **VS Code**, then the **PlatformIO IDE** extension (Extensions panel, search "PlatformIO"). Restart VS Code when it asks.
2. Windows only: install the ST-Link USB driver (STSW-LINK009 from st.com).
   Linux only: install the PlatformIO udev rules (`99-platformio-udev.rules`) and re-plug the board.
3. `File > Open Folder` and pick the `foc_tmc6200` folder (the one containing `platformio.ini`).
4. Click the tick (Build) in the bottom status bar. The first build downloads the ARM GCC compiler, the STM32 Arduino core and SimpleFOC. **Start this first, it is the slow part.**
5. Plug the Nucleo in by USB. Click the arrow (Upload). It flashes through the on-board ST-Link.
6. Click the plug icon (Serial Monitor). 115200 baud. Type a command and press Enter.

No STM32CubeIDE, no separate programmer, no bootloader needed.

## 2. Wiring

| TMC6200 pin | MCU pin | Nucleo header | Function |
|---|---|---|---|
| UH | PA8 | D7 | TIM1_CH1, high-side gate input U |
| UL | PB13 | CN10-30 | TIM1_CH1N, low-side gate input U |
| VH | PA9 | D8 | TIM1_CH2 |
| VL | PB14 | CN10-28 | TIM1_CH2N |
| WH | PA10 | D2 | TIM1_CH3 |
| WL | PB15 | CN10-26 | TIM1_CH3N |
| SCK_IDRV1 | PA5 | D13 | SPI1 SCK |
| SDO_SINGLE | PA6 | D12 | SPI1 MISO |
| SDI_AMPLx10 | PA7 | D11 | SPI1 MOSI |
| CSN_IDRV0 | PB6 | D10 | chip select |
| SPE | PB5 | D4 | driven high = SPI mode |
| DRV_EN | PB10 | D6 | high = power stage on |
| FAULT | PB4 | D5 | interrupt input |
| CURU | PA0 | A0 | ADC1_IN0 |
| CURV | PA1 | A1 | ADC1_IN1 |
| CURW | PA4 | A2 | ADC1_IN4 |
| VCC_IO | 3V3 | CN7-16 | logic level reference |
| GND | GND | CN7-20 | common ground |
| CLK | GND | - | internal 24 MHz clock |
| VOFS/TEST | open | - | amplifier offset = 5VOUT/3 |

Rotor sensor: PB8 (D15), PB9 (D14), PC7 (D9). UART: USART2 on PA2/PA3, already wired to the ST-Link USB port.

Hardware points to check before power:

- **Pull-down on DRV_EN** (10 k to GND). The pin has no internal pull, so without it the bridge state is undefined while the MCU is in reset or being flashed.
- **CURx can swing to about 4.9 V** if an amplifier saturates; the STM32 ADC pins take 3.3 V max. Put 1 k to 2.2 k in series with each CURx line, and pick shunt and gain so normal currents stay inside the linear range.
- The TMC6200 only answers on SPI when **VS (motor supply) is present**.
- Use a **current-limited bench supply** for all bring-up. Power the Nucleo from USB only.
- If your board hard-wires SPE high, set `PIN_TMC_SPE` to `-1`.

## 3. Bring-up (set `CONTROL_STAGE` in board_config.h, rebuild, flash)

**Stage 0: SPI link.** No PWM. Expect `TMC6200 found (version 0x10)` and a register dump showing `6-PWM` and your gain. Send `G` for the dump again. Do not go on until this is clean.

**Stage 1: open loop.** Set `MOTOR_POLE_PAIRS` and keep `OPENLOOP_VOLTAGE` low. Supply current limit at 1 to 2 A. Send `T5`: the rotor should turn smoothly at 5 rad/s. `T-5` reverses. Send `I` and check the three phase currents look sinusoidal and sum to about zero; that proves the current sense before you rely on it.

**Stage 2: closed loop, voltage mode.** Set `SENSOR_TYPE`. At boot the motor aligns itself (it jerks and turns slowly). Then `R300` runs at 300 rpm. Tune `VEL_PID_P/I` live with the `M` commands.

**Stage 3: closed loop with current loops.** Same as stage 2 plus Id/Iq PI controllers. Set `CUR_PID_P/I` and `CL_CURRENT_LIMIT` first. This is the structure of your Simulink model.

## 4. UART commands (115200, end with Enter)

| Command | Action |
|---|---|
| `T30` / `T-30` | speed in rad/s, sign is direction |
| `R500` / `R-500` | speed in rpm, sign is direction |
| `D` | reverse direction at the same speed |
| `S` | stop (ramps to zero) |
| `E0` / `E1` | disable / enable power stage |
| `C` | clear a trip, re-write TMC6200 config, re-enable |
| `G` | TMC6200 register dump with decoded faults |
| `I` | phase currents, speed, target, Uq |
| `M...` | SimpleFOC motor interface (e.g. `MVP0.3` velocity P, `MVI5` velocity I, `MLU6` voltage limit). Compatible with SimpleFOCStudio |
| `?` | list commands |

Blue button on the Nucleo is an emergency stop. Any TMC6200 fault, a chip reset, or the button latches the stage off until `C`.

## 5. What was and was not verified

- Register map, SPI protocol and pin functions were checked against the TMC6200 datasheet rev 1.08.
- All stage/sensor combinations were syntax-checked against the SimpleFOC 2.4.0 sources.
- Pin-to-peripheral assignments were checked against the STM32 Arduino core tables for the F401RE.
- **Not done:** a real ARM build (the build machine could not reach the PlatformIO registry) and anything on hardware. Expect to fix small things on first build.
