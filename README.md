# Syringe Filler Control (PlatformIO)

Firmware and helper sketches for a syringe filling system using stepper drivers,
limit switches, and optional I2C peripherals (PCA9685 + ADS1115).

## Repository Layout

- `HandheldStepperControl/HandheldStepperControl.ino`: Handheld, hold-to-run
  stepper controller for quick manual movement.
- `SyringeFillControl/SyringeFillControl.ino`: Primary ESP32 firmware with
  multi-axis motion, servo control, and pot feedback.
- `SyringeFillControl/test.ino`: Bench-test sketch for steppers, homing, and
  servo control.
- `syringe-filler-pio/platformio.ini`: PlatformIO configuration for the ESP32
  build.

## Build (PlatformIO)

1. Install PlatformIO (`pio`) and the ESP32 toolchain.
2. Open `syringe-filler-pio/` as the project root.
3. Build and upload:

```bash
pio run
pio run -t upload
```

Optional serial monitor:

```bash
pio device monitor
```

## Serial Command Summary

The main firmware prints a command list at startup. Common commands include:

- `on` / `off`: Enable or disable the main stepper driver.
- `speed <sps>`: Set main axis speed (steps per second).
- `dir <0|1>`: Set direction for the main axis.
- `home`: Run the homing routine.
- `pos`, `pos2`, `pos3`: Position readouts.
- `move <steps>`, `movemm <mm>`: Relative moves on the main axis.
- `goto <steps>`, `gotomm <mm>`: Absolute moves on the main axis.
- `move2 <steps>`, `move3 <steps>`: Single-axis moves for auxiliaries.
- `m23 <steps2> <steps3>` / `link <steps>`: Synchronized auxiliary moves.
- `servo <channel> <angle>`: Servo angle control via PCA9685.
- `servoslow <channel> <angle> [delay_ms]`: Slow servo sweep.
- `potmove <target_adc> <sps>`: Pot-driven move on axis 2.

## License

Commercial use is permitted worldwide except within the United States, where a
separate commercial license is required.
