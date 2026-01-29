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
- `syringe-filler-multiple/platformio.ini`: PlatformIO configuration for the
  multi-syringe ESP32 build.
- `syringe-filler-single/`: Placeholder for single-syringe firmware work.
- `shared/lib/`: Shared PlatformIO libraries available to firmware projects.

## Build (PlatformIO)

1. Install PlatformIO (`pio`) and the ESP32 toolchain.
2. Open `syringe-filler-multiple/` as the project root.
3. Build and upload:

```bash
pio run
pio run -t upload
```

Optional serial monitor:

```bash
pio device monitor
```

## Build (Arduino IDE)

1. Open `HandheldStepperControl/HandheldStepperControl.ino` in the Arduino IDE.
2. Select the appropriate board and port.
3. Compile and upload to the handheld controller target.

## Recipe JSON Client

A lightweight serial JSON client is available in
`syringe-filler-multiple/tools/` and is documented in
`syringe-filler-multiple/docs/recipes.md`. It validates recipe JSON using the
firmware's `Util::Recipe` schema before sending `sfc.recipe.save`.

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
