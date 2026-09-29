# Motor controller sample

Runs a closed speed loop on the motor of the board and tunes it from the
shell, through the [motor control subsystem](../../../subsys/motor_control/README.md).
The application holds no motor code: each board overlay describes the power
stage, picks the motor driver and declares the controller over it.

## Nucleo-G431RB + X-NUCLEO-IHM16M1

The overlay is the one of the [motor driver sample](../../driver/motor/README.md),
plus a controller over its motor: see there for the shield rework, the
jumpers and the wiring.

The loop runs at 500 Hz. Its gains are conservative seeds, and the ramp
accelerates by 1000 RPM/s: tune both on the bench for the motor and its
load.

## Building and running

```console
make
make flash
```

On the console (ST-Link virtual COM port, 115200 baud):

```console
uart:~$ mc start
uart:~$ mc target 1000
uart:~$ sensor get feedback
uart:~$ mc speed kp 0.05
uart:~$ mc list
uart:~$ mc stop
```

The setpoint is in RPM, its sign sets the direction. The gains are in
per-mille of the supply voltage per RPM of error for kp, and per RPM and
per second for ki. Starting resets the setpoint to 0, so set it after
`mc start`.
