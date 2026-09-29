# Motor controller

Closed speed loop over a motor of this module: it commands the motor through
the generic [motor](../../include/drivers/motor.h) class and reads its speed
from a sensor, the motor's
[`zephyr,motor-feedback`](../../drivers/motor/feedback/README.md) child
typically. It owns no hardware and holds no motor or vendor code, so it runs
over any motor that implements the class. The setpoint ramp and the speed PID
are the ramp and pid libraries of this module.

This follows the split of the Zephyr motor control RFC
([#102158](https://github.com/zephyrproject-rtos/zephyr/issues/102158)): the
drivers are the power stage and the sensors, and the controller, with its
loops and state, is a subsystem above them. Each controller is a device
declared in devicetree, driven by plain functions rather than a driver API.

## Contract

- the requested speed is reached through a ramp, whose slew rate is gentler
  toward 0 when a deceleration limit is set, and which stops on 0 before a
  reversal;
- a speed PID turns the error into the signed voltage applied to the motor,
  within the output limits; there is no inner current loop;
- starting resets the setpoint to 0 and the PID, so a speed requested while
  stopped is lost;
- a failed speed read applies a zero voltage, which brakes the rotor rather
  than releasing it;
- stopping zeroes the voltage and stops the motor, which releases the rotor;
- every controller runs its tick on one shared work queue, fired by its own
  timer at the regulation period.

## Units

| Quantity | Unit |
| -------- | ---- |
| setpoint and speed | RPM, signed by the direction |
| loop output | per-mille of the supply voltage, -1000..1000 |
| kp | per-mille per RPM of error |
| ki | per-mille per RPM of error and per second |
| kd | per-mille per RPM/s of error change |

Devicetree has no floating-point type: gains are given × 1000000 and the
integral bounds × 1000, then converted to floats at init.

## Devicetree

- [`zephyr,motor-control.yaml`](../../dts/bindings/motor_control/zephyr,motor-control.yaml):
  `motor`, `motor-sensor`, `control-period-ms`, the `speed-pid-*` gains and
  integral bounds, `output-min-milli`, `output-max-milli`,
  `ramp-acceleration-rpm-per-s`, `ramp-deceleration-rpm-per-s`.

```dts
/ {
        mc0: mc0 {
                compatible = "zephyr,motor-control";
                status = "okay";

                motor = <&motor0>;
                motor-sensor = <&motor0_feedback>;
                control-period-ms = <2>;
                speed-pid-kp-micro = <20000>;
                speed-pid-ki-micro = <100000>;
                ramp-acceleration-rpm-per-s = <1000>;
        };
};
```

The build fails if `motor` is not a `zephyr,motor` node, or if the output
limits leave -1000..1000 or are out of order. Init returns an error on a
motor or a sensor not ready, or a PID configuration that is invalid.

## Shell

With `CONFIG_MOTOR_CONTROL_SHELL`, the `mc` command lists the controllers
and their gains, starts and stops them, sets the setpoint, resets the PID and
tunes each gain at runtime. The controller name may be omitted when only one
is declared. Printing the gains needs `CONFIG_CBPRINTF_FP_SUPPORT`.
