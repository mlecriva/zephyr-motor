# Motor feedback sensor driver

Zephyr sensor serving the shaft speed and angle of a motor, and its phase
currents when the motor measures them, as a child of its motor node. It owns
no hardware and holds no motor or vendor code: it polls its parent through
the motor API, so it works with any motor that reports feedback.

It implements the Zephyr sensor API, so the control layer above the motor
(speed and position loops, software safeties) and the sensor shell read it
like any other sensor.

Filtering stays in the consumer, which gets raw values only, since the right
filter depends on its own rate.

## Contract

- a fetch polls the motor once and latches every value together, a get
  returns the latched value;
- the angle is cumulative and signed since init, the speed signed by the
  measured direction;
- a phase current is served only when the motor measures that phase, and
  reading it otherwise fails;
- the sensor fails to initialize on a motor that reports no feedback.

## Channels

| Channel | Unit | Value |
| ------- | ---- | ----- |
| `SENSOR_CHAN_ROTATION` | degrees | shaft angle, cumulative and signed since init |
| `SENSOR_CHAN_RPM` | RPM | shaft speed, signed by the measured direction |
| `SENSOR_CHAN_MOTOR_PHASE_CURRENT_U` | A | current of the first phase, signed, last sample |
| `SENSOR_CHAN_MOTOR_PHASE_CURRENT_V` | A | current of the second phase, signed, last sample |
| `SENSOR_CHAN_MOTOR_PHASE_CURRENT_W` | A | current of the third phase, signed, last sample |

All come from `motor_get_feedback()`, in micro-degrees, milli-RPM and
milliamps. `val2` carries the fractional part in millionths, with the same
sign as `val1`. The integer part of the angle wraps past `INT32_MAX`
degrees.

The phase-current channels are private, numbered from
`SENSOR_CHAN_PRIV_START` in
[motor_feedback.h](../../../include/drivers/sensor/motor_feedback.h). The
sensor shell only reads the standard channels unless given a channel number.

## Devicetree

- [`zephyr,motor-feedback.yaml`](../../../dts/bindings/motor/zephyr,motor-feedback.yaml):
  no property of its own.

The node is a child of the `zephyr,motor` node it reads.

```dts
motor0: motor0 {
        compatible = "zephyr,bldc-6step-bridge-hall", "zephyr,motor";
        /* ... */

        motor0_feedback: feedback {
                compatible = "zephyr,motor-feedback";
        };
};
```

The build fails if the parent is not a `zephyr,motor` node. Init returns an
error on a motor not ready, one that reports no feedback, or a first read
that fails.
