# 6-step BLDC driver on pwm_bridge and bldc_hall

Zephyr driver for the trapezoidal commutation of a three-phase BLDC motor
with Hall sensors, built on the device classes of this module: a
[pwm_bridge](../../pwm_bridge/README.md) drives the phases, a
[bldc_hall](../../bldc_hall/README.md) reports the Hall edges, and a
[bldc_adc](../../bldc_adc/README.md) sampler of the phase currents can be
bound. It owns no hardware and holds no vendor code: on every Hall edge it
decides what each phase does.

It implements the generic device class, **motor**
([motor.h](../../../include/drivers/motor.h)), the only API a controller
sees: every motor is driven the same way, whatever the implementation behind
it, with start, stop, and a signed voltage whose sign selects the direction.

Regulation loops (current, speed, position) and filtering stay in the
consumer, which commands a voltage and reads back the raw shaft speed and
angle, and the raw phase currents when a sampler is bound. The driver only
runs what has to follow the switching: the commutation.

## Contract

- every Hall edge commutates the bridge from the interrupt, before the
  feedback is updated;
- the voltage may be set before starting, and applies at start;
- a zero voltage brakes the rotor, only stopping the motor releases it;
- an impossible Hall state releases every phase until a valid one returns,
  and the motor refuses to start on one;
- the feedback is raw, and follows the rotor even while the motor is
  stopped;
- the feedback carries the current of each phase only when a sampler is
  bound, measured from the zero-current point found at init.

## Commutation

The Hall state packs the three lines, U being the most significant bit. A
forward table gives, for each of the six valid states, one leg `PWM` at the
commanded duty, one `FORCE_LOW` and one `HI_Z`. States 0 and 7 are sensor
faults and set all three `HI_Z`. The pattern goes to the bridge in one
commit, so every leg switches on the same carrier cycle.

The voltage magnitude sets the duty against the bridge full scale, rounded
to the nearest cycle. A negative voltage swaps the `PWM` and `FORCE_LOW`
legs, which reverses the rotation.

Both active legs are complementary pairs: the low side of the chopped leg
carries the freewheel current, and the grounded leg keeps the dead-time
generator in the path for the next commutation. At zero duty the two low
sides therefore short the winding and hold the rotor. A fail-safe path that
expects zero voltage to coast has to stop the motor instead.

## Feedback

On each edge, after the commutation, the previous and new Hall states give
the direction and move a signed step count. An invalid pair moves nothing
and is not a reversal. The angle is recomputed from the count, 60° per step
divided by `motor-pole-pairs`, so rounding never accumulates.

The speed comes from the interval the Hall device timed, signed by the
measured direction. A read bounds it by the time since the last edge, so a
stopping rotor reads slower and slower instead of frozen at its last speed,
and reads 0 once the Hall counter reports no valid interval.

With an `adc`, the phase currents come from the last sample of the bldc_adc,
taken once per PWM period, in U/V/W order. At init, before the motor can
run, the driver averages a few sequences per phase to find the zero-current
count; each reading is then its deviation from that count, scaled to
milliamps by the reference voltage the sampler measured, the amplifier gain
and the shunt. The offset is the only averaged value: every current read
is a single sample. Without an `adc`, the feedback reports no phase.

A [`zephyr,motor-feedback`](../feedback/README.md) child serves the angle,
the speed and the phase currents through the sensor API.

## Devicetree

- [`zephyr,motor.yaml`](../../../dts/bindings/motor/zephyr,motor.yaml):
  the generic compatible, no property;
- [`zephyr,bldc-6step-bridge-hall.yaml`](../../../dts/bindings/motor/zephyr,bldc-6step-bridge-hall.yaml):
  `pwm`, `hall`, `adc`, `shunt-resistance-uohm`, `current-gain-milli`,
  `motor-pole-pairs`.

The node lists its own compatible first and `zephyr,motor` second, which is
what lets the `motor` shell and other motor consumers enumerate it. It has
no cells: consumers reference it by phandle.

```dts
/ {
        motor0: motor0 {
                compatible = "zephyr,bldc-6step-bridge-hall", "zephyr,motor";
                status = "okay";

                pwm = <&bridge0>;
                hall = <&hall_motor0>;
                adc = <&adc_motor0>;
                shunt-resistance-uohm = <60000>;
                current-gain-milli = <1318>;
                motor-pole-pairs = <4>;

                motor0_feedback: feedback {
                        compatible = "zephyr,motor-feedback";
                };
        };
};
```

`pwm` points to a pwm_bridge with three legs, `hall` to the bldc_hall
device of the same motor. `adc` is optional and points to a bldc_adc
triggered by that same bridge, its channels in U/V/W order; the shunt and
the gain of the current-sense front-end come with it. The conversion
assumes 12-bit counts, which the bldc_adc class does not report.

The build fails if the node does not list `zephyr,motor`, if
`motor-pole-pairs` is 0, or if `adc` names a sampler synchronized to another
bridge or comes without `shunt-resistance-uohm` and `current-gain-milli`.
Init returns an error on a bound device not ready, a bridge that does not
drive three legs or lacks the `HI_Z`, `PWM` or `FORCE_LOW` state, a Hall
device that fails to report its counter rate or edge count, or a sampler
that fails to report its reference voltage or to deliver the calibration
sequences.
