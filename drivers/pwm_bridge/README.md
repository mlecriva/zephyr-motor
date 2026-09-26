# STM32 bridge output stage driver

Zephyr driver for the half-bridge legs of a switched power stage, built on an
STM32 advanced-control timer (TIM1/TIM8), which it owns exclusively. Each leg
is one capture/compare channel plus its complementary output with hardware
dead-time.

It implements its own device class, **pwm_bridge**
([pwm_bridge.h](../../include/drivers/pwm_bridge.h)), not the Zephyr PWM API.
The PWM API lacks what a power bridge needs: switching every leg at once,
holding a switch on for a whole period, and cutting the outputs at runtime.

Modulation stays in the consumer, which gives a state and a pulse width for
each leg. A 6-step modulator chops one leg, grounds one and releases the
third. A sinusoidal or SVPWM modulator chops all three with different widths.

## Contract

- one carrier shared by every leg of a device;
- a new pattern switches all legs on the same carrier cycle, can be applied
  from an interrupt, and re-applying unchanged states is harmless;
- pulse widths take effect at the next period boundary, never mid-pulse;
- outputs stay off at boot until the consumer explicitly enables them.

## Legs

Leg count (1 to 4) follows from `st,leg-channels` and is read back with
`pwm_bridge_get_caps()`: 1 = chopper/PFC, 2 = H-bridge, 3 = three-phase
inverter, 4 = stepper or three-phase + neutral. More legs would need several
synchronised timers, i.e. another driver behind the same API.

| State | Behavior |
| ----- | -------- |
| `HI_Z` | both switches off, phase released |
| `PWM` | high side chopped, low side complementary with dead-time |
| `FORCE_LOW` | low side conducts the whole period |
| `FORCE_HIGH` | high side conducts the whole period |

`caps.state_mask` advertises the supported states. Every state but `HI_Z`
drives both switches, so freewheel current goes through the MOSFET instead of
the body diode. As a result, a `PWM` leg at 0% duty **brakes** (the low side
shorts the winding). To coast, use `HI_Z` or disable the outputs.

## Carrier and sync trigger

The counter is center-aligned: `ARR = f_counter / (2 × f_pwm)`, which is also
`caps.max_pulse_cycles`. Because `ARR` is truncated, `caps.carrier_hz` gives
the actual rate, which may differ from the requested `pwm-frequency-hz`.

With `sync-trigger-enable`, `TRGO2` fires once per period on the update event
(`CNT = 0`, center of the high-side pulse), e.g. to trigger an ADC. It uses
no channel, and the counter runs from init, so the trigger keeps firing while
the outputs are off.

## Devicetree

- [`pwm-bridge-device.yaml`](../../dts/bindings/pwm_bridge/pwm-bridge-device.yaml):
  common properties `pwm-frequency-hz`, `deadtime-ns`, `sync-trigger-enable`,
  `break-input`;
- [`st,stm32-pwm-bridge.yaml`](../../dts/bindings/pwm_bridge/st,stm32-pwm-bridge.yaml):
  `st,leg-channels`, the timer channel of each leg.

The node is a child of `st,stm32-timers`, which provides `reg`, `clocks`,
`resets` and `st,prescaler`. It has no `#pwm-cells`: consumers reference it
by phandle.

```dts
&timers1 {
        status = "okay";
        st,prescaler = <0>;

        bridge0: pwm-bridge {
                compatible = "st,stm32-pwm-bridge";
                status = "okay";

                pinctrl-0 = <&tim1_ch1_pe9  &tim1_ch1n_pe8
                             &tim1_ch2_pe11 &tim1_ch2n_pe10
                             &tim1_ch3_pe13 &tim1_ch3n_pe12>;
                pinctrl-names = "default";

                st,leg-channels = <1 2 3>;

                pwm-frequency-hz = <20000>;
                deadtime-ns = <500>;
                sync-trigger-enable;
        };
};
```

The build fails if the parent is not TIM1/TIM8, if a leg count or channel is
outside 1..4, or if a sibling `pwm`/`counter`/`qdec` node is enabled on the
same timer. Init returns an error on duplicate channels, a frequency the
prescaler cannot reach, or a dead-time `DTG` cannot encode.
