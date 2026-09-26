# STM32 synchronized ADC acquisition driver

Zephyr driver for three analog channels sampled at the same point of every PWM
period, e.g. the currents of a three-shunt motor bridge. It is built on the
injected group of an STM32 ADC, which it owns exclusively, and triggered in
hardware by a [pwm_bridge](../pwm_bridge/README.md).

It implements its own device class, **bldc_adc**
([bldc_adc.h](../../include/drivers/bldc_adc.h)), not the Zephyr ADC API.
The ADC API lacks what current sensing on a bridge needs: a hardware trigger
tied to the carrier, and several channels converted in one sequence at the
same instant of every period.

Scaling stays in the consumer, which gets raw counts only, plus the reference
voltage measured at init to turn them into volts. The shunt value, amplifier
gain and zero-input bias are the consumer's to know.

## Contract

- the three channels are converted once per PWM period, always in the same
  order;
- until the consumer registers its handler, a read waits for the next period
  and returns fresh counts, e.g. to calibrate offsets at init;
- once a handler is registered, it gets the counts every period from an
  interrupt, and reads are refused;
- the reference voltage is measured once at init and does not change
  afterwards.

## Injected group

The bridge's sync trigger
([`TRGO2`](../pwm_bridge/README.md#carrier-and-sync-trigger)) starts the
three-rank injected sequence on its rising edge. The bridge counter runs from
init, so the trigger fires even while the outputs are off, and a consumer can
calibrate at its own init.

The end-of-sequence flag `JEOS` has one owner at a time. Until a handler is
registered, its interrupt stays masked and a read polls the flag. Registering
a handler unmasks it and hands the flag to the ISR, unregistering hands it
back.

## Shunt topologies

The driver serves three shunts only. The sampling instant is set by the
bridge, not by this driver: the center of the high-side pulse, fixed relative
to the carrier. In 6-step commutation, with one leg chopped and another held
low:

- three low-side or inline shunts work: the leg held low conducts the whole
  period, so its shunt always carries the phase current;
- a single DC-link shunt carries the phase current during the high-side pulse,
  so the instant would suit it, but the driver needs exactly three channels.
  The fixed sampling time, 247.5 ADC cycles or about 5.8 µs at 42.5 MHz, would
  also have to end within the second half of the pulse, i.e. above roughly
  12 µs of on-time;
- two low-side shunts miss the current in two sectors out of six, when the leg
  held low is the one without a shunt. They would need a sample during the
  off-time, which the bridge does not provide.

SVPWM would need the sequencer reprogrammed every sector, as the instant moves
with the duty cycles, and single-shunt FOC two samples per period. This driver
does neither.

## Reference voltage

At init, once the ADC is enabled and before the injected group starts, one
regular conversion samples the internal `VREFINT` band-gap. Its factory
calibration gives the board's real `VREF+` rather than a nominal value.

That conversion waits until uptime reaches `frontend-settle-ms`, so the
reference and the analog front-end settle first. The deadline counts from
boot, so several instances pay it once.

## Devicetree

- [`bldc-adc-device.yaml`](../../dts/bindings/bldc_adc/bldc-adc-device.yaml):
  common properties `pwm`, `channels`, `frontend-settle-ms`;
- [`st,stm32-bldc-adc.yaml`](../../dts/bindings/bldc_adc/st,stm32-bldc-adc.yaml):
  no STM32-specific property.

The node is a child of `st,stm32-adc`, which provides `reg`, `clocks`,
`interrupts` and `st,adc-prescaler`. That parent must be enabled, but
`adc_stm32` must not bind it, so `CONFIG_ADC_STM32` stays off. The node has
no cells: consumers reference it by phandle.

```dts
&adc1 {
        status = "okay";
        st,adc-clock-source = "SYNC";
        st,adc-prescaler = <4>;

        adc_motor0: bldc-adc {
                compatible = "st,stm32-bldc-adc";
                status = "okay";

                pwm = <&bridge0>;
                channels = <6 7 8>;
        };
};
```

`pwm` points to the bridge, which must set `sync-trigger-enable`. `channels`
lists the ADC input numbers in the order the counts are reported.

The build fails if `CONFIG_ADC_STM32` is enabled, if the ADC clock source is
not `SYNC` or its prescaler not 1, 2 or 4, if the bridge timer is not
TIM1/TIM8, if its sync trigger is disabled, or if `channels` does not list
exactly three inputs. Init returns an error on a clock controller not ready,
or a calibration, enable or reference conversion that times out.
