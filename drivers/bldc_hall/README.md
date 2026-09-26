# STM32 Hall sensor acquisition driver

Zephyr driver for the three U/V/W Hall sensors of a BLDC motor, built on the
first three capture channels of an STM32 general-purpose timer, which it owns
exclusively. The timer XORs the three lines and times every edge in hardware.

It implements its own device class, **bldc_hall**
([bldc_hall.h](../../include/drivers/bldc_hall.h)), not plain GPIO
interrupts. A GPIO interrupt gives the state, but not a hardware-timed
interval between edges.

Direction, sensor faults, position and speed stay in the consumer, which gets
raw measurements only: the 3-bit state, and on every edge the ticks since the
previous one. The counter frequency and the edges per electrical revolution
are read back from the device to turn ticks into speed.

## Contract

- every edge reports the state and the ticks since the previous edge, from
  an interrupt;
- the first edge after init or after the counter wraps reports 0 ticks, as it
  closes no whole interval;
- the time since the last edge can be read at any moment, and reads as the
  maximum value before the first edge or once the counter wraps, so a stall
  shows up without any edge;
- no edge is reported until the consumer registers its handler, so it can
  read the initial state first.

## Timer XOR

Setting `TI1S` feeds the XOR of CH1/CH2/CH3 to TI1. The timer runs in
slave-mode reset on `TI1F_ED`, so every edge clears the counter, and an input
capture on `TRC` latches the count that just elapsed: one interrupt per edge,
six per electrical revolution, and `CNT` reads as the time since the last
edge.

`URS = 1` limits the update event to a real overflow. Without it, the
slave-mode reset would raise one on every edge, and an overflow would stop
meaning "no edge for a whole counter span", which is what invalidates `CNT`
and the next capture.

## Devicetree

- [`bldc-hall-device.yaml`](../../dts/bindings/bldc_hall/bldc-hall-device.yaml):
  common property `hall-gpios`;
- [`st,stm32-bldc-hall-xor.yaml`](../../dts/bindings/bldc_hall/st,stm32-bldc-hall-xor.yaml):
  `pinctrl-0`, the pins muxed to CH1/CH2/CH3.

The node is a child of `st,stm32-timers`, which provides `reg`, `clocks`,
`interrupts` and `st,prescaler`. It has no cells: consumers reference it by
phandle.

```dts
&timers3 {
        status = "okay";
        st,prescaler = <63>;

        hall_motor0: hall-xor {
                compatible = "st,stm32-bldc-hall-xor";
                status = "okay";

                pinctrl-0 = <&tim3_ch1_pa6 &tim3_ch2_pa4 &tim3_ch3_pb0>;
                pinctrl-names = "default";

                hall-gpios = <&gpioa 6 0>, <&gpioa 4 0>, <&gpiob 0 0>;
        };
};
```

`hall-gpios` repeats the pins of `pinctrl-0` in U/V/W order: that order alone
defines the phases, U being the most significant bit of the state. Pinctrl
sets the pins up and the driver reads their levels raw, which works while
they are muxed to the timer, so every flag cell must be 0 and any pull goes
in `pinctrl-0`.

Size `st,prescaler` so the counter (16 or 32 bits, depending on the timer)
does not wrap between two edges at the slowest expected speed, or the
measured speed is wrong
([AN4013](https://www.st.com/resource/en/application_note/an4013-stm32-crossseries-timer-overview-stmicroelectronics.pdf)
§4.3.4).

The build fails if the parent timer has no single global interrupt (TIM1/TIM8),
or if `hall-gpios` does not list exactly three lines or sets a flag. Init
returns an error on a GPIO controller not ready, or a timer clock
or pinctrl setup failure.
