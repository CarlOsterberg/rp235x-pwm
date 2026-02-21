# Pwm motor controller on rp pico 235x

## What do

This program communicates on UART tx on Gpio0/pin 1 and UART rx on Gpio1/pin 2.
The UART settings are: baud rate 115200, 8 bits, no parity and 1 stop bit.
Gpio4/pin6, gpio8/pin11, gpio12/pin16 and Gpio16/pin21 are configured to transmit a PWM signal to a ESC,
*30A XT60 Electronic speed regulator* which is controlling a *A2212 / 13T 1000KV* brushless motor.

### Installing dependancies

```sh
rustup target install thumbv8m.main-none-eabihf
cargo install flip-link
cargo install probe-rs-tools # If debugging with SWD
```

## Flashing and running the code

There are two ways, either you can use the picotool or probe-rs. The pciotool
requires less tools but you need to put the device into mass storage mode somehow.
probe-rs is easier to use but you need a debugger supporting swd and to solder the debug pins.
For details see the [cargo config](.cargo/config.toml).

## License

The contents of this repository are dual-licensed under the _[MIT](LICENSE-MIT) OR [Apache-2.0](LICENSE-APACHE-2.0)_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See [`LICENSE-MIT`](LICENSE-MIT) or [`LICENSE-APACHE-2.0`](LICENSE-APACHE-2.0) for more
information on each specific licence.
