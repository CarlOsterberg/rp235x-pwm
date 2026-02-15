# Pwm motor controller on rp pico 235x

## What do

This program communicates on uart tx on pin 1 and uart rx on pin 2.
Baudrate 115200, 8 bits, no parity and 1 stop bit.
Pin 6 is configured to transmit a PWM signal toa ESC,
*30A XT60 Electronic speed regulator* which is controlling a *A2212 / 13T 1000KV* brushless motor.

## Known bugs

Special chars eg, å, ä, ö crashes the program currently. 

### Installing dependancies

```sh
rustup target install thumbv8m.main-none-eabihf
cargo install flip-link
```

  
For a debug build
```sh
cargo run
```
For a release build
```sh
cargo run --release
```

**Loading with picotool**  
  As ELF files produced by compiling Rust code are completely compatible with ELF
  files produced by compiling C or C++ code, you can also use the Raspberry Pi
  tool [picotool](https://github.com/raspberrypi/picotool). The only thing to be
  aware of is that picotool expects your ELF files to have a `.elf` extension, and
  by default Rust does not give the ELF files any extension. You can fix this by
  simply renaming the file.

  This means you can't easily use it as a cargo runner - yet.

  Also of note is that the special
  [pico-sdk](https://github.com/raspberrypi/pico-sdk) macros which hide
  information in the ELF file in a way that `picotool info` can read it out, are
  not supported in Rust. An alternative is TBC.


## License

The contents of this repository are dual-licensed under the _[MIT](LICENSE-MIT) OR [Apache-2.0](LICENSE-APACHE-2.0)_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See [`LICENSE-MIT`](LICENSE-MIT) or [`LICENSE-APACHE-2.0`](LICENSE-APACHE-2.0) for more
information on each specific licence.
