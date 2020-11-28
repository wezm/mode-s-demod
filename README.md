# mode-s-demod

This crate implements Mode-S demodulation as used by ADS-B. It can be coupled
with a software defined radio to demodulate the raw I/Q samples.

The code in this crate was ported from C to Rust from [Malcolm Robb's
fork][MalcolmRobb] of [antirez's dump1090 tool][dump1090] using [C2Rust].
The machine conversion was tidied up and massaged into what you see now. There is
still plenty of room for making the code safer, and more idiomatic.

[MalcolmRobb]: https://github.com/MalcolmRobb/dump1090
[dump1090]: https://github.com/antirez/dump1090
[C2Rust]: https://c2rust.com/manual/
