# `arw-archive`

The Rust implementation of the lossless Sony ARW2 archive codec. See the
repository [README](../../README.md) for CLI and project overview, and
[`docs/INTEGRATION.md`](../../docs/INTEGRATION.md) for Rust and
C/C++ integration examples. The codec implementation is Rust; the C header
lets C and C++ programs call it and does not contain a separate codec.

Build the CLI and library artifacts from the repository root:

```sh
cargo build --locked --release -p arw-archive
```
