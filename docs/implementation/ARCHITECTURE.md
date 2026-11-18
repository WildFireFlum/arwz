# Implementation architecture

This diagram follows one file through the Rust crate. It focuses on module
boundaries and the data passed between them; the archive byte layout is in
[FORMAT.md](../FORMAT.md).

```mermaid
flowchart TB
    subgraph EntryPoints["Entry points"]
        CLI["CLI<br/>src/main.rs"]
        RustAPI["Rust API<br/>archive::pack_archive / unpack_archive"]
        CAPI["C API<br/>ffi.rs"]
    end

    CLI --> Archive
    RustAPI --> Archive
    CAPI --> Boundary["Pointer checks, panic containment,<br/>thread-local errors and buffer ownership"]
    Boundary --> Archive

    subgraph ArchiveLayer["Archive layer · archive.rs"]
        Archive{"Pack or unpack"}
        TIFF["TIFF scanner<br/>IFD and SubIFD links"]
        Geometry["Supported strip geometry<br/>compression 32767, one strip,<br/>rows even, width divisible by 32"]
        Frame["Archive framing<br/>header, section lengths,<br/>SHA-256 digests"]
        Side["Bytes outside sensor strip"]
        Helpers["Endpoint maps + RG2<br/>predictor table"]
        Sensor["Packed sensor strip"]
    end

    Archive -->|pack| TIFF
    TIFF --> Geometry
    Geometry -->|strip offset and dimensions| Sensor
    Geometry -->|prefix and suffix| Side
    Sensor --> Maps["Observed-value maps<br/>sensor_maps"]
    Sensor --> Fit["Optional predictor fit<br/>fit_cross_profile"]
    Maps --> Helpers
    Fit --> Helpers
    Sensor --> Encoder["Sensor encoder<br/>encode_sensor"]
    Helpers --> Encoder
    Encoder -->|range-coded strip| Frame
    Helpers -->|XZ| Frame
    Side -->|XZ| Frame

    Frame -->|unpack| Validate["Check marker, revision,<br/>codec ID, lengths and helper digest"]
    Validate -->|helper data| DecodeHelpers["XZ decode maps and RG2"]
    Validate -->|outside-strip bytes| DecodeSide["XZ decode prefix and suffix"]
    Validate -->|sensor stream| Decoder["Sensor decoder<br/>decode_sensor"]
    DecodeHelpers --> Decoder
    Decoder -->|original packed strip bytes| Restore["Reassemble prefix + strip + suffix"]
    DecodeSide --> Restore
    Restore --> Digest["Check SHA-256 of restored ARW"]
    Digest --> Result["Return original bytes"]

    subgraph SensorCodec["Sensor codec · lib.rs"]
        Block["16-byte Sony blocks<br/>two blocks per 32-pixel row group"]
        HeaderModels["Endpoint range/minimum<br/>residual models"]
        PositionModel["Adaptive extremum-position<br/>model"]
        Predictor["Causal pixel predictor<br/>five-row sample ring"]
        DeltaModel["Observed delta alphabet<br/>+ escape path"]
        CDF["Context-adaptive cumulative<br/>frequency model"]
        RangeCoder["32-bit range coder"]

        Block --> HeaderModels
        Block --> PositionModel
        PositionModel --> Predictor
        Predictor --> DeltaModel
        DeltaModel --> CDF
        HeaderModels --> RangeCoder
        PositionModel --> RangeCoder
        CDF --> RangeCoder
    end

    Encoder -. uses .-> SensorCodec
    Decoder -. uses .-> SensorCodec
```

## Why the pieces are separate

- The archive layer owns TIFF discovery, section framing and file integrity.
  The sensor codec only transforms the supported strip.
- Endpoint maps and the optional `RG2` table are derived from the strip and
  stored as helper data. The decoder reads them first so it can reproduce the
  encoder's ranks and predictor choices.
- The sensor pass immediately reconstructs samples and updates its predictor
  and adaptive models. Encoder and decoder share this pass so their state
  remains synchronized without storing a full sample image in the archive.
- The final digest verifies exact restoration of the input ARW. It does not
  reverse any lossy compression already applied by the camera.
