# Flash-size cap regression tests

Run `python3 tests/test_flash_size_limit.py` from the SDK checkout. Requires
Python 3 and a GCC/Clang-compatible `CC` (default `cc`) with
UndefinedBehaviorSanitizer. The test compiles the actual RP2040 boot stage of
`src/fs/low_flash.c` (the phymarker stage, `low_flash_init_rp2040()` and
`low_flash_init()`) together with the real pure `flash_layout` module; JEDEC,
ROM partition lookup and the NOR flash backing the marker sector are stubs.
This is a host regression harness, **not** a Pico/ESP32 SDK build or a
hardware test.

`PICO_FLASH_SIZE_LIMIT_BYTES` is opt-in and RP2040-only. It is rejected at
compile time on RP2350, ESP32 and emulation, and `picokeys_sdk_import.cmake`
rejects it at configure time on ESP32 and emulation as well. Cap VALUES are
validated at boot in `compute_layout()`: a value that is zero, not
sector-aligned or larger than the clamped capacity locks the storage instead
of failing the build, so there is no compile-time power-of-two range check.
That 28cd6a4 restriction was superseded in the merge with the M1 storage
design (architecture.md section 3): the runtime fail-closed rule is what lets
`tests/flash_harness` build and test misaligned (`0x200800`) and too-small
(`0x100000`) cap variants, and any positive sector-aligned cap that leaves a
valid data region is supported, powers of two are not required.

RP2040 accepts a JEDEC capacity exponent between 16 (64 KiB) and 24 (16 MiB)
and rejects any other byte before any shift, so an abnormal ID is never read
as a large capacity. A capacity that passes the window can still fail closed
when the data region cannot hold the minimum pool+journal headroom. The
half-flash data pool starts above the physical-marker sector in capped builds
only: a 2 MiB effective size yields offsets `[0x101000, 0x200000)`; uncapped
boards keep the upstream layout (2 MiB yields `[0x100000, 0x200000)`, 4 MiB
`[0x200000, 0x400000)`). The cap never increases detected capacity, and a cap
larger than the detected capacity is an explicit error, not a silent "no cap".

Boot-stage failure is the storage-locked state (the device still boots and
enumerates, but every flash writer refuses and nothing is wiped), not a
`panic()`: every locked case in the suite asserts zero marker writes and zero
bounds publications.

The marker scenarios drive the real `phymarker_stage()` over a NOR-accurate
mapping at the XIP marker address: a valid marker means zero writes, a blank
sector gets one full 256-byte 0xFF-padded page program plus readback, the
legacy truncated artifact (`53 59 45 4B`, "SYEK", the little-endian low half
of the "PICOKEYS" magic) gets an erase plus program, foreign content locks
the storage, and a marker sector beyond the chip is skipped entirely. Page
content (magic, version, flags, UID, CRC and 0xFF padding) is verified after
every program. CRC computation is stubbed to a constant; no hardware
persistence or readback claim is made. RP2350 partition selection, the
RP2350 fallback and ESP32 sizing are checked uncapped and unchanged; a cap on
those platforms fails compilation.

Existing reported hardware evidence is limited to YD-RP2040 4MB with a 2 MiB cap
and the original marker-offset fix. The follow-up guards and marker-call ordering
have not been retested on hardware; neither RP2350 nor ESP32 hardware is claimed.

Changing the cap changes persistent-storage locations. Use a blank key or make a
recoverable flash backup first; do not alternate capped/uncapped firmware on a
provisioned key. Moving the lower bound past the marker can exclude existing
records on a 2 MiB key whose pool reached that first sector. No storage migration
or automatic erase is added by this change.
