# pico-keys-sdk — Yusoofs Pipico fork

Fork of the [pico-keys SDK](https://github.com/polhenarejos/pico-keys-sdk)
carrying the **Yusoofs Pipico V1** storage and companion work. The mission
branches are `pipico/storage-baseline` (flash-storage baseline) and
`pipico/companion-hooks` (stacked on top; the firmware pin points there).
The consuming firmware lives in the
[pico-fido fork](https://github.com/yusoofsh/pico-fido), branch
`pipico/integration-v1`, which builds this SDK as a pinned submodule; its
[`docs/pipico/`](https://github.com/yusoofsh/pico-fido/tree/pipico/integration-v1/docs/pipico)
directory is the documentation of record (baseline, layout, threat model,
hardware checklist, handoff, manifest).

**Status: SOURCE REVIEWED · BUILT · AUTOMATED TESTS PASSED (software
level). FLASHED = NOT_RUN — no board was ever flashed with firmware built
from this fork in this mission (no board was attached).**

## What the mission branches change (summary)

- `src/fs/flash_layout.{c,h}`: pure, host-testable layout module — JEDEC
  capacity validation before any shift, build-time clamp + optional cap,
  marker classification (BLANK / VALID / LEGACY_TRUNCATED / FOREIGN), and
  the unchanged v1 marker page format.
- `src/fs/low_flash.c` (RP2040): reordered `low_flash_init` — detect and
  validate geometry before any mutation; a valid marker writes nothing, a
  blank one gets one full-page program plus readback, the legacy truncated
  marker is repaired, foreign content fails closed (storage locked, device
  still boots); the marker-gap guard applies only to capped RP2040 builds.
  Uncapped and non-RP2040 layouts are unchanged except the build-time clamp.
- Storage-locked builds skip storage-dependent startup and refuse writes
  without wiping data.
- `src/usb/emulation/`: an emulation-only BOOT button control (test seam;
  absent from firmware builds), plus the CTAPHID-cancel transaction fixes
  in `src/usb/hid/`.
- `src/usb/hid/kb_tx.{c,h}`: single-owner keyboard transmitter (readiness
  on `tud_hid_n_ready`, no overwrite of in-flight text, guaranteed
  key-up), and weak `picokey_early_init` / `picokey_task` hooks in
  `main.c` for the companion.
- `USB_PRODUCT_STRING` compile definition: the product string is set by the
  consumer ("Yusoofs Pipico" in the Pipico build); the upstream default
  "Pico Key" is kept when the define is absent.
- CI: `.github/workflows/pipico-sdk-tests.yml` (host tests, permissions
  `contents: read`, no secrets).

## Run the host tests

The test configure does **not** clone dependencies: `tests/CMakeLists.txt`
compiles mbedtls sources from `third-party/mbedtls` and fails to configure
without them. Clone the pinned mbedtls v3.6.7 there first (never committed):

```sh
git clone -q --depth 1 -b v3.6.7 https://github.com/Mbed-TLS/mbedtls.git third-party/mbedtls
git -C third-party/mbedtls rev-parse HEAD   # 068ff080b369adfac81509f9b57b2afabaf82dc5
```

Then, with cmake, ninja and a host C compiler on `PATH`:

```sh
cmake -S tests -B build-tests -G Ninja
ninja -C build-tests
ctest --test-dir build-tests --output-on-failure
```

The suite covers the pure `flash_layout` unit tests and the integration
harness that compiles the unmodified production `low_flash.c` against a mock
NOR flash, plus the button, kb_tx and cancel-path harnesses. (The firmware
build is different: its configure fetches `third-party/` dependencies on
demand through `cmake/deps.cmake` — it is the host-test configure above that
needs the explicit preclone.)

The flash-size-limit python regression runs standalone (needs `python3` and
a C compiler with UBSan; gcc 13 works):

```sh
python3 tests/test_flash_size_limit.py
```

See `tests/README.flash-size-limit.md` for its scope and the documented M1
adaptations.

## Building the firmware

The firmware is built from the pico-fido fork with its pinned Pipico preset:

```sh
scripts/pipico/build.sh   # in the pico-fido checkout
```

That script pins the whole toolchain tuple, runs the image-bounds, clock and
budget gates and fails on any warning. Build and test prerequisites are
documented in the pico-fido fork's README and `docs/pipico/BASELINE.md`.

## Publication order and the two checkouts

- **The SDK branch is published first.** Push `pipico/companion-hooks`, let
  its `pipico-sdk-tests` CI run conclude success on the new SHA, and only
  then bump the `pico-keys-sdk` gitlink in the pico-fido fork
  (`pipico/integration-v1`) to that SHA. Never push a root commit whose
  gitlink names an SDK commit that is not yet on this fork.
- **This repository exists as two checkouts**: this standalone clone and the
  `pico-fido/pico-keys-sdk` submodule of the firmware fork. They diverge
  silently unless synchronized: a commit made in one must be fetched and
  checked out in the other (`git fetch <other-checkout> && git checkout
  <sha>`) before any build, gate run or push, and the root then stages the
  new gitlink (`git add pico-keys-sdk`).

## Test status labels

- SOURCE REVIEWED / BUILT / AUTOMATED TESTS PASSED: achieved and evidenced
  (SDK ctest receipts; CI run on the pushed branch head — see
  `docs/pipico/MANIFEST.md` in the pico-fido fork).
- FLASHED, HARDWARE TESTED: **NOT_RUN** for this mission. The board-side
  checklist is `docs/pipico/HARDWARE-TESTS.md` in the pico-fido fork.
