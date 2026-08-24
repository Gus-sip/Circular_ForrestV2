# circular_forest_v2

Module C (sensor node) and Module B (relay) flash code in one PlatformIO
folder, kept fully separate under `src/module_c/` and `src/module_b/` -
different `main.cpp` each, different `Config.h` each, no shared source
between them. This is a restructured copy of `../combined_modules/` (itself
merged from two sibling project folders, `../sensor_node/` and `../relay/`,
that still exist independently and are unaffected by anything here).

Fixes made in one place don't propagate to the others automatically - if you
patch a bug here, port it back (or vice versa) by hand.

## Layout

```
src/
  module_c/
    sensor/          - sensor drivers (BME690, SEN0466, BMV080, CM1106,
                        Calypso wind meter), their ISensor/Reading interface,
                        the standalone single-sensor bring-up sketches,
                        PowerManager/SleepManager/Sampler, Config.h, pins.h
    communications/   - the real LoRa-TX shipping firmware
                        (chip_forest_lora_tx.cpp) + RYLR998 bring-up/test
                        sketches (rylr998_bridge/demo/param_probe.cpp)
  module_b/
    communications/   - all of Module B: LoRa RX, BC660K-GL NB-IoT uplink,
                        AP-mode web dashboard. No sensor/ subfolder - Module B
                        has no sensors of its own, it only relays what Module C
                        sends over LoRa.
  module_a/            - no code here; Module A is the central ThingsBoard
                        server Module B uplinks to, not local firmware
  shared/
    radio/              - RYLR998 driver, used by both module_c/communications
                        and module_b/communications
bosch_bmv080_sdk/        Module C's vendored BMV080 SDK blobs (see below)
test/                    Module B's native unit tests (NbiotProtocol parsing)
notes/                   docs carried over from each source project (this
                        file, module_c_README.md, module_b_README.md,
                        module_c_DEVLOG.md, module_c_STATUS.md,
                        module_c_SENSOR_TIMELINE.md)
```

`chip_forest_lora_tx.cpp` is the one file that genuinely straddles both
concerns (reads all five sensors *and* transmits over LoRa) - it lives in
`module_c/communications/` since that's its role in the system, and its
`#include`s for `pins.h`/`Config.h`/`PowerManager.h`/`SleepManager.h`/the
sensor headers were updated to `../sensor/...` accordingly. No other file
needed its include paths touched - everything else's dependencies stayed
within whichever subfolder it moved into.

## Environments

Every env from `combined_modules` is here unchanged (same `module-c-`/
`module-b-` prefixed env ids), just repointed at the new `sensor/`/
`communications/` paths. The two that actually ship:

- **`module-c-lora-tx`** - Module C's real firmware: five sensors, LoRa TX to
  Module B, light-sleep between cycles, answers CFG downlinks.
- **`module-b-main`** - Module B's real firmware: LoRa RX, BC660K-GL NB-IoT
  uplink to Module A, AP-mode web dashboard.

Everything else is a bring-up/isolation-test rig carried over from each
project's history (per-sensor test envs, AT-liveness bridges, param probes,
etc.) - see the comment above each `[env:...]` block in `platformio.ini` for
what it's for.

`module-b-native` runs `src/module_b/communications/nbiot/NbiotProtocol.*`'s
unit tests on the host (`pio test -e module-b-native`) rather than the ESP32 -
this needs a system GCC/G++ toolchain, which this machine doesn't currently
have installed (pre-existing, unrelated to this restructure).

## BMV080 SDK restore

The SparkFun BMV080 Arduino Library's own repo is missing files this project
needs; `bosch_bmv080_sdk/` vendors them, and they have to be copied into
each env's own `.pio/libdeps/` after that env's first `pio run` (library
fetch happens then, not before). **Per-env**, not once - `.pio/libdeps` is
separate per environment:

```
cd bosch_bmv080_sdk
./restore.ps1 -EnvName module-c-lora-tx
```

Already done for `module-c-main`, `module-c-lora-tx`, `module-c-bringup`, and
`module-c-chip-forest-v1` (the four envs that pull in the BMV080 library) as
of this restructure. Needed again if `.pio` ever gets wiped. Note:
`restore.ps1` needs PowerShell script execution enabled
(`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` or similar) - if it's
blocked, copy the four files by hand per the commands inside the script.

## Verified

All 17 ESP32 environments (both `module-c-*` and `module-b-*`) build clean
as of this restructure, including the two production envs above. `module-b-native`
was not verified - it needs a system GCC/G++ toolchain, which this machine
doesn't currently have installed (pre-existing, unrelated to this restructure).

## Note on a pre-existing bug (carried over from `combined_modules`)

`module-c-cm1106-test` and `module-c-cm1106-read` originally (in
`../sensor_node/`) didn't exclude `ulp_pro_uart_test.cpp` from their source
filter, which also defines `setup()`/`loop()` - a link-time "multiple
definition" error the moment either env was actually built. Already fixed in
`combined_modules` and carried over fixed here.
