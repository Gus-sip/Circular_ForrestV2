param(
    [Parameter(Mandatory = $true)]
    [string]$EnvName
)

# Copies the manually-vendored Bosch BMV080 SDK blobs into a specific env's
# .pio/libdeps checkout of the SparkFun BMV080 Arduino Library. Needed
# because that library's own repo doesn't ship these files. .pio/libdeps is
# per-env, so this must be re-run once per Module C env that uses the BMV080
# (module-c-main, module-c-lora-tx, module-c-bringup, module-c-chip-forest-v1)
# after that env's first `pio run` has fetched the library.
#
# Usage: ./restore.ps1 -EnvName module-c-lora-tx

$libDir = "..\.pio\libdeps\$EnvName\SparkFun BMV080 Arduino Library\src"
Copy-Item "bmv080.h" "$libDir\sfTk\bmv080.h" -Force
Copy-Item "bmv080_defs.h" "$libDir\sfTk\bmv080_defs.h" -Force
Copy-Item "lib_bmv080.a" "$libDir\esp32s3\lib_bmv080.a" -Force
Copy-Item "lib_postProcessor.a" "$libDir\esp32s3\lib_postProcessor.a" -Force
Write-Host "Bosch BMV080 SDK files restored for env '$EnvName'."
