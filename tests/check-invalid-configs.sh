#!/usr/bin/env bash
set -euo pipefail

check_invalid() {
  local config=$1
  local expected=$2
  local output

  if output=$(esphome config "$config" 2>&1); then
    echo "Expected $config to be rejected, but it was accepted" >&2
    return 1
  fi

  if ! grep -Fq "$expected" <<<"$output"; then
    echo "Expected $config to fail with: $expected" >&2
    echo "$output" >&2
    return 1
  fi

  echo "Rejected $config as expected"
}

check_invalid tests/invalid/arduino.yaml \
  "This feature is only available with framework(s) esp-idf."
check_invalid tests/invalid/unsupported-variant.yaml \
  "SMA Bluetooth Classic is only available on ESP32."
check_invalid tests/invalid/mac-address.yaml \
  "MAC Address must consist of 6 : (colon) separated parts."
check_invalid tests/invalid/password.yaml \
  "length of value must be at most 12."
check_invalid tests/invalid/latitude.yaml \
  "value must be at most 90."
check_invalid tests/invalid/longitude.yaml \
  "value must be at most 180."
