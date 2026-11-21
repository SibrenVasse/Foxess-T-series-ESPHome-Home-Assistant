#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
test_dir=$(mktemp -d)
trap 'rm -rf -- "$test_dir"' EXIT

cp foxess-inverter.yaml.example "$test_dir/foxess-inverter.yaml"
cp -R esphome "$test_dir/esphome"
api_key=$(openssl rand -base64 32)
sed "s|^api_key:.*|api_key: '$api_key'|" secrets.yaml.example > "$test_dir/secrets.yaml"

esphome config "$test_dir/foxess-inverter.yaml" > /dev/null
esphome compile "$test_dir/foxess-inverter.yaml"
