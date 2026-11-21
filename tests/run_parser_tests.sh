#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
test_dir=$(mktemp -d)
trap 'rm -rf -- "$test_dir"' EXIT

component=esphome/components/foxess_solar
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -g \
  -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
  -I "$component" "$component/frame_parser.cpp" tests/frame_parser_test.cpp \
  -o "$test_dir/frame_parser_test"
"$test_dir/frame_parser_test"
