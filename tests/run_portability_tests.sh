#!/usr/bin/env sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_dir=$(CDPATH= cd -- "$test_dir/.." && pwd)
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++20 -I"$project_dir/include" "$test_dir/vector3_header_isolation_test.cpp" -o "$build_dir/vector3_header_isolation_test"
"$build_dir/vector3_header_isolation_test"

c++ -std=c++20 -U_DEFAULT_SOURCE -U_GNU_SOURCE -D_POSIX_C_SOURCE=200809L -I"$project_dir/include" "$test_dir/quaternion_portability_test.cpp" -o "$build_dir/quaternion_portability_test"
"$build_dir/quaternion_portability_test"
