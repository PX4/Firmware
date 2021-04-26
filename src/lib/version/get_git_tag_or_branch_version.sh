#!/usr/bin/env bash

set -e

# Script to extract PX4_GIT_TAG_OR_BRANCH_NAME to be used by CI

build_dir="$1"
if [ ! -d "$build_dir" ]; then
	echo "usage: $0 <build directory>"
	exit -1
fi

version_file="$build_dir/src/lib/version/build_git_version.h"

if [ ! -f "$version_file" ]; then
	echo "Version file not found. Did you run the target 'ver_gen'?"
	exit -1
fi

sed -n 's/.*PX4_GIT_TAG_OR_BRANCH_NAME\s*"\(.*\)".*/version=\1/p' "$version_file"

