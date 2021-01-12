#! /usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# update PATH to include kconfiglib scripts
export PATH=${DIR}:${PATH}

make --no-print-directory --silent CONFIG_ARCH_BOARD_CUSTOM=y CONFIG_APPS_DIR="../apps" olddefconfig
