#!/usr/bin/env python3
""" Script to generate a JSON config with all build targets (for CI) """

import argparse
import os
import sys
import json
import re

source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

parser = argparse.ArgumentParser(description='Generate build targets')

parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')
parser.add_argument('-p', '--pretty', dest='pretty', action='store_true',
                    help='Pretty output instead of a single line')

args = parser.parse_args()
verbose = args.verbose

build_configs = []
excluded_manufacturers = ['atlflight']
excluded_platforms = ['qurt']
excluded_labels = [
    'stackcheck',
    'nolockstep', 'replay', 'test',
    'uavcanv1' # TODO: fix and enable
    ]

def process_target(cmake_file, target_name):
    ret = None
    is_board_def = False
    platform = None
    toolchain = None
    for line in open(cmake_file, 'r'):
        if 'px4_add_board' in line:
            is_board_def = True
        if not is_board_def:
            continue

        re_platform = re.search('PLATFORM\s+([^\s]+)', line)
        if re_platform: platform = re_platform.group(1)

        re_toolchain = re.search('TOOLCHAIN\s+([^\s]+)', line)
        if re_toolchain: toolchain = re_toolchain.group(1)

    if is_board_def:
        assert platform, f"PLATFORM not found in {cmake_file}"

        if platform not in excluded_platforms:
            # get the container based on the platform and toolchain
            container = platform
            if platform == 'posix':
                container = 'base-focal'
                if toolchain:
                    if toolchain.startswith('aarch64'):
                        container = 'aarch64'
                    elif toolchain == 'arm-linux-gnueabihf':
                        container = 'armhf'
                    else:
                        if verbose: print(f'possibly unmatched toolchain: {toolchain}')
            elif platform == 'nuttx':
                container = 'nuttx-focal'

            ret = {'target': target_name, 'container': container}
    return ret

for manufacturer in os.scandir(os.path.join(source_dir, 'boards')):
    if not manufacturer.is_dir():
        continue
    if manufacturer.name in excluded_manufacturers:
        if verbose: print(f'excluding manufacturer {manufacturer.name}')
        continue

    for board in os.scandir(manufacturer.path):
        if not board.is_dir():
            continue
        for files in os.scandir(board.path):
            if files.is_file() and files.name.endswith('.cmake'):
                label = files.name[:-6]
                target_name = manufacturer.name + '_' + board.name + '_' + label
                if label in excluded_labels:
                    if verbose: print(f'excluding label {label} ({target_name})')
                    continue
                target = process_target(files.path, target_name)
                if target is not None:
                    build_configs.append(target)


github_action_config = { 'include': build_configs }
extra_args = {}
if args.pretty:
    extra_args['indent'] = 2
print(json.dumps(github_action_config, **extra_args))

