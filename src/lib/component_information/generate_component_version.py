#!/usr/bin/env python

import argparse
import json
import lzma #to create .xz file

parser = argparse.ArgumentParser(description="""Generate the COMPONENT_VERSION json file""")
parser.add_argument('filename', metavar='component_version.json', help='JSON output file')
parser.add_argument('--compress', action='store_true', help='Add a compressed output file')
parser.add_argument('--no-parameters', action='store_true', help='disable parameter support')

args = parser.parse_args()
filename = args.filename
compress = args.compress

def save_compressed(filename):
    #create lzma compressed version
    xz_filename=filename+'.xz'
    with lzma.open(xz_filename, 'wt', preset=9) as f:
        with open(filename, 'r') as content_file:
            f.write(content_file.read())

component_version = {}
component_version['version'] = 1
# types:
# 0: COMP_METADATA_TYPE_VERSION
# 1: COMP_METADATA_TYPE_PARAMETER
# 2: COMP_METADATA_TYPE_COMMANDS
supported_types = [0]
if not args.no_parameters: supported_types.append(1)
component_version['supportedCompMetadataTypes'] = supported_types

with open(filename, 'w') as outfile:
    json.dump(component_version, outfile)

if compress:
    save_compressed(filename)
