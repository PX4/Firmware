#! /usr/bin/env python3
""" Script to validate JSON file(s) against a schema file according to
https://json-schema.org/"""

import argparse
import sys
import json

try:
    from jsonschema import validate
except ImportError as e:
    print("Failed to import jsonschema: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user jsonschema")
    print("")
    sys.exit(1)


parser = argparse.ArgumentParser(description='Validate JSON file(s) against a schema')
parser.add_argument('json_file', nargs='+', help='JSON config file(s)')
parser.add_argument('--schema-file', type=str, action='store',
                    help='JSON schema file', required=True)
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()
schema_file = args.schema_file
json_files = args.json_file
verbose = args.verbose

# load the schema
with open(schema_file, 'r') as stream:
    schema = json.load(stream)

# validate files
for json_file in json_files:
    if verbose: print("Validating {:}".format(json_file))
    with open(json_file, 'r') as stream:
        json_data = json.load(stream)

    try:
        validate(instance=json_data, schema=schema)
    except:
        print("JSON validation for {:} failed (schema: {:})".format(json_file, schema_file))
        raise

