#!/usr/bin/env python

import argparse
import zlib
import os

parser = argparse.ArgumentParser(description="""Generate the COMPONENT_INFORMATION hashes header file""")
parser.add_argument('--output', metavar='hashes.h', help='output file')
parser.add_argument('files', nargs='+', metavar='hashes.json', help='files to generate hashes from')

args = parser.parse_args()
filename = args.output
files = args.files

with open(filename, 'w') as outfile:
    outfile.write("#include <stdint.h>\n")
    outfile.write("namespace component_information {\n")
    for filename in files:
        # get CRC
        file_hash = 0
        for line in open(filename, "rb"):
            file_hash = zlib.crc32(line, file_hash)

        basename = os.path.basename(filename)
        identifier = basename.split('.')[0]
        outfile.write("static constexpr uint32_t {:}_hash = {:};\n".format(identifier, file_hash))


    outfile.write("}\n")


