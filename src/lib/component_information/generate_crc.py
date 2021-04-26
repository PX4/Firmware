#!/usr/bin/env python

import argparse
import os

parser = argparse.ArgumentParser(description="""Generate the COMPONENT_INFORMATION checksums (CRC32) header file""")
parser.add_argument('--output', metavar='checksums.h', help='output file')
parser.add_argument('files', nargs='+', metavar='input_file.json', help='files to generate CRC from')

args = parser.parse_args()
filename = args.output
files = args.files

def create_table():
    a = []
    for i in range(256):
        k = i
        for j in range(8):
            if k & 1:
                k ^= 0x1db710640
            k >>= 1
        a.append(k)
    return a

# MAVLink's CRC32
def crc_update(buf, crc_table, crc):
    for k in buf:
        crc = (crc >> 8) ^ crc_table[(crc & 0xff) ^ k]
    return crc

crc_table = create_table()

with open(filename, 'w') as outfile:
    outfile.write("#include <stdint.h>\n")
    outfile.write("namespace component_information {\n")
    for filename in files:
        file_crc = 0
        for line in open(filename, "rb"):
            file_crc = crc_update(line, crc_table, file_crc)

        basename = os.path.basename(filename)
        identifier = basename.split('.')[0]
        outfile.write("static constexpr uint32_t {:}_crc = {:};\n".format(identifier, file_crc))


    outfile.write("}\n")


