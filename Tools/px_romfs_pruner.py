#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2014-2018 PX4 Development Team. All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


"""
px_romfs_pruner.py:
Try to keep size of the ROMFS minimal.

This script goes through the temporarily copied ROMFS data and deletes all
comments, empty lines and unnecessary whitespace.
It also deletes hidden files such as auto-saved backups that a text editor
might have left in the tree.

@author: Julian Oes <julian@oes.ch>
"""

from __future__ import print_function
import argparse
import re
import os
import io


def main():
    # Parse commandline arguments
    parser = argparse.ArgumentParser(description="ROMFS pruner.")
    parser.add_argument('--folder', action="store",
                        help="ROMFS scratch folder.")
    parser.add_argument('--board', action="store",
                        help="Board architecture for this run")
    args = parser.parse_args()

    err_count = 0

    # go through temp folder
    for (root, dirs, files) in os.walk(args.folder):
        for file in files:
            file_path = os.path.join(root, file)

            # delete hidden files
            if file.startswith("."):
                os.remove(file_path)
                continue

            # delete documentation
            if file.startswith("README"):
                os.remove(file_path)
                continue

            # delete CMakeLists
            if file.startswith("CMakeLists"):
                os.remove(file_path)
                continue

            # only prune text files
            if ".zip" in file or ".bin" in file or ".swp" in file \
                    or ".gz" in file or ".xz" in file or ".bz2" in file \
                    or ".data" in file or ".DS_Store" in file:
                continue

            # read file line by line
            pruned_content = ""
            board_excluded = False

            with io.open(file_path, "r", newline=None) as f:
                for line in f:
                    # abort if spurious tabs are found
                    if re.search(r"[a-zA-Z0-9]+\t.+", line):
                        file_local = re.sub(args.folder, '', file_path)
                        print("ERROR: Spurious TAB character in file " + file_local)
                        print("Line: " + line)
                        err_count += 1

                    # find excluded boards
                    if re.search(r'\b{0} exclude\b'.format(args.board), line):
                        board_excluded = True
                    # handle mixer files differently than startup files
                    if file_path.endswith(".mix"):
                        if line.startswith(("Z:", "M:", "R: ", "O:", "S:",
                                            "H:", "T:", "P:", "A:")):
                            # reduce multiple consecutive spaces into a
                            # single space
                            line_reduced = re.sub(' +', ' ', line)
                            pruned_content += line_reduced
                    else:
                        if not line.isspace() \
                                and not line.strip().startswith("#"):
                            pruned_content += line.strip() + "\n"
            # delete the file if it doesn't contain the architecture
            # write out the pruned content else
            if not board_excluded:
                # overwrite old scratch file
                with open(file_path, "wb") as f:
                    pruned_content = re.sub("\r\n", "\n", pruned_content)
                    f.write(pruned_content.encode("ascii", errors='strict'))
            else:
                os.remove(file_path)

    if (err_count > 0):
        exit(1)


if __name__ == '__main__':
    main()
