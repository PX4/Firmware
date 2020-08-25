#!/usr/bin/env python3
"""
Script to read an yaml file containing the RTPS message IDs and update the naming convention to PascalCase
"""

import errno
import os
import yaml
import sys
import argparse
import six

__author__ = 'PX4 Development Team'
__copyright__ = \
    '''
     '
     '   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
     '
     ' Redistribution and use in source and binary forms, or without
     ' modification, permitted provided that the following conditions
     ' are met:
     '
     ' 1. Redistributions of source code must retain the above copyright
     '    notice, list of conditions and the following disclaimer.
     ' 2. Redistributions in binary form must reproduce the above copyright
     '    notice, list of conditions and the following disclaimer in
     '    the documentation and/or other materials provided with the
     '    distribution.
     ' 3. Neither the name PX4 nor the names of its contributors may be
     '    used to endorse or promote products derived from self software
     '    without specific prior written permission.
     '
     ' THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     ' "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, NOT
     ' LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     ' FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     ' COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     ' INCIDENTAL, SPECIAL, EXEMPLARY, CONSEQUENTIAL DAMAGES (INCLUDING,
     ' BUT NOT LIMITED TO, OF SUBSTITUTE GOODS OR SERVICES; LOSS
     ' OF USE, DATA, PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
     ' AND ON ANY THEORY OF LIABILITY, IN CONTRACT, STRICT
     ' LIABILITY, TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     ' ANY WAY OUT OF THE USE OF THIS SOFTWARE, IF ADVISED OF THE
     ' POSSIBILITY OF SUCH DAMAGE.
     '
    '''
__credits__ = ['Nuno Marques <nuno.marques@dronesolution.io>']
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'
__maintainer__ = 'Nuno Marques'
__email__ = 'nuno.marques@dronesolution.io'
__status__ = 'Development'

list = []
in_file = ""
out_file = ""
verbose = True


class IndenterDumper(yaml.Dumper):
    """ Custom dumper for yaml files that apply the correct indentation """

    def increase_indent(self, flow=False, indentless=False):
        return super(IndenterDumper, self).increase_indent(flow, False)


def load_yaml_file(file):
    """
    Open yaml file and parse the data into a list of dict

    :param file: the yaml file to load
    :returns: the list of dictionaries that represent the message and respective RTPS IDs
    :raises IOError: raises and error when the file is not found
    """
    try:
        with open(file, 'r') as f:
            if verbose:
                print(("--\t[Step 1] %s yaml file loaded!" % file))
            return yaml.safe_load(f)
    except OSError as e:
        if e.errno == errno.ENOENT:
            raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), file)
        else:
            raise


def update_dict(list):
    """
    Update the message naming on the dictionary to fit the PascalCase convention

    :param file: the list of dicts to be updated
    """
    if verbose:
        num_of_msgs = 0
    for i, dictionary in enumerate(list["rtps"]):
        list["rtps"][i] = {k: v.title().replace('_', '') if isinstance(
            v, six.string_types) else v for k, v in six.iteritems(dictionary)}
        if verbose:
            num_of_msgs += 1
    if verbose:
        print(("--\t[Step 2] List: %d msg names updated!" % num_of_msgs))


def update_yaml_file(list, file):
    """
    Open the yaml file to dump the new list of dict toself.

    .. note:: Since the the dump method automatically sorts the keys alphabetically,
    the 'id' fields will appear first than the 'msg' fields.

    :param list: the list of updated dicts
    :param file: the yaml file to load and write the new data
    :raises IOError: raises and error when the file is not found
    """
    try:
        with open(file, 'w') as f:
            yaml.dump(list, f, Dumper=IndenterDumper, default_flow_style=False)
            if verbose:
                if in_file == out_file:
                    print(("--\t[Step 3] %s updated!" % in_file))
                else:
                    print(("--\t[Step 3] %s created!" % out_file))

    except OSError as e:
        if e.errno == errno.ENOENT:
            raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), file)
        else:
            raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Read an yaml file containing the RTPS message IDs and update the naming convention to PascalCase')
    optional = parser._action_groups.pop()
    required = parser.add_argument_group('Required')
    required.add_argument("-i", "--input-file", dest="input_file",
                          help="Yaml file to read", metavar="INFILE")
    optional.add_argument("-o", "--output-file", dest="output_file",
                          help="Yaml file to dump. If not set, it is the same as the input",
                          metavar="OUTFILE", default="")
    optional.add_argument("-q", "--quiet", action="store_false", dest="verbose",
                          default=True, help="Don't print status messages to stdout")

    args = parser.parse_args()
    verbose = args.verbose
    in_file = args.input_file
    out_file = args.output_file if (args.output_file != in_file and args.output_file != "") else in_file

    if verbose:
        print("-- PX4 to ROS RTPS Ids --")

    list = load_yaml_file(in_file)
    update_dict(list)
    update_yaml_file(list, out_file)
