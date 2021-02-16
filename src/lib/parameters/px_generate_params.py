#!/usr/bin/env python
"""
Param source code generation script.
"""
from __future__ import print_function
import xml.etree.ElementTree as ET
import codecs
import argparse
import sys

try:
    from jinja2 import Environment, FileSystemLoader
except ImportError as e:
    print("Failed to import jinja2: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user jinja2")
    print("")
    sys.exit(1)

import os

def generate(xml_file, dest='.'):
    """
    Generate px4 param source from xml.

    @param xml_file: input parameter xml file
    @param dest: Destination directory for generated files
        None means to scan everything.
    """
    # pylint: disable=broad-except
    tree = ET.parse(xml_file)
    root = tree.getroot()

    params = []
    for group in root:
        if group.tag == "group" and "no_code_generation" not in group.attrib:
            for param in group:
                params.append(param)

    params = sorted(params, key=lambda name: name.attrib["name"])

    script_path = os.path.dirname(os.path.realpath(__file__))

    # for jinja docs see: http://jinja.pocoo.org/docs/2.9/api/
    env = Environment(
        loader=FileSystemLoader(os.path.join(script_path, 'templates')))

    if not os.path.isdir(dest):
        os.path.mkdir(dest)

    template_files = [
        'px4_parameters.hpp.jinja',
    ]
    for template_file in template_files:
        template = env.get_template(template_file)
        with open(os.path.join(
                dest, template_file.replace('.jinja','')), 'w') as fid:
            fid.write(template.render(params=params))

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--xml", help="parameter xml file")
    arg_parser.add_argument("--dest", help="destination path", default=os.path.curdir)
    args = arg_parser.parse_args()
    generate(xml_file=args.xml, dest=args.dest)

#  vim: set et fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :
