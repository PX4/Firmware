#!/usr/bin/env python

import em
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-t", "--tasks", dest='tasks_all', nargs='+', required=True, help="All tasks to be generated")
parser.add_argument("-s", "--tasks_additional", dest='tasks_add', nargs='+', help="Additional tasks to be generated (on top of the core)")
parser.add_argument("-i", "--input_directory", dest='directory_in', required=True, help="Output directory")
parser.add_argument("-o", "--output_directory", dest='directory_out', required=True, help="Input directory")
parser.add_argument("-f", "--files", dest='gen_files', nargs='+', required=True, help="Files to generate")

# Parse arguments
args = parser.parse_args()

for gen_file in args.gen_files:
    ofile = args.directory_out + "/" + gen_file
    output_file = open(ofile, 'w')
    # need to specify the em_globals inside the loop -> em.Error: interpreter globals collision
    em_globals = {
        "tasks": args.tasks_all,
        "tasks_add": args.tasks_add,
    }
    interpreter = em.Interpreter(output=output_file, globals=em_globals)
    interpreter.file(open(args.directory_in + "/" + gen_file + ".em"))
    interpreter.shutdown()
