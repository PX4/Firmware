#!/bin/bash

# Script to run ShellCheck (a static analysis tool for shell scripts) over a
# script directory

if [ -z "$1" ]; then
	echo "usage: $0 <directory>"
	echo ""
	echo "  <directory>     Directory to search for scripts"
	exit -1
fi
search_directory="$1"

command -v shellcheck >/dev/null 2>&1 || { echo -e >&2 \
"Error: shellcheck required but it's not installed. On Ubuntu use:\n sudo apt-get install shellcheck\n\nAborting."; exit 1; }

scripts="$(find "$search_directory" -type f ! -name '*.txt' ! -name '*.mix' ! -name '*.bin')"

echo "Running shellcheck in '$search_directory'."

# Disabled rules:
# SC2121: allow 'set' as assignment (NuttX-style)
# SC1008: unrecognized shebang
# SC1090: use of source (.) - Can't follow non-constant source. Use a directive to specify location.
# SC1091: use of source (.) - Not following: xxxx openBinaryFile: does not exist (No such file or directory)
# SC2086: double quote to prevent globbing and word splitting
# SC2166: allow the form [ $OUTPUT_MODE == fmu -o $OUTPUT_MODE == io ]
# SC2169: In dash, 'source' in place of '.' is not supported. (we alias it)
# SC2148: allow files w/o shebang
# SC2039: In POSIX sh, array references are undefined. TODO: fix this
# SC2181: Check exit code directly with e.g. 'if mycmd;', not indirectly with $?.
shellcheck -x \
	-e SC1008 \
	-e SC1090 \
	-e SC1091 \
	-e SC2086 \
	-e SC2121 \
	-e SC2148 \
	-e SC2166 \
	-e SC2169 \
	-e SC2039 \
	-e SC2181 \
	--shell=dash \
	$scripts
ret=$?
if [ $ret -ne 0 ]; then
	echo "Please fix the above script problems."
	echo "If an error is raised that should be ignored, \
add the following right before the offending line:"
	echo "# shellcheck disable=SCxxxx"
	echo ""
	echo "Re-run the script with '$0 $@'"
	exit $ret
fi

echo "No problems found."
exit 0
