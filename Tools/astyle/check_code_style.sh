#!/usr/bin/env bash

FILE=$1
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

if [ -f "$FILE" ]; then
	CHECK_FAILED=$(${DIR}/fix_code_style.sh --dry-run --formatted $FILE)
	if [ -n "$CHECK_FAILED" ]; then
		${DIR}/fix_code_style.sh --quiet < $FILE > $FILE.pretty

		echo
		git --no-pager diff --no-index --minimal --histogram --color=always $FILE $FILE.pretty
		rm -f $FILE.pretty
		echo

		if [[ $PX4_ASTYLE_FIX -eq 1 ]]; then
			${DIR}/fix_code_style.sh $FILE
		else
			# Make sure this file is not staged for comitting
			git reset $FILE
			# Provide instructions
			echo $FILE 'bad formatting, please run "make format" or "./Tools/astyle/fix_code_style.sh' $FILE'"'
			exit 1
		fi
	fi
fi
