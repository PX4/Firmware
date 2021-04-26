#!/usr/bin/env bash
set -eu

# Check for the latest astyle version
ASTYLE_VER_REQUIRED_1="Artistic Style Version 2.06"
ASTYLE_VER_REQUIRED_2="Artistic Style Version 3.0"
ASTYLE_VER_REQUIRED_3="Artistic Style Version 3.0.1"
ASTYLE_VER_REQUIRED_4="Artistic Style Version 3.1"

astyle_ver() {
	echo "PX4 requires at least ${ASTYLE_VER_REQUIRED_1}"
	echo "You can get the correct version here: https://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/"
}

# check if astyle is installed
condition=$(which astyle 2>/dev/null | grep -v "not found" | wc -l)
if [ $condition -eq 0 ]; then
	echo "astyle is not installed"
	astyle_ver
	exit 1
else

	ASTYLE_VER=`astyle --version`

	if [ "$ASTYLE_VER" != "$ASTYLE_VER_REQUIRED_1" -a \
	     "$ASTYLE_VER" != "$ASTYLE_VER_REQUIRED_2" -a \
	     "$ASTYLE_VER" != "$ASTYLE_VER_REQUIRED_3" -a \
	     "$ASTYLE_VER" != "$ASTYLE_VER_REQUIRED_4" ]
	then
	    echo "Error: you're using ${ASTYLE_VER}"
	    echo "but should be using ${ASTYLE_VER_REQUIRED_1}, ${ASTYLE_VER_REQUIRED_2}, ${ASTYLE_VER_REQUIRED_3}, or ${ASTYLE_VER_REQUIRED_4} instead"
	    exit 1
	fi
fi

CI="${CI:-false}"
DIR=$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)

if [[ "$@" == "--fix" ]]; then
    export PX4_ASTYLE_FIX=1
fi


# install git pre-commit hook
HOOK_FILE="$DIR/../../.git/hooks/pre-commit"
if [ ! -f $HOOK_FILE ] && [ "$CI" != "true" ]; then
	echo ""
	echo -e "\033[31mNinja tip: add a git pre-commit hook to automatically check code style\033[0m"
	echo -e "Would you like to install one now? (\033[94mcp ./Tools/astyle/pre-commit .git/hooks/pre-commit\033[0m): [y/\033[1mN\033[0m]"

	read user_cmd
	if [ "$user_cmd" == "y" ]; then
		echo -e "copying ./Tools/astyle/pre-commit -> .git/hooks/pre-commit"
		mkdir -p $DIR/../.git/hooks
		cp $DIR/pre-commit $HOOK_FILE
		echo -e "\033[94mGreat, hook installed!\033[0m (checking style now)"
	else
		echo -e "\033[94mOk, I will remind you again later!\033[0m (checking style now)"
	fi
fi

${DIR}/files_to_check_code_style.sh | xargs -P 8 -I % ${DIR}/check_code_style.sh %

if [ $? -eq 0 ]; then
    echo "Format checks passed"
    exit 0
fi
