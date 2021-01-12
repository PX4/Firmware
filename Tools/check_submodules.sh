#!/usr/bin/env bash

function check_git_submodule {

# The .git exists in a submodule if init and update have been done.
if [[ -f $1"/.git" || -d $1"/.git" ]]; then

	# always update within CI environment or configuring withing VSCode CMake where you can't interact
	if [ "$CI" == "true" ] || [ -n "${VSCODE_PID+set}" ]; then
		git submodule --quiet sync --recursive -- $1
		git submodule --quiet update --init --recursive --jobs=8 -- $1  || true
		git submodule --quiet update --init --recursive --jobs=8 -- $1
		exit 0
	fi

	SUBMODULE_STATUS=$(git submodule summary "$1")
	STATUSRETVAL=$(echo $SUBMODULE_STATUS | grep -A20 -i "$1")
	if ! [[ -z "$STATUSRETVAL" ]]; then
		echo -e "\033[31mChecked $1 submodule, ACTION REQUIRED:\033[0m"
		echo ""
		echo -e "Different commits:"
		echo -e "$SUBMODULE_STATUS"
		echo ""
		echo ""
		echo -e " *******************************************************************************"
		echo -e " *   \033[31mIF YOU DID NOT CHANGE THIS FILE (OR YOU DON'T KNOW WHAT A SUBMODULE IS):\033[0m  *"
		echo -e " *   \033[31mHit 'u' and <ENTER> to update ALL submodules and resolve this.\033[0m            *"
		echo -e " *   (performs \033[94mgit submodule sync --recursive\033[0m                                  *"
		echo -e " *    and \033[94mgit submodule update --init --recursive\033[0m )                            *"
		echo -e " *******************************************************************************"
		echo ""
		echo ""
		echo -e "   Only for EXPERTS:"
		echo -e "   $1 submodule is not in the recommended version."
		echo -e "   Hit 'y' and <ENTER> to continue the build with this version. Hit <ENTER> to resolve manually."
		echo -e "   Use \033[94mgit add $1 && git commit -m 'Updated $1'\033[0m to choose this version (careful!)"
		echo ""
		read user_cmd
		if [ "$user_cmd" == "y" ]; then
			echo "Continuing build with manually overridden submodule.."
		elif [ "$user_cmd" == "u" ]; then
			git submodule sync --recursive -- $1
			git submodule update --init --recursive -- $1 || true
			git submodule update --init --recursive --force -- $1
			echo "Submodule fixed, continuing build.."
		else
			echo "Build aborted."
			exit 1
		fi
	fi
else
	git submodule --quiet sync --recursive --quiet -- $1
	git submodule --quiet update --init --recursive -- $1  || true
	git submodule --quiet update --init --recursive -- $1
fi

}

# If called with a path then respect $GIT_SUBMODULES_ARE_EVIL but do normal processing
if [ "$#" != "0" ]; then
	# called with a path then process only that path but respect $GIT_SUBMODULES_ARE_EVIL
	[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
		# GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules updated
		exit 0
	}

	check_git_submodule $1

else

	[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && {
		# GIT_SUBMODULES_ARE_EVIL is set, meaning user doesn't want submodules updated
		echo "GIT_SUBMODULES_ARE_EVIL is defined - Skipping All submodule checking!"
		exit 0
	}

	submodules=$(git submodule status | awk '{ print $2 }')
	for i in $submodules;
	do
		check_git_submodule $i
	done

fi

exit 0
