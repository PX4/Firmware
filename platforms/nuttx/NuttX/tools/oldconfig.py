#!/usr/bin/env python

# Copyright (c) 2018-2019, Ulf Magnusson
# SPDX-License-Identifier: ISC

# Implements oldconfig functionality:
#
#   1. Load existing .config
#   2. Prompt the user for the value of all modifiable symbols/choices that
#      aren't already set in the .config
#   3. Write new .config
#
# The default input/output filename is '.config'. A different filename can be
# passed in the KCONFIG_CONFIG environment variable.
#
# Unlike 'make oldconfig', this script doesn't print menu titles and comments,
# but gives Kconfig definition locations. Printing menus and comments would be
# pretty easy to add: Look at the parents of each item and print all menu
# prompts and comments unless they have already been printed (assuming you want
# to skip "irrelevant" menus).
#
# Entering '?' displays the help text of the symbol/choice, if any.

from __future__ import print_function

import sys

from six.moves import input

from kconfiglib import Symbol, Choice, BOOL, TRISTATE, HEX, standard_kconfig


# Note: Used as the entry point in setup.py
def _main():
    # Earlier symbols in Kconfig files might depend on later symbols and become
    # visible if their values change. This flag is set to True if the value of
    # any symbol changes, in which case we rerun the oldconfig to check for new
    # visible symbols.
    global conf_changed

    kconf = standard_kconfig()
    kconf.load_config()

    while True:
        conf_changed = False

        for node in kconf.node_iter():
            oldconfig(node)

        if not conf_changed:
            break

    kconf.write_config()


def oldconfig(node):
    """
    Prompts the user for a value if node.item is a visible symbol/choice with
    no user value.
    """
    # See main()
    global conf_changed

    # Only symbols and choices can be configured
    if not isinstance(node.item, (Symbol, Choice)):
        return

    # Skip symbols and choices that aren't visible
    if not node.item.visibility:
        return

    # Skip symbols and choices that don't have a prompt (at this location)
    if not node.prompt:
        return

    if isinstance(node.item, Symbol):
        sym = node.item

        # Skip symbols that already have a user value
        if sym.user_value is not None:
            return

        # Skip symbols that can only have a single value, due to selects
        if len(sym.assignable) == 1:
            return

        # Skip symbols in choices in y mode. We ask once for the entire choice
        # instead.
        if sym.choice and sym.choice.tri_value == 2:
            return

        # Loop until the user enters a valid value or enters a blank string
        # (for the default value)
        while True:
            val = input("{} ({}) [{}] ".format(
                node.prompt[0], _name_and_loc_str(sym),
                _default_value_str(sym)))

            if val == "?":
                _print_help(node)
                continue

            # Substitute a blank string with the default value the symbol
            # would get
            if not val:
                val = sym.str_value

            # Automatically add a "0x" prefix for hex symbols, like the
            # menuconfig interface does. This isn't done when loading .config
            # files, hence why set_value() doesn't do it automatically.
            if sym.type == HEX and not val.startswith(("0x", "0X")):
                val = "0x" + val

            old_str_val = sym.str_value

            # Kconfiglib itself will print a warning here if the value
            # is invalid, so we don't need to bother
            if sym.set_value(val):
                # Valid value input. We're done with this node.

                if sym.str_value != old_str_val:
                    conf_changed = True

                return

    else:
        choice = node.item

        # Skip choices that already have a visible user selection...
        if choice.user_selection and choice.user_selection.visibility == 2:
            # ...unless there are new visible symbols in the choice. (We know
            # they have y (2) visibility in that case, because m-visible
            # symbols get demoted to n-visibility in y-mode choices, and the
            # user-selected symbol had visibility y.)
            for sym in choice.syms:
                if sym is not choice.user_selection and sym.visibility and \
                   sym.user_value is None:
                    # New visible symbols in the choice
                    break
            else:
                # No new visible symbols in the choice
                return

        # Get a list of available selections. The mode of the choice limits
        # the visibility of the choice value symbols, so this will indirectly
        # skip choices in n and m mode.
        options = [sym for sym in choice.syms if sym.visibility == 2]

        if not options:
            # No y-visible choice value symbols
            return

        # Loop until the user enters a valid selection or a blank string (for
        # the default selection)
        while True:
            print("{} ({})".format(node.prompt[0], _name_and_loc_str(choice)))

            for i, sym in enumerate(options, 1):
                print("{} {}. {} ({})".format(
                    ">" if sym is choice.selection else " ",
                    i,
                    # Assume people don't define choice symbols with multiple
                    # prompts. That generates a warning anyway.
                    sym.nodes[0].prompt[0],
                    sym.name))

            sel_index = input("choice[1-{}]: ".format(len(options)))

            if sel_index == "?":
                _print_help(node)
                continue

            # Pick the default selection if the string is blank
            if not sel_index:
                choice.selection.set_value(2)
                break

            try:
                sel_index = int(sel_index)
            except ValueError:
                print("Bad index", file=sys.stderr)
                continue

            if not 1 <= sel_index <= len(options):
                print("Bad index", file=sys.stderr)
                continue

            # Valid selection

            if options[sel_index - 1].tri_value != 2:
                conf_changed = True

            options[sel_index - 1].set_value(2)
            break

        # Give all of the non-selected visible choice symbols the user value n.
        # This makes it so that the choice is no longer considered new once we
        # do additional passes, if the reason that it was considered new was
        # that it had new visible choice symbols.
        #
        # Only giving visible choice symbols the user value n means we will
        # prompt for the choice again if later user selections make more new
        # choice symbols visible, which is correct.
        for sym in choice.syms:
            if sym is not choice.user_selection and sym.visibility:
                sym.set_value(0)


def _name_and_loc_str(sc):
    # Helper for printing the name of the symbol/choice 'sc' along with the
    # location(s) in the Kconfig files where it is defined. Unnamed choices
    # return "choice" instead of the name.

    return "{}, defined at {}".format(
        sc.name or "choice",
        ", ".join("{}:{}".format(node.filename, node.linenr)
                  for node in sc.nodes))


def _print_help(node):
    print("\n" + (node.help or "No help text\n"))


def _default_value_str(sym):
    # Returns the "m/M/y" string in e.g.
    #
    #   TRISTATE_SYM prompt (TRISTATE_SYM, defined at Kconfig:9) [n/M/y]:
    #
    # For string/int/hex, returns the default value as-is.

    if sym.type in (BOOL, TRISTATE):
        return "/".join(("NMY" if sym.tri_value == tri else "nmy")[tri]
                        for tri in sym.assignable)

    # string/int/hex
    return sym.str_value


if __name__ == "__main__":
    _main()
