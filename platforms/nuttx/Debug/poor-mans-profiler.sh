#!/usr/bin/env bash
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#
# Poor man's sampling profiler for NuttX.
#
# Usage: Install flamegraph.pl in your PATH, configure your .gdbinit, run the script with proper arguments and go
#        have a coffee. When you're back, you'll see the flamegraph. Note that frequent calls to GDB significantly
#        interfere with normal operation of the target, which means that you can't profile real-time tasks with it.
#        For best results, ensure that the PC is not overloaded, the USB host controller to which the debugger is
#        connected is not congested. You should also allow the current user to set negative nice values.
#
# The FlameGraph script can be downloaded from https://github.com/brendangregg/FlameGraph. Thanks Mr. Gregg.
#
# Requirements: ARM GDB with Python support. You can get one by downloading the sources from
#               https://launchpad.net/gcc-arm-embedded and building them with correct flags.
#               Note that Python support is not required if no per-task sampling is needed.
#
# Usage with PX4:
#   0) See above regarding flamegraph.pl
#   1) Connect your device, e.g. Pixhawk 4
#   2) Build PX4: make px4_fmu-v5_default
#   3) Flash PX4: make px4_fmu-v5_default upload
#   4) Connect a JTAG/SWD debugger, e.g. a DroneCode Probe
#   5) Profile: make px4_fmu-v5_default profile

set -e
root=$(dirname $0)

function die()
{
    echo "$@"
    exit 1
}

function usage()
{
    echo "Invalid usage. Supported options:"
    cat $0 | sed -n 's/^\s*--\([^)\*]*\).*/\1/p' # Don't try this at home.
    exit 1
}

if [ -z `which flamegraph.pl` ]; then
    echo "Install flamegraph.pl first.  Available from:"
    echo -e "\thttps://github.com/brendangregg/FlameGraph"
    exit 1
fi

#
# Parsing the arguments. Read this section for usage info.
#
nsamples=0
sleeptime=0.1    # Doctors recommend 7-8 hours a day
taskname=
elf=
append=0
fgfontsize=10
fgwidth=1900
gdbdev=`ls /dev/serial/by-id/*Black_Magic_Probe*-if00`

for i in "$@"
do
    case $i in
        --nsamples=*)
            nsamples="${i#*=}"
            ;;
        --sleeptime=*)
            sleeptime="${i#*=}"
            ;;
        --taskname=*)
            taskname="${i#*=}"
            ;;
        --elf=*)
            elf="${i#*=}"
            ;;
        --append)
            append=1
            ;;
        --fgfontsize=*)
            fgfontsize="${i#*=}"
            ;;
        --fgwidth=*)
            fgwidth="${i#*=}"
            ;;
        --gdbdev=*)
            gdbdev="${i#*=}"
            ;;
        *)
            usage
            ;;
    esac
    shift
done

[[ -z "$elf" ]] && die "Please specify the ELF file location, e.g.: build/px4_fmu-v4_default/px4_fmu-v4_default.elf"

#
# Temporary files
#
stacksfile=/tmp/pmpn-stacks.log
foldfile=/tmp/pmpn-folded.txt
graphfile=/tmp/pmpn-flamegraph.svg
gdberrfile=/tmp/pmpn-gdberr.log

#
# Sampling if requested. Note that if $append is true, the stack file will not be rewritten.
#
cd $root

if [[ $nsamples > 0 ]]
then
    [[ $append = 0 ]] && (rm -f $stacksfile; echo "Old stacks removed")

    echo "Sampling the task '$taskname'..."

    for x in $(seq 1 $nsamples)
    do
        if [[ "$taskname" = "" ]]
        then
            arm-none-eabi-gdb $elf --nx --quiet --batch \
                                           -ex "set print asm-demangle on" \
                                           -ex "target extended $gdbdev" \
                                           -ex "monitor swdp_scan" \
                                           -ex "attach 1" \
                                           -ex bt \
                2> $gdberrfile \
                | sed -n 's/\(#.*\)/\1/p' \
                >> $stacksfile
        else
            arm-none-eabi-gdb $elf --nx --quiet --batch \
                                           -ex "set print asm-demangle on" \
                                           -ex "target extended $gdbdev" \
                                           -ex "monitor swdp_scan" \
                                           -ex "attach 1" \
                                           -ex "source $root/Nuttx.py" \
                                           -ex "show mybt $taskname" \
                2> $gdberrfile \
                | sed -n 's/0\.0:\(#.*\)/\1/p' \
                >> $stacksfile
        fi
        echo -e '\n\n' >> $stacksfile
        echo -ne "\r$x/$nsamples"
        sleep $sleeptime
    done

    echo
    echo "Stacks saved to $stacksfile"
else
    echo "Sampling skipped - set 'nsamples' to re-sample."
fi

#
# Folding the stacks.
#
[ -f $stacksfile ] || die "Where are the stack samples? Expected file not found: $stacksfile"

cat << 'EOF' > /tmp/pmpn-folder.py
#
# This stack folder correctly handles C++ types.
#
from __future__ import print_function, division
import fileinput, collections, os, sys

def enforce(x, msg='Invalid input'):
    if not x:
        raise Exception(msg)

def split_first_part_with_parens(line):
    LBRACES = {'(':'()', '<':'<>', '[':'[]', '{':'{}'}
    RBRACES = {')':'()', '>':'<>', ']':'[]', '}':'{}'}
    QUOTES = set(['"', "'"])
    quotes = collections.defaultdict(bool)
    braces = collections.defaultdict(int)
    out = ''
    for ch in line:
        out += ch
        # escape character cancels further processing
        if ch == '\\':
            continue
        # special cases
        if out.endswith('operator>') or out.endswith('operator>>') or out.endswith('operator->'):  # gotta love c++
            braces['<>'] += 1
        if out.endswith('operator<') or out.endswith('operator<<'):
            braces['<>'] -= 1
        # switching quotes
        if ch in QUOTES:
            quotes[ch] = not quotes[ch]
        # counting parens only when outside quotes
        if sum(quotes.values()) == 0:
            if ch in LBRACES.keys():
                braces[LBRACES[ch]] += 1
            if ch in RBRACES.keys():
                braces[RBRACES[ch]] -= 1
        # sanity check
        for v in braces.values():
            enforce(v >= 0, 'Unaligned braces: ' + str(dict(braces)))
        # termination condition
        if ch == ' ' and sum(braces.values()) == 0:
            break
    out = out.strip()
    return out, line[len(out):]

def parse(line):
    def take_path(line, output):
        line = line.strip()
        if line.startswith('at '):
            line = line[3:].strip()
        if line:
            output['file_full_path'] = line.rsplit(':', 1)[0].strip()
            output['file_base_name'] = os.path.basename(output['file_full_path'])
            output['line'] = int(line.rsplit(':', 1)[1])
        return output

    def take_args(line, output):
        line = line.lstrip()
        if line[0] == '(':
            output['args'], line = split_first_part_with_parens(line)
        return take_path(line.lstrip(), output)

    def take_function(line, output):
        output['function'], line = split_first_part_with_parens(line.lstrip())
        return take_args(line.lstrip(), output)

    def take_mem_loc(line, output):
        line = line.lstrip()
        if line.startswith('0x'):
            end = line.find(' ')
            num = line[:end]
            output['memloc'] = int(num, 16)
            line = line[end:].lstrip()
            end = line.find(' ')
            enforce(line[:end] == 'in')
            line = line[end:].lstrip()
        return take_function(line, output)

    def take_frame_num(line, output):
        line = line.lstrip()
        enforce(line[0] == '#')
        end = line.find(' ')
        num = line[1:end]
        output['frame_num'] = int(num)
        return take_mem_loc(line[end:], output)

    return take_frame_num(line, {})

stacks = collections.defaultdict(int)
current = ''

stack_tops = collections.defaultdict(int)
num_stack_frames = 0

for idx,line in enumerate(fileinput.input()):
    try:
        line = line.strip()
        if line:
            inf = parse(line)
            fun = inf['function']
            current = (fun + ';' + current) if current else fun

            if inf['frame_num'] == 0:
                num_stack_frames += 1
                stack_tops[fun] += 1
        elif current:
            stacks[current] += 1
            current = ''
    except Exception, ex:
        print('ERROR (line %d):' % (idx + 1), ex, file=sys.stderr)

for s, f in sorted(stacks.items(), key=lambda (s, f): s):
    print(s, f)

print('Total stack frames:', num_stack_frames, file=sys.stderr)
print('Top consumers (distribution of the stack tops):', file=sys.stderr)
for name,num in sorted(stack_tops.items(), key=lambda (name, num): num, reverse=True)[:300]:
    print('% 7.3f%%   ' % (100 * num / num_stack_frames), name, file=sys.stderr)
EOF

cat $stacksfile | python /tmp/pmpn-folder.py > $foldfile

echo "Folded stacks saved to $foldfile"

#
# Graphing.
#
cat $foldfile | flamegraph.pl --fontsize=$fgfontsize --width=$fgwidth > $graphfile
echo "FlameGraph saved to $graphfile"

# On KDE, xdg-open prefers Gwenview by default, which doesn't handle interactive SVGs, so we need a browser.
# The current implementation is hackish and stupid. Somebody, please do something about it.
opener=xdg-open
which firefox       > /dev/null && opener=firefox
which google-chrome > /dev/null && opener=google-chrome

$opener $graphfile
