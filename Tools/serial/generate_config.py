#!/usr/bin/env python3
""" Script to generate Serial (UART) parameters and the ROMFS startup script """

from __future__ import print_function

import argparse
import os
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

try:
    import yaml
except ImportError as e:
    print("Failed to import yaml: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyyaml")
    print("")
    sys.exit(1)


## Configuration

# All possible Serial ports
# Note: do not re-use or change indexes. When adding a port, always use an
# index that has never been used before. This is important for compatibility
# with QGC (parameter metadata)
serial_ports = {
    # index 0 means disabled
    # index 1000 means ethernet condiguration

    # Generic
#     "URT1": {
#         "label": "UART 1",
#         "index": 1,
#         "default_baudrate": 57600,
#         },
#     "URT2": {
#         "label": "UART 2",
#         "index": 2,
#         "default_baudrate": 57600,
#         },
#     "URT3": {
#         "label": "UART 3",
#         "index": 3,
#         "default_baudrate": 57600,
#         },
#     "URT4": {
#         "label": "UART 4",
#         "index": 4,
#         "default_baudrate": 57600,
#         },
#     "URT5": {
#         "label": "UART 5",
#         "index": 5,
#         "default_baudrate": 57600,
#         },
    "URT6": {
        "label": "UART 6",
        "index": 6,
        "default_baudrate": 57600,
        },
#     "URT7": {
#         "label": "UART 7",
#         "index": 7,
#         "default_baudrate": 57600,
#         },
#     "URT8": {
#         "label": "UART 8",
#         "index": 8,
#         "default_baudrate": 57600,
#         },
#     "URT9": {
#         "label": "UART 9",
#         "index": 9,
#         "default_baudrate": 57600,
#         },

    # Telemetry Ports
    "TEL1": { # telemetry link
        "label": "TELEM 1",
        "index": 101,
        "default_baudrate": 57600,
        },
    "TEL2": { # companion port
        "label": "TELEM 2",
        "index": 102,
        "default_baudrate": 921600,
        },
    "TEL3": {
        "label": "TELEM 3",
        "index": 103,
        "default_baudrate": 57600,
        },
    "TEL4": {
        "label": "TELEM/SERIAL 4",
        "index": 104,
        "default_baudrate": 57600,
        },

    # GPS Ports
    "GPS1": {
        "label": "GPS 1",
        "index": 201,
        "default_baudrate": 0,
        },
    "GPS2": {
        "label": "GPS 2",
        "index": 202,
        "default_baudrate": 0,
        },
    "GPS3": {
        "label": "GPS 3",
        "index": 203,
        "default_baudrate": 0,
        },

    # RC Port
    "RC": {
        "label": "Radio Controller",
        "index": 300,
        "default_baudrate": 0,
        },

    # WIFI Port (PixRacer)
    "WIFI": {
        "label": "Wifi Port",
        "index": 301,
        "default_baudrate": 1, # set default to an unusable value to detect that this serial port has not been configured
        },


    }

parser = argparse.ArgumentParser(description='Generate Serial params & startup script')

parser.add_argument('--serial-ports', type=str, nargs='*', metavar="TAG:DEVICE",
                    default=[],
                    help='Serial ports: mappings from the tag name to the device (e.g. GPS1:/dev/ttyS1)')
parser.add_argument('--config-files', type=str, nargs='*', default=[],
                    help='YAML module config file(s)')
parser.add_argument('--all-ports', action='store_true',
                    help='Generate output for all known ports (params file only)')
parser.add_argument('--constrained-flash', action='store_true',
                    help='Reduce verbosity in ROMFS scripts to reduce flash size')
parser.add_argument('--rc-dir', type=str, action='store',
                    help='ROMFS output directory', default=None)
parser.add_argument('--params-file', type=str, action='store',
                    help='Parameter output file', default=None)
parser.add_argument('--ethernet', action='store_true',
                    help='Ethernet support')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()

arg_board_serial_ports = args.serial_ports
verbose = args.verbose
rc_serial_output_dir = args.rc_dir
rc_serial_template = 'rc.serial.jinja'
rc_serial_port_template = 'rc.serial_port.jinja'
serial_params_output_file = args.params_file
serial_params_template = 'serial_params.c.jinja'
generate_for_all_ports = args.all_ports
constrained_flash = args.constrained_flash
ethernet_supported = args.ethernet

if generate_for_all_ports:
    board_ports = [(key, "") for key in serial_ports]
else:
    # convert arg_board_serial_ports list [ "TAG:DEVICE" ] into [ ("TAG", "DEVICE") ]
    board_ports = [tuple(port.split(":")) for port in arg_board_serial_ports]


if rc_serial_output_dir is None and serial_params_output_file is None:
    raise Exception("At least one of --rc-dir or --params-file "
        "(e.g. serial_params.c) needs to be specified")


# parse the YAML files
serial_commands = []
ethernet_configuration = []
additional_params = ""

if ethernet_supported:
    ethernet_configuration.append({
        'tag': "ETH",
        'label': "Ethernet",
        'index': 1000
        })

def parse_yaml_serial_config(yaml_config):
    """ parse the serial_config section from the yaml config file """
    if 'serial_config' not in yaml_config:
        return []
    ret = []
    module_name = yaml_config['module_name']
    for serial_config in yaml_config['serial_config']:
        if 'label' not in serial_config:
            serial_config['label'] = module_name
        ret.append(serial_config)
    return ret

def parse_yaml_parameters_config(yaml_config, ethernet_supported):
    """ parse the parameters section from the yaml config file """
    if 'parameters' not in yaml_config:
        return ''
    parameters_section_list = yaml_config['parameters']
    for parameters_section in parameters_section_list:
        if 'definitions' not in parameters_section:
            return ''
        definitions = parameters_section['definitions']
        ret = ''
        param_group = parameters_section.get('group', None)
        for param_name in definitions:
            param = definitions[param_name]
            if param.get('requires_ethernet', False) and not ethernet_supported:
                continue
            num_instances = param.get('num_instances', 1)
            instance_start = param.get('instance_start', 0) # offset

            # get the type and extract all tags
            tags = '@group {:}'.format(param_group)
            if param['type'] == 'enum':
                param_type = 'INT32'
                for key in param['values']:
                    tags += '\n * @value {:} {:}'.format(key, param['values'][key])
            elif param['type'] == 'boolean':
                param_type = 'INT32'
                tags += '\n * @boolean'
            elif param['type'] == 'int32':
                param_type = 'INT32'
            elif param['type'] == 'float':
                param_type = 'FLOAT'
            else:
                raise Exception("unknown param type {:}".format(param['type']))

            for tag in ['decimal', 'increment', 'category', 'volatile', 'bit',
                        'min', 'max', 'unit', 'reboot_required']:
                if tag in param:
                    tags += '\n * @{:} {:}'.format(tag, param[tag])

            for i in range(num_instances):
                # default value
                default_value = 0
                if 'default' in param:
                    # default can be a list of num_instances or a single value
                    if type(param['default']) == list:
                        assert len(param['default']) == num_instances
                        default_value = param['default'][i]
                    else:
                        default_value = param['default']

                if type(default_value) == bool:
                    default_value = int(default_value)

                # output the existing C-style format
                ret += '''
/**
 * {short_descr}
 *
 * {long_descr}
 *
 * {tags}
 */
PARAM_DEFINE_{param_type}({name}, {default_value});
'''.format(short_descr=param['description']['short'].replace("\n", "\n * "),
           long_descr=param['description']['long'].replace("\n", "\n * "),
           tags=tags,
           param_type=param_type,
           name=param_name,
           default_value=default_value,
          ).replace('${i}', str(i+instance_start))
    return ret

for yaml_file in args.config_files:
    with open(yaml_file, 'r') as stream:
        try:
            yaml_config = yaml.load(stream, Loader=yaml.Loader)
            serial_commands.extend(parse_yaml_serial_config(yaml_config))

            # TODO: additional params should be parsed in a separate script
            additional_params += parse_yaml_parameters_config(
                yaml_config, ethernet_supported)

        except yaml.YAMLError as exc:
            print(exc)
            raise


# sanity check (makes sure the param names don't exceed the max length of 16 chars)
for key in serial_ports:
    if len(key) > 4:
        raise Exception("Serial tag {:} is too long (max length=4)".format(key))

serial_devices = []
for tag, device in board_ports:
    if tag not in serial_ports:
        raise Exception("Unknown serial port {:}. "
            "You might have to add it to serial_ports in\n {:}".format(tag,
                os.path.realpath(__file__)))
    serial_devices.append({
        'tag': tag,
        'device': device,
        'label': serial_ports[tag]["label"],
        'index': serial_ports[tag]["index"],
        'default_baudrate': serial_ports[tag]["default_baudrate"]
        })


# construct commands based on selected board
commands = []
for serial_command in serial_commands:
    num_instances = serial_command.get('num_instances', 1)
    # TODO: use a loop in the script instead of explicitly enumerating all instances
    for i in range(num_instances):
        port_config = serial_command['port_config_param']
        port_param_name = port_config['name'].replace('${i}', str(i))

        # check if a port dependency is specified
        if 'depends_on_port' in port_config:
            depends_on_port = port_config['depends_on_port']
            if not any(p['tag'] == depends_on_port for p in serial_devices):
                if verbose:
                    print("Skipping {:} (missing dependent port)".format(port_param_name))
                continue

        default_port = 0 # disabled
        if 'default' in port_config:
            if type(port_config['default']) == list:
                assert len(port_config['default']) == num_instances
                default_port_str = port_config['default'][i]
            else:
                default_port_str = port_config['default']
            if default_port_str != "":
                if default_port_str not in serial_ports:
                    raise Exception("Default Port {:} not found for {:}".format(default_port_str, serial_command['label']))
                default_port = serial_ports[default_port_str]['index']

        commands.append({
            'command': serial_command['command'],
            'label': serial_command['label'],
            'instance': i,
            'multi_instance': num_instances > 1,
            'port_param_name': port_param_name,
            'default_port': default_port,
            'param_group': port_config['group'],
            'description_extended': port_config.get('description_extended', ''),
			'supports_networking': serial_command.get('supports_networking', False)
            })

if verbose:
    print("Serial Devices: {:}".format(serial_devices))
    #print("Commands: {:}".format(commands))


jinja_env = Environment(loader=FileSystemLoader(
    os.path.dirname(os.path.realpath(__file__))))

# generate the ROMFS script using a jinja template
if rc_serial_output_dir is not None:
    if generate_for_all_ports:
        raise Exception("Cannot create rc file for --all-ports")
    rc_serial_output_file = os.path.join(rc_serial_output_dir, "rc.serial")
    rc_serial_port_output_file = os.path.join(rc_serial_output_dir, "rc.serial_port")

    if verbose: print("Generating {:}".format(rc_serial_output_file))
    if len(serial_devices) == 0:
        # if the board has no UARTs, create an empty rc file
        open(rc_serial_output_file, 'w').close()
    else:
        template = jinja_env.get_template(rc_serial_template)
        with open(rc_serial_output_file, 'w') as fid:
            fid.write(template.render(serial_devices=serial_devices,
                commands=commands,
                constrained_flash=constrained_flash))

        if verbose: print("Generating {:}".format(rc_serial_port_output_file))
        template = jinja_env.get_template(rc_serial_port_template)
        with open(rc_serial_port_output_file, 'w') as fid:
            fid.write(template.render(serial_devices=serial_devices,
                ethernet_configuration=ethernet_configuration,
                constrained_flash=constrained_flash))

# parameter definitions
if serial_params_output_file is not None:
    if verbose: print("Generating {:}".format(serial_params_output_file))
    template = jinja_env.get_template(serial_params_template)
    with open(serial_params_output_file, 'w') as fid:
        fid.write(template.render(serial_devices=serial_devices,
            ethernet_configuration=ethernet_configuration,
            commands=commands, serial_ports=serial_ports,
            additional_definitions=additional_params))

