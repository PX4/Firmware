#! /usr/bin/env python3
import sys
import re
import math
import textwrap
from functools import reduce


class ModuleDocumentation(object):
    """
    documentation for a single module
    """

    # If you add categories or subcategories, they also need to be added to the
    # TOC in https://github.com/PX4/Devguide/blob/master/en/SUMMARY.md
    valid_categories = ['driver', 'estimator', 'controller', 'system',
                        'communication', 'command', 'template', 'simulation']
    valid_subcategories = ['', 'distance_sensor', 'imu', 'airspeed_sensor',
                           'magnetometer', 'baro', 'optical_flow']

    max_line_length = 80 # wrap lines that are longer than this

    def __init__(self, function_calls, scope):
        """
        :param function_calls: list of tuples (function_name, [str(arg)])
        """
        self._name = ''
        self._category = ''
        self._subcategory = ''
        self._doc_string = ''
        self._usage_string = ''
        self._first_command = True
        self._scope = scope

        self._options = '' # all option chars
        self._explicit_options = '' # all option chars (explicit in the module)
        self._all_values = [] # list of all values
        self._all_commands = []

        self._paring_implicit_options = False

        for func_name, args in function_calls:
            attribute_name = '_handle_'+func_name.lower()
            try:
                f = getattr(self, attribute_name)
                f(args)
            except AttributeError:
                raise Exception('unhandled function: PRINT_MODULE_'+func_name)

        self._usage_string = self._wrap_long_lines(self._usage_string, 17)


    def _handle_description(self, args):
        assert(len(args) == 1) # description
        self._doc_string = self._get_string(args[0])

    def _handle_usage_name(self, args):
        assert(len(args) == 2) # executable_name, category
        self._name = self._get_string(args[0])
        self._category = self._get_string(args[1])

        self._usage_string = "%s <command> [arguments...]\n" % self._name
        self._usage_string += " Commands:\n"

    def _handle_usage_subcategory(self, args):
        assert(len(args) == 1) # description
        self._subcategory = self._get_string(args[0])

    def _handle_usage_name_simple(self, args):
        assert(len(args) == 2) # executable_name, category
        self._name = self._get_string(args[0])
        self._category = self._get_string(args[1])

        self._usage_string = "%s [arguments...]\n" % self._name

    def _handle_usage_command_descr(self, args):
        assert(len(args) == 2) # name, description
        name = self._get_string(args[0])
        self._all_commands.append(name)

        if self._first_command:
            self._first_command = False
        else:
            self._usage_string += "\n"

        if self._is_string(args[1]):
            description = self._get_string(args[1])
            self._usage_string += "   %-13s %s\n" % (name, description)
        else:
            self._usage_string += "   %s\n" % name

    def _handle_usage_command(self, args):
        assert(len(args) == 1) # name
        args.append('nullptr')
        self._handle_usage_command_descr(args)

    def _handle_usage_default_commands(self, args):
        assert(len(args) == 0)
        self._handle_usage_command(['"stop"'])
        self._handle_usage_command_descr(['"status"', '"print status info"'])

    def _handle_usage_param_int(self, args):
        assert(len(args) == 6) # option_char, default_val, min_val, max_val, description, is_optional
        option_char = self._get_option_char(args[0])
        default_val = int(args[1], 0)
        description = self._get_string(args[4])
        if self._is_bool_true(args[5]):
            self._usage_string += "     [-%s <val>]  %s\n" % (option_char, description)
            if default_val != -1:
                self._usage_string += "                 default: %i\n" % default_val
        else:
            self._usage_string += "     -%s <val>    %s\n" % (option_char, description)

    def _handle_usage_param_float(self, args):
        assert(len(args) == 6) # option_char, default_val, min_val, max_val, description, is_optional
        option_char = self._get_option_char(args[0])
        default_val = self._get_float(args[1])
        description = self._get_string(args[4])
        if self._is_bool_true(args[5]):
            self._usage_string += "     [-%s <val>]  %s\n" % (option_char, description)
            if not math.isnan(default_val):
                self._usage_string += "                 default: %.1f\n" % default_val
        else:
            self._usage_string += "     -%s <val>    %s\n" % (option_char, description)

    def _handle_usage_params_i2c_spi_driver(self, args):
        assert(len(args) == 2) # i2c_support, spi_support
        self._paring_implicit_options = True
        if self._is_bool_true(args[0]):
            self._handle_usage_param_flag(['\'I\'', "\"Internal I2C bus(es)\"", 'true'])
            self._handle_usage_param_flag(['\'X\'', "\"External I2C bus(es)\"", 'true'])
        if self._is_bool_true(args[1]):
            self._handle_usage_param_flag(['\'s\'', "\"Internal SPI bus(es)\"", 'true'])
            self._handle_usage_param_flag(['\'S\'', "\"External SPI bus(es)\"", 'true'])

        self._handle_usage_param_int(['\'b\'', '-1', '0', '16',
            "\"board-specific bus (default=all) (external SPI: n-th bus (default=1))\"", 'true'])

        if self._is_bool_true(args[1]):
            self._handle_usage_param_int(['\'c\'', '1', '1', '10',
                "\"chip-select index (for external SPI)\"", 'true'])
            self._handle_usage_param_int(['\'m\'', '-1', '0', '3', "\"SPI mode\"", 'true'])

        self._handle_usage_param_int(['\'f\'', '-1', '0', '1000000', "\"bus frequency in kHz\"", 'true'])
        self._handle_usage_param_flag(['\'q\'', "\"quiet startup (no message if no device found)\"", 'true'])
        self._paring_implicit_options = False

    def _handle_usage_params_i2c_address(self, args):
        assert(len(args) == 1) # i2c_address
        self._paring_implicit_options = True
        self._handle_usage_param_int(['\'a\'', args[0], '0', '0xff', "\"I2C address\"", 'true'])
        self._paring_implicit_options = False

    def _handle_usage_params_i2c_keep_running_flag(self, args):
        assert(len(args) == 0)
        self._paring_implicit_options = True
        self._handle_usage_param_flag(['\'k\'', "\"if initialization (probing) fails, keep retrying periodically\"", 'true'])
        self._paring_implicit_options = False

    def _handle_usage_param_flag(self, args):
        assert(len(args) == 3) # option_char, description, is_optional
        option_char = self._get_option_char(args[0])
        description = self._get_string(args[1])
        if self._is_bool_true(args[2]):
            self._usage_string += "     [-%c]        %s\n" % (option_char, description)
        else:
            self._usage_string += "     -%c          %s\n" % (option_char, description)

    def _handle_usage_param_string(self, args):
        assert(len(args) == 5) # option_char, default_val, values, description, is_optional
        option_char = self._get_option_char(args[0])
        description = self._get_string(args[3])
        if self._is_bool_true(args[4]):
            self._usage_string += "     [-%c <val>]  %s\n" % (option_char, description)
        else:
            self._usage_string += "     -%c <val>    %s\n" % (option_char, description)

        if self._is_string(args[2]):
            values = self._get_string(args[2])
            self._all_values.append(values)
            if self._is_string(args[1]):
                default_val = self._get_string(args[1])
                self._usage_string += "                 values: %s, default: %s\n" %(values, default_val)
            else:
                self._usage_string += "                 values: %s\n" % values
        else:
            if self._is_string(args[1]):
                default_val = self._get_string(args[1])
                self._usage_string += "                 default: %s\n" % default_val

    def _handle_usage_param_comment(self, args):
        assert(len(args) == 1) # comment
        comment = self._get_string(args[0])
        self._usage_string += self._wrap_long_lines("\n %s\n" % comment, 1)

    def _handle_usage_arg(self, args):
        assert(len(args) == 3) # values, description, is_optional
        values = self._get_string(args[0])
        self._all_values.append(values)
        description = self._get_string(args[1])
        if self._is_bool_true(args[2]):
            values += ']'
            self._usage_string += "     [%-10s %s\n" % (values, description)
        else:
            self._usage_string += "     %-11s %s\n" % (values, description)


    def _get_string(self, string):
        return string[1:-1] # remove the " at start & end

    def _get_float(self, string):
        f = string
        if f[-1] == 'f':
            f = f[:-1]
        return float(f)

    def _is_string(self, argument):
        return len(argument) > 0 and argument[0] == '"'

    def _is_bool_true(self, argument):
        return len(argument) > 0 and argument == 'true'

    def _get_option_char(self, argument):
        assert(len(argument) == 3) # must have the form: 'p' (assume there's no escaping)
        option_char = argument[1]
        self._options += option_char
        if not self._paring_implicit_options:
            self._explicit_options += option_char
        return option_char


    def _wrap_long_lines(self, string, indentation_spaces):
        """
        wrap long lines in a string
        :param indentation_spaces: number of added spaces on continued lines
        """
        ret = ''
        for s in string.splitlines():
            ret += textwrap.fill(s, self.max_line_length,
                    subsequent_indent=' '*indentation_spaces)+'\n'
        return ret

    def name(self):
        return self._name

    def category(self):
        return self._category

    def subcategory(self):
        return self._subcategory

    def scope(self):
        return self._scope

    def documentation(self):
        doc_string = self._doc_string

        # convert '$ cmd' commands into code blocks (e.g. '$ logger start')
        # use lookahead (?=...) so the multiple consecutive command lines work
        doc_string = re.sub(r"\n\$ (.*)(?=\n)", r"\n```\n\1\n```", doc_string)
        # now merge consecutive blocks
        doc_string = re.sub(r"\n```\n```\n", r"\n", doc_string)

        return doc_string

    def usage_string(self):
        usage_string = self._usage_string

        while len(usage_string) > 1 and usage_string[-1] == '\n':
            usage_string = usage_string[:-1]

        return usage_string

    def options(self):
        """
        get all the -p options as string of chars, that are explicitly set in
        the module
        """
        return self._explicit_options

    def all_values(self):
        """
        get a list of all command values
        """
        return self._all_values

    def all_commands(self):
        """
        get a list of all commands
        """
        return self._all_commands


class SourceParser(object):
    """
    Parses provided data and stores all found parameters internally.
    """

    # Regex to extract module doc function calls, starting with PRINT_MODULE_
    re_doc_definition = re.compile(r'PRINT_MODULE_([A-Z0-9_]*)\s*\(')

    def __init__(self):
        self._modules = {} # all found modules: key is the module name
        self._consistency_checks_failure = False # one or more checks failed

        self._comment_remove_pattern = re.compile(
            r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
            re.DOTALL | re.MULTILINE)

    def Parse(self, scope, contents):
        """
        Incrementally parse program contents and append all found documentations
        to the list.
        """

        # remove comments from source
        contents = self._comment_remover(contents)

        extracted_function_calls = [] # list of tuples: (FUNC_NAME, list(ARGS))

        start_index = 0
        while start_index < len(contents):
            # skip whitespace
            while start_index < len(contents) and contents[start_index] in [ ' ', '\t']:
                start_index += 1

            end_index = contents.find('\n', start_index)
            if end_index == -1: end_index = len(contents)
            line = contents[start_index:end_index]

            # Ignore empty lines and macro #if's
            if line == "" or line.startswith('#if'):
                start_index = end_index + 1
                continue

            m = self.re_doc_definition.match(contents, start_index, end_index)
            if m:
                func_name = m.group(1)
                end_index_match = m.span()[1]
                next_start_index, arguments = self._parse_arguments(contents, end_index_match)

                extracted_function_calls.append((func_name, arguments))

                start_index = end_index + 1
                if next_start_index > start_index:
                    start_index = next_start_index

                continue
            start_index = end_index + 1


        if len(extracted_function_calls) > 0:
            # add the module to the dict
            module_doc = ModuleDocumentation(extracted_function_calls, scope)

            if module_doc.name() == '':
                raise  Exception('PRINT_MODULE_USAGE_NAME not given for ' + scope)
            if not module_doc.category() in ModuleDocumentation.valid_categories:
                raise  Exception('Invalid/unknown category ' +
                        module_doc.category() + ' for ' + scope)
            if not module_doc.subcategory() in ModuleDocumentation.valid_subcategories:
                raise  Exception('Invalid/unknown subcategory ' +
                        module_doc.subcategory() + ' for ' + scope)

            self._do_consistency_check(contents, scope, module_doc)

            self._modules[module_doc.name()] = module_doc

        return True

    def _comment_remover(self, text):
        """ remove C++ & C style comments.
            Source: https://stackoverflow.com/a/241506 """
        def replacer(match):
            s = match.group(0)
            if s.startswith('/'):
                return " " # note: a space and not an empty string
            else:
                return s
        return re.sub(self._comment_remove_pattern, replacer, text)

    def _do_consistency_check(self, contents, scope, module_doc):
        """
        check the documentation for consistency with the code (arguments to
        getopt() and others). This is only approximative, but should catch cases
        where an option was added and not documented.
        """

        # search all option chars in getopt() calls, combine them & compare
        # against the documented set
        getopt_args = re.findall(r"\b(px4_|)getopt\b.*\"([a-zA-Z:]+)\"", contents)
        # there could be several getopt calls and it is not simple to find which
        # command it belongs to, so combine all into a single string
        getopt_args = reduce(lambda a, b: a + b[1], getopt_args, '').replace(':', '')

        # some modules don't use getopt or parse the options in another file,
        # so only check if both lists are not empty
        if len(getopt_args) > 0 and len(module_doc.options()) > 0:
            # sort & remove duplicates
            sorted_getopt_args = ''.join(set(sorted(getopt_args)))
            sorted_module_options = ''.join(set(sorted(module_doc.options())))
            if sorted_getopt_args != sorted_module_options:
                failed = True

                # do one more test: check if strcmp(..."-x"... is used instead
                if len(sorted_getopt_args) < len(sorted_module_options):
                    failed = False
                    # iterate options that are only in module doc
                    for c in set(sorted_module_options) - set(sorted_getopt_args):
                        if len(re.findall(r"\bstrcmp\b.*\"-"+c+r"\"", contents)) == 0:
                            failed = True

                if failed:
                    print(("Warning: documentation inconsistency in %s:" % scope))
                    print((" Documented options       : %s" % sorted_module_options))
                    print((" Options found in getopt(): %s" % sorted_getopt_args))
                    self._consistency_checks_failure = True


        # now check the commands: search for strcmp(argv[i], "command".
        # this will also find the value arguments, so append them too to the
        # module doc strings
        commands = re.findall(r"\bstrcmp\b.*argv\[.*\"(.+)\"", contents) + \
                   re.findall(r"\bstrcmp\b.*\"(.+)\".*argv\[", contents) + \
                   re.findall(r"\bstrcmp\b.*\bverb\b.*\"(.+)\"", contents)

        doc_commands = module_doc.all_commands() + \
                [x for value in module_doc.all_values() for x in value.split('|')]

        for command in commands:
            if len(command) == 2 and command[0] == '-':
                continue # skip options

            if command in ['start', 'stop', 'status']:
                continue # handled in the base class

            if not command in doc_commands:
                print(("Warning: undocumented command '%s' in %s" %(command, scope)))
                self._consistency_checks_failure = True

        # limit the maximum line length in the module doc string
        max_line_length = 120
        module_doc = module_doc.documentation()
        verbatim_mode = False
        line_nr = 0
        for line in module_doc.split('\n'):
            line_nr += 1
            if line.strip().startswith('```'):
                # ignore preformatted blocks
                verbatim_mode = not verbatim_mode
            elif not verbatim_mode:
                if not 'www.' in line and not 'http' in line:
                    if len(line) > max_line_length:
                        print(('Line too long (%i > %i) in %s:' % (len(line), max_line_length, scope)))
                        print((' '+line))
                        self._consistency_checks_failure = True


    def _parse_arguments(self, contents, start_index):
        """
        parse function arguments into a list and return a tuple with (index, [str(args)])
        where the index points to the start of the next line.

        example: contents[start_index:] may look like:
        'p', nullptr, "<topic_name>");
        [...]
        """
        args = []
        next_position = start_index
        current_string = ''

        while next_position < len(contents):
            # skip whitespace
            while next_position < len(contents) and contents[next_position] in [' ', '\t', '\n']:
                next_position += 1

            if next_position >= len(contents):
                continue

            if contents[next_position] == '\"':
                next_position += 1
                string = ''
                string_start = next_position
                while next_position < len(contents):
                    if contents[next_position] == '\\': # escaping
                        if contents[next_position + 1] != '\n': # skip if continued on next line
                            string += contents[next_position:next_position+2].encode().decode('unicode_escape')
                        next_position += 2
                    elif contents[next_position] == '"':
                        next_position += 1
                        break
                    else:
                        string += contents[next_position]
                        next_position += 1
                # store the string, as it could continue in the form "a" "b"
                current_string += string

            elif contents.startswith('//', next_position): # comment
                next_position = contents.find('\n', next_position)
            elif contents.startswith('/*', next_position): # comment
                next_position = contents.find('*/', next_position) + 2
            else:
                if current_string != '':
                    args.append('"'+current_string+'"')
                    current_string = ''

                if contents.startswith('R\"', next_position): # C++11 raw string literal
                    bracket = contents.find('(', next_position)
                    identifier = contents[next_position+2:bracket]
                    raw_string_end = contents.find(')'+identifier+'"', next_position)
                    args.append('"'+contents[next_position+3+len(identifier):raw_string_end]+'"')
                    next_position = raw_string_end+len(identifier)+2
                elif contents[next_position] == ')':
                    break # finished
                elif contents[next_position] == ',':
                    next_position += 1 # skip
                elif contents[next_position] == '(':
                    raise Exception('parser error: unsupported "(" in function arguments')
                else:
                    # keyword (true, nullptr, ...), number or char (or variable).
                    # valid separators are: \n, ,, ), //, /*
                    next_arg_pos = contents.find(',', next_position)
                    m = re.search(r"\n|,|\)|//|/\*", contents[next_position:])
                    if m:
                        next_arg_pos = m.start() + next_position
                        args.append(contents[next_position:next_arg_pos].strip())
                    else:
                        raise Exception('parser error')
                    next_position = next_arg_pos
        #print(args)

        # find the next line
        next_position = contents.find('\n', next_position)
        if next_position >= 0: next_position += 1

        return next_position, args

    def HasValidationFailure(self):
        return self._consistency_checks_failure

    def GetModuleGroups(self):
        """
        Returns a dictionary of all categories with a dictonary of subcategories
        that contain a list of associated modules.
        """
        groups = {}
        for module_name in self._modules:
            module = self._modules[module_name]
            subcategory = module.subcategory()
            if module.category() in groups:
                if subcategory in groups[module.category()]:
                    groups[module.category()][subcategory].append(module)
                else:
                    groups[module.category()][subcategory] = [module]
            else:
                groups[module.category()] = {subcategory: [module]}

        # sort by module name
        for category in groups:
            group = groups[category]
            for subcategory in group:
                group[subcategory] = sorted(group[subcategory], key=lambda x: x.name())
        return groups
