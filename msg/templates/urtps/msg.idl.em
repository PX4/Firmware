@###############################################
@#
@# ROS message to IDL converter
@#
@# EmPy template for generating <msg>.idl files
@#
@################################################################################
@#
@# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
@# Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
@#
@# Redistribution and use in source and binary forms, with or without
@# modification, are permitted provided that the following conditions are met:
@#
@# 1. Redistributions of source code must retain the above copyright notice, this
@# list of conditions and the following disclaimer.
@#
@# 2. Redistributions in binary form must reproduce the above copyright notice,
@# this list of conditions and the following disclaimer in the documentation
@# and/or other materials provided with the distribution.
@#
@# 3. Neither the name of the copyright holder nor the names of its contributors
@# may be used to endorse or promote products derived from this software without
@# specific prior written permission.
@#
@# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
@# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
@# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
@# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
@# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
@# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
@# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
@# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
@# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
@# POSSIBILITY OF SUCH DAMAGE.
@#
@################################################################################
@{
from packaging import version
import genmsg.msgs

from px_generate_uorb_topic_helper import *  # this is in Tools/

builtin_types = set()
array_types = set()

topic = alias if alias else spec.short_name
}@
@#################################################
@# Searching for serialize function per each field
@#################################################
@{

def get_include_directives(spec):
    include_directives = set()
    for field in (spec.parsed_fields()):
        if genmsg.msgs.is_valid_constant_type(genmsg.msgs.bare_msg_type(field.type)):
            continue
        builtin_type = str(field.base_type).replace('px4/', '')
        if version.parse(fastrtps_version) <= version.parse('1.7.2'):
            include_directive = '#include "%s_.idl"' % builtin_type
        else:
            include_directive = '#include "%s.idl"' % builtin_type
        builtin_types.add(builtin_type)
        include_directives.add(include_directive)
    return sorted(include_directives)


def get_idl_type_name(field_type):
    if field_type in type_idl_map:
        return type_idl_map[field_type]
    else:
        (package, name) = genmsg.names.package_resource_name(field_type)
        return name


def add_msg_field(field):
    if (not field.is_header):
        if field.is_array:
            if version.parse(fastrtps_version) <= version.parse('1.7.2'):
                print('    {0}__{1}_array_{2} {3}_;'.format(topic, str(get_idl_type_name(field.base_type)).replace(" ", "_"), str(field.array_len), field.name))
            else:
                print('    {0}__{1}_array_{2} {3};'.format(topic, str(get_idl_type_name(field.base_type)).replace(" ", "_"), str(field.array_len), field.name))
        else:
            if version.parse(fastrtps_version) <= version.parse('1.7.2'):
                base_type = get_idl_type_name(field.base_type) + "_" if get_idl_type_name(field.base_type) in builtin_types else get_idl_type_name(field.base_type)
            else:
                base_type = get_idl_type_name(field.base_type) if get_idl_type_name(field.base_type) in builtin_types else get_idl_type_name(field.base_type)
            print('    {0} {1}_;'.format(base_type, field.name))

def add_msg_fields():
    for field in spec.parsed_fields():
        add_msg_field(field)


def add_array_typedefs():
    for field in spec.parsed_fields():
        if not field.is_header and field.is_array:
            if version.parse(fastrtps_version) <= version.parse('1.7.2'):
                base_type = get_idl_type_name(field.base_type) + "_" if get_idl_type_name(field.base_type) in builtin_types else get_idl_type_name(field.base_type)
            else:
                base_type = get_idl_type_name(field.base_type) if get_idl_type_name(field.base_type) in builtin_types else get_idl_type_name(field.base_type)
            array_type = 'typedef {0} {1}__{2}_array_{3}[{4}];'.format(base_type, topic, get_idl_type_name(field.base_type).replace(" ", "_"), field.array_len, field.array_len)
            if array_type not in array_types:
                array_types.add(array_type)
    for atype in array_types:
        print(atype)


def add_msg_constants():
    sorted_constants = sorted(spec.constants,
                       key=sizeof_field_type, reverse=True)
    for constant in sorted_constants:
        print('const {0} {1}__{2} = {3};'.format(get_idl_type_name(constant.type), topic, constant.name, constant.val))

}
#ifndef __@(topic)__idl__
#define __@(topic)__idl__

@#############################
@# Include dependency messages
@#############################
@[for line in get_include_directives(spec)]@
@(line)@
@[end for]@


@# Constants
@add_msg_constants()
@# Array types
@add_array_typedefs()
@[if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
struct @(topic)_
@[else]@
struct @(topic)
@[end if]@
{
@add_msg_fields()
@[if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
}; // struct @(topic)_

#pragma keylist @(topic)_
@[else]@
}; // struct @(topic)

#pragma keylist @(topic)
@[end if]@

#endif  // __@(topic)__idl__
