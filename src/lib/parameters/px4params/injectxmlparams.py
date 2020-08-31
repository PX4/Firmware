#!/usr/bin/env python
# Injects params from XML file into a param_group
#

from __future__ import print_function
from px4params import srcscanner, srcparser, xmlout, markdownout
from .srcparser import ParameterGroup, Parameter

import xml.etree.ElementTree as ET
import sys


class XMLInject():
    def __init__(self, injected_xml_filename):
        self.groups=[]

        valid_parameter_attributes = set(["category", "default", "name", "type", "volatile"])
        valid_field_tags = set(["board","short_desc", "long_desc", "min", "max", "unit", "decimal", "increment", "reboot_required"])
        valid_other_top_level_tags = set(["group","values"])

        importtree = ET.parse(injected_xml_filename)
        injectgroups = importtree.getroot().findall("group")
        not_handled_parameter_tags=set()
        not_handled_parameter_attributes=set()
        for igroup in injectgroups:
            group_name=igroup.get('name')
            imported_group = ParameterGroup(group_name)
            no_code_generation=igroup.get('no_code_generation')
            if no_code_generation:
                imported_group.no_code_generation=no_code_generation
            for iparam in igroup:
                param_name=iparam.get('name')
                param_type=iparam.get('type')
                new_param= Parameter(param_name, param_type)
                #get other param info stored as attributes
                for param_attrib in iparam.attrib:
                    if param_attrib not in valid_parameter_attributes:
                        not_handled_parameter_attributes.add(param_attrib)
                    elif param_attrib == 'category':
                        new_param.SetCategory(iparam.get('category'))
                    elif param_attrib == 'default':
                        new_param.default = iparam.get('default')
                    elif param_attrib == 'volatile':
                        new_param.SetVolatile()


                #get param info stored as child tags
                for child in iparam:
                    if child.tag in valid_field_tags:
                        new_param.SetField(child.tag, child.text)
                    elif child.tag == 'values':
                        for value in child:
                            new_param.SetEnumValue(value.get('code'), value.text)
                    elif child.tag == 'bitmask':
                        for bit in child:
                            new_param.SetBitmaskBit(bit.get('index'), bit.text)
                    else:
                        not_handled_parameter_tags.add(child.tag)                      
            
                imported_group.AddParameter(new_param)
            self.groups.append(imported_group)

                    
        if not_handled_parameter_tags or not_handled_parameter_attributes:
            print("WARNING: Injected file parameter has unhandled child tags: %s" % not_handled_parameter_tags)
            print("WARNING: Injected file parameter has unhandled attributes: %s" % not_handled_parameter_attributes)

        

    def injected(self):
        return self.groups
