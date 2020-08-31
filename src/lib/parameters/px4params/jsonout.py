from xml.sax.saxutils import escape
import codecs
import json
import sys


class JsonOutput():
    def __init__(self, groups, board, inject_xml_file_name):
        all_json=dict()
        all_json['version']=1
        all_json['uid']=1  #COMPONENT_INFORMATION.comp_metadata_type
        all_json['scope']="Firmware"
        all_params=[]
        all_json['parameters']=all_params

        #xml_parameters = ET.Element("parameters")
        #xml_version = ET.SubElement(xml_parameters, "version")
        #xml_version.text = "3"
        #xml_version = ET.SubElement(xml_parameters, "parameter_version_major")
        #xml_version.text = "1"
        #xml_version = ET.SubElement(xml_parameters, "parameter_version_minor")
        #xml_version.text = "15"

        schema_map = {
                        "short_desc": "shortDesc",
			"long_desc": "longDesc",
			"unit": "units",
			"decimal": "decimalPlaces",
			"min": "min",
			"max": "max",
			"increment": "increment",
			"reboot_required": "rebootRequired"
			}
        allowed_types = { "Uint8", "Int8", "Uint16", "Int16", "Uint32", "Int32", "Float"}

        last_param_name = ""
        board_specific_param_set = False
        for group in groups:
            group_name=group.GetName()

            for param in group.GetParams():
                if (last_param_name == param.GetName() and not board_specific_param_set) or last_param_name != param.GetName():
                    curr_param=dict()
                    curr_param['name'] = param.GetName()
                    curr_param['default'] = param.GetDefault()
                    curr_param['type'] = param.GetType().capitalize()
                    if not curr_param['type'] in allowed_types:
                        print("Error: %s type not supported: curr_param['type']" % (curr_param['name'],curr_param['type']) )
                        sys.Exit(1)

                    curr_param['group'] = group_name
                    if (param.GetCategory()):
                        curr_param['category'] = param.GetCategory()

                    if (param.GetVolatile() == "true"):
                        curr_param['volatile'] = param.GetVolatile()

                    last_param_name = param.GetName()
                    for code in param.GetFieldCodes():
                        value = param.GetFieldValue(code)
                        if code == "board":
                            if value == board:
                                board_specific_param_set = True
                                # JSON schema has no field for board_specific schema. Ignore.
                            else:
                                #xml_group.remove(xml_param)
                                continue
                        else:
                            #map PX4 param field names to schema names
                           if code in schema_map:
                               curr_param[schema_map[code]] = value
                           else:
                               print('ERROR: Field not in json schema: %s' % code)
                               sys.exit(1)


                if last_param_name != param.GetName():
                    board_specific_param_set = False

                enum_codes=param.GetEnumCodes() or '' # Gets numerical values for parameter.
                if enum_codes:
                    enum_codes=sorted(enum_codes,key=float)
                    codes_list=list()
                    for item in enum_codes:
                        code_dict=dict()
                        code_dict['value']=item
                        code_dict['description']=param.GetEnumValue(item)
                        codes_list.append(code_dict)
                    curr_param['values'] = codes_list


                if len(param.GetBitmaskList()) > 0:
                    bitmasks_list=list()
                    for index in param.GetBitmaskList():
                        bitmask_dict=dict()
                        bitmask_dict['index']=index
                        bitmask_dict['description']=param.GetBitmaskBit(index)
                        bitmasks_list.append(bitmask_dict)
                    curr_param['bitmask'] = bitmasks_list


                all_params.append(curr_param)


        #Json string output.
        self.output = json.dumps(all_json,indent=2)


    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)

