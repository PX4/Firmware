#!/usr/bin/env python3
################################################################################
#
# Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

import sys
import os
import argparse
import errno
import yaml
import re
import difflib


class Classifier():
    """
    Class to classify RTPS msgs as senders, receivers or to be ignored
    """

    def __init__(self, yaml_file, msg_folder):
        self.msg_folder = msg_folder
        self.all_msgs_list = self.set_all_msgs()
        self.msg_id_map = self.parse_yaml_msg_id_file(yaml_file)
        self.alias_space_init_id = 170

        # Checkers
        self.check_if_listed(yaml_file)
        self.check_base_type()
        self.check_id_space()

        self.msgs_to_send, self.alias_msgs_to_send = self.set_msgs_to_send()
        self.msgs_to_receive, self.alias_msgs_to_receive = self.set_msgs_to_receive()
        self.msgs_to_ignore, self.alias_msgs_to_ignore = self.set_msgs_to_ignore()
        self.msg_files_send = self.set_msg_files_send()
        self.msg_files_receive = self.set_msg_files_receive()
        self.msg_files_ignore = self.set_msg_files_ignore()

    # setters (for class init)
    def set_all_msgs(self):
        msg_list = []
        for filename in os.listdir(self.msg_folder):
            if '.msg' in filename:
                # add base messages
                msg_list.append(re.sub(".msg", "", filename))

                # add alias / multi-topics messages
                with open(os.path.join(self.msg_folder, filename), 'r') as f:
                    alias_msgs = []
                    lines = f.readlines()
                    for line in lines:
                        if '# TOPICS' in line:
                            fileUpdated = True
                            alias_msgs += line.split()
                            alias_msgs.remove('#')
                            alias_msgs.remove('TOPICS')
                            for msg in alias_msgs:
                                if msg not in msg_list:
                                    msg_list.append(msg)
        return msg_list

    def set_msgs_to_send(self):
        send = {}
        send_alias = []
        for dict in self.msg_id_map['rtps']:
            if 'send' in list(dict.keys()):
                if 'alias' in list(dict.keys()):
                    send_alias.append(
                        ({dict['msg']: dict['id']}, dict['alias']))
                else:
                    send.update({dict['msg']: dict['id']})
        return send, send_alias

    def set_msgs_to_receive(self):
        receive = {}
        receive_alias = []
        for dict in self.msg_id_map['rtps']:
            if 'receive' in list(dict.keys()):
                if 'alias' in list(dict.keys()):
                    receive_alias.append(
                        ({dict['msg']: dict['id']}, dict['alias']))
                else:
                    receive.update({dict['msg']: dict['id']})
        return receive, receive_alias

    def set_msgs_to_ignore(self):
        ignore = {}
        ignore_alias = []
        for dict in self.msg_id_map['rtps']:
            if (('send' not in list(dict.keys())) and ('receive' not in list(dict.keys()))):
                if 'alias' in list(dict.keys()):
                    ignore_alias.append(
                        ({dict['msg']: dict['id']}, dict['alias']))
                else:
                    ignore.update({dict['msg']: dict['id']})
        return ignore, ignore_alias

    def set_msg_files_send(self):
        return [os.path.join(self.msg_folder, msg + ".msg")
                for msg in list(self.msgs_to_send.keys())]

    def set_msg_files_receive(self):
        return [os.path.join(self.msg_folder, msg + ".msg")
                for msg in list(self.msgs_to_receive.keys())]

    def set_msg_files_ignore(self):
        return [os.path.join(self.msg_folder, msg + ".msg")
                for msg in list(self.msgs_to_ignore.keys())]

    def check_if_listed(self, yaml_file):
        """
        Checks if all msgs are listed under the RTPS ID yaml file
        """
        none_listed_msgs = []
        for msg in self.all_msgs_list:
            result = not any(
                dict['msg'] == msg for dict in self.msg_id_map['rtps'])
            if result:
                none_listed_msgs.append(msg)

        if len(none_listed_msgs) > 0:
            if len(none_listed_msgs) == 1:
                error_msg = "The following message is not listen under "
            elif len(none_listed_msgs) > 1:
                error_msg = "The following messages are not listen under "

            raise AssertionError(
                "\n%s %s: " % (error_msg, yaml_file) + ", ".join('%s' % msg for msg in none_listed_msgs) +
                "\n\nPlease add them to the yaml file with the respective ID and, if applicable, mark them " +
                "to be sent or received by the micro-RTPS bridge.\n"
                "NOTE: If the message is an alias / part of multi-topics (#TOPICS) of a base message, it should be added as well.\n")

    def check_base_type(self):
        """
        Check if alias message has correct base type
        """
        registered_alias_msgs = list(
            dict['alias'] for dict in self.msg_id_map['rtps'] if 'alias' in list(dict.keys()))

        base_types = []
        for dict in self.msg_id_map['rtps']:
            if 'alias' not in list(dict.keys()):
                base_types.append(dict['msg'])

        incorrect_base_types = list(
            set(registered_alias_msgs) - set(base_types))

        base_types_suggestion = {}
        for incorrect in incorrect_base_types:
            base_types_suggestion.update({incorrect: difflib.get_close_matches(
                incorrect, base_types, n=1, cutoff=0.6)})

        if len(base_types_suggestion) > 0:
            raise AssertionError(
                ('\n' + '\n'.join('\t- The multi-topic message base type \'{}\' does not exist.{}'.format(k, (' Did you mean \'' + v[0] + '\'?' if v else '')) for k, v in list(base_types_suggestion.items()))))

    def check_id_space(self):
        """
        Check if msg ID is in the correct ID space
        """
        incorrect_base_ids = {}
        incorrect_alias_ids = {}
        for dict in self.msg_id_map['rtps']:
            if 'alias' not in list(dict.keys()) and dict['id'] >= self.alias_space_init_id:
                incorrect_base_ids.update({dict['msg']: dict['id']})
            elif 'alias' in list(dict.keys()) and dict['id'] < self.alias_space_init_id:
                incorrect_alias_ids.update({dict['msg']: dict['id']})

        if len(incorrect_base_ids) > 0:
            raise AssertionError(
                ('\n' + '\n'.join('\t- The message \'{} with ID \'{}\' is in the wrong ID space. Please use any of the available IDs from 0 to 169'.format(k, v) for k, v in list(incorrect_base_ids.items()))))

        if len(incorrect_alias_ids) > 0:
            raise AssertionError(
                ('\n' + '\n'.join('\t- The alias message \'{}\' with ID \'{}\' is in the wrong ID space. Please use any of the available IDs from 170 to 255'.format(k, v) for k, v in list(incorrect_alias_ids.items()))))

    @staticmethod
    def parse_yaml_msg_id_file(yaml_file):
        """
        Parses a yaml file into a dict
        """
        try:
            with open(yaml_file, 'r') as f:
                return yaml.safe_load(f)
        except OSError as e:
            if e.errno == errno.ENOENT:
                raise IOError(errno.ENOENT, os.strerror(
                    errno.ENOENT), yaml_file)
            else:
                raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-s", "--send", dest='send',
                        action="store_true", help="Get topics to be sent")
    parser.add_argument("-a", "--alias", dest='alias',
                        action="store_true", help="Get alias topics")
    parser.add_argument("-r", "--receive", dest='receive',
                        action="store_true", help="Get topics to be received")
    parser.add_argument("-i", "--ignore", dest='ignore',
                        action="store_true", help="Get topics to be ignored")
    parser.add_argument("-p", "--path", dest='path',
                        action="store_true", help="Get msgs and its paths")
    parser.add_argument("-m", "--topic-msg-dir", dest='msgdir', type=str,
                        help="Topics message dir, by default msg/", default="msg")
    parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                        help="RTPS msg IDs definition file absolute path, by default use relative path to msg, tools/uorb_rtps_message_ids.yaml",
                        default='tools/uorb_rtps_message_ids.yaml')

    # Parse arguments
    args = parser.parse_args()

    msg_dir = args.msgdir
    if args.msgdir == 'msg':
        msg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    else:
        msg_dir = os.path.abspath(args.msgdir)
    classifier = (Classifier(os.path.abspath(args.yaml_file), msg_dir) if os.path.isabs(args.yaml_file)
                  else Classifier(os.path.join(msg_dir, args.yaml_file), msg_dir))

    if args.send:
        if args.path:
            print(('send files: ' + ', '.join(str(msg_file)
                                              for msg_file in classifier.msgs_files_send) + '\n'))
        else:
            if args.alias:
                print((', '.join(str(msg)
                                 for msg in list(classifier.msgs_to_send.keys())) + (' alias ' + ', '.join(str(list(msg[0].keys())[0])
                                                                                                         for msg in classifier.alias_msgs_to_send) if len(classifier.alias_msgs_to_send) > 0 else '') + '\n'))
            else:
                print((', '.join(str(msg)
                                 for msg in list(classifier.msgs_to_send.keys()))))
    if args.receive:
        if args.path:
            print(('receive files: ' + ', '.join(str(msg_file)
                                                 for msg_file in classifier.msgs_files_receive) + '\n'))
        else:
            if args.alias:
                print((', '.join(str(msg)
                                 for msg in list(classifier.msgs_to_receive.keys())) + (' alias ' + ', '.join(str(list(msg[0].keys())[0])
                                                                                                            for msg in classifier.alias_msgs_to_receive) if len(classifier.alias_msgs_to_receive) > 0 else '') + '\n'))
            else:
                print((', '.join(str(msg)
                                 for msg in list(classifier.msgs_to_receive.keys()))))
    if args.ignore:
        if args.path:
            print(('ignore files: ' + ', '.join(str(msg_file)
                                                for msg_file in classifier.msgs_files_ignore) + '\n'))
        else:
            if args.alias:
                print((', '.join(str(msg)
                                 for msg in list(classifier.msgs_to_ignore.keys())) + (' alias ' + ', '.join(str(list(msg[0].keys())[0])
                                                                                                           for msg in classifier.alias_msgs_to_ignore) if len(classifier.alias_msgs_to_ignore) > 0 else '') + '\n'))
            else:
                print((', '.join(str(msg)
                                 for msg in list(classifier.msgs_to_ignore.keys()))))
