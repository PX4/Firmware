#!/usr/bin/env python3
################################################################################
#
# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
# Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

# This script can generate the client and agent code based on a set of topics
# to sent and set to receive. It uses fastrtpsgen to generate the code from the
# IDL for the topic messages. The PX4 msg definitions are used to create the IDL
# used by fastrtpsgen using templates.

import sys
import os
import argparse
import shutil
import px_generate_uorb_topic_files
import px_generate_uorb_topic_helper
from uorb_rtps_classifier import Classifier
import subprocess
import glob
import errno
import re

try:
    from six.moves import input
except ImportError as e:
    print("Failed to import six: " + e)
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user six")
    print("")
    sys.exit(1)

try:
    from packaging import version
except ImportError as e:
    print("Failed to import packaging: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user packaging")
    print("")
    sys.exit(1)


def check_rtps_id_uniqueness(classifier):
    """
    Checks if there are no ID's for different msgs repeated on the map
    """

    repeated_ids = dict()

    full_send_list = dict(list(msg for msg in list(classifier.msgs_to_send.items(
    ))) + list(list(msg[0].items())[0] for msg in classifier.alias_msgs_to_send))
    full_receive_list = dict(list(msg for msg in list(classifier.msgs_to_receive.items(
    ))) + list(list(msg[0].items())[0] for msg in classifier.alias_msgs_to_receive))
    full_ignore_list = dict(list(msg for msg in list(classifier.msgs_to_ignore.items(
    ))) + list(list(msg[0].items())[0] for msg in classifier.alias_msgs_to_ignore))

    # check if there are repeated ID's on the messages to send
    for key, value in list(full_send_list.items()):
        if list(full_send_list.values()).count(value) > 1:
            repeated_ids.update({key: value})

    # check if there are repeated ID's on the messages to receive
    for key, value in list(full_receive_list.items()):
        if list(full_receive_list.values()).count(value) > 1:
            repeated_ids.update({key: value})

    # check if there are repeated ID's on the messages to ignore
    for key, value in list(full_ignore_list.items()):
        if list(full_ignore_list.values()).count(value) > 1:
            repeated_ids.update({key: value})

    # check if there are repeated IDs between classified and unclassified msgs
    # check send and ignore lists
    send_ignore_common_ids = list(set(full_ignore_list.values(
    )).intersection(list(full_send_list.values())))
    for item in list(full_send_list.items()):
        for repeated in send_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})
    for item in list(full_ignore_list.items()):
        for repeated in send_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})

    # check receive and ignore lists
    receive_ignore_common_ids = list(set(full_ignore_list.values(
    )).intersection(list(full_receive_list.values())))
    for item in list(full_receive_list.items()):
        for repeated in receive_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})
    for item in list(full_ignore_list.items()):
        for repeated in receive_ignore_common_ids:
            if item[1] == repeated:
                repeated_ids.update({item[0]: item[1]})

    all_msgs = {}
    all_msgs.update(full_send_list)
    all_msgs.update(full_receive_list)
    all_msgs.update(full_ignore_list)
    all_ids = list()
    all_ids = list(all_msgs.values())
    all_ids.sort()

    if not repeated_ids:
        print("All good. RTPS ID's are unique")
    else:
        raise AssertionError(", ".join('%s' % msgs for msgs in list(repeated_ids.keys())) +
                             " have their ID's repeated. Please choose from the following pool:\n" +
                             ", ".join('%d' % id for id in px_generate_uorb_topic_helper.check_available_ids(all_ids)))


default_client_out = "src/modules/micrortps_bridge/micrortps_client"
default_agent_out = "src/modules/micrortps_bridge/micrortps_agent"
default_uorb_templates_dir = "templates/uorb_microcdr"
default_urtps_templates_dir = "templates/urtps"
default_rtps_id_file = "tools/uorb_rtps_message_ids.yaml"
default_package_name = px_generate_uorb_topic_files.PACKAGE

parser = argparse.ArgumentParser()

parser.add_argument("-a", "--agent", dest='agent', action="store_true",
                    help="Flag for generate the agent, by default is true if -c is not specified")
parser.add_argument("-c", "--client", dest='client', action="store_true",
                    help="Flag for generate the client, by default is true if -a is not specified")
parser.add_argument("-i", "--generate-idl", dest='gen_idl',
                    action="store_true", help="Flag for generate idl files for each msg")
parser.add_argument("-j", "--idl-dir", dest='idl_dir',
                    type=str, help="IDL files dir", default='')
parser.add_argument("-m", "--mkdir-build", dest='mkdir_build',
                    action="store_true", help="Flag to create 'build' dir")
parser.add_argument("-l", "--generate-cmakelists", dest='cmakelists',
                    action="store_true", help="Flag to generate a CMakeLists.txt file for the micro-RTPS agent")
parser.add_argument("-t", "--topic-msg-dir", dest='msgdir', type=str,
                    help="Topics message, by default using relative path 'msg/'", default="msg")
parser.add_argument("-b", "--uorb-templates-dir", dest='uorb_templates', type=str,
                    help="uORB templates, by default using relative path to msgdir 'templates/uorb_microcdr'", default=default_uorb_templates_dir)
parser.add_argument("-q", "--urtps-templates-dir", dest='urtps_templates', type=str,
                    help="uRTPS templates, by default using relative path to msgdir 'templates/urtps'", default=default_urtps_templates_dir)
parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                    help="RTPS msg IDs definition path, by default using relative path to msgdir 'tools/uorb_rtps_message_ids.yaml'", default=default_rtps_id_file)
parser.add_argument("-p", "--package", dest='package', type=str,
                    help="Msg package naming, by default px4", default=default_package_name)
parser.add_argument("-o", "--agent-outdir", dest='agentdir', type=str,
                    help="Agent output dir, by default using relative path 'src/modules/micrortps_bridge/micrortps_agent'", default=default_agent_out)
parser.add_argument("-u", "--client-outdir", dest='clientdir', type=str,
                    help="Client output dir, by default using relative path 'src/modules/micrortps_bridge/micrortps_client'", default=default_client_out)
parser.add_argument("-f", "--fastrtpsgen-dir", dest='fastrtpsgen', type=str, nargs='?',
                    help="fastrtpsgen installation dir, only needed if fastrtpsgen is not in PATH, by default empty", default="")
parser.add_argument("-g", "--fastrtpsgen-include", dest='fastrtpsgen_include', type=str,
                    help="directory(ies) to add to preprocessor include paths of fastrtpsgen, by default empty", default="")
parser.add_argument("-r", "--ros2-distro", dest='ros2_distro', type=str, nargs='?',
                    help="ROS2 distro, only required if generating the agent for usage with ROS2 nodes, by default empty", default="")
parser.add_argument("--delete-tree", dest='del_tree',
                    action="store_true", help="Delete dir tree output dir(s)")


if len(sys.argv) <= 1:
    parser.print_usage()
    exit(-1)

# Parse arguments
args = parser.parse_args()
agent = args.agent
client = args.client
cmakelists = args.cmakelists
del_tree = args.del_tree
gen_idl = args.gen_idl
mkdir_build = args.mkdir_build
package = args.package

# Msg files path
msg_dir = os.path.abspath(args.msgdir)
px_generate_uorb_topic_files.append_to_include_path(
    {msg_dir}, px_generate_uorb_topic_files.INCL_DEFAULT, package)

# Agent files output path
agent_out_dir = os.path.abspath(args.agentdir)

# Client files output path
client_out_dir = os.path.abspath(args.clientdir)

# IDL files path
idl_dir = args.idl_dir
if idl_dir != '':
    idl_dir = os.path.abspath(args.idl_dir)
else:
    idl_dir = os.path.join(agent_out_dir, "idl")

if args.fastrtpsgen is None or args.fastrtpsgen == '':
    # Assume fastrtpsgen is in PATH
    fastrtpsgen_path = 'fastrtpsgen'
    for dirname in os.environ['PATH'].split(':'):
        candidate = os.path.join(dirname, 'fastrtpsgen')
        if os.path.isfile(candidate):
            fastrtpsgen_path = candidate
else:
    # Path to fastrtpsgen is explicitly specified
    if os.path.isdir(args.fastrtpsgen):
        fastrtpsgen_path = os.path.join(
            os.path.abspath(args.fastrtpsgen), 'fastrtpsgen')
    else:
        fastrtpsgen_path = args.fastrtpsgen

fastrtpsgen_include = args.fastrtpsgen_include
if fastrtpsgen_include is not None and fastrtpsgen_include != '':
    fastrtpsgen_include = "-I " + \
        os.path.abspath(
            args.fastrtpsgen_include) + " "

# get FastRTPSGen version
# .. note:: since Fast-RTPS 1.8.0 release, FastRTPSGen is a separated repository
# and not included in the Fast-RTPS project.
# The starting version since this separation is 1.0.0, which follows its own
# versioning
fastrtpsgen_version = version.Version("1.0.0")
if(os.path.exists(fastrtpsgen_path)):
    try:
        fastrtpsgen_version_out = subprocess.check_output(
            [fastrtpsgen_path, "-version"]).decode("utf-8").strip()[-5:]
    except OSError:
        raise

    try:
        fastrtpsgen_version = version.parse(fastrtpsgen_version_out)
    except version.InvalidVersion:
        raise Exception(
            "'fastrtpsgen -version' returned None or an invalid version")
else:
    raise Exception(
        "FastRTPSGen not found. Specify the location of fastrtpsgen with the -f flag")


# get ROS 2 version, if exists
ros2_distro = ''
ros_version = os.environ.get('ROS_VERSION')
if ros_version == '2':
    if args.ros2_distro != '':
        ros2_distro = args.ros2_distro
    else:
        ros2_distro = os.environ.get('ROS_DISTRO')

# get FastRTPS version
fastrtps_version = ''
if not ros2_distro:
    # grab the version installed system wise
    fastrtps_version = subprocess.check_output(
        "ldconfig -v 2>/dev/null | grep libfastrtps", shell=True).decode("utf-8").strip().split('so.')[-1]
else:
    # grab the version of the ros-<ros_distro>-fastrtps package
    fastrtps_version = re.search(r'Version:\s*([\dd.]+)', subprocess.check_output(
        "dpkg -s ros-" + ros2_distro + "-fastrtps 2>/dev/null | grep -i version", shell=True).decode("utf-8").strip()).group(1)


# If nothing specified it's generated both
if agent == False and client == False:
    agent = True
    client = True

if del_tree:
    if agent:
        _continue = str(input("\nFiles in " + agent_out_dir +
                              " will be erased, continue?[Y/n]\n"))
        if _continue == "N" or _continue == "n":
            print("Aborting execution...")
            exit(-1)
        else:
            if agent and os.path.isdir(agent_out_dir):
                shutil.rmtree(agent_out_dir)

    if client:
        _continue = str(input(
            "\nFiles in " + client_out_dir + " will be erased, continue?[Y/n]\n"))
        if _continue.strip() in ("N", "n"):
            print("Aborting execution...")
            exit(-1)
        else:
            if client and os.path.isdir(client_out_dir):
                shutil.rmtree(client_out_dir)

if agent and os.path.isdir(os.path.join(agent_out_dir, "idl")):
    shutil.rmtree(os.path.join(agent_out_dir, "idl"))

# uORB templates path
uorb_templates_dir = (args.uorb_templates if os.path.isabs(args.uorb_templates)
                      else os.path.join(msg_dir, args.uorb_templates))

# uRTPS templates path
urtps_templates_dir = (args.urtps_templates if os.path.isabs(args.urtps_templates)
                       else os.path.join(msg_dir, args.urtps_templates))

# parse yaml file into a map of ids
classifier = (Classifier(os.path.abspath(args.yaml_file), msg_dir) if os.path.isabs(args.yaml_file)
              else Classifier(os.path.join(msg_dir, args.yaml_file), msg_dir))

# check if there are no ID's repeated
check_rtps_id_uniqueness(classifier)


uRTPS_CLIENT_TEMPL_FILE = 'microRTPS_client.cpp.em'
uRTPS_AGENT_TOPICS_H_TEMPL_FILE = 'RtpsTopics.h.em'
uRTPS_AGENT_TOPICS_SRC_TEMPL_FILE = 'RtpsTopics.cpp.em'
uRTPS_AGENT_TEMPL_FILE = 'microRTPS_agent.cpp.em'
uRTPS_TIMESYNC_CPP_TEMPL_FILE = 'microRTPS_timesync.cpp.em'
uRTPS_TIMESYNC_H_TEMPL_FILE = 'microRTPS_timesync.h.em'
uRTPS_AGENT_CMAKELISTS_TEMPL_FILE = 'microRTPS_agent_CMakeLists.txt.em'
uRTPS_PUBLISHER_SRC_TEMPL_FILE = 'Publisher.cpp.em'
uRTPS_PUBLISHER_H_TEMPL_FILE = 'Publisher.h.em'
uRTPS_SUBSCRIBER_SRC_TEMPL_FILE = 'Subscriber.cpp.em'
uRTPS_SUBSCRIBER_H_TEMPL_FILE = 'Subscriber.h.em'


def generate_agent(out_dir):
    global fastrtps_version

    if classifier.msgs_to_send:
        for msg_file in classifier.msgs_to_send:
            if gen_idl:
                if out_dir != agent_out_dir:
                    px_generate_uorb_topic_files.generate_idl_file(msg_file, msg_dir, "", os.path.join(out_dir, "/idl"), urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
                else:
                    px_generate_uorb_topic_files.generate_idl_file(msg_file, msg_dir, "", idl_dir, urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_H_TEMPL_FILE)

    if classifier.alias_msgs_to_send:
        for msg_file in classifier.alias_msgs_to_send:
            msg_alias = list(msg_file[0].keys())[0]
            msg_name = msg_file[1]
            if gen_idl:
                if out_dir != agent_out_dir:
                    px_generate_uorb_topic_files.generate_idl_file(msg_name, msg_dir, msg_alias, os.path.join(out_dir, "/idl"), urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
                else:
                    px_generate_uorb_topic_files.generate_idl_file(msg_name, msg_dir, msg_alias, idl_dir, urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_PUBLISHER_H_TEMPL_FILE)

    if classifier.msgs_to_receive:
        for msg_file in classifier.msgs_to_receive:
            if gen_idl:
                if out_dir != agent_out_dir:
                    px_generate_uorb_topic_files.generate_idl_file(msg_file, msg_dir, "", os.path.join(out_dir, "/idl"), urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
                else:
                    px_generate_uorb_topic_files.generate_idl_file(msg_file, msg_dir, "", idl_dir, urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, msg_dir, "", out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_H_TEMPL_FILE)

    if classifier.alias_msgs_to_receive:
        for msg_file in classifier.alias_msgs_to_receive:
            msg_alias = list(msg_file[0].keys())[0]
            msg_name = msg_file[1]
            if gen_idl:
                if out_dir != agent_out_dir:
                    px_generate_uorb_topic_files.generate_idl_file(msg_name, msg_dir, msg_alias, os.path.join(out_dir, "/idl"), urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
                else:
                    px_generate_uorb_topic_files.generate_idl_file(msg_name, msg_dir, msg_alias, idl_dir, urtps_templates_dir,
                                                                   package, px_generate_uorb_topic_files.INCL_DEFAULT, fastrtps_version, ros2_distro, classifier.msg_id_map)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_name, msg_dir, msg_alias, out_dir, urtps_templates_dir,
                                                             package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_SUBSCRIBER_H_TEMPL_FILE)

    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, out_dir,
                                                        urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_AGENT_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, out_dir,
                                                        urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_TIMESYNC_CPP_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, out_dir,
                                                        urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_TIMESYNC_H_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, out_dir,
                                                        urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_AGENT_TOPICS_H_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, out_dir,
                                                        urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_AGENT_TOPICS_SRC_TEMPL_FILE)
    if cmakelists:
        px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir, os.path.dirname(out_dir),
                                                            urtps_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_AGENT_CMAKELISTS_TEMPL_FILE)

    # Final steps to install agent
    mkdir_p(os.path.join(out_dir, "fastrtpsgen"))
    prev_cwd_path = os.getcwd()
    os.chdir(os.path.join(out_dir, "fastrtpsgen"))
    if not glob.glob(os.path.join(idl_dir, "*.idl")):
        raise Exception("No IDL files found in %s" % idl_dir)

    # If it is generating the bridge code for interfacing with ROS2, then set
    # the '-typeros2' option in fastrtpsgen.
    # .. note:: This is only available in FastRTPSGen 1.0.4 and above
    gen_ros2_typename = ""
    if ros2_distro and ros2_distro in ['dashing', 'eloquent', 'foxy'] and fastrtpsgen_version >= version.Version("1.0.4"):
        gen_ros2_typename = "-typeros2 "

    for idl_file in glob.glob(os.path.join(idl_dir, "*.idl")):
        try:
            ret = subprocess.check_call(fastrtpsgen_path + " -d " + out_dir +
                                        "/fastrtpsgen -example x64Linux2.6gcc " + gen_ros2_typename + fastrtpsgen_include + idl_file, shell=True)
        except OSError:
            raise

    rm_wildcard(os.path.join(out_dir, "fastrtpsgen/*PubSubMain*"))
    rm_wildcard(os.path.join(out_dir, "fastrtpsgen/makefile*"))
    rm_wildcard(os.path.join(out_dir, "fastrtpsgen/*Publisher*"))
    rm_wildcard(os.path.join(out_dir, "fastrtpsgen/*Subscriber*"))
    for f in glob.glob(os.path.join(out_dir, "fastrtpsgen/*.cxx")):
        os.rename(f, f.replace(".cxx", ".cpp"))
    cp_wildcard(os.path.join(out_dir, "fastrtpsgen/*"), out_dir)
    if os.path.isdir(os.path.join(out_dir, "fastrtpsgen")):
        shutil.rmtree(os.path.join(out_dir, "fastrtpsgen"))
    cp_wildcard(os.path.join(urtps_templates_dir,
                             "microRTPS_transport.*"), agent_out_dir)
    if cmakelists:
        os.rename(os.path.join(os.path.dirname(out_dir), "microRTPS_agent_CMakeLists.txt"),
                  os.path.join(os.path.dirname(out_dir), "CMakeLists.txt"))
    if (mkdir_build):
        mkdir_p(os.path.join(os.path.dirname(out_dir), "build"))
    os.chdir(prev_cwd_path)
    return 0


def rm_wildcard(pattern):
    for f in glob.glob(pattern):
        os.remove(f)


def cp_wildcard(pattern, destdir):
    for f in glob.glob(pattern):
        shutil.copy(f, destdir)


def mkdir_p(dirpath):
    try:
        os.makedirs(dirpath)
    except OSError as e:
        if e.errno == errno.EEXIST and os.path.isdir(dirpath):
            pass
        else:
            raise


def generate_client(out_dir):
    global fastrtps_version

    # Rename work in the default path
    if default_client_out != out_dir:
        def_file = os.path.join(default_client_out, "microRTPS_client.cpp")
        if os.path.isfile(def_file):
            os.rename(def_file, def_file.replace(".cpp", ".cpp_"))
        def_file = os.path.join(default_client_out, "microRTPS_transport.cpp")
        if os.path.isfile(def_file):
            os.rename(def_file, def_file.replace(".cpp", ".cpp_"))
        def_file = os.path.join(default_client_out, "microRTPS_transport.h")
        if os.path.isfile(def_file):
            os.rename(def_file, def_file.replace(".h", ".h_"))

    px_generate_uorb_topic_files.generate_uRTPS_general(classifier.msgs_to_send, classifier.alias_msgs_to_send, classifier.msgs_to_receive, classifier.alias_msgs_to_receive, msg_dir,
                                                        out_dir, uorb_templates_dir, package, px_generate_uorb_topic_files.INCL_DEFAULT, classifier.msg_id_map, fastrtps_version, ros2_distro, uRTPS_CLIENT_TEMPL_FILE)

    # Final steps to install client
    cp_wildcard(os.path.join(urtps_templates_dir,
                             "microRTPS_transport.*"), out_dir)

    return 0


if agent:
    generate_agent(agent_out_dir)
    print(("\nAgent created in: " + agent_out_dir))

if client:
    generate_client(client_out_dir)
    print(("\nClient created in: " + client_out_dir))
