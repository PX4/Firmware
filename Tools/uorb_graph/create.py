#! /usr/bin/env python3

from __future__ import print_function

import argparse
import os
import codecs
import re
import colorsys
import json
import sys


parser = argparse.ArgumentParser(
    description='Generate uORB pub/sub dependency graph from source code')

parser.add_argument('-s', '--src-path', action='append',
                    help='Source path(s) (default=src, can be specified multiple times)',
                    default=[])
parser.add_argument('-e', '--exclude-path', action='append',
                    help='Excluded path(s), can be specified multiple times',
                    default=[])
parser.add_argument('-f', '--file', metavar='file', action='store',
                    help='output file name prefix',
                    default='graph')
parser.add_argument('-o', '--output', metavar='output', action='store',
                    help='output format (json or graphviz)',
                    default='json')
parser.add_argument('--use-topic-union', action='store_true',
                    help='''
Use the union of all publication and subscription topics (useful for complete
graphs or only few/single module(s)). The default is to use the intersection
(remove topics that have no subscriber or no publisher)''')
parser.add_argument('-m', '--modules', action='store',
                    help='Comma-separated whitelist of modules (the module\'s '+
                    'MAIN, e.g. from a startup script)',
                    default='')

args = parser.parse_args()


g_debug = False
def dbg_print(string):
    if g_debug:
        print(string)


def get_N_colors(N, s=0.8, v=0.9):
    """ get N distinct colors as a list of hex strings """
    HSV_tuples = [(x*1.0/N, s, v) for x in range(N)]
    hex_out = []
    for rgb in HSV_tuples:
        rgb = map(lambda x: int(x*255), colorsys.hsv_to_rgb(*rgb))
        hex_out.append("#"+"".join(map(lambda x: format(x, '02x'), rgb)))
    return hex_out


class PubSub(object):
    """ Collects either publication or subscription information for nodes
    (modules and topics) & edges """

    def __init__(self, is_publication, topic_blacklist, orb_pub_sub_regexes, special_cases):
        """
            :param is_publication: if True, publications, False for
            subscriptions
            :param topic_blacklist: list of topics to blacklist
            :param orb_pub_sub_regexes: list of regexes to extract orb calls
                (e.g. orb_subscribe). They need to have 2 captures, the second
                one is the one capturing ORB_ID(<topic>
        """
        self._module_pubsubs = {} # key = module name, value = set of topic names
        self._special_cases = special_cases
        self._special_cases_matched = None
        self._topic_blacklist = topic_blacklist
        self._orb_pub_sub_regexes = orb_pub_sub_regexes
        if is_publication:
            self._method = 'Publication'
        else:
            self._method = 'Subscription'

    def reset(self):
        self._special_cases_matched = [False]*len(self._special_cases)

    def filter_modules(self, module_whitelist):
        remove = [k for k in self._module_pubsubs if k not in module_whitelist]
        for k in remove: del self._module_pubsubs[k]


    def check_if_match_found(self, modules):
        """ check if all special cases got a match (if not, it means the source
            code got changed)
        """
        for i, (module_match, file_match_re, src_match_re, _) in enumerate(self._special_cases):
            if module_match in modules and src_match_re is not None:
                if not self._special_cases_matched[i]:
                    raise Exception('Module '+module_match+
                            ': no match for '+self._method+' special case'+
                            src_match_re.pattern+'. The case needs to be updated')


    def extract(self, file_name, src_str, module, orb_id_vehicle_attitude_controls_topic):
        """ Extract subscribed/published topics from a source string
        :param src_str: string of C/C++ code with comments and whitespace removed
        """

        orb_pubsub_matches = []
        for regex in self._orb_pub_sub_regexes:
            orb_pubsub_matches += re.findall(regex, src_str)

        orb_id = 'ORB_ID('

        for _, match in orb_pubsub_matches:
            if match == 'ORB_ID_VEHICLE_ATTITUDE_CONTROLS': # special case
                match = orb_id+orb_id_vehicle_attitude_controls_topic

            # match has the form: '[ORB_ID(]<topic_name>'
            if match.startswith(orb_id):
                topic_name = match[len(orb_id):]
                self._add_topic(topic_name, file_name, module)
            else:
                ignore_found = False
                for module_match, file_match_re, _, ignore_re in self._special_cases:
                    if module == module_match:
                        if file_match_re.search(file_name):
                            if ignore_re.search(match):
                                ignore_found = True
                if not ignore_found:
                    # If we land here, we need to add another special case
                    raise Exception(self._method+' w/o ORB_ID(): '+match+' in '
                            +file_name+' ('+module+'). You need to add another special case.')

        # handle special cases
        for i, (module_match, file_match_re, src_match_re, _) in enumerate(self._special_cases):
            if src_match_re is None:
                continue
            if module == module_match:
                if file_match_re.search(file_name):
                    matches = src_match_re.findall(src_str)
                    for match in matches:
                        # match has the form: '[ORB_ID(]<topic_name>'
                        if match.startswith(orb_id):
                            topic_name = match[len(orb_id):]
                            dbg_print('Found '+self._method+' for special case in '
                                      +module+': '+topic_name)
                            self._add_topic(topic_name, file_name, module)
                            self._special_cases_matched[i] = True
                        else:
                            # this is not fatal, as it could be a method delaration/definition
                            dbg_print('Special case '+self._method+' w/o ORB_ID(): '
                                      +match+' in '+file_name+' ('+module+')')

    def _add_topic(self, topic_name, file_name, module):
        """ add a subscription/publication for a module
        """
        if topic_name in self._topic_blacklist:
            dbg_print('ignoring blacklisted topic '+topic_name)
            return

        if module is None:
            if not file_name.endswith('hott/messages.cpp'): # hott has a special module structure. just ignore it
                print('Warning: found '+self._method+' without associated module: '
                      +topic_name+' in '+file_name)
            return

        if not module in self._module_pubsubs:
            self._module_pubsubs[module] = set()
        self._module_pubsubs[module].add(topic_name)


    def get_topics(self, modules):
        """ get the set of topics
            :param modules: list of modules to take into account
        """
        topics = set()
        for module in modules:
            if module in self._module_pubsubs:
                topics |= self._module_pubsubs[module]
        return topics

    @property
    def pubsubs(self):
        """ get dict of all publication/subscriptions (key=modules, value=set of
            topic names"""
        return self._module_pubsubs


class Graph(object):
    """ Collects Node and Edge information by parsing the source tree """
    def __init__(self, module_whitelist=[], topic_blacklist=[]):
        self._current_module = [] # stack with current module (they can be nested)
        self._all_modules = set() # set of all found modules

        self._comment_remove_pattern = re.compile(
            r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
            re.DOTALL | re.MULTILINE)
        self._whitespace_pattern = re.compile(r'\s+')
        self._module_whitelist = module_whitelist
        self._excluded_paths = []

        self._orb_id_vehicle_attitude_controls_topic = 'actuator_controls_0'
        self._orb_id_vehicle_attitude_controls_re = \
            re.compile(r'\#define\s+ORB_ID_VEHICLE_ATTITUDE_CONTROLS\s+([^,)]+)')

        self._module_subscriptions = {} # key = module name, value = set of topic names
        self._module_publications = {} # key = module name, value = set of topic names

        self._modules = set() # all modules
        self._topics = set() # all topics
        self._topic_colors = {} # key = topic, value = color (html string)

    # handle special cases
    # format: list of tuples with 4 entries:
    # - module name to match (module MAIN)
    # - regex for file name(s) to match within the module (matched against the full path)
    # - regex to extract the topic name: the match must be ORB_ID(<topic_name>
    #   Note: whitespace is removed from source code, so it does not need to be
    #   accounted for in the regex.
    #   If this is None, it will just be ignored
    # - regex to ignore matches in the form orb_[subscribe|advertise](<match>
    #   (the expectation is that the previous matching ORB_ID() will be passed
    #   to this, so that we can ignore it)
        special_cases_sub = [
    ('listener', r'.*', None, r'^(id)$'),
    ('logger', r'.*', None, r'^(topic|sub\.metadata|_polling_topic_meta)$'),

    ('tap_esc', r'.*', r'\b_control_topics\[[0-9]\]=([^,)]+)', r'^_control_topics\[i\]$'),
    ('snapdragon_pwm_out', r'.*', r'\b_controls_topics\[[0-9]\]=([^,)]+)', r'^_controls_topics\[i\]$'),
    ('linux_pwm_out', r'.*', r'\b_controls_topics\[[0-9]\]=([^,)]+)', r'^_controls_topics\[i\]$'),
    ]
        special_cases_sub = [(a, re.compile(b), re.compile(c) if c is not None else None, re.compile(d))
                                   for a,b,c,d in special_cases_sub]

        self._subscriptions = PubSub(False, topic_blacklist,
                [r"\borb_subscribe(_multi|)\b\(([^,)]+)"],
                special_cases_sub)


        special_cases_pub = [
    ('replay', r'Replay\.cpp$', None, r'^sub\.orb_meta$'),

    ('uavcan', r'sensors/.*\.cpp$', r'\bUavcanCDevSensorBridgeBase\([^{]*DEVICE_PATH,([^,)]+)', r'^_orb_topic$'),
    ]
        special_cases_pub = [(a, re.compile(b), re.compile(c) if c is not None else None, re.compile(d))
                                   for a,b,c,d in special_cases_pub]
        self._publications = PubSub(True, topic_blacklist,
                [r"\borb_advertise(_multi|_queue|_multi_queue|)\b\(([^,)]+)",
                 r"\borb_publish_auto()\b\(([^,)]+)"],
                special_cases_pub)


    def _get_current_module(self):
        if len(self._current_module) == 0:
            return None
        return self._current_module[-1]

    def build(self, src_path_list, excluded_paths=[], use_topic_pubsub_union=True):
        """ parse the source tree & extract pub/sub information.
            :param use_topic_pubsub_union: if true, use all topics that have a
            publisher or subscriber. If false, use only topics with at least one
            publisher and subscriber.

            fill in self._module_subsciptions & self._module_publications
        """

        self._subscriptions.reset()
        self._publications.reset()

        self._excluded_paths = [os.path.normpath(p) for p in excluded_paths]
        for path in src_path_list:
            self._build_recursive(path)

        # filter by whitelist
        if len(self._module_whitelist) > 0:
            self._subscriptions.filter_modules(self._module_whitelist)
            self._publications.filter_modules(self._module_whitelist)

        # modules & topics sets
        self._modules = set(list(self._publications.pubsubs.keys()) +
                list(self._subscriptions.pubsubs.keys()))
        print('number of modules: '+str(len(self._modules)))
        self._topics = self._get_topics(use_topic_pubsub_union=use_topic_pubsub_union)
        print('number of topics: '+str(len(self._topics)))

        # initialize colors
        color_list = get_N_colors(len(self._topics), 0.7, 0.85)
        self._topic_colors = {}
        for i, topic in enumerate(self._topics):
            self._topic_colors[topic] = color_list[i]


        # validate that all special rules got used
        self._subscriptions.check_if_match_found(self._all_modules)
        self._publications.check_if_match_found(self._all_modules)


    def _get_topics(self, use_topic_pubsub_union=True):
        """ get the set of topics
        """
        subscribed_topics = self._subscriptions.get_topics(self._modules)
        published_topics = self._publications.get_topics(self._modules)
        if use_topic_pubsub_union:
            return subscribed_topics | published_topics
        return subscribed_topics & published_topics

    def _build_recursive(self, path):

        if os.path.normpath(path) in self._excluded_paths:
            dbg_print('ignoring excluded path '+path)
            return

        entries = os.listdir(path)

        # check if entering a new module
        cmake_file = 'CMakeLists.txt'
        new_module = False
        if cmake_file in entries:
            new_module = self._extract_module_name(os.path.join(path, cmake_file))

        # iterate directories recursively
        for entry in entries:
            file_name = os.path.join(path, entry)
            if os.path.isdir(file_name):
                self._build_recursive(file_name)


        # iterate source files
        # Note: we could skip the entries if we're not in a module, but we don't
        # so that we get appropriate error messages to know where we miss subs
        # or pubs
        for entry in entries:
            file_name = os.path.join(path, entry)
            if os.path.isfile(file_name):
                _, ext = os.path.splitext(file_name)
                if ext in ['.cpp', '.c', '.h', '.hpp']:
                    self._process_source_file(file_name)


        if new_module:
            self._current_module.pop()


    def _extract_module_name(self, file_name):
        """ extract the module name from a CMakeLists.txt file and store
            in self._current_module if there is any """
        datafile = open(file_name)
        found_module_def = False
        for line in datafile:
            if 'px4_add_module' in line: # must contain 'px4_add_module'
                found_module_def = True

            words = line.split()
            # get the definition of MAIN
            if found_module_def and 'MAIN' in words and len(words) >= 2:
                self._current_module.append(words[1])
                self._all_modules.add(words[1])
                dbg_print('Found module name: '+words[1])
                return True
        return False


    def _process_source_file(self, file_name):
        """ extract information from a single source file """

        with codecs.open(file_name, 'r', 'utf-8') as f:
            try:
                content = f.read()
            except:
                print('Failed reading file: %s, skipping content.' % file_name)
                return


            current_module = self._get_current_module()
            if current_module == 'uorb_tests': # skip this
                return
            if current_module == 'uorb':

                # search and validate the ORB_ID_VEHICLE_ATTITUDE_CONTROLS define
                matches = self._orb_id_vehicle_attitude_controls_re.findall(content)
                for match in matches:
                    if match != 'ORB_ID('+self._orb_id_vehicle_attitude_controls_topic:
                        # if we land here, you need to change _orb_id_vehicle_attitude_controls_topic
                        raise Exception(
                            'The extracted define for ORB_ID_VEHICLE_ATTITUDE_CONTROLS '
                            'is '+match+' but expected ORB_ID('+
                            self._orb_id_vehicle_attitude_controls_topic)

                return # skip uorb module for the rest



            if content.lower().find('orb_') != -1: # approximative filter to quickly
                                           # discard files we're not interested in
                                           # (speedup the parsing)
                src = self._comment_remover(content)
                src = re.sub(self._whitespace_pattern, '', src) # remove all whitespace


                # subscriptions
                self._subscriptions.extract(file_name, src, current_module,
                        self._orb_id_vehicle_attitude_controls_topic)

                # publications
                self._publications.extract(file_name, src, current_module,
                        self._orb_id_vehicle_attitude_controls_topic)

                # TODO: handle Publication & Subscription template classes



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


    @property
    def modules(self):
        """ get the set of all modules """
        return self._modules

    @property
    def topics(self):
        """ get set set of all topics """
        return self._topics

    @property
    def topic_colors(self):
        """ get a dict of all topic colors with key=topic, value=color """
        return self._topic_colors

    @property
    def module_subscriptions(self):
        """ get a dict of all subscriptions with key=module name, value=set(topic names) """
        return self._subscriptions.pubsubs

    @property
    def module_publications(self):
        """ get a dict of all publications with key=module name, value=set(topic names) """
        return self._publications.pubsubs



class OutputGraphviz(object):
    """ write graph using Graphviz """

    def __init__(self, graph):
        self._graph = graph

    def write(self, file_name, engine='fdp',
              show_publications=True, show_subscriptions=True):
        """ write the graph to a file
            :param engine: graphviz engine
                - fdp works for large graphs
                - neato works better for smaller graphs
                - circo works for single modules
                CLI: fdp graph.fv -Tpdf -o test.pdf
        """

        print('Writing to '+file_name)

        ratio = 1 # aspect ratio

        modules = self._graph.modules
        topics = self._graph.topics
        topic_colors = self._graph.topic_colors
        module_publications = self._graph.module_publications
        module_subscriptions = self._graph.module_subscriptions

        graph_attr={'splines': 'true', 'ratio': str(ratio), 'overlap': 'false'}
        graph_attr['sep'] = '"+15,15"' # increase spacing between nodes
        graph = Digraph(comment='autogenerated graph with graphviz using uorb_graph.py',
                engine=engine, graph_attr=graph_attr)


        # nodes
        for module in modules:
            graph.node('m_'+module, module, shape='box', fontcolor='#ffffff',
                    style='filled', color='#666666', fontsize='16')

        for topic in topics:
            graph.node('t_'+topic, topic, shape='ellipse', fontcolor='#ffffff',
                    style='filled', color=topic_colors[topic])


        # edges
        if show_publications:
            for module in modules:
                if module in module_publications:
                    for topic in module_publications[module]:
                        if topic in topics:
                            graph.edge('m_'+module, 't_'+topic,
                                    color=topic_colors[topic], style='dashed')

        if show_subscriptions:
            for module in modules:
                if module in module_subscriptions:
                    for topic in module_subscriptions[module]:
                        if topic in topics:
                            graph.edge('t_'+topic, 'm_'+module,
                                    color=topic_colors[topic])

        graph.render(file_name, view=False)


class OutputJSON(object):
    """ write graph to a JSON file (that can be used with D3.js) """

    def __init__(self, graph):
        self._graph = graph

    def write(self, file_name):

        print('Writing to '+file_name)

        modules = self._graph.modules
        topics = self._graph.topics
        topic_colors = self._graph.topic_colors
        module_publications = self._graph.module_publications
        module_subscriptions = self._graph.module_subscriptions

        data = {}
        nodes = []

        # nodes
        # (sort by length, such that short names are last. The rendering order
        # will be the same, so that in case of an overlap, the shorter label
        # will be on top)
        for module in sorted(modules, key=len, reverse=True):
            node = {}
            node['id'] = 'm_'+module
            node['name'] = module
            node['type'] = 'module'
            node['color'] = '#666666'
            # TODO: add url to open module documentation?
            nodes.append(node)

        for topic in sorted(topics, key=len, reverse=True):
            node = {}
            node['id'] = 't_'+topic
            node['name'] = topic
            node['type'] = 'topic'
            node['color'] = topic_colors[topic]
            # url is opened when double-clicking on the node
            # TODO: does not work for multi-topics
            node['url'] = 'https://github.com/PX4/Firmware/blob/master/msg/'+topic+'.msg'
            nodes.append(node)

        data['nodes'] = nodes

        edges = []

        # edges
        for module in modules:
            if module in module_publications:
                for topic in module_publications[module]:
                    if topic in topics:
                        edge = {}
                        edge['source'] = 'm_'+module
                        edge['target'] = 't_'+topic
                        edge['color'] = topic_colors[topic]
                        edge['style'] = 'dashed'
                        edges.append(edge)

        for module in modules:
            if module in module_subscriptions:
                for topic in module_subscriptions[module]:
                    if topic in topics:
                        edge = {}
                        edge['source'] = 't_'+topic
                        edge['target'] = 'm_'+module
                        edge['color'] = topic_colors[topic]
                        edge['style'] = 'normal'
                        edges.append(edge)

        data['links'] = edges

        with open(file_name, 'w') as outfile:
            json.dump(data, outfile) # add indent=2 for readable formatting



# ignore topics that are subscribed/published by many topics, but are not really
# useful to show in the graph
topic_blacklist = [ 'parameter_update', 'mavlink_log', 'log_message' ]
print('Excluded topics: '+str(topic_blacklist))

if len(args.modules) == 0:
    module_whitelist = []
else:
    module_whitelist = [ m.strip() for m in args.modules.split(',')]

graph = Graph(module_whitelist=module_whitelist, topic_blacklist=topic_blacklist)
if len(args.src_path) == 0:
    args.src_path = ['src']
graph.build(args.src_path, args.exclude_path, use_topic_pubsub_union=args.use_topic_union)


if args.output == 'json':
    output_json = OutputJSON(graph)
    output_json.write(args.file+'.json')

elif args.output == 'graphviz':
    try:
        from graphviz import Digraph
    except ImportError as e:
        print("Failed to import graphviz: " + e)
        print("")
        print("You may need to install it with:")
        print("    pip3 install --user graphviz")
        print("")
        sys.exit(1)
    output_graphviz = OutputGraphviz(graph)
    engine='fdp' # use neato or fdp
    output_graphviz.write(args.file+'.fv', engine=engine)
    output_graphviz.write(args.file+'_subs.fv', show_publications=False, engine=engine)
    output_graphviz.write(args.file+'_pubs.fv', show_subscriptions=False, engine=engine)
else:
    print('Error: unknown output format '+args.output)
