#! /usr/bin/env python3

from __future__ import print_function

import argparse
import codecs
import colorsys
import json
import logging
import os
import re
import sys
from typing import Optional, Set, Tuple


parser = argparse.ArgumentParser(
    description='Generate uORB pub/sub dependency graph from source code')

parser.add_argument('-s', '--src-path', action='append',
                    help='Source path(s) (default=src, can be specified multiple times)',
                    default=[])
parser.add_argument('-e', '--exclude-path', action='append',
                    help='Excluded path(s), can be specified multiple times',
                    default=[])
parser.add_argument('--merge-depends', action='store_true',
                    help='Merge library topics inte the modules that depend on them.')
parser.add_argument('-v','--verbosity', action='count',
                    help='increase output verbosity; primarily for debugging; repeat for more detail',
                    default=0)
parser.add_argument('-f', '--file', metavar='file', action='store',
                    help='output file name prefix',
                    default='graph')
parser.add_argument('-o', '--output', metavar='output', action='store',
                    help='output format (json or graphviz)',
                    default='json')
parser.add_argument('-u','--use-topic-union', action='store_true',
                    help='''
Use the union of all publication and subscription topics (useful for complete
graphs or only few/single module(s)). The default is to use the intersection
(remove topics that have no subscriber or no publisher)''')
parser.add_argument('-m', '--modules', action='store',
                    help='Comma-separated whitelist of modules (the module\'s '+
                    'MAIN, e.g. from a startup script)',
                    default='')


logging.basicConfig(level=logging.WARNING,format='%(message)s')
log = logging.getLogger()

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

    # special value to signal an ambiguous was found -- don't record this topic, and stop processing.
    AMBIGUOUS_SITE_TOPIC = "AMBIGUOUS"

    def __init__(self, name, topic_blacklist, regexes):
        """
            :param is_publication: if True, publications, False for
            subscriptions
            :param topic_blacklist: list of topics to blacklist
            :param orb_pub_sub_regexes: list of regexes to extract orb calls
                (e.g. orb_subscribe). They need to have 2 captures, the second
                one is the one capturing ORB_ID(<topic>
        """
        self._name = name
        self._topic_blacklist = topic_blacklist
        self._regexes = set([ re.compile(regex) for regex in regexes])

    def match(self, source_line: str) -> str:
        """ Extract subscribed/published topics from a source string
        :param src_str: string of C/C++ code with comments and whitespace removed
        :return: if any topic was found, returned as a str.   On error, raise on exception.  On ambiguous line, return `AMBIGUOUS_SITE_TOPIC`.  Otherwise, return `None`
        """

        for regex in self._regexes:
            # just the matches for this particular pattern:
            match = regex.search(source_line)

            if match is None:
                continue

            # # all regexes should contain 2 capture groups  (or else this code block crashes)
            route_group, topic_group = match.groups()

            log.debug("          ####:{}:  {}, {}".format( self._name, route_group, topic_group))

            # # TODO: handle this case... but not sure where, yet
            # if match == 'ORB_ID_VEHICLE_ATTITUDE_CONTROLS': # special case
            #     match = orb_id+orb_id_vehicle_attitude_controls_topic

            # match has the form: '[ORB_ID(]<topic_name>'
            if route_group:
                if route_group == 'ORB_ID':
                    log.debug("            >>> Found ORB_ID topic: " + topic_group +  "    w/regex: " + str(regex.pattern))
                    return self._filter_topic(topic_group)
                elif route_group == '[':
                    if not topic_group:
                        log.debug("            !! found an ambiguous site => return an empty set")
                        return PubSub.AMBIGUOUS_SITE_TOPIC
                else:
                    raise SyntaxError('!!! Encountered regex case:  `route_group` contains unrecognized value!: '+ route_group+'  (::'+str(regex.pattern)+')\n'
                                        + "        ("+ route_group+', '+topic_group +")\n"
                                        + "        " + source_line)
            elif route_group.empty() and topic_group.empty():
                log.debug('!!! Found ambiguous site, without `ORB_ID` or topic (::'+str(regex.pattern))
                return PubSub.AMBIGUOUS_SITE_TOPIC

            else:
                raise SyntaxError("            !!! unhandled case:  unknown-variant: "+route_group+", " + topic_group + " ....from regex: " + str(regex.pattern))

        return None

    def _filter_topic(self, topic_name: str) -> str:
        """ add topic to set, unless the topic is ignored """
        if topic_name in self._topic_blacklist:
            log.debug("                    XX Ignoring blacklisted topic " + topic_name)
            return None
        else:
            return topic_name

class Publications(PubSub):
    """ Collects topic publication information for scopes """

    def __init__(self, topic_blacklist, regexes):
        super().__init__('PUB', topic_blacklist, regexes)


class Subscriptions(PubSub):
    """ Collects topic subscription information for scopes """

    def __init__(self, topic_blacklist, regexes):
        super().__init__('SUB', topic_blacklist, regexes)


class Ambiguities(PubSub):
    """ Collects topic information that cannot be classified """

    def __init__(self, topic_blacklist, regexes):
        super().__init__('AMB', topic_blacklist, regexes)


class Scope(object):
    """ Defines a scope to add dependencies or topics to """
    def __init__(self, typename, name):
        self.publications = set()
        self.subscriptions = set()
        self.dependencies = set()
        self.ambiguities = set()
        self._name = name
        self._typename = typename

    def add_dependency(self, dependency_name: str):
        if isinstance( dependency_name, str):
            self.dependencies.add(dependency_name)

    def is_empty(self):
        return (0 == len(self.publications)) and (0==len(self.subscriptions))

    @property
    def name(self):
        return self._name

    def reduce_ambiguities(self) -> Set[str]:
        self.ambiguities = self.ambiguities - self.subscriptions - self.publications
        return self.dependencies

    @property
    def typename(self):
        return self._typename

    # define these so we can hash these classes in dicts and sets
    def __hash__(self):
        return self._name.__hash__()

    def __eq__(self, other):
        if isinstance(other, str):
            return self._name == other
        else:
            return self._name == other._name

class LibraryScope(Scope):
    def __init__(self, name):
        super().__init__('Library',name)

class ModuleScope(Scope):
    def __init__(self, name):
        super().__init__('Module',name)

class Graph(object):
    """ Collects Node and Edge information by parsing the source tree """

    def __init__(self, **kwargs):
        """
        :kwargs:
            - scope_whitelist
            - scope_blacklist
            - topic_blacklist
        """

        self._whitespace_pattern = re.compile(r'\s+')

        self._scope_blacklist = set(kwargs.get('scope_blacklist',set()))
        self._scope_whitelist = set(kwargs.get('scope_whitelist',set()))

        self._path_blacklist = []

        self._topic_blacklist = set(kwargs.get('topic_blacklist',set()))

        self._orb_id_vehicle_attitude_controls_topic = 'actuator_controls_0'
        self._orb_id_vehicle_attitude_controls_re = re.compile(r'\#define\s+ORB_ID_VEHICLE_ATTITUDE_CONTROLS\s+([^,)]+)')

        self._warnings = [] # list of all ambiguous scan sites

        self._current_scope = [] # stack with current module (they can be nested)

        self._found_modules = {}   # dict of all found modules
        self._found_libraries = {} # dict of all found modules

        self._print_nodes = set()  # combination of libraries + modules
        self._print_topics = set() # all topics
        self._topic_colors = {} # key = topic, value = color (html string)

        # note: the source-file-string is pre-processed to remove whitespace -- regexes should ignore whitespace
        # note: the regexes should 2 capture groups '()' to correctly register with downstream code
        capture_cases_sub = [r"orb_subscribe\w*\((ORB_ID)(?:\(|::)(\w+)",
                             r"orb_copy\((ORB_ID)(?:\(|::)(\w+)",
                             r"Subscription\w*(?:<[^>]+>|)\w*(?:\[[^]]+\]|)[\{\(](ORB_ID)(?:\(|::)(\w+)",
                             r"SubscriptionCallbackWorkItem\w+\{this,(ORB_ID)(?:\(|::)(\w+)",
                            ]
        self._subscriptions = Subscriptions( self._topic_blacklist, capture_cases_sub)

        # note: the source-file-string is pre-processed to remove whitespace -- regexes should ignore whitespace
        # note: the regexes should 2 capture groups '()' to correctly register with downstream code
        capture_cases_pub = [r"orb_advertise(?:_multi|_queue|_multi_queue|)\((ORB_ID)(?:\(|::)(\w+)",
                             r"orb_publish(?:_auto|)\((ORB_ID)(?:\(|::)(\w+)",
                             r"Publication\w*<\w+>\w+(?:\[[^]]+\]|)[\(\{]*(ORB_ID)(?:\(|::)(\w+)",
                            ]
        self._publications = Publications( self._topic_blacklist, capture_cases_pub)

        # note: the source-file-string is pre-processed to remove whitespace -- regexes should ignore whitespace
        # note: the regexes should 2 capture groups '()' to correctly register with downstream code
        capture_cases_ambiguous = [ r"Publication\w*(?:\<\w+\>|)\w+(\[)()",
                                    r"Subscription\w*(?:\<\w+\>|)\w+(\[)()",
                                    r"(ORB_ID)(?:\(|::)(\w+)",
                                  ]
        self._ambiguities = Ambiguities( self._topic_blacklist, capture_cases_ambiguous)

    def _get_current_scope(self):
        if len(self._current_scope) == 0:
            return None
        return self._current_scope[-1]

    def build(self, src_path_list, **kwargs):
        """ parse the source tree & extract pub/sub information.
            :param use_topic_pubsub_union: if true, use all topics that have a
            publisher or subscriber. If false, use only topics with at least one
            publisher and subscriber.

            fill in self._module_subsciptions & self._module_publications
        """


        self._path_blacklist =  set([ os.path.normpath(p) for p in kwargs.get('path_blacklist',[]) ])

        for path in src_path_list:
            log.info("## Add src path: " + path )
            self._build_recursive(path, **kwargs)

        # Summarize the found counts: (all topics are defined in 'dependency' library)
        log.info('### Summary: Total Scanned:')
        log.info('    Library Count:      '+str(len(self._found_libraries)))
        log.info('    Module Count:       '+str(len(self._found_modules)))
        log.info('    Warning Count:      '+str(len(self._warnings)))

        if kwargs['merge_depends']:
            graph.merge_depends()

        # filter all scopes, topics into only the scopes + topics to output
        self._generate_print_lists(use_topic_pubsub_union=kwargs['use_topic_pubsub_union'], merge_depends=kwargs['merge_depends'])

        # Summarize the found counts:
        log.info('### Summary (in-scope):')
        log.info('    Scope Count:        '+str(len(self._print_scopes)))
        log.info('    Ambiguous Topics:   '+str(len(self._print_ambiguities)))
        log.info('    Linked Topics:      '+str(len(self._print_topics)))
        log.info('    Warnings:           '+str(len(self._warnings)))

        if 0 < len(self._warnings):
            # print out the list of warning-sites:
            log.info('## Warning Sites:')
            for w in self._warnings:
                scope_name = 'no-scope'
                if None is not w[0]:
                    scope_name = w[0].name
                # warnings tuple contains: (current_scope, file_name, line_number, line)
                log.info("    -['{}']:{:<64s}:{} = {}".format(scope_name, w[1].lstrip('/.'), w[2], w[3] ))

        # initialize colors
        color_list = get_N_colors(len(self._print_topics), 0.7, 0.85)
        self._topic_colors = {}
        for i, topic in enumerate(self._print_topics):
            self._topic_colors[topic] = color_list[i]


    def _generate_print_lists(self, use_topic_pubsub_union, merge_depends):
        """ generate the set of scopes (modules + libraries) and topics to print to output """

        subscribed_topics = set()
        published_topics = set()
        ambiguous_topics = set()

        # gather all found scopes:
        all_scopes = { **self._found_libraries, **self._found_modules }

        if 0 == len(self._scope_whitelist):
            select_scopes = self._found_modules
        else:
            select_scopes = {}
            for scope_name in self._scope_whitelist:
                if scope_name in all_scopes:
                    select_scopes[scope_name] = all_scopes[scope_name]
        if not isinstance(select_scopes, dict) or 0 == len(select_scopes):
            log.error("!! No requested modules not found -- exiting.")
            sys.exit(0)

        log.debug('### Condensing found topics: scope -> total')
        for name,scope in select_scopes.items():
            log.debug('    # Scope: '+ name )

            log.debug('        ## Subs: ' + str(len(scope.subscriptions)))
            for topic in sorted(scope.subscriptions):
                log.debug('            - ' + topic)
                subscribed_topics.add(topic)

            log.debug('        ## Pubs: ' + str(len(scope.publications)))
            for topic in sorted(scope.publications):
                log.debug('            - ' + topic )
                published_topics.add(topic)

            scope.reduce_ambiguities()

            log.debug('        ## Ambiguities: ' + str(len(scope.ambiguities)))
            for topic in sorted(scope.ambiguities):
                log.debug('            - ' + topic )
                ambiguous_topics.add(topic)

        # filter modules iff they have at least a subscription or a publication
        scopes_with_topic = {}
        for name,scope in select_scopes.items():
            if not scope.is_empty():
                scopes_with_topic[name] = scope

        self._print_ambiguities = ambiguous_topics
        if use_topic_pubsub_union:
            self._print_topics = subscribed_topics | published_topics
            self._print_scopes = scopes_with_topic
        else:
            self._print_topics = subscribed_topics & published_topics

            # cull scopes to only those that pub or sub to a topic that has both
            intersect_scopes = {}
            for name,scope in scopes_with_topic.items():
                all_scope_topics = scope.publications | scope.subscriptions
                for topic in all_scope_topics:
                    if topic in self._print_topics:
                        intersect_scopes[scope.name] = scope
                        break
            self._print_scopes = intersect_scopes


    def _build_recursive(self, path, **kwargs):
        if os.path.normpath(path) in self._path_blacklist:
            log.debug('ignoring excluded path '+path)
            return

        entries = os.listdir(path)

        # check if entering a new scope
        cmake_file = 'CMakeLists.txt'
        new_scope = False
        if cmake_file in entries:
            new_scope = self._extract_build_information(os.path.join(path, cmake_file), **kwargs)

        # iterate directories recursively
        for entry in entries:
            file_name = os.path.join(path, entry)
            if os.path.isdir(file_name):
                self._build_recursive(file_name, **kwargs)


        # iterate source files
        # Note: Skip all entries if we're not in a scope -- both finding known pubs/subs and emitting warnings
        for entry in entries:
            file_name = os.path.join(path, entry)
            if os.path.isfile(file_name):
                _, ext = os.path.splitext(file_name)
                if ext in ['.cpp', '.c', '.h', '.hpp']:
                    self._process_source_file(file_name)

        if new_scope:
            self._current_scope.pop()


    def _extract_build_information(self, file_name, **kwargs):
        """ extract the module or library name from a CMakeLists.txt file and store
            in self._current_scope if there is any
            Also records dependencies, if any are specified.
        """

        datafile = open(file_name)
        found_module_def = False
        found_module_depends = False
        found_library_def = False
        for line in datafile:
            if 'px4_add_module' in line: # must contain 'px4_add_module'
                found_module_def = True
            elif 'px4_add_library' in line: # must contain 'px4_add_module'
                tokens = line.split('(')
                if 1 < len(tokens):
                    found_library_def = True
                    library_name = tokens[1].split()[0].strip().rstrip(')')
                    library_scope = LibraryScope(library_name)
                    self._current_scope.append(library_scope)
                    self._found_libraries[library_name] = library_scope
                    if self._in_scope():
                        log.debug('    >> found library: ' + library_name)

                    # we can return early because we have no further information to collect from libraries
                    return True
            elif found_module_def and 'DEPENDS' in line.upper():
                found_module_depends = True
            elif found_module_depends:
                # two tabs is a *sketchy* heuristic -- spacing isn't guaranteed by cmake;
                # ... but the hard-tabs *is* specified by PX4 coding standards, so it's likely to be consistent
                if line.startswith('\t\t'):
                    depends = [dep.strip() for dep in line.split()]
                    for name in depends:
                        self._current_scope[-1].add_dependency(name)
                        if kwargs['merge_depends']:
                            if (0 < len(self._scope_whitelist)) and self._current_scope[-1].name in self._scope_whitelist:
                                # if we whitelist a module with dependencies, whitelist the dependencies, too
                                self._scope_whitelist.add(name)

                else:
                    found_module_depends = False  ## done with the 'DEPENDS' section.

            words = line.split()
            # get the definition of MAIN
            if found_module_def and 'MAIN' in words and len(words) >= 2:
                module_name = words[1]
                module_scope = ModuleScope(module_name)
                self._current_scope.append(module_scope)
                self._found_modules[module_name] = module_scope
                if self._in_scope():
                    log.debug('    >> Found module name: ' + module_scope.name)

        return (found_library_def or found_module_def)


    def _process_source_file(self, file_name):
        """ extract information from a single source file """

        log.debug( "        >> extracting topics from file: " + file_name )

        with codecs.open(file_name, 'r', 'utf-8') as f:
            try:
                content = f.read()
            except:
                print('Failed reading file: %s, skipping content.' % file_name)
                return


            current_scope = self._get_current_scope()
            if current_scope:
                if current_scope.name in self._scope_blacklist:
                    return
                elif current_scope.name == 'uorb_tests': # skip this
                    return
                elif current_scope.name == 'uorb':

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

            line_number = 0
            for full_line in content.splitlines():
                line_number += 1

                short_line = re.sub(self._whitespace_pattern, '', full_line)

                topic = self._publications.match(short_line)
                if topic:
                    if current_scope:
                        current_scope.publications.add(topic)
                        continue
                    else:
                        raise AssertionError("Encountered Publication topic outside of any scope! " + file_name + " Aborting!")

                topic = self._subscriptions.match(short_line)
                if topic:
                    if current_scope:
                        current_scope.subscriptions.add(topic)
                        continue
                    else:
                        raise AssertionError("Encountered Subscription topic outside of any scope! " + file_name + " Aborting!")

                topic = self._ambiguities.match(short_line)
                if topic:
                    if current_scope:
                        if topic != PubSub.AMBIGUOUS_SITE_TOPIC:
                            current_scope.ambiguities.add(topic)
                        self._warnings.append((current_scope, file_name, line_number, full_line))
                        continue
                    else:
                        raise AssertionError("Encountered Ambiguous topic outside of any scope! " + file_name + " Aborting!")

    def _in_scope(self, scope_name = None):
        if 0 < len(self._current_scope):
            if None is scope_name:
                scope_name = self._current_scope[-1].name
            if scope_name in self._scope_whitelist:
                return True

        return False

    def merge_depends(self):
        log.info('### Merge Depends:')

        for modname,module in self._found_modules.items():
            if modname in self._scope_whitelist or 0==len(self._scope_whitelist):
                for depname in module.dependencies:
                    if depname in self._found_libraries:
                        dep = self._found_libraries[depname]
                        # copy topics from library to depending library
                        for topic in dep.publications:
                            module.publications.add(topic)
                        for topic in dep.subscriptions:
                            module.subscriptions.add(topic)
                        for topic in dep.ambiguities:
                            module.ambiguities

        # omit all libraries -- they've already been merged into their respective dependees
        self._scope_whitelist = set([ str(s) for s in self._scope_whitelist if s not in self._found_libraries])


    @property
    def output_scopes(self):
        """ get the set of all modules """
        return self._print_scopes

    @property
    def output_topics(self):
        """ get set set of all topics """
        return self._print_topics

    @property
    def topic_colors(self):
        """ get a dict of all topic colors with key=topic, value=color """
        return self._topic_colors

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

        output_topics = self._graph.output_topics
        output_scopes = self._graph.output_scopes
        topic_colors = self._graph.topic_colors

        graph_attr={'splines': 'true', 'ratio': str(ratio), 'overlap': 'false'}
        graph_attr['sep'] = '"+15,15"' # increase spacing between nodes
        graph = Digraph(comment='autogenerated graph with graphviz using uorb_graph.py',
                engine=engine, graph_attr=graph_attr)

        # scopes: modules
        log.info('    > Writing scopes')
        for name,_ in output_scopes.items():
            graph.node('m_'+name, name, shape='box', fontcolor='#ffffff',
                    style='filled', color='#666666', fontsize='16')

        log.info('    > Writing topics')
        for topic in output_topics:
            graph.node('t_'+topic, topic, shape='ellipse', fontcolor='#ffffff',
                    style='filled', color=topic_colors[topic])

        # edges
        log.info('    > Writing publish edges')
        if show_publications:
            for scope_name,scope in output_scopes.items():
                for topic in scope.publications:
                    if topic in output_topics:
                        graph.edge('m_'+scope_name, 't_'+topic, color=topic_colors[topic], style='dashed')

        log.info('    > Writing subscribe edges')
        if show_subscriptions:
            for scope_name,scope in output_scopes.items():
                for topic in scope.subscriptions:
                    if topic in output_topics:
                        graph.edge('t_'+topic, 'm_'+scope_name, color=topic_colors[topic])

        graph.render(file_name, view=False)


class OutputJSON(object):
    """ write graph to a JSON file (that can be used with D3.js) """

    def __init__(self, graph):
        self._graph = graph

    def write(self, file_name):

        print('Writing to '+file_name)

        output_topics = self._graph.output_topics
        output_scopes = self._graph.output_scopes
        topic_colors = self._graph.topic_colors

        data = {}
        nodes = []

        # nodes
        # (sort by length, such that short names are last. The rendering order
        # will be the same, so that in case of an overlap, the shorter label
        # will be on top)
        for scope_tuple in sorted(output_scopes.items(), key=(lambda st: len(st[0])), reverse=True):
            node = {}
            node['id'] = 'm_'+scope_tuple[0]
            node['name'] = scope_tuple[0]
            node['type'] = scope_tuple[1].typename
            node['color'] = '#666666'
            # TODO: add url to open module documentation?
            nodes.append(node)

        for topic in sorted(output_topics, key=len, reverse=True):
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
        for name,scope in output_scopes.items():
            for topic in scope.publications:
                if topic in output_topics:
                    edge = {}
                    edge['source'] = 'm_'+name
                    edge['target'] = 't_'+topic
                    edge['color'] = topic_colors[topic]
                    edge['style'] = 'dashed'
                    edges.append(edge)

        for name,scope in output_scopes.items():
            for topic in scope.subscriptions:
                if topic in output_topics:
                    edge = {}
                    edge['source'] = 't_'+topic
                    edge['target'] = 'm_'+name
                    edge['color'] = topic_colors[topic]
                    edge['style'] = 'normal'
                    edges.append(edge)

        data['links'] = edges

        with open(file_name, 'w') as outfile:
            json.dump(data, outfile) # add indent=2 for readable formatting


if "__main__" == __name__:

    args = parser.parse_args()

    if 0 < args.verbosity:
        if 1 == args.verbosity:
            log.setLevel(logging.INFO)
            print("set log level to INFO")
        else: # implicity 1<
            log.setLevel(logging.DEBUG)
            print("set log level to DEBUG")

    # ignore topics that are subscribed/published by many topics, but are not really
    # useful to show in the graph
    topic_blacklist = [ 'parameter_update', 'mavlink_log', 'log_message' ]
    print('Excluded topics: '+str(topic_blacklist))

    if len(args.modules) == 0:
        scope_whitelist = []
    else:
        scope_whitelist = [ m.strip() for m in args.modules.split(',')]
        scope_whitelist = set(scope_whitelist)

    graph = Graph(scope_whitelist=scope_whitelist, topic_blacklist=topic_blacklist)

    # if no source paths are supplied, guess that we're in the project root, and apply it to the entire 'src/' tree
    if len(args.src_path) == 0:
        args.src_path = ['src']

    # transcribe only the source paths that actually exist:
    source_paths = []
    for path in args.src_path:
        if os.path.exists(path):
            source_paths.append(path)
        else:
            log.warn("Could not find path: " + path)

    if 0 == len(source_paths):
        print("!! None of the source directories were valid -- Exiting.")
        sys.exit(-1)

    # ignore certain paths
    path_blacklist = ['src/lib/parameters/']
    if 0 < len(args.exclude_path):
        path_blacklist = args.exclude_path
    if path_blacklist:
        print('Excluded Path: '+str(path_blacklist))

    graph.build(source_paths, path_blacklist=path_blacklist, use_topic_pubsub_union=args.use_topic_union, merge_depends=args.merge_depends)

    if args.output == 'json':
        output_json = OutputJSON(graph)
        output_json.write(args.file+'.json')

    elif args.output in ('graphviz','gv'):
        try:
            from graphviz import Digraph
        except ImportError as e:
            print("Failed to import graphviz: " + str(e))
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
    elif args.output == 'none':
        pass
    else:
        print('Error: unknown output format '+args.output)
