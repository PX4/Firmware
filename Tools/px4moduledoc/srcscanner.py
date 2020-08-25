#! /usr/bin/env python3
import os
import re
import codecs
import sys

class SourceScanner(object):
    """
    Traverses directory tree, reads all source files, and passes their contents
    to the Parser.
    """

    def ScanDir(self, srcdirs, parser):
        """
        Scans provided path and passes all found contents to the parser using
        parser.Parse method.
        """
        extensions = tuple([".cpp", ".c"])
        for srcdir in srcdirs:
            for dirname, dirnames, filenames in os.walk(srcdir):
                for filename in filenames:
                    if filename.endswith(extensions):
                        path = os.path.join(dirname, filename)
                        try:
                            if not self.ScanFile(path, parser):
                                return False
                        except:
                            print(("Exception in file %s" % path))
                            raise
        return True

    def ScanFile(self, path, parser):
        """
        Scans provided file and passes its contents to the parser using
        parser.Parse method.
        """
        # Extract the scope: it is the directory within the repo. Either it
        # starts directly with 'src/module/abc', or it has the form 'x/y/z/src/module/abc'.
        # The output is 'module/abc' in both cases.
        prefix = "^(|.*" + os.path.sep + ")src" + os.path.sep
        scope = re.sub(prefix.replace("\\", "/"), "", os.path.dirname(os.path.relpath(path)).replace("\\", "/"))

        with codecs.open(path, 'r', 'utf-8') as f:
            try:
                contents = f.read()
            except:
                contents = ''
                print(('Failed reading file: %s, skipping content.' % path))
                pass
        return parser.Parse(scope, contents)
