#!/usr/bin/env python3

# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

import os
import shlex
import sys
import re
import yaml

from typing import Callable,Dict,List,Optional,Set,Tuple
from typing import cast
from argparse import Namespace as NS # type alias

from cc import CommandCollection

class Service:
    def __init__(self, svcid:str, fn:str) -> None:
        self.id = svcid
        self.srcs     : List[str]                      = []
        self.hooks    : List[List[str]]                = []
        self.hookdefs : Dict[str,List[str]]            = {}
        self.require  : List[str]                      = []
        self.defines  : List[Tuple[str,Optional[str]]] = []
        self.fn = fn
        with open(fn, 'r') as fh:
            d = yaml.safe_load(fh)
        for k, v in d.items():
            if k == 'src':
                if isinstance(v, list):
                    self.srcs.extend(v)
                else:
                    self.srcs.append(v)
            elif k == 'hooks':
                if not isinstance(v, list):
                    v = [ v ]
                self.hooks.extend([Service.parse_hook(h, fn) for h in v])
            elif k == 'require':
                if not isinstance(v, list):
                    v = [ v ]
                self.require.extend(v)
            elif k == 'define':
                if not isinstance(v, list):
                    v = [ v ]
                self.defines.extend([Service.parse_define(d, fn) for d in v])
            elif k.startswith('hook.'):
                h = k[5:]
                if h not in self.hookdefs:
                    self.hookdefs[h] = []
                if not isinstance(v, list):
                    v = [ v ]
                self.hookdefs[h].extend(v)
            else:
                raise ValueError('%s: unknown key %s' % (fn, k))

    @staticmethod
    def parse_hook(hd:str, fn:str) -> List[str]:
        m = re.match(r'^\s*(.+)\s+(\w+)\s*(\([^\)]+\))\s*$', hd)
        if m:
            return [ m.group(2), m.group(1) + ' %s ' + m.group(3) ]
        else:
            raise ValueError('%s: invalid function declaration "%s"' % (fn, hd))

    @staticmethod
    def parse_define(dd:str, fn:str) -> Tuple[str,Optional[str]]:
        m = re.match(r'^([^=]+)(?:=(.*))?', dd)
        if m:
            return cast(Tuple[str,Optional[str]], m.groups())
        else:
            raise ValueError('%s: invalid define declaration "%s"' % (fn, dd))

class ServiceCollection:
    def __init__(self) -> None:
        self.svcs : Dict[str,Service] = {}

    def add(self, svc:Service) -> None:
        self.svcs[svc.id] = svc

    def validate(self) -> None:
        pass

    def sources(self) -> List[str]:
        return [src for svc in self.svcs.values() for src in svc.srcs]

    def files(self) -> List[str]:
        return [svc.fn for svc in self.svcs.values()]

    def defines(self) -> List[str]:
        return ['SVC_' + s for s in self.svcs.keys()] + [
                '%s%s' % (k, '' if v is None else '=%s' % shlex.quote(v))
                for svc in self.svcs.values() for k,v in svc.defines]

    def hookdefs(self) -> Dict[str,Tuple[str,List[str]]]:
        return { h[0]: (h[1], [hd for hds in (sv2.hookdefs.get(h[0])
            for sv2 in self.svcs.values()) if hds is not None for hd in hds])
            for sv1 in self.svcs.values() for h in sv1.hooks }

    def unresolved(self) -> Set[str]:
        return set([s for sl in [svc.require for svc in self.svcs.values()] for s in sl if s not in self.svcs])

class ServiceToolUtil:
    @staticmethod
    def arg(name:str) -> Callable:
        if name == 'svc':
            return CommandCollection.arg(name, type=str,
                    metavar='svcid',
                    nargs='*', help='service identifier')
        if name == '--path':
            return CommandCollection.arg('-p', '--path', type=str,
                    action='append',
                    help='paths to search for service definitions')
        raise ValueError()


class ServiceTool:
    def run(self) -> None:
        CommandCollection.run(self)

    @staticmethod
    def load(svcid:str, paths:List[str]) -> Optional[Service]:
        for p in paths:
            fn = os.path.join(p, svcid + '.svc')
            if os.path.isfile(fn):
                return Service(svcid, fn)
        return None

    @staticmethod
    def collect(args:NS) -> ServiceCollection:
        sc = ServiceCollection()
        ss = set(args.svc)
        while len(ss):
            s = ss.pop()
            svc = ServiceTool.load(s, args.path or ['.'])
            if svc is None:
                raise ValueError('Cannot find service description for "%s"' % s)
            sc.add(svc)
            ss.update(sc.unresolved())
        sc.validate()
        return sc

    @ServiceToolUtil.arg('svc')
    @ServiceToolUtil.arg('--path')
    @CommandCollection.cmd(help='validate the service configuration')
    def check(self, args:NS) -> None:
        try:
            ServiceTool.collect(args)
        except:
            print(str(sys.exc_info()))

    @ServiceToolUtil.arg('svc')
    @ServiceToolUtil.arg('--path')
    @CommandCollection.cmd(help='output a list of source files')
    def sources(self, args:NS) -> None:
        sc = ServiceTool.collect(args)
        print(' '.join(sc.sources()))

    @ServiceToolUtil.arg('svc')
    @ServiceToolUtil.arg('--path')
    @CommandCollection.cmd(help='output a list defines for the compiler')
    def defines(self, args:NS) -> None:
        sc = ServiceTool.collect(args)
        print(' '.join(sc.defines()))

    @ServiceToolUtil.arg('svc')
    @CommandCollection.arg('-d', action='store_true', help='create a dependency file for make')
    @CommandCollection.arg('-o', '--output', type=str, help='output file', required=True)
    @ServiceToolUtil.arg('--path')
    @CommandCollection.cmd(help='create the svcdef header file')
    def svcdefs(self, args:NS) -> None:
        sc = ServiceTool.collect(args)
        with open(args.output, 'w') as fh:
            fh.write('// Automatically generated by %s\n\n' % ' '.join(sys.argv))
            for h, defs in sc.hookdefs().items():
                fh.write('#define SVCHOOK_%s(...) do { %s } while (0)\n' % (h,
                    ' '.join(['{ extern %s; %s(__VA_ARGS__); }'
                        % (defs[0] % f, f) for f in defs[1]])))
        if args.d:
            with open(os.path.splitext(args.output)[0] + '.d', 'w') as fh:
                deps = sc.files()
                fh.write('%s: %s\n\n' % (args.output, ' '.join(deps)))
                for d in deps:
                    fh.write('%s:\n\n' % d)

if __name__ == '__main__':
    ServiceTool().run()
