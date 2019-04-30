# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

import argparse

from typing import Any,Callable,List,Optional,Union
from argparse import Namespace as NS, ArgumentParser as AP # type aliases

class CommandCollection:
    parser = argparse.ArgumentParser()
    subs = parser.add_subparsers(dest='command')
    subs.required = True # type: ignore
    sub = None

    @staticmethod
    def cmd(name:Optional[str]=None, **kwargs:Any) -> Callable:
        def cmd_decorator(f:Callable) -> Callable:
            p = CommandCollection.subs.add_parser(name if name else f.__name__, **kwargs)
            def cmd_do(obj:Any, args:NS) -> None:
                CommandCollection.sub = p
                f(obj, args)
            p.set_defaults(func=cmd_do)
            cmd_do.parser = p # type: ignore
            return cmd_do
        return cmd_decorator

    @staticmethod
    def arg(*args:Any, **kwargs:Any) -> Callable:
        def arg_decorator(f:Callable) -> Callable:
            p:AP = f.parser # type: ignore
            p.add_argument(*args, **kwargs)
            return f
        return arg_decorator

    @staticmethod
    def argf(af:Callable, **kwargs:Any) -> Callable:
        def arg_decorator(f:Callable) -> Callable:
            p:AP = f.parser # type: ignore
            af(p, **kwargs)
            return f
        return arg_decorator

    @staticmethod
    def run(obj:Any) -> None:
        args = CommandCollection.parser.parse_args()
        args.func(obj, args)

    @staticmethod
    def error(message:str) -> None:
        (CommandCollection.sub or CommandCollection.parser).error(message)
