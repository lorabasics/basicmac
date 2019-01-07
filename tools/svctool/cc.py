# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

import argparse

from typing import Any,Callable,List,Optional,Union

class CommandCollection:
    parser = argparse.ArgumentParser()
    subs = parser.add_subparsers(dest='command')
    subs.required = True
    sub = None

    def cmd(name:Optional[str]=None, **kwargs) -> Callable:
        def cmd_decorator(f:Callable) -> Callable:
            p = CommandCollection.subs.add_parser(name if name else f.__name__, **kwargs)
            def cmd_do(obj, args):
                CommandCollection.sub = p
                f(obj, args)
            p.set_defaults(func=cmd_do)
            cmd_do.parser = p
            return cmd_do
        return cmd_decorator

    def arg(*args, **kwargs) -> Callable:
        def arg_decorator(f:Callable) -> Callable:
            f.parser.add_argument(*args, **kwargs)
            return f
        return arg_decorator

    def argf(af:Callable, **kwargs) -> Callable:
        def arg_decorator(f:Callable) -> Callable:
            af(f.parser, **kwargs)
            return f
        return arg_decorator

    def run(obj:Any) -> None:
        args = CommandCollection.parser.parse_args()
        args.func(obj, args)

    def error(message:str) -> None:
        (CommandCollection.sub or CommandCollection.parser).error(message)
