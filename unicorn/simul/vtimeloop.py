# Copyright (C) 2018-2018, Semtech Corporation.
# All rights reserved.
#
# This file is subject to the terms and conditions
# defined in file 'LICENSE', which is part of this
# source code package.

from typing import Any, Awaitable, Callable, Dict, Generator, List, \
        Optional, TypeVar, Union
from typing import cast

import asyncio
import heapq

T = TypeVar('T')

class VirtualTimeLoop(asyncio.AbstractEventLoop):

    class VirtualTimerHandle(asyncio.TimerHandle):
        # Python 3.7 makes this unnecessary by adding when() and cancelled() APIs
        def __init__(self, when:float, callback:Callable, args:List[Any],
                loop:asyncio.AbstractEventLoop, **kwargs:Any) -> None:
            super().__init__(when, callback, args, loop, **kwargs)
            self.__when = when
            self.__cancelled = False

        def cancel(self) -> None:
            super().cancel()
            self.__cancelled = True

        def when(self) -> float:
            return self.__when

        def cancelled(self) -> bool:
            return self.__cancelled

    def __init__(self) -> None:
        self._time:float = 0
        self._tasks:List[VirtualTimeLoop.VirtualTimerHandle] = list()
        self._ex:Optional[BaseException] = None
        
    def get_debug(self) -> bool:
        return False

    def time(self) -> float:
        return self._time

    def call_exception_handler(self, context:Dict[str,Any]) -> None:
        self._ex = context.get('exception', None)

    def _run(self, future:Optional[asyncio.Future]) -> None:
        while len(self._tasks) and (future is None or not future.done()):
            th = heapq.heappop(self._tasks)
            self._time = th.when()
            if not th.cancelled():
                th._run()
            if self._ex is not None:
                raise self._ex

    def run_until_complete(self,
            future:Union[Generator[Any,None,T],Awaitable[T]]) -> T:
        f = asyncio.ensure_future(future, loop=self)
        self._run(f)
        return f.result()

    def create_task(self, coro): # type: ignore
        async def wrap_for_ex() -> Any:
            try:
                return await coro
            except asyncio.CancelledError:
                pass
            except BaseException as e:
                self._ex = e
        return asyncio.Task(wrap_for_ex(), loop=self)

    def create_future(self) -> asyncio.Future:
        return asyncio.Future(loop=self)

    def call_at(self, when:float, callback:Callable,
            *args:Any, **kwargs:Any) -> asyncio.TimerHandle:
        th = VirtualTimeLoop.VirtualTimerHandle(when, callback, list(args), loop=self, **kwargs)
        heapq.heappush(self._tasks, th)
        return th

    def call_later(self, delay:float, callback:Callable,
            *args:Any, **kwargs:Any) -> asyncio.TimerHandle:
        return self.call_at(self._time + delay, callback, *args, **kwargs)

    def call_soon(self, callback:Callable, *args, **kwargs):
        return self.call_later(0, callback, *args, **kwargs)


    def _timer_handle_cancelled(self, handle:asyncio.TimerHandle) -> None:
        pass
