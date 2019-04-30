# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

from typing import Any,Mapping,MutableMapping

__all__ = ['Conf', 'Msg', 'State' ]

Conf = Mapping[str,Any]
Msg = MutableMapping[str,Any]
State = MutableMapping[str,Any]
