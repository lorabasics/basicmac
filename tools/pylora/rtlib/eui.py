# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

import functools
import re
import struct
from base64 import b16decode as _b16decode


__all__ = ['Eui']


@functools.total_ordering
class Eui(object):
    """Class to represent EUIs. The normal representation is an string with 23 characters
    and a format of HH-...-HH. This matches the JSON and SQL representation.
    There are methods to convert to binary string or 64bit integer.
    """
    
    HEX_REGEX = re.compile("^[0-9A-Fa-f]{16}$")
    """Pure hexadecimal."""
    NUM_REGEX = re.compile("^(\-?\d+|0[xX][0-9A-Fa-f]+)$")
    """Standard decimal or hexadecimal number"""
    REGEX = re.compile("^([0-9a-fA-F]){2}(-([0-9a-fA-F]){2}){7}$")
    """Regular expression matching a well formed EUI"""
    REGEX2 = re.compile("^([0-9a-fA-F]){2}(:([0-9a-fA-F]){2}){7}$")
    """Regular expression matching a well formed EUI using colon instead of dashes"""
    
    
    @staticmethod
    def int2str (num:int) -> str:
        return "-".join(["{0:02X}".format((num>>(i*8)&0xFF)) for i in range(7,-1,-1)])
    
    @staticmethod
    def str2int (s:str) -> int:
        if Eui.HEX_REGEX.match(s):
            return int(s,16)
        if Eui.NUM_REGEX.match(s):
            return int(s,0)
        if Eui.REGEX.match(s):
            b = _b16decode(s.replace("-","").encode('ascii'))
        elif Eui.REGEX2.match(s):
            b = _b16decode(s.replace(":","").encode('ascii'))
        else:
            raise ValueError("Illegal Eui: {}".format(s))
        return struct.unpack_from('>q',b)[0]
    
    @staticmethod
    def from_str(euistr:str) -> 'Eui':
        return Eui(euistr)
    
    @staticmethod
    def from_int(euinum:int) -> 'Eui':
        return Eui(euinum)
    
    @staticmethod
    def from_bytes(euibytes:bytes) -> 'Eui':
        return Eui(euibytes)
    
    def __bool__(self) -> bool:
        # Bool value is not dependent on EUI's int value
        return True
    
    def __index__(self) -> int:
        return struct.unpack_from('>q',self.as_bytes(),0)[0]
    
    def as_int(self) -> int:
        return self.__index__()
    
    def as_bytes(self) -> bytes:
        return _b16decode(self.euistr.replace("-","").encode('ascii'))
    
    def __str__(self):
        return self.euistr
    
    def __repr__(self) -> str:
        return "<{}: {}>".format(self.__class__.__name__, str(self))
    
    def __hash__(self):
        return hash(self.euistr)
    
    def __lt__(self, other):
        return self.euistr < other.euistr
    
    def __eq__(self, other):
        return True if isinstance(other,Eui) and self.euistr==other.euistr else False
    
    def __init__(self, euispec) -> None:
        if isinstance(euispec,Eui):
            self.euistr = euispec.euistr   # type: str
        elif isinstance(euispec,str):
            if Eui.HEX_REGEX.match(euispec):
                self.euistr = Eui.int2str(int(euispec,16))
            elif Eui.NUM_REGEX.match(euispec):
                self.euistr = Eui.int2str(int(euispec,0))
            else:
                if Eui.REGEX.match(euispec):
                    self.euistr = euispec.upper()
                elif Eui.REGEX2.match(euispec):
                    self.euistr = euispec.replace(':','-').upper()
                else:
                    raise ValueError("Illegal EUI: {}".format(euispec))
        elif isinstance(euispec,bytes):
            if len(euispec)!=8:
                raise ValueError("Illegal EUI length: {}".format(len(euispec)))
            self.euistr = "-".join([ "{:02X}".format(euispec[i]) for i in range(8) ])
        elif isinstance(euispec,int):
            self.euistr = Eui.int2str(euispec)
        else:
            raise ValueError("Illegal EUI type: "+str(type(euispec)))


