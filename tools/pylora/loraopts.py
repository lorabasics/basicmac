# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

from typing import Callable,Dict,List,Tuple,Union
import re

class BF():
    """Bitfield - might also cover entire bytes."""
    def __init__(self, name:str='', bits:Union[int,Tuple[int,int]]=(0,0), signed=False) -> None:
        self.name = name
        self.brange = (bits,bits) if isinstance(bits,int) else bits
        self.signed = signed
        self.value = 0

    def decode(self, v:int) -> int:
        eb = 2<<self.brange[1]
        x = (v & (eb-1)) >>self.brange[0]
        if self.signed and x >= (eb>>1):
            x = x-eb
        self.value = x
        return x

    def encode(self) -> int:
        eb = 2<<self.brange[1]
        lo = self.brange[0]
        return (self.value << lo) & (eb-1)

    def value_as_int(self) -> int:
        return self.value

    def value_as_str(self, fmt:str='s') -> str:
        return '{:d}'.format(self.value_as_int())

    def bits_as_str(self) -> str:
        if self.brange[0] != self.brange[1]:
            return '%d-%d' % self.brange
        return '%d' % self.brange[0]

    def __repr__(self) -> str:
        return '%s:%s=%s' % (self.name, self.bits_as_str(), self.value_as_str('s'))

    def __str__(self) -> str:
        return '%s=%s' % (self.name, self.value_as_str('r'))

    def format(self, fmt:str) -> str:
        if 'S' in fmt:  # structure spec
            return '%s:%s' % (self.name, self.bits_as_str())
        if 'D' in fmt:  # details
            return repr(self)
        return str(self)


class Ux():
    def __init__(self, n, fields:Tuple[BF,...]) -> None:
        self.n = n
        self.fields = fields
        for f in fields:
            self.__setattr__(f.name, f)
        self.value = 0

    def decode(self, ba:bytes, off:int) -> int:
        v = 0
        n = self.n
        for i in range(n):
            v |= ba[off+i]<<(i*8)
        self.value = v
        for f in self.fields:
            f.decode(v)
        return off+n

    def encode(self, ba:bytearray, off:int) -> int:
        v = 0
        for f in self.fields:
            v |= f.encode()
        n = self.n
        for i in range(n):
            ba[off+i] = (v>>(i*8)) & 0xFF
        return off+n

    def __repr__(self) -> str:
        return 'u%d(%s)' % (self.n, ','.join(map(repr, self.fields)))

    def __str__(self) -> str:
        return ' '.join(map(str, filter(lambda a: not isinstance(a,RFU), self.fields)))

    def format(self, fmt:str) -> str:
        """Format element U disables rendering of Ux elements. Element u/text defines the
        separator between Ux elements (default is comma)."""
        sep = ','
        m = re.search(r'[Uu]/([^/]*)/', fmt)
        if m: sep = m.group(1)
        f = sep.join(filter(lambda f:f, map(lambda f:f.format(fmt), self.fields)))
        if 'U' in fmt:
            return f
        return 'u%d(%s)' % (self.n, f)



class U8(Ux):
    def __init__(self, *fields:BF) -> None:
        super().__init__(1, fields)

class U16(Ux):
    def __init__(self, *fields:BF) -> None:
        super().__init__(2, fields)

class U24(Ux):
    def __init__(self, *fields:BF) -> None:
        super().__init__(3, fields)

class U32(Ux):
    def __init__(self, *fields:BF) -> None:
        super().__init__(4, fields)


class BFx(BF):
    def __init__(self, name:str='', bits:Union[int,Tuple[int,int]]=(0,0), signed=False) -> None:
        super().__init__(name,bits,signed)

    def value_as_str(self, fmt:str='s') -> str:
        return 'x{:X}'.format(self.value_as_int())


class RFU(BFx):
    def __init__(self, from_bit=0, to_bit=None, ext:str='') -> None:
        super().__init__('RFU'+ext,bits=(from_bit, to_bit or from_bit))

    def value_as_str(self, fmt:str='s') -> str:
        v = self.value_as_int()
        if v < 10:
            return str(v)
        return 'x{:X}'.format(v)

    def format(self, fmt:str) -> str:
        if 'R' in fmt:
            return ''
        return super().format(fmt)


class Freq(BF):
    def __init__(self, name:str='Freq') -> None:
        super().__init__(name, (0,23))

    def value_as_str(self, fmt:str='s') -> str:
        v = self.value_as_int()
        if v<1000000:
            return '%d' % v
        return '%.1fkHz' % (v/10) if (v % 10) else '%.3fMHz' % (v/1e4)


class Ack(BF):
    def __init__(self, name:str, bit:int) -> None:
        super().__init__(name, bit)

    def value_as_str(self, fmt:str='s') -> str:
        v = self.value_as_int()
        return '1/ACK' if v else '0/NACK'



class Opt():
    CMD = 0
    NAME = ''
    def __init__(self, *args:Ux, **kwargs) -> None:
        self.args = args
        for a in args:
            for f in a.fields:
                self.__setattr__(f.name, f)
        self.set(**kwargs)

    def __repr__(self) -> str:
        a = ' '.join(map(repr, self.args))
        if a: a=' '+a
        return '%s<x%02X%s>' % (self.NAME, self.CMD & 0xFF, a)

    def __str__(self) -> str:
        return '%s<%s>' % (self.NAME, ' '.join(map(str, self.args)))

    def set(self, **kwargs) -> None:
        for k,v in kwargs.items():
            getattr(self,k).value = v

    def format(self, fmt:str) -> str:
        """Format element A print arguments only. a/text/ print command and arguments and
        separates arguments with text (default: space). Char 'C' suppresses the hex command code."""
        sep = ' '
        m = re.search(r'[aA]/([^/]*)/', fmt)
        if m: sep = m.group(1)
        a = sep.join(map(lambda a:a.format(fmt), self.args))
        if 'A' in fmt:
            return a
        chex = 'x%02X ' % (self.CMD & 0xFF) if 'C' not in fmt else ''
        return '%s<%s%s>' % (self.NAME, chex, a)

    def decode(self, ba:bytes, off:int) -> int:
        assert ba[off] == self.CMD & 0xFF
        off += 1
        for a in self.args:
            off = a.decode(ba,off)
        return off

    def encode(self, ba:bytearray, off:int) -> int:
        ba[off] = self.CMD & 0xFF
        off += 1
        for a in self.args:
            off = a.encode(ba,off)
        return off

class OptUp(Opt):
    pass

class OptDn(Opt):
    pass

class Unknown(Opt):
    CMD = 0
    NAME = '?'
    def __init__(self, n:int) -> None:
        self.n = n
        self.un = b''

    def __repr__(self) -> str:
        return '%s<x%s>' % (self.NAME, self.un.hex().upper())

    def __str__(self) -> str:
        return self.__repr__()

    def format(self, fmt:str) -> str:
        return self.__repr__()

    def decode(self, ba:bytes, off:int) -> int:
        n = self.n
        self.un = ba[off:off+n]
        return off+n

    def encode(self, ba:bytearray, off:int) -> int:
        n = self.n
        ba[off:off+n] = self.un
        return off+n


# ================================================================================
# Reset Conf/Ind  --- ABP only ---
# ================================================================================

class ResetInd(OptUp):
    CMD = 1
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(BF('Minor',(0,3)), RFU(4,7)), **kwargs)

class ResetConf(OptDn):
    CMD = 1
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(BF('Minor',(0,3)), RFU(4,7)), **kwargs)


# ================================================================================
# LinkCheck Req/Ans
# ================================================================================

class LinkCheckReq(OptUp):
    CMD = 2

class LinkCheckAns(OptDn):
    CMD = 2
    def __init__(self, **kwargs) -> None:
        super().__init__(
            U8(BF('Margin',(0,7))),
            U8(BF('GwCnt', (0,7))), **kwargs)


# ================================================================================
# LinkADR Req/Ans
# ================================================================================

class LinkADRReq(OptDn):
    CMD = 3
    def __init__(self, **kwargs) -> None:
        super().__init__(
            U8 ( BF ('TXPow',(0,3)), BF('DR',(4,7)) ),
            U16( BFx('ChMask',(0,15)) ),
            U8 ( BF ('NbTrans', (0,3)), BF('ChMaskCntl', (4,6)), RFU(7) ), **kwargs)

class LinkADRAns(OptUp):
    CMD = 3
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(Ack('ChAck',0), Ack('DRAck',1), Ack('TXPowAck',2), RFU(3,7)), **kwargs)


# ================================================================================
# DutyCycle Req/Ans
# ================================================================================

class DutyCycleReq(OptDn):
    CMD = 4
    def __init__(self, **kwargs) -> None:
        super().__init__(
            # dc = 2^-MaxDC  (aggr. over all channels)
            U8( BF('MaxDC',(0,3), RFU(4,7)) ), **kwargs)

class DutyCycleAns(OptUp):
    CMD = 4


# ================================================================================
# RXParamSetup Req/Ans
# ================================================================================

class RXParamSetupReq(OptDn):
    CMD = 5
    def __init__(self, **kwargs) -> None:
        super().__init__(
            U8 ( BF('RX2DR',(0,3)), BF('RX1DRoff',(4,6)), RFU(7) ),
            U24( Freq() ), **kwargs)

class RXParamSetupAns(OptUp):
    CMD = 5
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(Ack('FreqAck',0), Ack('RX2DRAck',1), Ack('RX1DRoffAck',2), RFU(3,7)), **kwargs)


# ================================================================================
# DevStatus Req/Ans
# ================================================================================

class DevStatusReq(OptDn):
    CMD = 6

class DevStatusAns(OptUp):
    CMD = 6
    def __init__(self, **kwargs) -> None:
        super().__init__(
            U8(BF('Batt',  (0,7))),
            U8(BF('Margin',(0,5), signed=True), RFU(6,7)), **kwargs)


# ================================================================================
# NewChannel Req/Ans
# ================================================================================

class NewChannelReq(OptDn):
    CMD = 7
    def __init__(self, **kwargs) -> None:
        super().__init__(
            U8 ( BF('Chnl',(0,7)) ),
            U24( Freq() ),
            U8 ( BF('MinDR',(0,3)), BF('MaxDR',(4,7)) ), **kwargs)

class NewChannelAns(OptUp):
    CMD = 7
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(Ack('ChnlAck',0), Ack('DRAck',1), RFU(2,7)), **kwargs)


# ================================================================================
# RXTimingSetup Req/Ans
# ================================================================================

class RXTimingSetupReq(OptDn):
    CMD = 8
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( BF('Delay',(0,3)), RFU(4,7) ), **kwargs)

class RXTimingSetupAns(OptUp):
    CMD = 8


# ================================================================================
# TXParamSetup Req/Ans
# ================================================================================

class MaxEIRP(BF):
    def __init__(self, name:str='MaxEIRP', bits:Union[int,Tuple[int,int]]=(0,3)) -> None:
        super().__init__(name,bits)
    def value_as_str(self, fmt:str='s') -> str:
        v = self.value_as_int()
        return '%d(%ddBm)' % (v,[8,10,12,13,14,16,18,20,21,24,26,27,29,30,33,36][v])

class TXParamSetupReq(OptDn):
    CMD = 9
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( MaxEIRP(), BF('UpDwell',4), BF('DnDwell',5), RFU(6,7) ), **kwargs)

class TXParamSetupAns(OptUp):
    CMD = 9


# ================================================================================
# DlChannel Req/Ans
# ================================================================================

class DlChannelReq(OptDn):
    CMD = 10
    def __init__(self, **kwargs) -> None:
        super().__init__( U8( BF('Chnl',(0,8)) ), U24(Freq()), **kwargs )

class DlChannelAns(OptUp):
    CMD = 10
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(Ack('FreqAck',0), Ack('ChnlAck',1), RFU(2,7)), **kwargs)


# ================================================================================
# Rekey Ind/Conf  --- OTAA only ---
# ================================================================================

class RekeyIndOLD(OptUp):
    """Old TrackNet impl of RekeyInd based on an intermediate spec."""
    CMD = 0x10B   # disambiguate from correct option
    SPECIAL = b'\x0B\x11\x00\x03'
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(BF('Minor',(0,3)), BF('Major',(4,7))),
                          U8(BF('Nonce',(0,7))),
                          U8(BF('DUOFCNT',0), BF('FCNT32',1), BF('RFU',(2,3)), BF('Extra',(4,7))), **kwargs)

class RekeyInd(OptUp):
    CMD = 11
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( BF('Minor',(0,3)), RFU(4,7) ), **kwargs)

class RekeyConf(OptDn):
    CMD = 11
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( BF('Minor',(0,3)), RFU(4,7) ), **kwargs)


# ================================================================================
# ADRParamSetup Req/Ans
# ================================================================================

class ADRParamSetupReq(OptDn):
    CMD = 12
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( BF('Delay',(0,3)), BF('Limit',(4,7)) ), **kwargs)

class ADRParamSetupAns(OptUp):
    CMD = 12


# ================================================================================
# DeviceTime Req/Ans
# ================================================================================

class DeviceTimeReq(OptUp):
    CMD = 13

class DeviceTimeAns(OptDn):
    CMD = 13
    def __init__(self, **kwargs) -> None:
        # time = epoch_secs + frac/2^8
        super().__init__(U32( BF('epoch_secs',(0,31)) ), U8( BF('frac',(0,7)) ), **kwargs)


# ================================================================================
# ForceRejoin Req
# ================================================================================

class ForceRejoinReq(OptDn):
    CMD = 14
    def __init__(self, **kwargs) -> None:
        super().__init__(U16( BF('DR',(0,3)),
                              BF('RejoinType',(4,6)),
                              BF('MaxRetries',(8,10)),
                              BF('Period',(11,13)),
                              RFU(14,15) ), **kwargs)


# ================================================================================
# RejoinParam Req/Ans
# ================================================================================

class RejoinParamReq(OptDn):
    CMD = 15
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( BF('MaxCountN',(0,3)), BF('MaxTimeN',(4,7)) ), **kwargs)

class RejoinParamAns(OptUp):
    CMD = 15
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(Ack('TimeAck',0), RFU(1,7)), **kwargs)


# ================================================================================
# PingSlotInfo Req/Ans
# ================================================================================

class Intv(BF):
    def __init__(self, name:str='Intv', bits:Union[int,Tuple[int,int]]=(0,2)) -> None:
        super().__init__(name,bits)
    def value_as_str(self, fmt:str='s') -> str:
        v = self.value_as_int()
        return '%d(%ds)' % (v,1<<v)

class PingSlotInfoReq(OptUp):
    CMD = 16
    def __init__(self, **kwargs) -> None:
        # periodicity: 2^Intv secs
        super().__init__(U8( Intv(bits=(0,2)), RFU(3,7) ), **kwargs)

class PingSlotInfoAns(OptDn):
    CMD = 16


# ================================================================================
# PingSlotChnl Req/Ans
# ================================================================================

class PingSlotChnlReq(OptDn):
    CMD = 17
    def __init__(self, **kwargs) -> None:
        super().__init__(
            U24( Freq() ),
            U8 ( BF('DR',(0,3)), RFU(4,7) ), **kwargs)

class PingSlotChnlAns(OptUp):
    CMD = 17
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(Ack('FreqAck',0), Ack('DRAck',1), RFU(2,7)), **kwargs)


# ================================================================================
# BeaconTiming Req/Ans
# ================================================================================

class BeaconTimingReq(OptUp):
    CMD = 18

class BeaconTimingAns(OptDn):
    CMD = 18
    def __init__(self, **kwargs) -> None:
        # deprecated as of 1.1
        super().__init__(U16( BF('Delay',(0,15)) ), U8( BF('Chnl',(0,7)) ), **kwargs)


# ================================================================================
# BeaconFreq Req/Ans
# ================================================================================

class BeaconFreqReq(OptDn):
    CMD = 19
    def __init__(self, **kwargs) -> None:
        super().__init__(U24( Freq() ), **kwargs)

class BeaconFreqAns(OptUp):
    CMD = 19
    def __init__(self, **kwargs) -> None:
        super().__init__( U8(Ack('FreqAck',0), RFU(1,7)), **kwargs)


# ================================================================================
# DeviceMode Ind/Conf
# ================================================================================

class DeviceModeInd(OptUp):
    CMD = 32
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( BF('DevClass',(0,7)) ), **kwargs)

class DeviceModeConf(OptDn):
    CMD = 32
    def __init__(self, **kwargs) -> None:
        super().__init__(U8( BF('DevClass',(0,7)) ), **kwargs)



# ================================================================================
# ----- common lookup tables -----
# ================================================================================

T_CtorOpt = Callable[[],Opt]
T_CtorOptMap = Dict[int,T_CtorOpt]

def _scan_classes(refcls:T_CtorOpt) -> T_CtorOptMap:
    d = {} # type: T_CtorOptMap
    type_refcls = type(refcls)
    for name,cls in globals().items():
        if type(cls) is not type_refcls or cls is refcls or not issubclass(cls,refcls): # type: ignore
            continue
        if not cls.NAME:     # pragma:nobranch
            cls.NAME = name
        d[cls.CMD] = cls
    return d


OPTSUP = _scan_classes(OptUp)
OPTSDN = _scan_classes(OptDn)


def unpack_opts(opts:Union[str,bytes], D:T_CtorOptMap) -> List[Opt]:
    if isinstance(opts,str):
        binopts = bytes.fromhex(opts)
    else:
        binopts = opts
    res = [] # type: List[Opt]
    off = 0
    n = len(binopts)
    while off < n:
        offini = off
        cmd = binopts[off]
        optcls = D.get(cmd)
        if optcls is None:
            opt = Unknown(n-off)  # type: Opt
        else:
            if optcls is RekeyInd and binopts[off:off+4] == RekeyIndOLD.SPECIAL:
                opt = RekeyIndOLD()
            else:
                opt = optcls()
        off = opt.decode(binopts,off)
        res.append(opt)
    return res

def unpack_optsup(opts:Union[str,bytes]) -> List[Opt]:
    return unpack_opts(opts, OPTSUP)

def unpack_optsdn(opts:Union[str,bytes]) -> List[Opt]:
    return unpack_opts(opts, OPTSDN)

def pack_opts(opts:List[Opt]) -> bytes:
    ba = bytearray(256)
    off = 0
    for o in opts:
        off = o.encode(ba,off)
    return bytes(ba[0:off])

