# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

from typing import Dict,List,NamedTuple,Tuple,Union
import re
import struct
from bisect import bisect
import rtlib

# Join accept RX1 delay - same accross all regions
JaccRxDelay = 5

ChDef = NamedTuple('ChDef', [('freq',int),('minDR',int),('maxDR',int)])

NoneChDef = ChDef(0,0,0)


MAX_EIRPs = [dBm-0.0001 for dBm in [8,10,12,13,14,16,18,20,21,24,26,27,29,30,33,36,1000000]]

def encode_max_eirp(max_eirp:float) -> int:
    return max(0, bisect(MAX_EIRPs, max_eirp)-1)


class DRxDef():
    def __init__(self,dr:int,sf:int,bw:int,dn:int=0) -> None:
        self.dr = dr
        self.sf = sf
        self.bw = bw
        self.dn = dn    # used for down link only
        if self.sf == -1:
            self.name = "DR%d:RFU" % self.dr
        elif self.sf == 0:
            self.name = "DR%d:FSK" % self.dr
        else:
            self.name = "DR%d:SF%dBW%d" % (self.dr, self.sf, self.bw)
    
    @property
    def invalid(self) -> bool:
        return (self.sf == -1)
    
    @property
    def isFSK(self) -> bool:
        return self.sf==0
    
    def __str__(self) -> str:
        return self.name

NONE_DR = DRxDef(0,-1,0)
EU_DRs = [
    DRxDef( 0,12,125),
    DRxDef( 1,11,125),
    DRxDef( 2,10,125),
    DRxDef( 3, 9,125),
    DRxDef( 4, 8,125),
    DRxDef( 5, 7,125),
    DRxDef( 6, 7,250),
    DRxDef( 7, 0,  0), # FSK
]
US_DRs = [
    # UP data rates
    DRxDef( 0,10,125),
    DRxDef( 1, 9,125),
    DRxDef( 2, 8,125),
    DRxDef( 3, 7,125),
    DRxDef( 4, 8,500),
    # DN data rates
    DRxDef( 8,12,500,1),
    DRxDef( 9,11,500,1),
    DRxDef(10,10,500,1),
    DRxDef(11, 9,500,1),
    DRxDef(12, 8,500,1),
    DRxDef(13, 7,500,1),
]
AU_DRs = [
    # UP data rates
    DRxDef( 0,12,125),
    DRxDef( 1,11,125),
    DRxDef( 2,10,125),
    DRxDef( 3, 9,125),
    DRxDef( 4, 8,125),
    DRxDef( 5, 7,125),
    DRxDef( 6, 8,500),
    # DN data rates
    DRxDef( 8,12,500,1),
    DRxDef( 9,11,500,1),
    DRxDef(10,10,500,1),
    DRxDef(11, 9,500,1),
    DRxDef(12, 8,500,1),
    DRxDef(13, 7,500,1),
]


def mkchlist(f:Union[int,float],n:int=1,d:int=200,minDR:int=0,maxDR:int=5,DR:int=-1):
    if DR>=0: minDR=maxDR=DR
    if abs(d) < 2000:
        d = int(d*1e3)
    if f < 10000:
        f = int(f*1e6)
    return [ ChDef(int(f+i*d), minDR, maxDR) for i in range(n) ]


def chmask_from_json(j:List[int]) -> int:
    return j[0] | (j[1]<<64)

def chmask_to_json(chmask:int) -> List[int]:
    return [chmask & ((1<<64)-1), chmask>>64]


class Region(object):
    def __init__(self,
                 regionid:int,
                 name:str,
                 upchannels:List[ChDef],
                 drs:List[DRxDef],
                 num_updr:int,
                 num_rx1droff:int,
                 txpow_steps:int=8,
                 adr_params:Tuple[float,float]=(-9.0,2.75)) -> None:
        self.regionid = regionid
        self.name = name          # region base name
        self.setup = ''
        self.remark = ''
        self.freq_range = [0,0]   # override in subclass
        self.RX2DR = 0
        self.PingDR = 0
        self.RX2Freq = 0   # override in subclass
        self.RxDelay = 1
        self.PingFreq = 0
        self.BCNFreq = []  # type: List[int]
        self.BCNDR = 0
        self.bcn_layout = (0,0,0)  # offset time & info, len -- see loramsg.pack_beacon
        self.upDwellTime = 0
        self.dnDwellTime = 0
        self.dnfreqs = [0]*16
        self.dnfreq_map = {}  # type: Dict[int,int]
        self.maxDR125 = 0  # max default DR (fastest with BW=125kHz)
        self.DRs = [ DRxDef(i,-1,  0) for i in range(16) ]
        for dr in drs:
            self.DRs[dr.dr] = dr
            if not dr.invalid and not dr.dn and dr.bw==125:
                self.maxDR125 = dr.dr
        self.sfbw2dr = { (dr.sf, dr.bw):dr for dr in self.DRs if not dr.dn and dr.sf >= 0 }
        self.txpow = [ -2*i for i in range(txpow_steps) ]
        # map datarate offset to real values - array defined only for valid offsets
        self.effectiveRX1DROffsets = [off for off in range(num_rx1droff)]
        self.num_updr = num_updr
        # snr2dr[dr] => SNR threshold for datarate dr
        # - use dr upto this SNR value
        # - if SNR bigger move to next faster DR
        # - fastest datarate has a biiiigly value snr2dr[num_updr-1]=100.0
        self.snr2dr = [ adr_params[0] + adr_params[1]*i for i in range(num_updr-1) ] + [ 100.0 ]
        self.std_upchannels = len(upchannels)    # default defined
        self.upchannels = upchannels
        self.cflist_type = 0
        self.max_eirp = 16.0    # dBm
        self.duty_cycle = 0.0   # none or e.g. 1/100.0
        self.avail_updr = 0     # type: int
        self.set_chmask()       # all ON + calc avail_updr
        self.max_pdulens   = [250]*16   # default max PDU lengths - without repeater option
        self.max_pdulens_R = [230]*16   #  ditto repeater compatible


    def __str__(self) -> str:
        return self.name+'/'+self.setup if self.setup else self.name

    @property
    def is_intermittent(self) -> bool:
        return self.regionid > 0x10000

    @property
    def base_region(self) -> 'Region':
        return REGIONS[self.name]

    def set_dnfreqs(self, dnfreqs:List[int]) -> None:
        dnlen = min(len(dnfreqs), len(self.upchannels))
        dnfreqs = dnfreqs[:dnlen]
        self.dnfreqs = dnfreqs + [0] * (16-dnlen)   # fewer corner cases
        dnfreq_map = {} # type: Dict[int,int]
        for chidx, dnfreq in enumerate(dnfreqs):
            if not dnfreq: continue
            dnfreq_map[self.upchannels[chidx].freq] = dnfreq
        self.dnfreq_map = dnfreq_map

    def set_chmask(self, chmask:int=0) -> None:
        """Recalculated available DRs (bit mask with 16 bits). Depends on available/defined/enabled channels."""
        if not chmask:
            chmask = self.chmask_allON
        self.chmask = chmask
        minDR = 16
        maxDR = -1
        updr_mask = 0
        for chidx,chdef in enumerate(self.upchannels):
            if (chmask & (1<<chidx)) != 0 and (chdef.minDR < minDR or chdef.maxDR > maxDR):
                updr_mask |= (0xFFFF << chdef.minDR) & (0xFFFF >> (15-chdef.maxDR))
        self.avail_updr = updr_mask

    def is_same(self, other:'Region') -> bool:
        return (self.name        == other.name and         # quick check for base region
                self.upchannels  == other.upchannels and
                self.chmask      == other.chmask and
                self.dnfreqs     == other.dnfreqs and
                self.RX2Freq     == other.RX2Freq and
                self.PingFreq    == other.PingFreq and
                self.max_eirp    == other.max_eirp and
                self.upDwellTime == other.upDwellTime and
                self.dnDwellTime == other.dnDwellTime)

    def check_freq(self, freq:int, exc:bool=True) -> bool:
        if freq < self.freq_range[0] or freq > self.freq_range[1]:
            if not exc: return False
            raise ValueError('Frequency %s not in range: [%s,%s]'
                             % (rtlib.freq_to_str(freq),
                                rtlib.freq_to_str(self.freq_range[0]),
                                rtlib.freq_to_str(self.freq_range[1])))
        return True

    def map_snr2dr(self, measured_snr:float) -> int:
        for thres_idx, thres_snr in enumerate(self.snr2dr):
            if measured_snr < thres_snr:
                break
        return thres_idx

    def fix_updr(self, updr:int, delta:int) -> int:
        """Move given up datarate by delta notches. Delta > 0 towards faster datarates.
        Used in ADR decisions in network server.
        """
        while delta != 0:
            if delta < 0:
                new_updr = max(0,updr+delta)
            else:
                new_updr = min(self.num_updr-1,updr+delta)
            # Never use FSK - LMIC does not implement it
            if (self.avail_updr & (1<<new_updr)) != 0 and not self.DRs[new_updr].isFSK:
                return new_updr
            # DR not available - (reduced) channel plan
            delta += 1 if delta < 0 else -1
        return updr

    def to_dr (self, sf:int, bw:int) -> DRxDef:
        """Convert low level Lora radio parameters into a datarate."""
        if bw >= 1000: bw //= 1000
        return self.sfbw2dr.get((sf,bw),NONE_DR)

    def get_dnfreq(self, freq:int) -> int:
        """Compute down frequency from the up frequency."""
        return self.dnfreq_map.get(freq,freq)
    
    def get_dndr(self, updr:int, rx1droffset:int) -> int:
        """Compute DN data rate from UP data rate and RX1DROffset."""
        return self.fix_updr(updr, -self.effectiveRX1DROffsets[rx1droffset])

    def get_upchannels(self) -> List[ChDef]:
        return self.upchannels
    
    def get_enabled_upchannels(self) -> List[ChDef]:
        chmask = self.chmask
        return [chdef for chidx,chdef in enumerate(self.upchannels) if (chmask & (1<<chidx)) != 0 ]
    
    def get_cflist(self) -> bytes:
        return b''

    def mcmds_linkadr_chmask(self, foptsdn:bytearray, off:int, dr:int, txpow:int=0,
                             chmask:int=0, optneg:int=None) -> int:
        return 0

    @property
    def chmask_allON(self) -> int:
        return (1<<len(self.upchannels))-1



class RegionLikeEU(Region):
    LinkADR_ALLON = 6

    def __init__(self, drs=EU_DRs, num_updr=8, rx2freq:int=0, **kwargs) -> None:
        super().__init__(drs=drs[:num_updr], num_updr=num_updr, **kwargs)
        self.max_pdulens   = [59,59,59,123,250,250,250,250]
        self.max_pdulens_R = [59,59,59,123,230,230,230,230]
        self.RX2Freq  = rx2freq
        self.PingFreq = rx2freq
        self.BCNFreq  = [rx2freq]
        self.RX2DR = 0
        self.PingDR = 0
        self.BCNDR = 3
        self.bcn_layout = (2,8,17)

    def get_cflist(self) -> bytes:
        upchannels = self.upchannels
        nup = len(upchannels)
        stdup = self.std_upchannels
        if stdup == nup:
            return b''
        freq_u3 = [ struct.pack('<I', upchannels[chidx].freq//100)[0:3]
                    for chidx in range(stdup, min(stdup+5, nup)) ]
        # Padding + CFList type 0
        freq_u3.append(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
        return (b''.join(freq_u3))[0:16]

    def mcmds_linkadr_chmask(self, foptsdn:bytearray, off:int, dr:int, txpow:int=0,
                             chmask:int=0, optneg:int=None) -> int:
        if not chmask: chmask = self.chmask_allON
        struct.pack_into('<BBHB', foptsdn, off, 3, (dr<<4)|txpow, chmask & 0xFFFF, 0)
        return 1


class Region_EU868(RegionLikeEU):
    def __init__(self) -> None:
        # For test short cuts - EU868 should be #1
        super().__init__(regionid=1, name='EU868',
                         upchannels=mkchlist(f=868.1,n=3,maxDR=5), num_updr=8,
                         num_rx1droff=6, txpow_steps=8,
                         rx2freq=869525000)
        self.max_eirp = 16.0  # dBm
        self.duty_cycle = 1/1000.0   # good for any channel; some channels go higher
        self.freq_range = [863000000,870000000]


class Region_EU433(RegionLikeEU):
    def __init__(self) -> None:
        super().__init__(regionid=3, name='EU433',
                         upchannels=mkchlist(f=433.175,n=3,maxDR=5), num_updr=8,
                         num_rx1droff=7, txpow_steps=8,
                         rx2freq=434665000)
        self.max_eirp = 12.15  # dBm
        self.freq_range = [433175000,434665000]


class RegionLikeUS(Region):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.cflist_type = 1

    def get_cflist(self) -> bytes:
        chmask = self.chmask
        if self.chmask_allON == chmask:
            return b''
        off = 0
        buf = bytearray([0]*15 + [1])    # CFList type 1
        while chmask:
            struct.pack_into('<H', buf, off, chmask & 0xFFFF)
            chmask >>= 16
            off += 2
        return bytes(buf)


def mcmds_linkadr_chmask_likeUS915(reg:Union['Region_US915','Region_AU915'],
                                   foptsdn:bytearray, off:int, dr:int, txpow:int=0,
                                   chmask:int=0, optneg:int=None) -> int:
    if not chmask: chmask = reg.chmask_allON
    if chmask == reg.chmask_allON:
        struct.pack_into('<BBHB', foptsdn, off, 3, (dr<<4)|txpow, (chmask>>64)&0xFF, reg.LinkADR_125ON<<4)
        return 1
    if optneg is not None:
        # LoRa WAN > 1.0 - new ctrl command for blocks of 8+1 channels
        blk8 = 0
        for i in range(8):
            m = (0xFF<<(i*8))|(1<<(64+i))  # channel mask for block i
            c = chmask & m
            if c == m:
                blk8 |= 1<<i
            elif c != 0: # Odd block - can't construct map with 8+1 blocks
                blk8 = 0
                break
        if blk8 > 0:
            # chmaks is a set of blocks
            struct.pack_into('<BBHB', foptsdn, off, 3, (dr<<4)|txpow, blk8, reg.LinkADR_BLK8ON<<4)
            return 1
    # chmask is not a set of blocks but something odd
    # (1) turn off all 125kHz and enable desird 500kHz channels
    # (2) then enable groups of 16x 125kHz channel if at least one channel used
    struct.pack_into('<BBHB', foptsdn, off, 3, (dr<<4)|txpow, (chmask>>64) & 0xFFFF, reg.LinkADR_125OFF<<4)
    ncmds = 1
    for j in range(0, 64, 16):
        mask = (chmask >> j) & 0xFFFF
        if mask:
            struct.pack_into('<BBHB', foptsdn, off+ncmds*5, 3, (dr<<4)|txpow, mask, (j//16)<<4)
            ncmds += 1
    return ncmds


class Region_US915(RegionLikeUS):
    LinkADR_BLK8ON = 5  # v1.1 - enable blocks (block = 8x125kHz + 1x500kHz channels)
    LinkADR_125ON  = 6
    LinkADR_125OFF = 7
    
    def __init__(self):
        # For test short cuts - US903 should be #2
        super().__init__(regionid=2, name='US915',
                         upchannels=mkchlist(f=902.3,n=64,maxDR=3) + mkchlist(f=903.0,n=8,d=1600,DR=4),
                         drs=US_DRs, num_updr=5, num_rx1droff=4, txpow_steps=11, adr_params=(-3.5,2.75))
        self.max_eirp = 30.0
        self.freq_range = [902000000,928000000]
        self.RX2Freq = 923300000
        self.RX2DR  = 8
        self.PingDR = 8
        self.BCNDR  = 8
        self.BCNFreq = [923300000 + chx * 600000 for chx in range(8)]
        self.bcn_layout = (5,11,23)
        self.max_pdulens   = [19,61,133,250,250,None,None,None,61,137,250,250,250,250]
        self.max_pdulens_R = [19,61,133,230,230,None,None,None,41,117,230,230,230,230]

    def mcmds_linkadr_chmask(self, foptsdn:bytearray, off:int, dr:int, txpow:int=0,
                             chmask:int=0, optneg:int=None) -> int:
        return mcmds_linkadr_chmask_likeUS915(self, foptsdn, off, dr, txpow, chmask, optneg)

    def get_dnfreq(self, freq:int) -> int:
        '''Compute down frequency from the up frequency.'''
        assert freq >= 902300000 and freq <= 914900000
        chx = (freq - 902300000) // 100000
        chx = (chx // (2 if (chx&1)==0 else 16)) % 8
        return 923300000 + chx * 600000

    def get_dndr(self, updr:int, rx1droffset:int) -> int:
        '''Compute DN data rate from UP data rate and RX1DROffset.'''
        return max(8, min(updr + 10 - self.effectiveRX1DROffsets[rx1droffset], 13))


class Region_AU915(RegionLikeUS):
    LinkADR_BLK8ON = 5  # enable blocks (block = 8x125kHz + 1x500kHz channels)
    LinkADR_125ON  = 6
    LinkADR_125OFF = 7

    def __init__(self):
        super().__init__(regionid=4, name='AU915',
                         upchannels=mkchlist(f=915.2,n=64,maxDR=5) + mkchlist(f=915.9,n=8,d=1600,DR=6),
                         drs=AU_DRs, num_updr=7, num_rx1droff=6, txpow_steps=11, adr_params=(-9.0,2.75))
        self.max_eirp = 30.0
        self.freq_range = [915000000,928000000]
        self.RX2Freq = 923300000
        self.RX2DR = 8
        self.PingDR = 8
        self.BCNDR = 10
        self.BCNFreq = [923300000 + chx * 600000 for chx in range(8)]
        self.bcn_layout = (3,9,19)
        self.max_pdulens   = [59,59,59,123,250,250,250,None,61,137,250,250,250,250]
        self.max_pdulens_R = [59,59,59,123,230,230,230,None,41,117,230,230,230,230]
    
    def mcmds_linkadr_chmask(self, foptsdn:bytearray, off:int, dr:int, txpow:int=0,
                             chmask:int=0, optneg:int=None) -> int:
        return mcmds_linkadr_chmask_likeUS915(self, foptsdn, off, dr, txpow, chmask, optneg)

    def get_dnfreq(self, freq:int) -> int:
        """Compute down frequency from the up frequency."""
        assert freq >= 915200000 and freq <= 927800000
        chx = (freq - 915200000) // 100000
        chx = (chx // (2 if (chx&1)==0 else 16)) % 8
        return 923300000 + chx * 600000
    
    def get_dndr(self, updr:int, rx1droffset:int) -> int:
        '''Compute DN data rate from UP data rate and RX1DROffset.'''
        return max(8, min(updr + 8 - self.effectiveRX1DROffsets[rx1droffset], 13))


class Region_CN470(RegionLikeUS):
    LinkADR_ALLON = 6

    def __init__(self):
        super().__init__(regionid=5, name='CN470',
                         upchannels=mkchlist(f=470.3,n=96,maxDR=5),
                         drs=EU_DRs[:6], num_updr=6, num_rx1droff=6, txpow_steps=8)
        self.max_eirp = 19.15
        self.freq_range = [470000000,510000000]
        self.RX2Freq = 505300000
        self.RX2DR = 0
        self.PingDR = 0
        self.BCNDR = 2
        self.BCNFreq = [508300000 + chx * 200000 for chx in range(8)]
        self.bcn_layout = (3,9,19)
        self.max_pdulens   = [59,59,59,123,250,250]
        self.max_pdulens_R = [59,59,59,123,230,230]
    
    def mcmds_linkadr_chmask(self, foptsdn:bytearray, off:int, dr:int, txpow:int=0,
                             chmask:int=0, optneg:int=None) -> int:
        if not chmask: chmask = self.chmask_allON
        struct.pack_into('<BBHB', foptsdn, off, 3, (dr<<4)|txpow, 0, Region_CN470.LinkADR_ALLON<<4)
        ncmds = 1
        for i in range(0, 96, 16):
            mask = (chmask >> i) & 0xFFFF
            if mask != 0xFFFF:
                struct.pack_into('<BBHB', foptsdn, off+5*ncmds, 3, (dr<<4)|txpow, mask, (i//16)<<4)
                ncmds += 1
        return ncmds

    def get_dnfreq(self, freq:int) -> int:
        """Compute down frequency from the up frequency."""
        assert freq >= 470200000 and freq < 489400000
        return 500300000 + ((freq - 470200000) // 200000 % 48) * 200000


class Region_CN779(RegionLikeEU):
    def __init__(self) -> None:
        super().__init__(regionid=6, name='CN779',
                         upchannels=mkchlist(f=779.5,n=3,maxDR=5),
                         num_updr=8, num_rx1droff=6, txpow_steps=6,
                         rx2freq=786000000)
        self.max_eirp = 12.15  # dBm
        self.duty_cycle = 1/100.0
        self.freq_range = [779000000,787000000]
        self.BCNFreq = [785000000]  # not the same as RX2Freq!
        self.PingFreq = 785000000


class Region_AS923(RegionLikeEU):
    # These countries cover regionids 30-49 (up to 39 in use)
    COUNTRIES = [
        # country code
        #     regionid
        #           freq_range
        #                   country name
        ('BN', 30, [923000000,925000000], 'Brunei'),
        ('KH', 31, [923000000,925000000], 'Cambodia'),
        ('ID', 32, [923000000,925000000], 'Indonesia'),
        ('JP', 33, [920000000,928000000], 'Japan'),
        ('LA', 34, [923000000,925000000], 'Laos'),
        ('NZ', 35, [915000000,928000000], 'New Zealand'),
        ('SG', 36, [920000000,925000000], 'Singapore'),
        ('TW', 37, [922000000,928000000], 'Taiwan'),
        ('TH', 38, [920000000,925000000], 'Thailand'),
        ('VN', 39, [920000000,925000000], 'Vietnam'),
    ]

    def __init__(self, regionid=7, country:str='', freq_range:List[int]=None, remark='') -> None:
        super().__init__(regionid=regionid, name='AS923'+country,
                         upchannels=mkchlist(f=923.2,n=2,maxDR=5),
                         num_updr=8, num_rx1droff=8, txpow_steps=8)
        self.remark = remark
        self.effectiveRX1DROffsets[-2:] = [-1,-2]
        self.max_eirp = 16.0  # dBm
        self.duty_cycle = 1/1000.0
        if freq_range:
            self.freq_range = freq_range
        if not country:
            self.freq_range = [min(map(lambda v:v[2][0],Region_AS923.COUNTRIES)),
                               max(map(lambda v:v[2][1],Region_AS923.COUNTRIES))]
        self.RX2Freq = 923200000
        self.RX2DR = 2
        self.PingDR = 2
        self.BCNFreq = [923400000]
        self.PingFreq = 923400000
        self.BCNDR = 3
        self.bcn_layout = (2,8,17)
        self.mindr = 0  # if dwell time limit of 400ms then 2
    
    def get_dndr(self, updr:int, rx1droffset:int) -> int:
        '''Compute DN data rate from UP data rate and RX1DROffset.'''
        return min(5, max(self.mindr, updr - self.effectiveRX1DROffsets[rx1droffset]))


class Region_KR920(RegionLikeEU):
    def __init__(self) -> None:
        super().__init__(regionid=8, name='KR920',
                         upchannels=mkchlist(f=922.1,n=3,maxDR=5),
                         num_updr=6, num_rx1droff=6, txpow_steps=8)
        self.max_eirp = 23.0   # dBm - gateway
        self.duty_cycle = 0.0  # none
        self.freq_range = [920900000,923300000]
        self.RX2Freq = 921900000
        self.RX2DR = 0
        self.PingDR = 0
        self.BCNFreq = [923100000]
        self.PingFreq = 923100000
        self.BCNDR = 3
        self.bcn_layout = (2,8,17)


class Region_IN865(RegionLikeEU):
    def __init__(self) -> None:
        super().__init__(regionid=9, name='IN865',
                         upchannels=[ChDef(865062500,0,5),ChDef(865402500,0,5),ChDef(865985000,0,5)],
                         num_updr=6, num_rx1droff=8, txpow_steps=11,
                         rx2freq=866550000)
        self.effectiveRX1DROffsets[-2:] = [-1,-2]
        self.max_eirp = 30.0         # dBm
        self.duty_cycle = 0.0        # good for any channel; some channels go higher
        self.freq_range = [865000000,867000000]
        self.RX2DR = 2
        self.PingDR = 2
        self.BCNDR = 4
        self.bcn_layout = (1,7,19)
    
    def get_dndr(self, updr:int, rx1droffset:int) -> int:
        '''Compute DN data rate from UP data rate and RX1DROffset.'''
        return min(5, max(0, updr - self.effectiveRX1DROffsets[rx1droffset]))


class Region_IL915(RegionLikeEU):
    def __init__(self) -> None:
        super().__init__(regionid=10, name='IL915',
                         upchannels=[ChDef(915700000,0,5),ChDef(915900000,0,5),ChDef(916100000,0,5)],
                         num_updr=8, num_rx1droff=6, txpow_steps=8,
                         rx2freq=916300000)
        self.max_eirp = 14.0         # dBm
        self.duty_cycle = 1/100.0    # good for any channel; some channels go higher
        self.freq_range = [915000000,917000000]
        self.RX2DR = 0
        self.PingDR = 0
        self.BCNDR = 3
        self.bcn_layout = (2,8,17)
    
    def get_dndr(self, updr:int, rx1droffset:int) -> int:
        '''Compute DN data rate from UP data rate and RX1DROffset.'''
        return min(5, max(0, updr - self.effectiveRX1DROffsets[rx1droffset]))


class Region_RU864(RegionLikeEU):
    def __init__(self) -> None:
        super().__init__(regionid=11, name='RU864',
                         upchannels=[ChDef(868900000,0,5),ChDef(869100000,0,5)],
                         num_updr=7, num_rx1droff=8, txpow_steps=8,
                         rx2freq=869100000)
        self.effectiveRX1DROffsets[-2:] = [-1,-2]
        self.max_eirp = 16.0         # dBm
        self.duty_cycle = 0.0        # good for any channel; some channels go higher
        self.freq_range = [864000000,870000000]
        self.PingFreq = 868900000



NoneRegion = Region(regionid=0, name='None',
                    upchannels=[], drs=[],
                    num_updr=0, num_rx1droff=0)

EU868 = Region_EU868()
EU433 = Region_EU433()
US915 = Region_US915()
AU915 = Region_AU915()
CN470 = Region_CN470()
CN779 = Region_CN779()
AS923 = Region_AS923()
KR920 = Region_KR920()
IN865 = Region_IN865()
IL915 = Region_IL915()
RU864 = Region_RU864()

REGIONS = {
    'EU868'  : EU868,
    'EU863'  : EU868,  # alias
    'EU433'  : EU433,
    'US915'  : US915,
    'US902'  : US915,  # alias
    'AU915'  : AU915,
    'CN470'  : CN470,
    'CN779'  : CN779,
    'AS923'  : AS923,
    'KR920'  : KR920,
    'IN865'  : IN865,
    'IL915'  : IL915,
    'RU864'  : RU864,
}

for country, regionid, freq_range, country_name in Region_AS923.COUNTRIES:
    fn = lambda: Region_AS923(regionid, country, freq_range, country_name)
    region = fn()
    globals()['Region_AS923'+country] = fn
    globals()['AS923'+country] = region
    REGIONS[region.name] = region


