# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

"""Encoding, decoding, encryption and verification of LoRa messages.
"""

from typing import cast,Optional,Tuple,Union
import math
import struct

from loracrypto import crypto
import rtlib
from rtlib.types import Conf, Msg, State


# Key derivation identifiers
KD_DevAddr    = 240 # proprietary use - derive 32bit DevAddr 
KD_NwkSKeyUp  = 1   # v1.1
KD_NwkSKeyDwn = 3   # spec 1.1 only
KD_NwkSKey    = 1   # spec 1.0 only
KD_AppSKey    = 2   # (de)cipher FRMPayload v1.0+v1.1


class VerifyError(Exception):
    """Exception thrown if verification of message integrity code (MIC) fails on a frame."""
    pass



class MHdr(object):
    FTYPE   = 0xE0
    RFU     = 0x1C
    MAJOR   = 0x03
    DNFLAG  = 0x20  # flags down except for FType.PROP


class Major(object):
    V1 = 0x00


class FrmType(object):
    JREQ = 0x00
    JACC = 0x20
    DAUP = 0x40  # data (unconfirmed) up
    DADN = 0x60  # data (unconfirmed) dn
    DCUP = 0x80  # data confirmed up
    DCDN = 0xA0  # data confirmed dn
    REJN = 0xC0  # rejoin for roaming
    PROP = 0xE0


class FCtrl(object):
    ADREN  =  0x80
    ADRARQ =  0x40
    ACK    =  0x20
    MORE   =  0x10   # in DN direction
    CLASSB =  0x10   # in UP direction
    OPTLEN =  0x0F
    FLAGS  =  0xF0


class DLSettings(object):
    OptNeg   = 0x80    # OptNeg in join accept, RFU in RXParamSetupReq
    RX1DRoff = 0x70
    RX2DR    = 0x0F
    @staticmethod
    def pack(rx1droff:int, rx2dr:int, optneg:bool=True) -> int:
        return (((rx1droff << 4) & DLSettings.RX1DRoff) |
                (rx2dr & DLSettings.RX2DR) |
                (DLSettings.OptNeg if optneg else 0))
    @staticmethod
    def unpack(dlset:int) -> Tuple[int,int,bool]:
        return ((dlset & DLSettings.RX1DRoff) >> 4,
                (dlset & DLSettings.RX2DR),
                (dlset & DLSettings.OptNeg)!=0)

class RXTiming(object):
    RFU     = 0xF0
    RxDelay = 0x0F



def to_hex(b:bytes) -> str:
    return bytes.hex(b).upper()

def to_bytes(s:str) -> bytes:
    return bytes.fromhex(s)

def as_bytes(x:Optional[Union[bytes,str]]) -> Optional[bytes]:
    if isinstance(x,str):
        return bytes.fromhex(x)
    return x


def sanity_check_frame (pdu:bytes) -> int:
    n = len(pdu)
    if n == 0:
        raise ValueError("Received empty frame")
    mhdr = pdu[0]
    if (mhdr & MHdr.MAJOR) != Major.V1:
        raise ValueError("Unsuppored major version: {:02X}".format(mhdr & MHdr.MAJOR))
    if (mhdr & MHdr.RFU) != 0:
        raise ValueError("Unsuppored MHdr.RFU: {:02X}".format(mhdr & MHdr.RFU))
    ftype = mhdr & MHdr.FTYPE
    
    if ftype == FrmType.JREQ or ftype == FrmType.REJN:
        if n != 23:
            raise ValueError("Frame JREQ with illegal length: expecting 23 but len={}".format(n))
        return ftype
    
    if ftype == FrmType.JACC:
        if n != 17 and n != 17+16:
            raise ValueError("Frame JACC with illegal length: expecting 17 or 17+16 but len={}".format(n))
        return ftype
    
    if ftype == FrmType.PROP:
        raise ValueError("Frame PROP detected - no proprietary frame formats supported")
    
    # remaining are DAUP/DADN/DCUP/DCDN - all having the same layout
    if n < 8+4:
        raise ValueError("Data frame too short: min 12 but len={}".format(n))
    foptlen = pdu[5] & FCtrl.OPTLEN
    if n < 8+4+foptlen:
        raise ValueError("Data frame has wrong length: expected 12+{} but len={}".format(foptlen,n))
    if n > 8+4+foptlen:
        port = pdu[8+foptlen]
        if port == 0 and foptlen > 0:
            raise ValueError("Data frame with FPort=0 has FOpts present")
            
    return ftype


def which_fcntdown (pdu:bytes) -> str:
    """Find the frame down counter being referred to by this down dataframe.
    If not a down dataframe return empty string. Assumes a sane frame.
    """
    mhdr = pdu[0]
    ftype = mhdr & MHdr.FTYPE
    if ftype != FrmType.DADN and ftype != FrmType.DCDN:
        return ""    # not a down dataframe
    n = len(pdu)
    foptlen = pdu[5] & FCtrl.OPTLEN
    if n == 8+4+foptlen:
        return 'NFCntDown'   # no port present
    port = pdu[8+foptlen]
    return 'NFCntDown' if port==0 else 'AFCntDown'


def pack_dataframe (mhdr:int = FrmType.DAUP|Major.V1,
                    devaddr:int = 0,
                    fctrl:int = 0,
                    fcnt:int = 0,            # 32bit counter required if nwkskey or appskey present
                    fopts:Union[bytes,bytearray] = None,
                    port:int = None,
                    payload:bytes = None,
                    nwkskey:bytes = None,
                    appskey:bytes = None) -> bytes:
    """Pack parameters into a data frame pdu with the following layout:
    
    +-----+---------+-----+-------+-------+------+---------+-----+---------+
    |  1  |    4    |  1  |   2   |  0/15 | 0/1  |   0-?   |  4  |   bytes |
    +=====+=========+=====+=======+=======+======+=========+=====+=========+
    | mhdr| devaddr |fctrl|  fcnt | fopts | port | payload | MIC |         |
    +-----+---------+-----+-------+-------+------+---------+-----+---------+
    | >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> | input to MIC  |
    +-----+---------+-----+-------+-------+------+---------+-----+---------+
    |                                            | >>>>>>> | ciphered      |
    +-----+---------+-----+-------+-------+------+---------+-----+---------+
    
    """
    if fopts is None: fopts = b''
    if payload is None: payload = b''
    
    prt = b'' if port is None or port < 0 else struct.pack("B",port)
    
    assert len(prt) == 1 or len(payload) == 0
    assert len(fopts) <= 15
    
    lfopts = len(fopts)
    fctrl = (fctrl & FCtrl.FLAGS) | lfopts
    ftype = mhdr & MHdr.FTYPE
    dndir = 1 if ftype == FrmType.DADN or ftype == FrmType.DCDN else 0
    
    if len(payload) > 0:
        key = nwkskey if port == 0 else appskey
        if key is not None:
            payload = crypto.cipher(key, devaddr, fcnt, dndir, payload)
    
    pdu = struct.pack("<BiBH{}s{}s{}si".format(lfopts, len(prt), len(payload)),
                      mhdr, devaddr, fctrl & 0xFF, fcnt & 0xFFFF, fopts, prt, payload, 0)
    if nwkskey is None:
        return pdu
    mic = crypto.calcMic(nwkskey, devaddr, fcnt, dndir, pdu)
    return pdu[0:-4] + struct.pack('<i', mic)


def peek_dataframe (pdu:bytes) -> Tuple[int,int]:
    """Peek at device short address and 16 bit sequence number of a data frame."""
    addr,fctrl,seqno16 = struct.unpack("<iBH", pdu[1:8])
    return (addr,seqno16)


def unpack_dataframe (pdu:bytes, lastseqno32:int, nwkskey:bytes, appskey:bytes=None) -> Msg:
    """Deconstruct a data frame into individual fields and verify integrity.
    Assumes package is sane (e.g. sanity_check_frame called before).
    If frame not sane then this might fail with an exception other than VerifyError.
    """
    mhdr, addr, fctrl, seqno = struct.unpack("<BiBH", pdu[0:8])
    lower16 = lastseqno32 & 0xFFFF
    upper16 = lastseqno32 >> 16
    if seqno < lower16 and upper16 < 0xFFFF:  # don't roll 32bit over to 0
        upper16 += 1
    seqno = (upper16<<16) + seqno
    ftype = mhdr & MHdr.FTYPE
    dndir = 1 if ftype == FrmType.DADN or ftype == FrmType.DCDN else 0
    pdulen = len(pdu)
    mic = struct.unpack('<i', pdu[pdulen-4:])[0]
    
    if mic != crypto.calcMic(nwkskey, addr, seqno, dndir, pdu):
        raise VerifyError("Verify of data frame failed")
    
    foptlen = fctrl & FCtrl.OPTLEN
    fopts = pdu[8:8+foptlen]         # assuming here 8+4+foptlen <= pdulen - otherwise malformed frame!!!
    if pdulen > 8+4+foptlen:
        port = pdu[8+foptlen]
        payload = pdu[9+foptlen:pdulen-4]
    else:
        port = -1
        payload = b''
    
    # Note: payload to be decrypted by appsrv
    if len(payload) > 0:
        key = nwkskey if port==0 else appskey
        if key is not None:
            payload = crypto.cipher(key, addr, seqno, dndir, payload)
    
    return {'msgtype'   :'updf',
            'MHdr'      : mhdr,
            'DevAddr'   : addr,
            'FCtrl'     : fctrl,
            'FCnt'      : seqno,
            'FOpts'     : fopts,      # field always present, if no options => ''
            'FPort'     : port,
            'FRMPayload': payload,    # field always present, if no payload => ''
            'MIC'       : mic}

def unpack_updf (pdu:bytes) -> Msg:
    """Deconstruct a data frame into individual fields and verify integrity.
    Note, there is no crypto verification nor MIC integrity verification.
    """
    mhdr, addr, fctrl, seqno = struct.unpack("<BiBH", pdu[0:8])
    ftype = mhdr & MHdr.FTYPE
    foptlen = fctrl & FCtrl.OPTLEN
    fopts = pdu[8:8+foptlen] # assuming here 8+4+foptlen <= pdulen - otherwise malformed frame!!!
    pdulen = len(pdu)
    if pdulen > 8+4+foptlen:
        port = pdu[8+foptlen]
        payload = pdu[9+foptlen:pdulen-4]
    else: # we have a malformed frame
        port = -1
        payload = b''
    mic = struct.unpack("<i",pdu[pdulen-4:])[0] # LNS expects a signed 32-bit for the MIC
    return {'msgtype'   :'updf',
            'MHdr'      : mhdr,
            'DevAddr'   : addr,
            'FCtrl'     : fctrl,
            'FCnt'      : seqno,
            'FOpts'     : fopts.hex(),   # field always present, if no options => ''
            'FPort'     : port,
            'FRMPayload': payload.hex(), # field always present, if no payload => ''
            'MIC'       : mic}

# This is special since it considers the two frame counters
# This is only used in device simulation code.
#
def unpack_dndf (sstate:State,  msg:Msg) -> str:
    """Deconstruct a down data frame into individual fields and verify it's integrity.
    Assumes package is sane (e.g. sanity_check_frame called before).
    If frame is not sane then this might fail with an exception other than VerifyError.
    """
    pdu = msg['pdu']
    if isinstance(pdu,str):
        pdu = to_bytes(pdu)
    mhdr, devaddr, fctrl, fcnt = struct.unpack("<BiBH", pdu[0:8])
    foptlen = fctrl & FCtrl.OPTLEN
    fopts = pdu[8:8+foptlen]         # assuming here 8+4+foptlen <= pdulen - otherwise malformed frame!!!
    pdulen = len(pdu)
    if pdulen > 8+4+foptlen:
        port = pdu[8+foptlen]
        payload = pdu[9+foptlen:pdulen-4]
    else:
        port = -1
        payload = b''
    mic = struct.unpack("<i", pdu[pdulen-4:])[0]
    
    # Select the correct down frame counter
    fcnt_key = 'NFCntDown' if port <= 0 else 'AFCntDown'
    s_fcntdn = sstate[fcnt_key]
    lower16 = s_fcntdn & 0xFFFF
    upper16 = s_fcntdn >> 16
    samelow = fcnt == lower16
    if fcnt < lower16 and upper16 < 0xFFFF:  # don't roll 32bit over to 0
        upper16 += 1
    fcnt = (upper16<<16) + fcnt
    ftype = mhdr & MHdr.FTYPE
    assert ftype == FrmType.DADN or ftype == FrmType.DCDN

    nwkskey = to_bytes(sstate['NwkSKeyDwn'])
    while mic != crypto.calcMic(nwkskey, devaddr, fcnt, 1, pdu):
        if not samelow:
            raise VerifyError("Verify of down data frame failed")
        samelow = False    # if lower16 the same try this upper page and next
        fcnt += 0x10000    # if not a replay of last fcnt then maybe jumped ahead
    
    if payload:
        cipkey = nwkskey if port==0 else to_bytes(sstate['AppSKey'])
        payload = crypto.cipher(cipkey, devaddr, fcnt, 1, payload)
        
    msg['MHdr'      ] = mhdr
    msg['DevAddr'   ] = devaddr
    msg['FCtrl'     ] = fctrl
    msg['FCnt'      ] = fcnt
    msg['FOpts'     ] = fopts
    msg['FPort'     ] = port
    msg['FRMPayload'] = payload
    msg['MIC'       ] = mic
    sstate[fcnt_key] = msg['FCnt']
    return fcnt_key


def pack_jreq (nwkkey:bytes,
               joineui:rtlib.Eui=None, deveui:rtlib.Eui=None,
               devnonce:int=0, mhdr:int=Major.V1|FrmType.JREQ) -> bytes:
    """Pack parameters into a JOIN REQUEST pdu with the following layout:
    |  1  |     8   |    8   |    2     |  4  |  bytes - all fields little endian
    | mhdr| joineui | deveui | devnonce | MIC |
    | < - - - - - - - - - - - - - - - ->|           input to MIC
    """
    if joineui is None: joineui = rtlib.Eui("00-00-00-00-00-00-00-00")
    if deveui  is None: deveui  = rtlib.Eui("00-00-00-00-00-00-00-00")
    
    pdu = struct.pack("<BqqHi", mhdr, joineui.as_int(), deveui.as_int(), devnonce & 0xFFFF, 0)
    mic = crypto.calcMicJoin(nwkkey,pdu)
    return pdu[0:-4] + struct.pack("<i", mic)

def unpack_nomic(pdu:bytes) -> Msg:
    """Verify sanity, and deconstruct a frame into individual fields. 
    Note, there is no crypto verification as we do not have keys."""
    #??? why do we have to be JSON serializable?
    ftype = sanity_check_frame(pdu)
    if (ftype==FrmType.JREQ):
        m = unpack_jreq(pdu)
        m['JoinEUI'] = str(m['JoinEUI'])
        m['DevEUI'] = str(m['DevEUI'])
        return m
    m = unpack_updf(pdu)
    return m

def unpack_jreq(pdu:bytes) -> Msg:
    """Unpack JREQ message and extract fields without verifying the integrity of the message..
    This call helps locate the device key and then the integrity must be checked by calling verifyJoinRequest.
    Return None if message is malformed, otherwise return mapping with frame fields.
    """
    mhdr, aeui, deui, devnonce, mic = struct.unpack("<BQQHi", pdu[0:23])
    return { 'msgtype'  : 'jreq', 
             'MHdr'     : mhdr,
             'JoinEUI'  : rtlib.Eui(aeui),
             'DevEUI'   : rtlib.Eui(deui),
             'DevNonce' : devnonce,
             'MIC'      : mic
         }


def verify_jreq(nwkkey:bytes, pdu:bytes) -> None:
    jr = unpack_jreq(pdu)
    if jr['MIC'] != crypto.calcMicJoin(nwkkey,pdu):
        raise VerifyError("Verify of join request failed")


def pack_jacc(nwkkey:Union[bytes,str],
              appnonce:int=0,
              netid:int=0, devaddr:int=0,
              dlset:int=0, rxdly:int=0,
              cflist:Optional[bytes]=None,
              mhdr:int=Major.V1|FrmType.JACC,
              devnonce:Optional[int]=None) -> bytes:
    """Pack parameters into a JOIN ACCEPT pdu with the following layout:
    |  1  |     3    |   3   |    4    |  1  |  1  |  0/16  |  4  |   bytes - all fields little endian
    | mhdr| appnonce | netid | devaddr |dlset|rxdly| cflist | MIC |
    |<- - - - - - - - - - - - - - - - - - - - - - - - - - ->|           input to MIC for v1.0
    |     |<- - - - - - - - - - - - - - - - - - - - - - - - - - ->|     ECB decrypted
    MIC input for v1.1: mhdr|devnonce|appnonce|netid|devaddr|dlset|rxdly|cflist
    """
    optneg = dlset & DLSettings.OptNeg
    nwkkey = cast(bytes, as_bytes(nwkkey))
    pdu = bytearray(17 + (16 if cflist else 0))
    pdu[0] = mhdr
    struct.pack_into("<I",   pdu, 1, appnonce & 0xFFFFFF)
    struct.pack_into("<I",   pdu, 1+3, netid)
    struct.pack_into("<iBB", pdu, 1+3+3, devaddr, dlset, rxdly)
    if cflist:
        struct.pack_into("16s", pdu, 17-4, cflist)
    if optneg:
        micpdu = pdu[0:1] + struct.pack("<H", devnonce) + pdu[1:]
    else:
        micpdu = pdu
    mic = crypto.calcMicJoin(nwkkey, bytes(micpdu))
    struct.pack_into("<i", pdu, len(pdu)-4, mic)
    return bytes(pdu[0:1] + crypto.decrypt(nwkkey, bytes(pdu[1:])))


def unpack_jacc(nwkkey:bytes, pdu:bytes, devnonce:int) -> Msg:
    pdu = bytes(pdu[0:1] + crypto.encrypt(nwkkey, pdu[1:]))
    appnonce = struct.unpack_from("<I", pdu, 1)[0] & 0xFFFFFF
    netid    = struct.unpack_from("<I", pdu, 1+3)[0] & 0xFFFFFF
    devaddr, dlset, rxdly = struct.unpack_from("<iBB", pdu, 1+3+3)
    cflist = None if len(pdu) == 17 else pdu[-20:-4]
    mic = struct.unpack_from("<i", pdu[-4:])[0]
    if dlset & DLSettings.OptNeg:
        micpdu = pdu[0:1] + struct.pack("<H", devnonce) + pdu[1:]
    else:
        micpdu = pdu
    if mic != crypto.calcMicJoin(nwkkey, micpdu):
        raise VerifyError("Verify of join accept failed")
    
    return {
        'MHdr'      : pdu[0],
        'AppNonce'  : appnonce,
        'NetID'     : netid,
        'DevAddr'   : devaddr,
        'DLSettings': dlset,
        'RxDelay'   : rxdly,
        'CFList'    : cflist,
        'MIC'       : mic
    }


def mic_join_request(sstate:State, jrmsg:Msg) -> int:
    """Use the sstate['NwkKey'] to append or verify the MIC of the jrmsg.
    jsmsg must contain the following fields: 'MHdr', 'JoinEUI', 'DevEUI' and 'DevNonce'.
    The method computes a MIC over an encoding of the join request message and
    either sets the field 'MIC' if it is absent or compare the computed value with the value
    of this field. If they don't match a VerifyError is raised.
    """
    pdu = pack_jreq(to_bytes(sstate['NwkKey']),
                    rtlib.Eui(jrmsg['JoinEUI']),
                    rtlib.Eui(jrmsg['DevEUI']),
                    jrmsg['DevNonce'],
                    jrmsg.get('MHdr', Major.V1|FrmType.JREQ))
    mic = struct.unpack("<i", pdu[-4:])[0]
    if 'MIC' in jrmsg:
        if jrmsg['MIC'] != mic:
            raise VerifyError('Verify of join request failed')
    else:
        jrmsg['MIC'] = mic
    return mic


def pack_msg(msg:Msg, mickey:Optional[Union[str,bytes]]=None, cipkey:Optional[Union[str,bytes]]=None) -> bytes:
    """Create a binary encoding of the frame described in msg."""
    mickey = as_bytes(mickey)
    cipkey = as_bytes(cipkey)
    
    fopts = msg.get('FOpts',None)
    fopts = to_bytes(fopts) if fopts else b''
    payld = msg.get('FRMPayload',None)
    payld = to_bytes(payld) if payld else b''
    
    return pack_dataframe(msg['MHdr'],
                          msg['DevAddr'],
                          msg['FCtrl'],
                          msg['FCnt'],
                          fopts,
                          msg.get('FPort',None),
                          payld,
                          cast(bytes,mickey), cast(bytes,cipkey))


def calc_mic(msg:Msg, mickey:Optional[Union[str,bytes]]=None, cipkey:Optional[Union[str,bytes]]=None) -> int:
    """Calculate MIC value from message object."""
    pdu = pack_msg(msg, mickey, cipkey)
    return struct.unpack("<i", pdu[-4:])[0]


def get_mic(pdu:bytes) -> int:
    return struct.unpack("<i", pdu[-4:])[0]


def verify_upmsg(nwkskeyup:Union[str,bytes], devaddr:int, fcnt32:int, pdu:bytes, mic:int) -> bool:
    return mic == crypto.calcMic(cast(bytes,as_bytes(nwkskeyup)), devaddr, fcnt32, 0, pdu)    # 0=dndir (up)


def cipher_msg_nwk(sstate:State, fcntkey:str, msg:Msg) -> None:
    """Cipher the 'FRMPayload' msg field is encrypted or decrypted using
    msg counter field fcntkey (one of 'AFCntDown', 'FCntUp' or 'FCntDown'.
    The msg field 'DevAddr' must hold the device short address.
    """
    assert msg['FPort'] == 0
    payload = msg['FRMPayload']
    if payload:
        try:             devaddr = msg['DevAddr']
        except KeyError: devaddr = sstate['DevAddr']
        if fcntkey == 'FCntUp':
            dndir = 0
            key = 'NwkSKeyUp'
        else:
            dndir = 1
            key = 'NwkSKeyDwn'
        cpdu = crypto.cipher(to_bytes(sstate[key]),
                             devaddr,
                             msg[fcntkey],
                             dndir,
                             to_bytes(payload))
        msg['FRMPayload'] = to_hex(cpdu)


def cipher_msg_app(sstate:State, fcntkey:str, msg:Msg) -> None:
    """Cipher the 'FRMPayload' msg field is encrypted or decrypted using
    msg counter field fcntkey (one of 'AFCntDown', 'FCntUp' or 'FCntDown'.
    The msg field 'DevAddr' must hold the device short address.
    """
    payload = msg['FRMPayload']
    if payload:
        try:             devaddr = msg['DevAddr']
        except KeyError: devaddr = sstate['DevAddr']
        cpdu = crypto.cipher(to_bytes(sstate['AppSKey']),
                             devaddr,
                             msg[fcntkey],
                             0 if fcntkey == 'FCntUp' else 1,
                             to_bytes(payload))
        msg['FRMPayload'] = to_hex(cpdu)


def recipher_msg_app(sstate:State, msg:Msg, prevaddr:Optional[int]=None, prevskey:str='') -> None:
    """Decipher the 'FRMPayload' msg field and encipher it again using
    the current session key and frame counter."""
    payload   = msg['FRMPayload']
    devaddr   = sstate['DevAddr']
    afcntdown = sstate['AFCntDown']
    appskey   = to_bytes(sstate['AppSKey'])
    if prevaddr is None:
        pdu = crypto.cipher(appskey, devaddr, msg['AFCntDown'], 1, to_bytes(payload))
    else:
        pdu = crypto.cipher(to_bytes(prevskey), prevaddr, msg['AFCntDown'], 1, to_bytes(payload))
    pdu = crypto.cipher(appskey, devaddr, afcntdown, 1, pdu)
    msg['AFCntDown'] = afcntdown
    msg['FRMPayload'] = to_hex(pdu)


def mic_msg(sstate:State, msg:Msg) -> None:
    """Compute message integrity code (MIC) over the fields contained in msg.
    This method encodes the fields and computes the MIC with the key sstate['NwkSKey{Up,Dwn}'] 
    depending on indicated direction in field msg['MHdr'].
    Msg must contain the following fields: 'MHdr', 'DevAddr', 'FCtrl', 'FCnt', 'FOpts',
    and optional fields 'FPort', 'FRMPayload'.
    This method verify the computed MIC with msg['MIC'] if this field is present and
    will raise an VerifyError if not equal. If the field msg['MIC'] is not present
    it will add the computed MIC to msg.
    """
    mhdr = msg['MHdr']
    ftype = mhdr & MHdr.FTYPE
    key = 'NwkSKeyDwn' if ftype == FrmType.DADN or ftype == FrmType.DCDN else 'NwkSKeyUp'
    code = pack_dataframe(mhdr,
                          msg['DevAddr'],
                          msg['FCtrl'],
                          msg['FCnt'],       # Note: must be 32bit frame counter
                          to_bytes(msg.get('FOpts','')),
                          msg.get('FPort',None),
                          to_bytes(msg.get('FRMPayload','')),
                          to_bytes(sstate[key]))
    mic = struct.unpack("<i", code[-4:])[0]
    if 'MIC' in msg:
        if msg['MIC'] != mic:
            raise VerifyError('Verify of data frame failed')
    else:
        msg['MIC'] = mic
    return mic


# ================================================================================
#
# DERIVE
#
# ================================================================================


def derive_apps_keys (sstate:State, msg:Msg, netid:int, lwversion10:bool=False) -> None:
    devnonce = msg['DevNonce']
    appnonce = msg['AppNonce']
    appkey   = sstate['NwkKey' if lwversion10 else 'AppKey']
    appskey = crypto.derive(to_bytes(appkey), devnonce, appnonce, netid, KD_AppSKey)
    sstate['AppSKey' ] = to_hex(appskey)


def derive_nwks_keys (sstate:State, sess:State, netid:int, lwversion10:bool=False) -> None:
    devnonce = sess['DevNonce']
    appnonce = sess['AppNonce']
    nwkkey   = sstate['NwkKey']

    # Derivation for V1.1/0 devices
    nwkskeyup  = to_hex(crypto.derive(to_bytes(nwkkey), devnonce, appnonce, netid, KD_NwkSKeyUp))
    nwkskeydwn = to_hex(crypto.derive(to_bytes(nwkkey), devnonce, appnonce, netid, KD_NwkSKeyDwn))
    sess['NwkSKeyUp' ] = nwkskeyup
    sess['NwkSKeyDwn'] = nwkskeyup if lwversion10 else nwkskeydwn
    if 'AppKey' in sstate:
        appkey = sstate['NwkKey' if lwversion10 else 'AppKey']
        appskey = crypto.derive(to_bytes(appkey), devnonce, appnonce, netid, KD_AppSKey)
        sess['AppSKey' ] = to_hex(appskey)


# ================================================================================
#
# AIRTIME
#
# ================================================================================

def airtime_symbols (bw:int=125, sf:int=7, nsyms:int=1) -> float:
    """Calculate airtime of n symbols in microseconds."""
    # Symbol rate / time for one symbol (secs)
    if bw in [125,250,500]:
        bw *= 1000
    assert bw in [125000,250000,500000]
    assert sf >= 7 and sf <= 12
    Rs = bw / (1<<sf)
    Ts = 1e6/Rs
    # Length/time of preamble
    return math.ceil( nsyms*Ts )


def airtime (bw:int=125, sf:int=7, plen:int=0, cr:int=1, ih:int=0, crc:int=1, dro:Optional[int]=None, npreamble:int=8) -> float:
    """Calculate air time of Lora packet in microseconds.
    The parameters are:
    | bandwidth (bw) one of 125,250,500 (kHz) or 12500,25000,50000 (Hz)
    | spreading factor (sf): one 7,..,12 or 0 for FSK @ 50kBit/s
    | packet payload length (plen): 0..255
    | coding rate (cr): 1..4
    | implicit header: 0 (explicit) or 1 (implicit)
    | cyclic redundancy check (crc): 0 (absent), 1 (present)
    | datarate optimization (dro): None sf=7,..,10: 0, sf=11,12 1, 0 (off), 1 (on)
    | number of preamble symbols (npreamble): default 8
    Return number of microseconds.
    """
    if sf == 0:
        # rough estimate of FSK @ 50kBit/s
        extra = 5+3+1+2   # preamble, syncword, len, crc
        bits = (plen+extra) * 8
        return  math.ceil(bits * 1e6 / 50e3)    # bits * us/sec / kBit/s
    
    if bw in [125,250,500]:
        bw *= 1000
    # dro=None 7-10 off, 11,12 on
    if dro is None:
        dro = int((sf>=11 and bw==125000) or (sf==12 and bw==250000))
    
    assert bw in [125000,250000,500000]
    assert sf >= 7 and sf <= 12
    assert plen >= 0 and plen <= 255
    assert cr >= 1 and cr <= 4
    assert ih==0 or ih==1
    assert crc==0 or crc==1
    assert dro==0 or dro==1
    
    # Symbol rate / time for one symbol (secs)
    Rs = bw / (1<<sf)
    Ts = 1/Rs
    # Length/time of preamble
    Tpreamble = (npreamble + 4.25) * Ts
    # Symbol length of payload and time
    tmp = math.ceil((8*plen - 4*sf + 28 + 16*crc - ih*20) / (4*sf-dro*8)) * (cr+4)
    npayload = 8 + max(0,tmp)
    Tpayload = npayload * Ts
    # Time on air 
    Tonair = Tpreamble + Tpayload
    # return us secs
    return math.ceil(Tonair*1e6);


