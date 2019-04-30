# Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
#
# This file is subject to the terms and conditions defined in file 'LICENSE',
# which is part of this source code package.

import struct

from Crypto.Hash import CMAC
from Crypto.Cipher import AES

class crypto:
    def encrypt (devkey:bytes, pdu:bytes) -> bytes:
        aes = AES.new(devkey, AES.MODE_ECB)
        return aes.encrypt(pdu)

    def decrypt (devkey:bytes, pdu:bytes) -> bytes:
        aes = AES.new(devkey, AES.MODE_ECB)
        return aes.decrypt(pdu)

    def calcMicJoin (key:bytes, pdu:bytes) -> int:
        pdu = pdu[:-4]
        cmac = CMAC.new(key, ciphermod=AES)
        cmac.update(pdu)
        mic, = struct.unpack_from('<i', cmac.digest())
        return mic

    def calcMic (key:bytes, devaddr:int, seqno:int, dndir:int, pdu:bytes) -> int:
        pdu = pdu[:-4]
        b0 = struct.pack('<BIBiIBB',
                0x49, 0, dndir, devaddr, seqno, 0, len(pdu))
        cmac = CMAC.new(key, ciphermod=AES, msg=b0)
        cmac.update(pdu)
        mic, = struct.unpack_from('<i', cmac.digest())
        return mic

    def cipher (key:bytes, devaddr:int, seqno:int, dndir:int, pdu:bytes) -> bytes:
        a0 = struct.pack('<BIBiIB', 1, 0, dndir, devaddr, seqno, 0)
        aes = AES.new(key, AES.MODE_CTR, nonce=a0, initial_value=1)
        return aes.encrypt(pdu)

    def derive (rootkey:bytes, devnonce:int, appnonce:int, netid:int, keytype:int) -> bytes:
        d = (bytes([keytype])
                + struct.pack('<I', appnonce)[:3]
                + struct.pack('<I', netid)[:3]
                + struct.pack('<H', devnonce)
                + (7 * b'\0'))
        aes = AES.new(rootkey, AES.MODE_ECB)
        return aes.encrypt(d)
