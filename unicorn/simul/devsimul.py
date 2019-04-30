# Copyright (C) 2018-2018, Semtech Corporation.
# All rights reserved.
#
# This file is subject to the terms and conditions
# defined in file 'LICENSE', which is part of this
# source code package.

from typing import Any, Awaitable, Callable, List, MutableMapping, Optional, TextIO, Tuple, Union
from typing import cast

import argparse
import asyncio
import math
import numpy
import os
import struct
import sys

from intelhex import IntelHex
from intervaltree import Interval, IntervalTree
from peripherals import InterruptController, GPIO, UART
from vtimeloop import VirtualTimeLoop

import unicorn as uc
import unicorn.arm_const as uca

import loramsg as lm

class Rps:
    @staticmethod
    def makeRps(sf:int=7, bw:int=125000, cr:int=1, crc:int=1, ih:int=0) -> int:
        return ((sf-6) | ([125000,250000,500000].index(bw)<<3)
                | ((cr-1)<<5) | ((crc^1)<<7) | ((ih&0xFF)<<8)) if sf else 0

    @staticmethod
    def getSf(rps:int) -> int:
        sf = rps & 0x7
        return sf + 6 if sf else 0

    @staticmethod
    def getBw(rps:int) -> int:
        return (1 << ((rps >> 3) & 0x3)) * 125000

    @staticmethod
    def getCr(rps:int) -> int:
        return ((rps >> 5) & 0x3) + 1

    @staticmethod
    def getCrc(rps:int) -> int:
        return ((rps >> 7) & 0x1) ^ 1

    @staticmethod
    def getIh(rps:int) -> int:
        return (rps >> 8) & 0xff

    @staticmethod
    def getParams(rps:int) -> Tuple[int,int,int,int,int]:
        return (Rps.getSf(rps),
                Rps.getBw(rps),
                Rps.getCr(rps),
                Rps.getCrc(rps),
                Rps.getIh(rps))

    @staticmethod
    def validate(rps:int) -> None:
        (sf, bw, cr, crc, ih) = Rps.getParams(rps)
        if sf:
            assert bw in [125000,250000,500000], f'unsupported bw: {bw}'
            assert sf >= 7 and sf <= 12,         f'unsupported sf: {sf}'
            assert cr >= 1 and cr <= 4,          f'unsupported cr: {cr}'
            assert ih==0 or ih==1,               f'unsupported ih: {ih}'
            assert crc==0 or crc==1,             f'unsupported crc: {crc}'

    @staticmethod
    def isFSK(rps:int) -> int:
        return (rps & 0x7) == 0

class LoraMsg:
    def __init__(self, time:float, pdu:bytes, freq:int, rps:int,
            xpow:Optional[int]=None, snr:Optional[int]=None,
            dro:Optional[int]=None, npreamble:int=8,
            rssi:Optional[float]=None) -> None:
        Rps.validate(rps)
        (sf, bw, cr, crc, ih) = Rps.getParams(rps)
        if sf:
            if dro is None:
                dro = 1 if ((sf>=11 and bw==125000)
                        or  (sf==12 and bw==250000)) else 0
            else:
                assert dro==0 or dro==1
            assert len(pdu) >= 0 and len(pdu) <= 255
        else:
            # TODO: FSK
            dro = 0

        self.pdu = pdu
        self.freq = freq
        self.rps = rps
        self.xpow = xpow
        self.snr = snr
        self.dro = dro
        self.npreamble = npreamble
        self.rssi = rssi

        self.xbeg = time
        self.xend = time + self.airtime()

    def __str__(self) -> str:
        sf = Rps.getSf(self.rps)
        bw = Rps.getBw(self.rps)
        return 'xbeg=%.6f, xend=%.6f, freq=%d, %s, data=%s' % (
                self.xbeg, self.xend,
                self.freq,
                'sf=%d, bw=%d' % (sf, bw) if sf else 'fsk',
                self.pdu.hex())

    # xbeg, xend, freq, rps, xpow, snr, dlen, data....
    SIM_RXTX_SPEC = '<iiIIiiI'
    SIM_RXTX_SPEC_SZ = struct.calcsize(SIM_RXTX_SPEC)
    SIM_RXTX_SPEC_MAXSZ = SIM_RXTX_SPEC_SZ + 256

    @staticmethod
    def from_simrxtx(rxtx:bytes, c:Optional[int]=None) -> 'LoraMsg':
        (xbeg, xend, freq, rps, xpow, snr, dlen) = struct.unpack_from(
                LoraMsg.SIM_RXTX_SPEC, rxtx)
        return LoraMsg(Simulation.ticks2time(xbeg, c),
                rxtx[LoraMsg.SIM_RXTX_SPEC_SZ:LoraMsg.SIM_RXTX_SPEC_SZ+dlen],
                freq, rps, xpow, snr)

    def simrxtx(self) -> bytes:
        return struct.pack(LoraMsg.SIM_RXTX_SPEC,
                numpy.int32(Simulation.time2ticks(self.xbeg)),
                numpy.int32(Simulation.time2ticks(self.xend)),
                self.freq, self.rps, self.xpow or 0, self.snr or 0,
                len(self.pdu)) + self.pdu

    def match(self, freq:int, rps:int) -> bool:
        return self.freq == freq and (Rps.isFSK(rps) if Rps.isFSK(self.rps)
                else (self.rps == rps))

    @staticmethod
    def symtime(rps:int, nsym:int=1) -> float:
        (sf, bw, cr, crc, ih) = Rps.getParams(rps)
        if sf == 0:
            return 8*nsym / 50000
        # Symbol rate / time for one symbol (secs)
        Rs = bw / (1<<sf)
        Ts = 1 / Rs
        return nsym * Ts

    def airtime(self) -> float:
        Ts = LoraMsg.symtime(self.rps)
        if Rps.isFSK(self.rps):
            return (5+3+1+2+len(self.pdu)) * Ts
        # Length/time of preamble
        Tpreamble = (self.npreamble + 4.25) * Ts
        # Symbol length of payload and time
        (sf, bw, cr, crc, ih) = Rps.getParams(self.rps)
        tmp = math.ceil(
                (8*len(self.pdu) - 4*sf + 28 + 16*crc - ih*20)
                / (4*sf-self.dro*8)) * (cr+4)
        npayload = 8 + max(0,tmp)
        Tpayload = npayload * Ts
        # Time on air
        return Tpreamble + Tpayload

    def tpreamble(self) -> float:
        return LoraMsg.symtime(self.rps, self.npreamble)

class DnMedium:
    def get_dn(self, rxon:int, rxtout:int, freq:int, rps:int, nsym:int=4) -> Optional[LoraMsg]:
        raise NotImplementedError()

class UpMedium:
    def put_up(self, msg:LoraMsg) -> None:
        raise NotImplementedError()

class SimpleDnMedium(DnMedium):
    def __init__(self) -> None:
        self.msgs = IntervalTree()

    def add_dn(self, msg:LoraMsg) -> None:
        t0 = Simulation.time2ticks(msg.xbeg)
        t1 = t0 + Simulation.time2ticks(msg.tpreamble())
        self.msgs[t0:t1] = msg

    @staticmethod
    def overlap(i1:Interval, i2:Interval) -> int:
        return min(i1.end, i2.end) - max(i1.begin, i2.begin) # type: ignore

    def get_dn(self, rxon:int, rxtout:int, freq:int, rps:int, nsym:int=4) -> Optional[LoraMsg]:
        rxw = Interval(rxon, rxon + rxtout)
        tpn = Simulation.time2ticks(LoraMsg.symtime(rps, nsym))
        for i in self.msgs.overlap(rxw[0], rxw[1]):
            m = i.data # type: LoraMsg
            if m.match(freq, rps) and SimpleDnMedium.overlap(i, rxw) >= tpn:
                break
        else:
            return None
        self.msgs.remove(i)
        return m

    def prune(self, ticks:int) -> None:
        exp = self.msgs.envelop(0, ticks)
        if exp:
            self.msgs.remove_envelop(0, ticks)
        return exp

class TraceWriter:
    def __init__(self, buf:TextIO) -> None:
        self.buf = buf

    def write(self, s:str, **kwargs:Any) -> None:
        self.buf.write(s)

class TrafficTrace:
    def __init__(self, io:Union[TextIO,TraceWriter]) -> None:
        if not isinstance(io, TraceWriter):
            io = TraceWriter(io)
        self.tw = io

    def trace(self, msg:LoraMsg, d2n:bool, lost:bool=False) -> None:
        self.tw.write('%s->%s: %s%s\n' % (
            'D' if d2n else 'N', 'x' if lost else 'N' if d2n else 'D',
            msg, self.msginfo(msg)),
            strike=lost)

    def msginfo(self, msg:LoraMsg) -> str:
        return ''

class SimEvent:
    def __init__(self) -> None:
        self.e = asyncio.Event()
        self.s = set()

    def set(self, key:str) -> None:
        self.s.add(key)
        self.e.set()

    def clear(self, key:str) -> None:
        self.s.discard(key)
        if not self.s:
            self.e.clear()

    def is_set(self, key:str) -> bool:
        return key in self.s

class Simulation:
    RAM_BASE   = 0x10000000
    FLASH_BASE = 0x20000000
    EE_BASE    = 0x30000000

    @staticmethod
    def time2ticks(t:float) -> int:
        return int(t * 32768)

    @staticmethod
    def ticks2time(t:int, c:Optional[int]=None) -> float:
        if c:
            t = Simulation.ticks_extend(t, c)
        return t / 32768

    @staticmethod
    def ticks_extend(t:int, c:int) -> int:
        return c + int(numpy.int32((int(numpy.int32(t)) - int(numpy.int32(c)))))

    def now2ticks(self) -> int:
        return Simulation.time2ticks(asyncio.get_event_loop().time() - self.epoch)
 
    def __init__(self, hexfiles:List[str], debug:Optional[TraceWriter]=None, traffic:Optional[TrafficTrace]=None,
            ramsz:int=16*1024, flashsz:int=128*1024, eesz:int=8*1024) -> None:

        self.upm:Optional[UpMedium] = None
        self.dnm:Optional[DnMedium] = None

        self.emu = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB)

        #self.emu.hook_add(uc.UC_HOOK_CODE,
        #        lambda uc, addr, size, sim: sim.trace(addr), self)
        self.emu.hook_add(uc.UC_HOOK_INTR,
                lambda uc, intno, sim: sim.intr(intno), self)
        self.emu.hook_add(uc.UC_HOOK_BLOCK,
                lambda uc, address, size, sim:
                sim.block_special(address), self,
                begin=0xfffff000, end=0xffffffff)

        self.emu.mem_map(Simulation.RAM_BASE, ramsz)
        self.emu.mem_map(Simulation.FLASH_BASE, flashsz)
        self.emu.mem_map(Simulation.EE_BASE, eesz)
        self.emu.mem_map(0xfffff000, 0x1000) # special

        self.ex:Optional[BaseException] = None

        # Peripherals
        self.event = SimEvent()
        self.ic = InterruptController(self.event)
        self.gpio = GPIO(self.ic, 0)
        self.uart = UART(self.ic, 1)

        # Load firmware
        for hf in hexfiles:
            self.load_hexfile(hf)

        # Debug settings
        self.debug = debug
        self.traffic = traffic

        # read SP and entry point address from header in flash
        (sp, ep) = struct.unpack('<II',
                self.emu.mem_read(Simulation.FLASH_BASE, 8))
        self.isp = sp
        self.ipc = ep

        self.ticks = 0
        self.reset()

        self.rxparams:MutableMapping[str,int] = {}
        self.rxmsg:Optional[LoraMsg] = None

    def load_hexfile(self, hexfile:str) -> None:
        ih = IntelHex()
        ih.loadhex(hexfile)
        beg = ih.minaddr()
        end = ih.maxaddr() + 1
        mem = bytes(ih.gets(beg, end - beg))
        try:
            self.emu.mem_write(beg, mem)
        except:
            print('Error loading %s at 0x%08x (%d bytes):' % (hexfile, beg, len(mem)))
            raise

    def step(self, ticks:int) -> int:
        if self.dnm:
            exp = self.dnm.prune(ticks if not self.rxing else self.rxparams['rxbeg'])
            for iv in exp:
                self.traffic.trace(iv[2], False, lost=True)
        self.ticks = ticks
        self.emu.emu_start(self.pc, 0xffffffff)
        if self.ex is not None:
            raise self.ex
        return self.sleep

    @staticmethod
    async def _sleepto(ticks:int, ev:Optional[asyncio.Event]) -> None:
        if ticks > 0:
            tts = Simulation.ticks2time(ticks)
            if ev:
                try:
                    await asyncio.wait_for(asyncio.shield(ev.wait()), timeout=tts)
                except asyncio.TimeoutError:
                    pass
            else:
                await asyncio.sleep(tts)

    async def vsleepto(self, tticks:int, ev:Optional[asyncio.Event]) -> int:
        await Simulation._sleepto(tticks - self.now2ticks(), ev)
        return self.now2ticks()

    async def rsleepto(self, tticks:int, ev:Optional[asyncio.Event]) -> int:
        now = self.now2ticks()
        # get closer with sleep
        while ev is None or not ev.is_set():
            tts = (tticks - now) - 10
            if tts <= 0:
                break
            await Simulation._sleepto(tts, ev)
            now = self.now2ticks()
        # last few steps busy-wait
        if ev and ev.is_set():
            return now
        while now < tticks:
            now = self.now2ticks()
        return tticks # pretend we didn't overshoot

    def check_irqs(self, ct:int) -> bool:
        if self.irq_enabled() and self.event.is_set('irq'):
            self.irq_invoke(ct)
            return True
        else:
            return False

    async def run(self) -> None:
        sleepto = self.vsleepto if isinstance(asyncio.get_event_loop(),
                VirtualTimeLoop) else self.rsleepto
        ct = self.now2ticks()
        while True:
            nt = self.step(ct)
            if self.check_irqs(ct):
                continue
            ct = await sleepto(nt, self.event.e)
            self.check_irqs(ct)
            self.event.clear('reset')

    def attach(self, upm:Optional[UpMedium], dnm:Optional[DnMedium]) -> None:
        self.upm = upm
        self.dnm = dnm

    def reset(self) -> None:
        self.vtor:Optional[int] = None

        self.gpio.reset()
        self.uart.reset()

        self.irq_invoking = False

        self.rxing:bool = False

        self.pc = self.ipc
        self.emu.reg_write(uca.UC_ARM_REG_SP, self.isp)
        self.emu.reg_write(uca.UC_ARM_REG_LR, 0xcafebabe)

        self.epoch = asyncio.get_event_loop().time()
        self.ticks = 0
        self.sleep = 0

        self.event.set('reset')

    def irq_enabled(self) -> bool:
        cpsr = cast(int, self.emu.reg_read(uca.UC_ARM_REG_CPSR))
        return (cpsr & (1 << 7)) == 0

    def stack_push(self, value:int) -> None:
        sp = self.emu.reg_read(uca.UC_ARM_REG_SP) - 4
        self.emu.mem_write(sp, struct.pack('<I', value))
        self.emu.reg_write(uca.UC_ARM_REG_SP, sp)

    def stack_pop(self) -> int:
        sp = self.emu.reg_read(uca.UC_ARM_REG_SP)
        (value,) = cast(Tuple[int], struct.unpack('<I', self.emu.mem_read(sp, 4)))
        self.emu.reg_write(uca.UC_ARM_REG_SP, sp + 4)
        return value

    def irq_invoke(self, ticks:int) -> None:
        if self.irq_invoking:
            raise RuntimeError('Nested interrupt invocation')
        if self.vtor is None:
            raise RuntimeError('VTOR not initialized')
        self.irq_invoking = True
        while self.event.is_set('irq') and self.irq_enabled():
            irq = self.ic.next()
            (addr,) = struct.unpack('<I',
                    self.emu.mem_read(self.vtor + 4*irq, 4))
            # push LR to stack
            self.stack_push(self.emu.reg_read(uca.UC_ARM_REG_LR))
            # set LR to magic value
            self.emu.reg_write(uca.UC_ARM_REG_LR, 0xfffffff1)
            self.pc = addr
            self.step(ticks)
        self.irq_invoking = False

    def get_string(self, addr:int) -> str:
        ba = bytearray()
        while True:
            ba += self.emu.mem_read(addr, 128)
            nt = ba.find(0, -128)
            if nt >= 0:
                return ba[:nt].decode('utf-8')
            addr += 128

    def svc_panic(self, params:Tuple[int,int,int], lr:int) -> bool:
        raise RuntimeError(
                'PANIC: type=%d (%s), reason=0x%x, addr=0x%08x, lr=0x%08x'
                % (params[0], {0: 'ex', 1: 'bl', 2: 'fw'}.get(params[0], '??'),
                    params[1], params[2], lr))

    def svc_debug_str(self, params:Tuple[int,int,int], lr:int) -> bool:
        if self.debug is not None:
            self.debug.write(self.get_string(params[0]))
        return True

    def svc_ticks(self, params:Tuple[int,int,int], lr:int) -> bool:
        self.emu.reg_write(uca.UC_ARM_REG_R0, numpy.uint32(self.ticks))
        self.emu.reg_write(uca.UC_ARM_REG_R1, numpy.uint32((self.ticks >> 32)))
        return True

    def svc_sleep(self, params:Tuple[int,int,int], lr:int) -> bool:
        if params[0] == 2: # forever
            target = self.ticks + Simulation.time2ticks(10 * 3600)
        else:
            target = Simulation.ticks_extend(params[1], self.ticks)
        if target > self.ticks:
            self.sleep = target
            self.emu.reg_write(uca.UC_ARM_REG_R0, 1)
            return False
        else:
            self.emu.reg_write(uca.UC_ARM_REG_R0, 0)
            return True

    def svc_tx(self, params:Tuple[int,int,int], lr:int) -> bool:
        m = LoraMsg.from_simrxtx(bytes(self.emu.mem_read(params[0],
            LoraMsg.SIM_RXTX_SPEC_MAXSZ)), self.ticks)
        if self.traffic is not None:
            self.traffic.trace(m, True)
        if self.upm:
            self.upm.put_up(m)
        return True

    def svc_rx_start(self, params:Tuple[int,int,int], lr:int) -> bool:
        self.rxparams['freq'] = params[0]
        self.rxparams['rps'] = params[1]
        self.rxparams['rxbeg'] = self.ticks
        self.rxparams['rxtout'] = params[2]
        self.rxing = True
        return True

    def get_dn(self) -> Optional[LoraMsg]:
        return None if self.dnm is None else self.dnm.get_dn(
                self.rxparams['rxbeg'], self.rxparams['rxtout'],
                self.rxparams['freq'], self.rxparams['rps'])

    def svc_rx_do(self, params:Tuple[int,int,int], lr:int) -> bool:
        m = self.get_dn()
        self.emu.reg_write(uca.UC_ARM_REG_R0, 1 if m else 0)
        self.rxmsg = m
        if m is None:
            self.rxing = False
        return True

    def svc_rx_done(self, params:Tuple[int,int,int], lr:int) -> bool:
        m = self.rxmsg
        if m:
            if self.traffic is not None:
                self.traffic.trace(m, False)
            self.emu.mem_write(params[0], m.simrxtx())
        self.emu.reg_write(uca.UC_ARM_REG_R0, 1 if m else 0)
        self.rxing = False
        return True

    def svc_rng_seed(self, params:Tuple[int,int,int], lr:int) -> bool:
        self.emu.reg_write(uca.UC_ARM_REG_R0, 0x12345678)
        return True

    def svc_vtor(self, params:Tuple[int,int,int], lr:int) -> bool:
        self.vtor = params[0]
        return True

    def svc_irq(self, params:Tuple[int,int,int], lr:int) -> bool:
        if not self.irq_invoking:
            self.sleep = self.ticks
            return False
        else:
            return True

    def svc_pio(self, params:Tuple[int,int,int], lr:int) -> bool:
        lut: List[Callable] = [
                lambda: self.gpio.config(self.emu.mem_read(params[1], 6 * 4)),
                lambda: self.emu.reg_write(uca.UC_ARM_REG_R0, self.gpio.read()),
                lambda: self.emu.reg_write(uca.UC_ARM_REG_R0,
                    self.gpio.read_irq()),
                lambda: self.gpio.clear_irq(params[1]) ]
        lut[params[0]]()
        return True

    def svc_uart(self, params:Tuple[int,int,int], lr:int) -> bool:
        cmd, p1, p2 = params
        if cmd == 0:
            if p1 == 0:
                v = self.uart.sr
            else:
                v = self.uart.read_rx()
            self.emu.reg_write(uca.UC_ARM_REG_R0, v)
        elif cmd == 1:
            self.uart.tx_char(p1)
        elif cmd == 2:
            self.uart.config(p1)
        elif cmd == 3:
            if p1 == 0:
                self.uart.setrx(not not p2)
            else:
                self.uart.settx(not not p2)
        else:
            raise KeyError('Unknown UART command %d' % cmd)

        return True

    svc_lookup = {
            0   : svc_panic,
            128 : svc_debug_str,
            129 : svc_ticks,
            130 : svc_sleep,
            131 : svc_tx,
            132 : svc_rx_start,
            133 : svc_rx_do,
            134 : svc_rx_done,
            135 : svc_rng_seed,
            136 : svc_vtor,
            137 : svc_irq,
            138 : svc_pio,
            139 : svc_uart,
            }

    def trace(self, addr:int) -> None:
        print('PC=%08x' % addr)

    def intr(self, intno:int) -> None:
        try:
            lr = self.emu.reg_read(uca.UC_ARM_REG_LR)
            if intno == 2: # SVC
                svcid = self.emu.reg_read(uca.UC_ARM_REG_R0)
                handler = Simulation.svc_lookup.get(svcid)
                if handler:
                    params = (self.emu.reg_read(uca.UC_ARM_REG_R1),
                            self.emu.reg_read(uca.UC_ARM_REG_R2),
                            self.emu.reg_read(uca.UC_ARM_REG_R3))
                    if handler(self, params, lr):
                        self.emu.reg_write(uca.UC_ARM_REG_PC, lr)
                    else:
                        self.pc = lr
                        self.emu.emu_stop()
                else:
                    raise RuntimeError('Unknown SVCID %d, lr=0x%08x'
                            % (svcid, lr))
            else:
                raise RuntimeError('Unexpected interrupt %d, lr=0x%08x'
                        % (intno, lr))
        except BaseException as ex:
            self.ex = ex
            self.emu.emu_stop()

    def block_special(self, address:int) -> None:
        if address == 0xfffffff0:
            self.pc = self.stack_pop()
            self.emu.emu_stop()
        else:
            raise RuntimeError('Unknown special PC: 0x%08x' % address)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-v', '--virtual-time', action='store_true',
            help='Use virtual time')
    p.add_argument('hexfiles', metavar='HEXFILE', nargs='+',
            help='Firmware files to load')
    args = p.parse_args()

    asyncio.get_event_loop().run_until_complete(Simulation(args.hexfiles).run())
