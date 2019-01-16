// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "board.h"
#include "hw.h"
#include "lmic.h"
#include "backtrace.h"

#if defined(BRD_sx1261_radio) || defined(BRD_sx1262_radio)

// ----------------------------------------
// Commands Selecting the Operating Modes of the Radio
#define CMD_SETSLEEP			0x84
#define CMD_SETSTANDBY			0x80
#define CMD_SETFS			0xC1
#define CMD_SETTX			0x83
#define CMD_SETRX			0x82
#define CMD_STOPTIMERONPREAMBLE		0x9F
#define CMD_SETRXDUTYCYCLE		0x94
#define CMD_SETCAD			0xC5
#define CMD_SETTXCONTINUOUSWAVE		0xD1
#define CMD_SETTXINFINITEPREAMBLE	0xD2
#define CMD_SETREGULATORMODE		0x96
#define CMD_CALIBRATE			0x89
#define CMD_CALIBRATEIMAGE		0x98
#define CMD_SETPACONFIG			0x95
#define CMD_SETRXTXFALLBACKMODE		0x93

// Commands to Access the Radio Registers and FIFO Buffer
#define CMD_WRITEREGISTER		0x0D
#define CMD_READREGISTER		0x1D
#define CMD_WRITEBUFFER			0x0E
#define CMD_READBUFFER			0x1E

// Commands Controlling the Radio IRQs and DIOs
#define CMD_SETDIOIRQPARAMS		0x08
#define CMD_GETIRQSTATUS		0x12
#define CMD_CLEARIRQSTATUS		0x02
#define CMD_SETDIO2ASRFSWITCHCTRL	0x9D
#define CMD_SETDIO3ASTCXOCTRL		0x97

// Commands Controlling the RF and Packets Settings
#define CMD_SETRFFREQUENCY		0x86
#define CMD_SETPACKETTYPE		0x8A
#define CMD_GETPACKETTYPE		0x11
#define CMD_SETTXPARAMS			0x8E
#define CMD_SETMODULATIONPARAMS		0x8B
#define CMD_SETPACKETPARAMS		0x8C
#define CMD_SETCADPARAMS		0x88
#define CMD_SETBUFFERBASEADDRESS	0x8F
#define CMD_SETLORASYMBNUMTIMEOUT	0xA0

// Commands Returning the Radio Status
#define CMD_GETSTATUS			0xC0
#define CMD_GETRSSIINST			0x15
#define CMD_GETRXBUFFERSTATUS		0x13
#define CMD_GETPACKETSTATUS		0x14
#define CMD_GETDEVICEERRORS		0x17
#define CMD_CLEARDEVICEERRORS		0x07
#define CMD_GETSTATS			0x10
#define CMD_RESETSTATS			0x00


// ----------------------------------------
// List of Registers

#define REG_WHITENINGMSB	0x06B8
#define REG_WHITENINGLSB	0x06B9
#define REG_CRCINITVALMSB	0x06BC
#define REG_CRCINITVALLSB	0x06BD
#define REG_CRCPOLYVALMSB	0x06BE
#define REG_CRCPOLYVALLSB	0x06BF
#define REG_SYNCWORD0		0x06C0
#define REG_SYNCWORD1		0x06C1
#define REG_SYNCWORD2		0x06C2
#define REG_SYNCWORD3		0x06C3
#define REG_SYNCWORD4		0x06C4
#define REG_SYNCWORD5		0x06C5
#define REG_SYNCWORD6		0x06C6
#define REG_SYNCWORD7		0x06C7
#define REG_NODEADDRESS		0x06CD
#define REG_BROADCASTADDR	0x06CE
#define REG_LORASYNCWORDMSB	0x0740
#define REG_LORASYNCWORDLSB	0x0741
#define REG_RANDOMNUMBERGEN0	0x0819
#define REG_RANDOMNUMBERGEN1	0x081A
#define REG_RANDOMNUMBERGEN2	0x081B
#define REG_RANDOMNUMBERGEN3	0x081C
#define REG_RXGAIN		0x08AC
#define REG_OCPCONFIG		0x08E7
#define REG_XTATRIM		0x0911
#define REG_XTBTRIM		0x0912

// sleep modes
#define SLEEP_COLD		0x00 // (no rtc timeout)
#define SLEEP_WARM		0x04 // (no rtc timesout)

// standby modes
#define STDBY_RC		0x00
#define STDBY_XOSC		0x01

// regulator modes
#define REGMODE_LDO		0x00
#define REGMODE_DCDC		0x01

// packet types
#define PACKET_TYPE_FSK		0x00
#define PACKET_TYPE_LORA	0x01

// irq types
#define IRQ_TXDONE		(1 << 0)
#define IRQ_RXDONE		(1 << 1)
#define IRQ_PREAMBLEDETECTED	(1 << 2)
#define IRQ_SYNCWORDVALID	(1 << 3)
#define IRQ_HEADERVALID		(1 << 4)
#define IRQ_HEADERERR		(1 << 5)
#define IRQ_CRCERR		(1 << 6)
#define IRQ_CADDONE		(1 << 7)
#define IRQ_CADDETECTED		(1 << 8)
#define IRQ_TIMEOUT		(1 << 9)

#define LORA_TXDONE_FIXUP       us2osticks(800) // XXX
#define FSK_TXDONE_FIXUP        us2osticks(800) // XXX
#define FSK_RXDONE_FIXUP        us2osticks(0) // XXX

// XXX
static const u2_t LORA_RXDONE_FIXUP_125[] = {
    [FSK]  =     us2osticks(0),
    [SF7]  =     us2osticks(0),
    [SF8]  =  us2osticks(1648),
    [SF9]  =  us2osticks(3265),
    [SF10] =  us2osticks(7049),
    [SF11] = us2osticks(13641),
    [SF12] = us2osticks(31189),
};

static const u2_t LORA_RXDONE_FIXUP_500[] = {
    [FSK]  = us2osticks(    0),
    [SF7]  = us2osticks(    0),
    [SF8]  = us2osticks(    0),
    [SF9]  = us2osticks(    0),
    [SF10] = us2osticks(    0),
    [SF11] = us2osticks(    0),
    [SF12] = us2osticks(    0),
};

// radio sleep state
static int sleeping;

// ----------------------------------------

static void writecmd (uint8_t cmd, const uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    sleeping = 0;
    hal_spi(cmd);
    for (u1_t i = 0; i < len; i++) {
        hal_spi(data[i]);
    }
    hal_spi_select(0);
}

static void WriteRegs (uint16_t addr, const uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    sleeping = 0;
    hal_spi(CMD_WRITEREGISTER);
    hal_spi(addr >> 8);
    hal_spi(addr);
    for (uint8_t i = 0; i < len; i++) {
        hal_spi(data[i]);
    }
    hal_spi_select(0);
}

static void WriteReg (uint16_t addr, uint8_t val) {
    WriteRegs(addr, &val, 1);
}

static void WriteBuffer (uint8_t off, const uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    sleeping = 0;
    hal_spi(CMD_WRITEBUFFER);
    hal_spi(off);
    for (uint8_t i = 0; i < len; i++) {
        hal_spi(data[i]);
    }
    hal_spi_select(0);
}

static uint8_t readcmd (uint8_t cmd, uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    sleeping = 0;
    hal_spi(cmd);
    uint8_t stat = hal_spi(0x00);
    for (u1_t i = 0; i < len; i++) {
        data[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
    return stat;
}

static void ReadRegs (uint16_t addr, uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    sleeping = 0;
    hal_spi(CMD_READREGISTER);
    hal_spi(addr >> 8);
    hal_spi(addr);
    hal_spi(0x00); // NOP
    for (uint8_t i = 0; i < len; i++) {
        data[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

static uint8_t ReadReg (uint16_t addr) {
    uint8_t val;
    ReadRegs(addr, &val, 1);
    return val;
}

static void ReadBuffer (uint8_t off, uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    sleeping = 0;
    hal_spi(CMD_READBUFFER);
    hal_spi(off);
    hal_spi(0x00); // NOP
    for (uint8_t i = 0; i < len; i++) {
        data[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

// set sleep mode SLEEP_COLD or SLEEP_WARM (from standby mode)
static void SetSleep (uint8_t cfg) {
    writecmd(CMD_SETSLEEP, &cfg, 1);
}

// set standby mode STDBY_RC or STANDBY_XOSC
static void SetStandby (uint8_t cfg) {
    writecmd(CMD_SETSTANDBY, &cfg, 1);
}

// set regulator mode REGMODE_LDO or REGMODE_DCDC
static void SetRegulatorMode (uint8_t mode) {
    writecmd(CMD_SETREGULATORMODE, &mode, 1);
}

// use DIO2 to drive antenna rf switch
static void SetDIO2AsRfSwitchCtrl (uint8_t enable) {
    writecmd(CMD_SETDIO2ASRFSWITCHCTRL, &enable, 1);
}

// write payload to fifo buffer at offset 0
static void WriteFifo (uint8_t *buf, uint8_t len) {
    static const uint8_t txrxbase[] = { 0, 0 };
    writecmd(CMD_SETBUFFERBASEADDRESS, txrxbase, 2);

    WriteBuffer(0, buf, len);
}

// read payload from fifo, return length
static uint8_t ReadFifo (uint8_t *buf) {
    // get buffer status
    uint8_t status[2];
    readcmd(CMD_GETRXBUFFERSTATUS, status, 2);

    // read buffer
    uint8_t len = status[0];
    uint8_t off = status[1];
    ReadBuffer(off, buf, len);

    // return length
    return len;
}

// set radio in transmit mode (abort after timeout [1/64ms])
static void SetTx (uint32_t timeout64ms) {
    uint8_t timeout[3] = { timeout64ms >> 16, timeout64ms >> 8, timeout64ms };
    writecmd(CMD_SETTX, timeout, 3);
}

// generate continuous (indefinite) wave
static void SetTxContinuousWave (void) {
    writecmd(CMD_SETTXCONTINUOUSWAVE, NULL, 0);
}

// set radio in receive mode (abort after timeout [1/64ms], or with timeout=0 after frame received)
static void SetRx (uint32_t timeout64ms) {
    uint8_t timeout[3] = { timeout64ms >> 16, timeout64ms >> 8, timeout64ms };
    writecmd(CMD_SETRX, timeout, 3);
    //debug_printf("SetRx(timeout64ms=%d, rxsyms=%d)\r\n", timeout64ms, LMIC.rxsyms);
}

// set radio to PACKET_TYPE_LORA or PACKET_TYPE_FSK mode
static void SetPacketType (uint8_t type) {
    writecmd(CMD_SETPACKETTYPE, &type, 1);
}

// set rf frequency (in Hz)
static void SetRfFrequency (uint32_t freq) {
    uint8_t buf[4];
    os_wmsbf4(buf, (uint32_t) (((uint64_t) freq << 25) / 32000000));
    writecmd(CMD_SETRFFREQUENCY, buf, 4);
}

// configure modulation parameters for LoRa
static void SetModulationParamsLora (u2_t rps) {
    uint8_t param[4];
    param[0] = getSf(rps) + 6; // SF (sf7=1)
    param[1] = getBw(rps) + 4; // BW (bw125=0)
    param[2] = getCr(rps) + 1; // CR (cr45=0)
    param[3] = ((1 << (getSf(LMIC.rps) + 1 - getBw(LMIC.rps) + 8)) >= 16384); // low-data-rate-opt (symbol time equal or above 16.38 ms)
    writecmd(CMD_SETMODULATIONPARAMS, param, 4);
}

// configure modulation parameters for FSK
static void SetModulationParamsFsk (void) {
    uint8_t param[8];
    param[0] = 0x00; // bitrate 50kbps (32 * fxtal / bitrate = 32 * 32000000 / 50000 = 0x005000)
    param[1] = 0x50;
    param[2] = 0x00;
    param[3] = 0x09; // pulse shape gaussian BT 0.5
    param[4] = 0x13; // bandwidth 93.8kHz DSB (~ 50kHz SSB)
    param[5] = 0x00; // frequency deviation 25kHz (deviation * 2^25 / fxtal = 25000 * 2^25 / 32000000 = 0x006666)
    param[6] = 0x66;
    param[7] = 0x66;
    writecmd(CMD_SETMODULATIONPARAMS, param, 8);
}

// configure packet handling for LoRa
static void SetPacketParamsLora (u2_t rps, int len, int inv) {
    uint8_t param[6];
    param[0] = 0x00; // 8 symbols preamble
    param[1] = 0x08;
    param[2] = getIh(rps); // implicit header
    param[3] = len;
    param[4] = !getNocrc(rps);
    param[5] = inv; // I/Q inversion
    writecmd(CMD_SETPACKETPARAMS, param, 6);
}

// configure packet handling for FSK
static void SetPacketParamsFsk (int crc, int len) {
    uint8_t param[9];
    param[0] = 0x00; // 40 bits / 5 bytes preamble
    param[1] = 0x28;
    param[2] = 0x05; // preamble detector length 16 bits
    param[3] = 0x18; // sync word length 24 bits / 3 bytes
    param[4] = 0x00; // node address filtering disabled
    param[5] = 0x01; // variable size packets, length is included
    param[6] = len;  // payload length
    param[7] = (crc) ? 0x06 : 0x01; // 16 bit inverted CRC or off
    param[8] = 0x01; // whitening enabled
    writecmd(CMD_SETPACKETPARAMS, param, 9);
}

// clear irq register
static void ClearIrqStatus (void) {
    uint8_t buf[2] = { 0xFF, 0xFF };
    writecmd(CMD_CLEARIRQSTATUS, buf, 2);
}

// stop timer on preamble detection or header/syncword detection
static void StopTimerOnPreamble (uint8_t enable) {
    writecmd(CMD_STOPTIMERONPREAMBLE, &enable, 1);
}

// set number of symbols for reception
static void SetLoRaSymbNumTimeout (uint8_t nsym) {
    writecmd(CMD_SETLORASYMBNUMTIMEOUT, &nsym, 1);
}

// return irq register
static uint16_t GetIrqStatus (void) {
    uint8_t buf[2];
    readcmd(CMD_GETIRQSTATUS, buf, 2);
    return (buf[0] << 8) | buf[1];
}

// get signal quality of received packet for LoRa
static void GetPacketStatusLora (s1_t *rssi, s1_t *snr) {
    uint8_t buf[3];
    readcmd(CMD_GETPACKETSTATUS, buf, 3);
    *rssi = -buf[0] / 2 + RSSI_OFF;
    *snr = buf[1] * SNR_SCALEUP / 4;
}

// get signal quality of received packet for FSK
static s1_t GetPacketStatusFsk (void) {
    uint8_t buf[3];
    readcmd(CMD_GETPACKETSTATUS, buf, 3);
    return -buf[2] / 2 + RSSI_OFF; // RssiAvg
}

// set and enable irq mask for dio1
static void SetDioIrqParams (uint16_t mask) {
    uint8_t param[] = { mask >> 8, mask & 0xFF, mask >> 8, mask & 0xFF, 0x00, 0x00, 0x00, 0x00 };
    writecmd(CMD_SETDIOIRQPARAMS, param, 8);
}

// set tx power (in dBm)
static void SetTxPower (s1_t pw) {
#if defined(BRD_sx1261_radio)
    // set over-current protection
    WriteReg(REG_OCPCONFIG, 0x18); // max 60mA
    // set pa config
    uint8_t paconfig[4] = { (pw > 14) ? 0x06 : 0x04, 0x00, 0x01, 0x01 };
    if (pw > 14) pw = 14;
    if (pw < -3) pw = -3;
#elif defined(BRD_sx1262_radio)
    // set over-current protection
    WriteReg(REG_OCPCONFIG, 0x38); // max 140mA
    // set pa config
    uint8_t paconfig[4] = { 0x04, 0x07, 0x00, 0x01 };
    if (pw > 22) pw = 22;
    if (pw < -3) pw = -3;
#endif
    writecmd(CMD_SETPACONFIG, paconfig, 4);

    // set tx params
    uint8_t pwramp[2];
    pwramp[0] = (uint8_t) pw;
    pwramp[1] = 0x04; // ramp time 200us
    writecmd(CMD_SETTXPARAMS, pwramp, 2);
}

// set sync word for LoRa
static void SetSyncWordLora (uint16_t syncword) {
    uint8_t buf[2] = { syncword >> 8, syncword & 0xFF };
    WriteRegs(REG_LORASYNCWORDMSB, buf, 2);
}

// set sync word for FSK
static void SetSyncWordFsk (uint32_t syncword) {
    uint8_t buf[3] = { syncword >> 16, syncword >> 8, syncword & 0xFF };
    WriteRegs(REG_SYNCWORD0, buf, 3);
}

// set seed for FSK data whitening
static void SetWhiteningSeed (uint16_t seed) {
    uint8_t buf[2] = { seed >> 8, seed & 0xFF };
    WriteRegs(REG_WHITENINGMSB, buf, 2);
}

// set CRC seed and polynomial for FSK
static void SetCrc16 (uint16_t seed, uint16_t polynomial) {
    // set seed
    uint8_t buf[2] = { seed >> 8, seed & 0xFF };
    WriteRegs(REG_CRCINITVALMSB, buf, 2);
    // set polynomial
    buf[0] = polynomial >> 8;
    buf[1] = polynomial;
    WriteRegs(REG_CRCPOLYVALMSB, buf, 2);
}

static uint32_t GetRandom (void) {
    uint8_t buf[4];
    // continuous rx
    SetRx(0xFFFFFF);
    // wait 1ms
    hal_waitUntil(os_getTime() + ms2osticks(1));
    // read random register
    ReadRegs(REG_RANDOMNUMBERGEN0, buf, 4);
    // standby
    SetStandby(STDBY_XOSC);
    return *((uint32_t*) buf);
}

void radio_sleep (void) {
    if (sleeping == 0) {
	SetSleep(SLEEP_COLD);
	sleeping = 1;
    }
}

static void txlora (void) {
    SetRegulatorMode(REGMODE_DCDC);
    SetDIO2AsRfSwitchCtrl(1);
    SetStandby(STDBY_XOSC);
    SetPacketType(PACKET_TYPE_LORA);
    SetRfFrequency(LMIC.freq);
    SetModulationParamsLora(LMIC.rps);
    SetPacketParamsLora(LMIC.rps, LMIC.dataLen, 0);
    SetTxPower(LMIC.txpow + LMIC.txPowAdj);
    SetSyncWordLora(0x3444);
    WriteFifo(LMIC.frame, LMIC.dataLen);
    ClearIrqStatus();
    SetDioIrqParams(IRQ_TXDONE | IRQ_TIMEOUT);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO1);

    // antenna switch / power accounting
    hal_pin_rxtx(1);

    // now we actually start the transmission
    BACKTRACE();
    SetTx(640000); // timeout 10s (should not happen, TXDONE irq will be raised)
}

static void txfsk (void) {
    SetRegulatorMode(REGMODE_DCDC);
    SetDIO2AsRfSwitchCtrl(1);
    SetStandby(STDBY_XOSC);
    SetPacketType(PACKET_TYPE_FSK);
    SetRfFrequency(LMIC.freq);
    SetModulationParamsFsk();
    SetPacketParamsFsk(1, LMIC.dataLen);
    SetCrc16(0x1D0F, 0x1021); // CCITT
    SetWhiteningSeed(0x01FF);
    SetSyncWordFsk(0xC194C1);
    SetTxPower(LMIC.txpow + LMIC.txPowAdj);
    WriteFifo(LMIC.frame, LMIC.dataLen);
    ClearIrqStatus();
    SetDioIrqParams(IRQ_TXDONE | IRQ_TIMEOUT);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO1);

    // antenna switch / power accounting
    hal_pin_rxtx(1);

    // now we actually start the transmission
    BACKTRACE();
    SetTx(64000); // timeout 1s (should not happen, TXDONE irq will be raised)
}

static void txcw (void) {
    SetRegulatorMode(REGMODE_DCDC);
    SetDIO2AsRfSwitchCtrl(1);
    SetStandby(STDBY_XOSC);
    SetRfFrequency(LMIC.freq);
    SetTxPower(LMIC.txpow + LMIC.txPowAdj);
    ClearIrqStatus();

    // antenna switch / power accounting
    hal_pin_rxtx(1);

    // start tx of wave (indefinitely, ended by RADIO_RST)
    BACKTRACE();
    SetTxContinuousWave();
}

void radio_starttx (bool txcontinuous) {
    if (txcontinuous) {
	txcw();
    } else {
	if (getSf(LMIC.rps) == FSK) { // FSK modem
	    txfsk();
	} else { // LoRa modem
	    txlora();
	}
	// the radio will go back to STANDBY mode as soon as the TX is finished
	// the corresponding IRQ will inform us about completion.
    }
}

static void rxfsk (bool rxcontinuous) {
    // configure radio (needs rampup time)
    ostime_t t0 = os_getTime();
    SetRegulatorMode(REGMODE_DCDC);
    SetDIO2AsRfSwitchCtrl(1);
    SetStandby(STDBY_XOSC);
    SetPacketType(PACKET_TYPE_FSK);
    SetRfFrequency(LMIC.freq);
    SetModulationParamsFsk();
    SetPacketParamsFsk(0, 255); // no crc on downlink
    //SetCrc16(0x1D0F, 0x1021); // CCITT
    SetWhiteningSeed(0x01FF);
    SetSyncWordFsk(0xC194C1);
    StopTimerOnPreamble(0);
    SetDioIrqParams(IRQ_RXDONE | IRQ_TIMEOUT | IRQ_CRCERR);

    ClearIrqStatus();

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO1);

    // now receive (lock interrupts only for final fine tuned rx timing...)
    hal_disableIRQs();
    if (rxcontinuous) { // continous rx
	BACKTRACE();
	// enable antenna switch for RX (and account power consumption)
	hal_pin_rxtx(0);
	// rx infinitely
	SetRx(0xFFFFFF);
    } else { // single rx
	BACKTRACE();
	// busy wait until exact rx time
	ostime_t now = os_getTime();
	if (LMIC.rxtime - now < 0) {
	    debug_printf("WARNING: rxtime is %d ticks in the past! (ramp-up time %d ms / %d ticks)\r\n",
			 now - LMIC.rxtime, osticks2ms(now - t0), now - t0);
	}
        hal_waitUntil(LMIC.rxtime);
	// enable antenna switch for RX (and account power consumption)
	hal_pin_rxtx(0);
	// rx for max LMIC.rxsyms symbols
	SetRx(64); // (1ms) XXX compute dynamic timeout based on LMIC.rxsyms!!!
    }
    hal_enableIRQs();
}

static void rxlora (bool rxcontinuous) {
    // configure radio (needs rampup time)
    ostime_t t0 = os_getTime();
    SetRegulatorMode(REGMODE_DCDC);
    SetDIO2AsRfSwitchCtrl(1);
    SetStandby(STDBY_XOSC);
    SetPacketType(PACKET_TYPE_LORA);
    SetRfFrequency(LMIC.freq);
    SetModulationParamsLora(LMIC.rps);
    SetPacketParamsLora(LMIC.rps, 255, !LMIC.noRXIQinversion);
    SetSyncWordLora(0x3444);
    StopTimerOnPreamble(1);
    SetLoRaSymbNumTimeout(8);
    SetDioIrqParams(IRQ_RXDONE | IRQ_TIMEOUT | IRQ_CRCERR);
    ClearIrqStatus();

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO1);

    // now receive (lock interrupts only for final fine tuned rx timing...)
    hal_disableIRQs();
    if (rxcontinuous) { // continous rx
	BACKTRACE();
	// enable antenna switch for RX (and account power consumption)
	hal_pin_rxtx(0);
	// rx infinitely
	SetRx(0xFFFFFF);
    } else { // single rx
	BACKTRACE();
	// busy wait until exact rx time
	ostime_t now = os_getTime();
	if (LMIC.rxtime - now < 0) {
	    debug_printf("WARNING: rxtime is %d ticks in the past! (ramp-up time %d ms / %d ticks)\r\n",
			 now - LMIC.rxtime, osticks2ms(now - t0), now - t0);
	}
        hal_waitUntil(LMIC.rxtime);
	// enable antenna switch for RX (and account power consumption)
	hal_pin_rxtx(0);
	// rx for max LMIC.rxsyms symbols
	// from lmic.c:
	// Table below defines the size of one symbol as
	//   symtime = 256us * 2^T(sf,bw)
	// 256us is called one symunit.
	//                 SF:
	//      BW:      |__7___8___9__10__11__12
	//      125kHz   |  2   3   4   5   6   7
	//      250kHz   |  1   2   3   4   5   6
	//      500kHz   |  0   1   2   3   4   5
	//
	// BW125=0, BW250, BW500, BWrfu
	// FSK=0, SF7, SF8, SF9, SF10, SF11, SF12, SFrfu
	//
	// SF=7 BW=125: symtime=1024us
	SetRx((LMIC.rxsyms << (getSf(LMIC.rps) + 1 - getBw(LMIC.rps) + 8 + 6)) / 1000); // N * 2^T(sf,bw) * 256 * 64 / 1000
    }
    hal_enableIRQs();
}

void radio_startrx (bool rxcontinuous) {
    if (getSf(LMIC.rps) == FSK) { // FSK modem
        rxfsk(rxcontinuous);
    } else { // LoRa modem
        rxlora(rxcontinuous);
    }
}

// reset radio
void radio_reset (void) {
    // drive RST pin low
    hal_pin_rst(0);

    // wait > 100us
    hal_waitUntil(os_getTime() + ms2osticks(1));

    // configure RST pin floating
    hal_pin_rst(2);

    // wait 1ms?
    hal_waitUntil(os_getTime() + ms2osticks(1));

    // check reset value
    ASSERT( ReadReg(REG_LORASYNCWORDLSB) == 0x24 );
}

void radio_init (void) {
    hal_disableIRQs();

    // reset radio (FSK/STANDBY)
    radio_reset();

    // check reset value
    ASSERT( ReadReg(REG_LORASYNCWORDLSB) == 0x24 );

    // go to SLEEP mode
    radio_sleep();

    hal_enableIRQs();
}

// called by radio irq job/stub
void radio_irq_process (ostime_t irqtime) {
    uint16_t irqflags = GetIrqStatus();

    // dispatch modem
    if (getSf(LMIC.rps) == FSK) { // FSK modem
	if (irqflags & IRQ_TXDONE) { // TXDONE
	    BACKTRACE();
            // save exact tx time
            LMIC.txend = irqtime - FSK_TXDONE_FIXUP;

        } else if (irqflags & IRQ_RXDONE) { // RXDONE
	    BACKTRACE();

            // read rx quality parameters
	    LMIC.rssi = GetPacketStatusFsk();
	    LMIC.snr = 0; // N/A

	    // read FIFO
	    LMIC.dataLen = ReadFifo(LMIC.frame);

            // save exact rx timestamps
            LMIC.rxtime  = irqtime - FSK_RXDONE_FIXUP; // end of frame timestamp
	    LMIC.rxtime0 = LMIC.rxtime - calcAirTime(LMIC.rps, LMIC.dataLen); // beginning of frame timestamp
#ifdef DEBUG_RX
	    debug_printf("RX[freq=%.1F,FSK,rssi=%d,len=%d]: %h\r\n",
			 LMIC.freq, 6, LMIC.rssi - RSSI_OFF, LMIC.dataLen, LMIC.frame, LMIC.dataLen);
#endif
	} else if (irqflags & IRQ_TIMEOUT) { // TIMEOUT
	    BACKTRACE();
            // indicate timeout
            LMIC.dataLen = 0;
        } else {
	    // unexpected irq
	    debug_printf("UNEXPECTED RADIO IRQ %04x\r\n", irqflags);
	    TRACE_VAL(irqflags);
	    ASSERT(0);
	}
    } else { // LORA modem
	if (irqflags & IRQ_TXDONE) { // TXDONE
	    BACKTRACE();

            // save exact tx time
            LMIC.txend = irqtime - LORA_TXDONE_FIXUP;

        } else if (irqflags & IRQ_RXDONE) { // RXDONE
	    BACKTRACE();

            // read rx quality parameters
	    GetPacketStatusLora(&LMIC.rssi, &LMIC.snr);

	    // read FIFO
	    LMIC.dataLen = ReadFifo(LMIC.frame);

            // save exact rx timestamps
            LMIC.rxtime = irqtime; // end of frame timestamp
            if (getBw(LMIC.rps) == BW125) {
                LMIC.rxtime -= LORA_RXDONE_FIXUP_125[getSf(LMIC.rps)];
            }
            else if (getBw(LMIC.rps) == BW500) {
                LMIC.rxtime -= LORA_RXDONE_FIXUP_500[getSf(LMIC.rps)];
            }
	    LMIC.rxtime0 = LMIC.rxtime - calcAirTime(LMIC.rps, LMIC.dataLen); // beginning of frame timestamp
#ifdef DEBUG_RX
	    debug_printf("RX[freq=%.1F,sf=%d,bw=%s,rssi=%d,snr=%.2F,len=%d]: %h\r\n",
			 LMIC.freq, 6,
			 getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)),
			 LMIC.rssi - RSSI_OFF, LMIC.snr * 100 / SNR_SCALEUP, 2,
			 LMIC.dataLen, LMIC.frame, LMIC.dataLen);
#endif
	} else if (irqflags & IRQ_TIMEOUT) { // TIMEOUT
	    BACKTRACE();
            // indicate timeout
            LMIC.dataLen = 0;
        } else {
	    // unexpected irq
	    debug_printf("UNEXPECTED RADIO IRQ %04x\r\n", irqflags);
	    TRACE_VAL(irqflags);
	    ASSERT(0);
	}
    }

    // mask all IRQs
    SetDioIrqParams(0);

    // clear IRQ flags
    ClearIrqStatus();
}

#endif
