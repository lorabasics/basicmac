// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

//#define DEBUG_TX
//#define DEBUG_RX

#include "board.h"
#include "lmic.h"
#include "backtrace.h"

#if !defined(BRD_sx1276_radio) && !defined(BRD_sx1272_radio)
#error "Missing board radio setting BRD_sx1272_radio/BRD_sx1276_radio"
#endif

// ----------------------------------------
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16
#define LORARegRxpacketCntValueLsb                 0x17
#define LORARegModemStat                           0x18
#define LORARegPktSnrValue                         0x19
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORAFeiMib                                 0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define FSKRegSyncValue8                           0x2F
#define FSKRegPacketConfig1                        0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define FSKRegSeqConfig1                           0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define FSKRegTimer2Coef                           0x3A
#define FSKRegImageCal                             0x3B
#define LORARegInvertIQ2                           0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
// #define RegTcxo                                    0x58 // common
#define SX1272_RegPaDac                            0x5A // common
#define SX1276_RegPaDac                            0x4D // common
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
// #define RegBitRateFrac                             0x70 // common

// ----------------------------------------
// SX1272 RegModemConfig1 settings
#define SX1272_MC1_CR_4_5                    0x08
#define SX1272_MC1_CR_4_6                    0x10
#define SX1272_MC1_CR_4_7                    0x18
#define SX1272_MC1_CR_4_8                    0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON   0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON          0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE    0x01 // mandated for SF11 and SF12

// SX1272 RegModemConfig2 settings
#define SX1272_MC2_AGCAUTO                   0x04

// SX1276 RegModemConfig1 settings
#define SX1276_MC1_BW_125                    0x70
#define SX1276_MC1_BW_250                    0x80
#define SX1276_MC1_BW_500                    0x90
#define SX1276_MC1_CR_4_5                    0x02
#define SX1276_MC1_CR_4_6                    0x04
#define SX1276_MC1_CR_4_7                    0x06
#define SX1276_MC1_CR_4_8                    0x08
#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON   0x01

// SX1276 RegModemConfig2
#define SX1276_MC2_RX_PAYLOAD_CRCON          0x04

// SX1276 RegModemConfig3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE    0x08
#define SX1276_MC3_AGCAUTO                   0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                    0x34

// opmodes
#define OPMODE_MASK           0x07
#define OPMODE_SLEEP          0
#define OPMODE_STANDBY        1
#define OPMODE_FSTX           2
#define OPMODE_TX             3
#define OPMODE_FSRX           4
#define OPMODE_RX             5
#define OPMODE_RX_SINGLE      6
#define OPMODE_CAD            7

// LoRa opmode bits:
#define OPMODE_LORA           0x80
// SX1272: bit7=1 (LoRa), bit6=0 (AccessSharedReg=LoRa), bit5+4+3=000 (unused), bit2+1+0=mode
// SX1276: bit7=1 (LoRa), bit6=0 (AccessSharedReg=LoRa), bit5+4=00 (reserved), bit3=0 (access HF test regs), bit2+1+0=mode
#define OPMODE_LORA_SLEEP     (0b10000000+OPMODE_SLEEP)
#define OPMODE_LORA_STANDBY   (0b10000000+OPMODE_STANDBY)
#define OPMODE_LORA_FSTX      (0b10000000+OPMODE_FSTX)
#define OPMODE_LORA_TX        (0b10000000+OPMODE_TX)
#define OPMODE_LORA_FSRX      (0b10000000+OPMODE_FSRX)
#define OPMODE_LORA_RX        (0b10000000+OPMODE_RX)
#define OPMODE_LORA_RX_SINGLE (0b10000000+OPMODE_RX_SINGLE)
#define OPMODE_LORA_CAD       (0b10000000+OPMODE_CAD)

// FSK opmode bits:
#ifdef BRD_sx1272_radio
// SX1272: bit7=0 (FSK), bit6+5=00 (modulation=FSK), bit4+3=00 (no shaping), bits2+1+0=mode
#define OPMODE_FSK_SLEEP      (0b00000000+OPMODE_SLEEP)
#define OPMODE_FSK_STANDBY    (0b00000000+OPMODE_STANDBY) // (reset value of RegOpMode)
#define OPMODE_FSK_FSTX       (0b00000000+OPMODE_FSTX)
#define OPMODE_FSK_TX         (0b00000000+OPMODE_TX)
#define OPMODE_FSK_FSRX       (0b00000000+OPMODE_FSRX)
#define OPMODE_FSK_RX         (0b00000000+OPMODE_RX)
#define OPMODE_FSK_RX_SINGLE  (0b00000000+OPMODE_RX_SINGLE)
#define OPMODE_FSK_CAD        (0b00000000+OPMODE_CAD)
#endif
#ifdef BRD_sx1276_radio
// SX1276: bit7=0 (FSK), bit6+5=00 (modulation=FSK), bit4=0 (reserved), bit3=1 (access LF test regs), bits2+1+0=mode
#define OPMODE_FSK_SLEEP      (0b00001000+OPMODE_SLEEP)
#define OPMODE_FSK_STANDBY    (0b00001000+OPMODE_STANDBY) // (reset value of RegOpMode)
#define OPMODE_FSK_FSTX       (0b00001000+OPMODE_FSTX)
#define OPMODE_FSK_TX         (0b00001000+OPMODE_TX)
#define OPMODE_FSK_FSRX       (0b00001000+OPMODE_FSRX)
#define OPMODE_FSK_RX         (0b00001000+OPMODE_RX)
#define OPMODE_FSK_RX_SINGLE  (0b00001000+OPMODE_RX_SINGLE)
#define OPMODE_FSK_CAD        (0b00001000+OPMODE_CAD)
#endif

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK            0x80
#define IRQ_LORA_RXDONE_MASK            0x40
#define IRQ_LORA_CRCERR_MASK            0x20
#define IRQ_LORA_HEADER_MASK            0x10
#define IRQ_LORA_TXDONE_MASK            0x08
#define IRQ_LORA_CDDONE_MASK            0x04
#define IRQ_LORA_FHSSCH_MASK            0x02
#define IRQ_LORA_CDDETD_MASK            0x01

#define IRQ_FSK1_MODEREADY_MASK         0x80
#define IRQ_FSK1_RXREADY_MASK           0x40
#define IRQ_FSK1_TXREADY_MASK           0x20
#define IRQ_FSK1_PLLLOCK_MASK           0x10
#define IRQ_FSK1_RSSI_MASK              0x08
#define IRQ_FSK1_TIMEOUT_MASK           0x04
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01
#define IRQ_FSK2_FIFOFULL_MASK          0x80
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10
#define IRQ_FSK2_PACKETSENT_MASK        0x08
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04
#define IRQ_FSK2_CRCOK_MASK             0x02
#define IRQ_FSK2_LOWBAT_MASK            0x01

// ----------------------------------------
// DIO function mappings
#define MAP1_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP1_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP1_DIO0_LORA_NOP      0xC0  // 11------
#define MAP1_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP1_DIO1_LORA_NOP      0x30  // --11----
#define MAP1_DIO2_LORA_NOP      0x0C  // ----11--
#define MAP2_DIO5_LORA_NOP      0x30  // --11----

#define MAP1_DIO0_FSK_READY     0x00  // 00------ (packet sent / payload ready)
#define MAP1_DIO1_FSK_NOP       0x30  // --11----
#define MAP1_DIO2_FSK_TXNOP     0x04  // ----01--
#define MAP1_DIO2_FSK_TIMEOUT   0x08  // ----10--

// FSK ImageCal defines
#define RF_IMAGECAL_IMAGECAL_START      0x40
#define RF_IMAGECAL_IMAGECAL_RUNNING    0x20

// IQ Inversion
// addr   bits  name                          mode  reset  description
// ----------------------------------------------------------------------
// 0x33   7     start_rambist                 rw    0      start RAM BIST
//        6     invert_i_q                    rw    0      invert I & Q
//        5     enable_quadrature_correction  rw    1      enable i/q compensation block (0=bypass)
//        4     invert_coef_amp               rw    0      check possible inversion between GSK and LoRa
//        3     invert_coef_phase             rw    0      check possible inversion between GSK and LoRa
//        2     sync_detect_th                rw    1      sync threshold: eliminates false preamble detection
//        1     chirp_invert_rx               rw    1      invert chirp direction in rx mode
//        0     chirp_invert_tx               rw    1      invert tx spreading sequence
// ----------------------------------------------------------------------
// 0x3B   7-0   freq_to_time_invert           rw    0x1D   tracking loop inversion optimization
// ----------------------------------------------------------------------
#define IQRXNORMAL  0x27 // (see AN1200.24 SX1276 settings for LoRaWAN)
#define IQRXINVERT  0x67
#define IQ2RXNORMAL 0x1D
#define IQ2RXINVERT 0x19

// radio-specific settings
#if defined(BRD_sx1276_radio)
#define RADIO_VERSION               0x12
#define RST_PIN_RESET_STATE         0
#define RSSI_HF_CONST               157
#define LNA_RX_GAIN                 0b00100011   // LnaGain=001 (max), LnaBoostLf=00 (default), reserved=0, LnaBoostHf=11 (150%)
#define RegPaDac                    SX1276_RegPaDac
#define LORA_TXDONE_FIXUP           us2osticks(800) // empirically determined 2017-09-25 mku/fhr (time since tx end till exec of irq handler)

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

#elif defined(BRD_sx1272_radio)
#define RADIO_VERSION               0x22
#define RST_PIN_RESET_STATE         1
#define RSSI_HF_CONST               139
#define LNA_RX_GAIN                 0b00100011
#define RegPaDac                    SX1272_RegPaDac
#define LORA_TXDONE_FIXUP           us2osticksRound(77) // based on Nucleo board regr test clockskew

static const u2_t LORA_RXDONE_FIXUP_125[] = {
    [FSK]  = us2osticksRound(    0),
    [SF7]  = us2osticksRound(  749),
    [SF8]  = us2osticksRound( 1343),
    [SF9]  = us2osticksRound( 3265),
    [SF10] = us2osticksRound( 7049),
    [SF11] = us2osticksRound(13641),
    [SF12] = us2osticksRound(31189),
};

// Based Nucleo board regr tests rxlatency-regr
static const u2_t LORA_RXDONE_FIXUP_500[] = {
    [FSK]  = us2osticksRound(   0),
    [SF7]  = us2osticksRound( 193),
    [SF8]  = us2osticksRound( 344),
    [SF9]  = us2osticksRound( 737),
    [SF10] = us2osticksRound(1521),
    [SF11] = us2osticksRound(3240),
    [SF12] = us2osticksRound(6972),
};

#endif

// ----------------------------------------
// RADIO STATE
static struct {
    ostime_t irqtime;
    osjob_t irqjob;
    u1_t randbuf[16]; // initialized by radio_init(), used by radio_rand1()
} state;

// ----------------------------------------
static void writeReg (u1_t addr, u1_t data) {
    hal_spi_select(1);
    hal_spi(addr | 0x80);
    hal_spi(data);
    hal_spi_select(0);
}

static u1_t readReg (u1_t addr) {
    hal_spi_select(1);
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    hal_spi_select(0);
    return val;
}

void radio_writeBuf (u1_t addr, xref2u1_t buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr | 0x80);
    for (u1_t i = 0; i < len; i++) {
        hal_spi(buf[i]);
    }
    hal_spi_select(0);
}

void radio_readBuf (u1_t addr, xref2u1_t buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr & 0x7F);
    for (u1_t i = 0; i < len; i++) {
        buf[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

// configure LoRa modem
static void configLoraModem (void) {
    sf_t sf = getSf(LMIC.rps); // (SF7=1)

#if defined(BRD_sx1276_radio)
    u1_t mc1 = 0, mc2 = 0, mc3 = 0;

    // set ModemConfig1 (bw=xxxx, cr=xxx, implicitheader=x)
    switch (getBw(LMIC.rps)) {
        case BW125: mc1 |= SX1276_MC1_BW_125; break;
        case BW250: mc1 |= SX1276_MC1_BW_250; break;
        case BW500: mc1 |= SX1276_MC1_BW_500; break;
        default: ASSERT(0);
    }
    switch (getCr(LMIC.rps)) {
        case CR_4_5: mc1 |= SX1276_MC1_CR_4_5; break;
        case CR_4_6: mc1 |= SX1276_MC1_CR_4_6; break;
        case CR_4_7: mc1 |= SX1276_MC1_CR_4_7; break;
        case CR_4_8: mc1 |= SX1276_MC1_CR_4_8; break;
        default: ASSERT(0);
    }
    if (getIh(LMIC.rps)) {
	mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
	writeReg(LORARegPayloadLength, getIh(LMIC.rps)); // required length
    }
    writeReg(LORARegModemConfig1, mc1);

    // set ModemConfig2 (sf=xxxx, txcont=0, rxpayloadcrc=x, symtimeoutmsb=00)
    mc2 = (7+sf-1) << 4;
    if (getNocrc(LMIC.rps) == 0) {
	mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
    }
    writeReg(LORARegModemConfig2, mc2);

    // set ModemConfig3 (unused=0000, lowdatarateoptimize=x, agcauto=1, reserved=00)
    mc3 = SX1276_MC3_AGCAUTO;
    if ((sf == SF11 || sf == SF12) && getBw(LMIC.rps) == BW125) {
	mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
    }
    writeReg(LORARegModemConfig3, mc3);

    // SX1276 Errata: 2.1 Sensitivity Optimization with a 500kHz Bandwith
    if (getBw(LMIC.rps) == BW500) {
	writeReg(0x36, 0x02);
	writeReg(0x3a, 0x64); // XXX TODO: correct value for low-band operation
    } else {
	writeReg(0x36, 0x03);
	// no need to reset register 0x3a
    }
#elif defined(BRD_sx1272_radio)
    // set ModemConfig1 (bw=xx, cr=xxx, implicitheader=x, rxpayloadcrc=x, lowdatarateoptimize=x)
    u1_t mc1 = getBw(LMIC.rps) << 6;
    switch (getCr(LMIC.rps)) {
        case CR_4_5: mc1 |= SX1272_MC1_CR_4_5; break;
        case CR_4_6: mc1 |= SX1272_MC1_CR_4_6; break;
        case CR_4_7: mc1 |= SX1272_MC1_CR_4_7; break;
        case CR_4_8: mc1 |= SX1272_MC1_CR_4_8; break;
        default: ASSERT(0);
    }
    if (getIh(LMIC.rps)) {
	mc1 |= SX1272_MC1_IMPLICIT_HEADER_MODE_ON;
	writeReg(LORARegPayloadLength, getIh(LMIC.rps)); // required length
    }
    if (getNocrc(LMIC.rps) == 0) {
	mc1 |= SX1272_MC1_RX_PAYLOAD_CRCON;
    }
    if ((sf == SF11 || sf == SF12) && getBw(LMIC.rps) == BW125) {
	mc1 |= SX1272_MC1_LOW_DATA_RATE_OPTIMIZE;
    }
    writeReg(LORARegModemConfig1, mc1);

    // set ModemConfig2 (sf=xxxx, txcont=0, agcauto=1 symtimeoutmsb=00)
    writeReg(LORARegModemConfig2, ((7+sf-1) << 4) | SX1272_MC2_AGCAUTO);
#endif // BRD_sx1272_radio
}

static void configChannel (void) {
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u4_t frf = ((u8_t)LMIC.freq << 19) / 32000000;
    writeReg(RegFrfMsb, frf >> 16);
    writeReg(RegFrfMid, frf >> 8);
    writeReg(RegFrfLsb, frf >> 0);
}


#ifdef CFG_tx_erp_adj
#define TX_ERP_ADJ	(CFG_tx_erp_adj)
#else
#define TX_ERP_ADJ	0
#endif

static void configPower (void) {
    s1_t pw = LMIC.txpow + LMIC.txPowAdj;
#if defined(CFG_eu868)
    // gain adjustment for regions where limit is ERP
    pw += TX_ERP_ADJ;
#endif
#if (defined(CFG_wailmer_board) || defined(CFG_wailord_board)) && defined(CFG_us915)
    // XXX - TODO - externalize this somehow
    // wailmer/wailord can only use 17dBm at DR4 (US)
    if (getBw(LMIC.rps) == BW500 && pw > 17) {
	pw = 17;
    }
#endif
#if defined(BRD_sx1276_radio)
    u1_t dac = readReg(RegPaDac), cfg, ocp;
    if (pw >= 20) {
	dac |= 0x07;
	cfg = 0x8f;
	ocp = 0x20 | 20;
    } else {
	if (pw > 17) {
	    pw = 17;
	} else if (pw < 2) {
	    pw = 2;
	}
	dac |= 0x04;
	cfg = 0x80 | (pw - 2);
	ocp = 0x2b;
    }
    writeReg(RegPaDac, dac);
    writeReg(RegPaConfig, cfg);
    writeReg(RegOcp, ocp);
#elif defined(BRD_sx1272_radio)
    // set PA config (2-17 dBm using PA_BOOST)
    if (pw > 17) {
        pw = 17;
    } else if (pw < 2) {
        pw = 2;
    }
    writeReg(RegPaConfig, (u1_t) (0x80 | (pw - 2)));
#endif /* BRD_sx1272_radio */
}

// continuous wave
static void txcw (void) {
    // select FSK modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_FSK_SLEEP);

    // enter standby mode (required for FIFO loading))
    writeReg(RegOpMode, OPMODE_FSK_STANDBY);

    // set frequency deviation
    writeReg(FSKRegFdevMsb, 0x00);
    writeReg(FSKRegFdevLsb, 0x00);

    // configure frequency
    configChannel();

    // configure output power
    configPower();

    // set continuous mode
    writeReg(LORARegModemConfig2, readReg(LORARegModemConfig2) | 0x08);

    // initialize the payload size and address pointers
    writeReg(FSKRegPayloadLength, 1);
    writeReg(RegFifo, 0);

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    writeReg(RegOpMode, OPMODE_FSK_TX);
}

static void txfsk (void) {
    // not supported
    ASSERT(0);

    // select FSK modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_FSK_SLEEP);

    // enter standby mode (required for FIFO loading))
    writeReg(RegOpMode, OPMODE_FSK_STANDBY);

    // set bitrate
    writeReg(FSKRegBitrateMsb, 0x02); // 50kbps
    writeReg(FSKRegBitrateLsb, 0x80);

    // set frequency deviation
    writeReg(FSKRegFdevMsb, 0x01); // +/- 25kHz
    writeReg(FSKRegFdevLsb, 0x99);

    // frame and packet handler settings
    writeReg(FSKRegPreambleMsb, 0x00);
    writeReg(FSKRegPreambleLsb, 0x05);
    writeReg(FSKRegSyncConfig, 0x12);
    writeReg(FSKRegPacketConfig1, 0xD0);
    writeReg(FSKRegPacketConfig2, 0x40);
    writeReg(FSKRegSyncValue1, 0xC1);
    writeReg(FSKRegSyncValue2, 0x94);
    writeReg(FSKRegSyncValue3, 0xC1);

    // configure frequency
    configChannel();

    // configure output power
    configPower();

    // set the IRQ mapping DIO0=PacketSent DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP1_DIO0_FSK_READY | MAP1_DIO1_FSK_NOP | MAP1_DIO2_FSK_TXNOP);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0);

    // initialize the payload size and address pointers
    writeReg(FSKRegPayloadLength, LMIC.dataLen + 1); // (insert length byte into payload))

    // download length byte and buffer to the radio FIFO
    writeReg(RegFifo, LMIC.dataLen);
    radio_writeBuf(RegFifo, LMIC.frame, LMIC.dataLen);

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    writeReg(RegOpMode, OPMODE_FSK_TX);
}

static void txlora (void) {
    // select LoRa modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_LORA_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_LORA_SLEEP);

    // enter standby mode (required for FIFO loading)
    writeReg(RegOpMode, OPMODE_LORA_STANDBY);

    // configure LoRa modem
    configLoraModem();

    // configure frequency
    configChannel();

    // configure output power
    writeReg(RegPaRamp, 0b00011000); // unused=000, LowPnTxPllOff=1, PaRamp=1000 (ramp-up time 50us)
    configPower();

    // set sync word
    writeReg(LORARegSyncWord, LORA_MAC_PREAMBLE);

    // set IQ inversion mode
    writeReg(LORARegInvertIQ,  IQRXNORMAL);
    writeReg(LORARegInvertIQ2, IQ2RXNORMAL);

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP1_DIO0_LORA_TXDONE | MAP1_DIO1_LORA_NOP | MAP1_DIO2_LORA_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // mask all IRQs but TxDone
    writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0);

    // initialize the payload size and address pointers
    writeReg(LORARegFifoTxBaseAddr, 0x00);
    writeReg(LORARegFifoAddrPtr, 0x00);
    writeReg(LORARegPayloadLength, LMIC.dataLen);

    // download buffer to the radio FIFO
    radio_writeBuf(RegFifo, LMIC.frame, LMIC.dataLen);

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    BACKTRACE();
    writeReg(RegOpMode, OPMODE_LORA_TX);
}

// start transmitter (buf=LMIC.frame, len=LMIC.dataLen)
static void starttx (void) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
#ifdef DEBUG_TX
    debug_printf("TX[freq=%.1F,sf=%d,bw=%s,len=%d%s]: %h\r\n",
	    LMIC.freq, 6,
	    getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)),
	    LMIC.dataLen,
	    (LMIC.pendTxPort != 0 && (LMIC.frame[OFF_DAT_FCT] & FCT_ADRARQ)) ? ",ADRARQ" : "",
	    LMIC.frame, LMIC.dataLen);
#endif
    if (getSf(LMIC.rps) == FSK) { // FSK modem
        txfsk();
    } else { // LoRa modem
        txlora();
    }
    // the radio will go back to STANDBY mode as soon as the TX is finished
    // the corresponding IRQ will inform us about completion.
}

enum { RXMODE_SINGLE = 0, RXMODE_SCAN, RXMODE_RSSI };

// start LoRa receiver (time=LMIC.rxtime, timeout=LMIC.rxsyms, result=LMIC.frame[LMIC.dataLen])
static void rxlora (u1_t rxmode) {
    // select LoRa modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_LORA_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_LORA_SLEEP);

    // enter standby mode (warm up)
    writeReg(RegOpMode, OPMODE_LORA_STANDBY);

    // configure LoRa modem (cfg1, cfg2, cfg3)
    configLoraModem();

    // configure frequency
    configChannel();

    // set LNA gain
    writeReg(RegLna, LNA_RX_GAIN);

    // set max payload size
    writeReg(LORARegPayloadMaxLength, MAX_LEN_FRAME);

    // set IQ inversion mode
    writeReg(LORARegInvertIQ,  (LMIC.noRXIQinversion) ? IQRXNORMAL  : IQRXINVERT);
    writeReg(LORARegInvertIQ2, (LMIC.noRXIQinversion) ? IQ2RXNORMAL : IQ2RXINVERT);

    // set symbol timeout (for single rx)
    writeReg(LORARegSymbTimeoutLsb, LMIC.rxsyms);

    // set sync word
    writeReg(LORARegSyncWord, LORA_MAC_PREAMBLE);

    // configure DIO mapping
    writeReg(RegDioMapping1, (const u1_t[]){ [RXMODE_SINGLE] = MAP1_DIO0_LORA_RXDONE | MAP1_DIO1_LORA_RXTOUT | MAP1_DIO2_LORA_NOP,
		                             [RXMODE_SCAN]   = MAP1_DIO0_LORA_RXDONE | MAP1_DIO1_LORA_NOP    | MAP1_DIO2_LORA_NOP,
		                             [RXMODE_RSSI]   = MAP1_DIO0_LORA_NOP    | MAP1_DIO1_LORA_NOP    | MAP1_DIO2_LORA_NOP,
		                           } [rxmode] );

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, ~ (const u1_t[]){ [RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK,
		                                    [RXMODE_SCAN]   = IRQ_LORA_RXDONE_MASK,
		                                    [RXMODE_RSSI]   = 0x00,
		                                  } [rxmode] );

    // enable IRQs in HAL
    hal_irqmask_set( (const u1_t[]){ [RXMODE_SINGLE] = HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO1,
		                     [RXMODE_SCAN]   = HAL_IRQMASK_DIO0,
		                     [RXMODE_RSSI]   = 0x00,
		                   } [rxmode] );

    // enable antenna switch for RX
    hal_pin_rxtx(0);

    // now instruct the radio to receive
    // it takes 1068us from beginning of rxlora() up to here (keep RX_RAMPUP in sync!)
    // (lock interrupts only for final fine tuned rx timing...)
    hal_disableIRQs();
    if (rxmode == RXMODE_SINGLE) { // single rx
	BACKTRACE();
        hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
#ifdef CFG_testpin
	// signal (rxstart) irq on GPIO line
	if (LMIC.testpinMode == 3) {
	    CFG_PIN_VAL(CFG_testpin, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE, 1);
	    LMIC.testpinMode |= 0x80;
	}
#endif
        writeReg(RegOpMode, OPMODE_LORA_RX_SINGLE);
    } else { // continous rx (scan or rssi)
	BACKTRACE();
        writeReg(RegOpMode, OPMODE_LORA_RX);
    }
    hal_enableIRQs();
}

static void rxfsk (u1_t rxmode) {
    // not supported
    ASSERT(0);

    // only single rx (no continuous scanning, no noise sampling)
    ASSERT( rxmode == RXMODE_SINGLE );

    // select FSK modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_FSK_SLEEP);

    // enter standby mode (warm up)
    writeReg(RegOpMode, OPMODE_FSK_STANDBY);

    // configure frequency
    configChannel();

    // set LNA gain
    writeReg(RegLna, LNA_RX_GAIN);

    // configure receiver
    writeReg(FSKRegRxConfig, 0x1E); // AFC auto, AGC, trigger on preamble?!?

    // set receiver bandwidth
    writeReg(FSKRegRxBw, 0x0B); // 50kHz SSb

    // set AFC bandwidth
    writeReg(FSKRegAfcBw, 0x12); // 83.3kHz SSB

    // set preamble detection
    writeReg(FSKRegPreambleDetect, 0xAA); // enable, 2 bytes, 10 chip errors

    // set sync config
    writeReg(FSKRegSyncConfig, 0x12); // no auto restart, preamble 0xAA, enable, fill FIFO, 3 bytes sync

    // set packet config
    writeReg(FSKRegPacketConfig1, 0xD8); // var-length, whitening, crc, no auto-clear, no adr filter
    writeReg(FSKRegPacketConfig2, 0x40); // packet mode

    // set sync value
    writeReg(FSKRegSyncValue1, 0xC1);
    writeReg(FSKRegSyncValue2, 0x94);
    writeReg(FSKRegSyncValue3, 0xC1);

    // set preamble timeout
    writeReg(FSKRegRxTimeout2, 0xFF);//(LMIC.rxsyms+1)/2);

    // set bitrate
    writeReg(FSKRegBitrateMsb, 0x02); // 50kbps
    writeReg(FSKRegBitrateLsb, 0x80);

    // set frequency deviation
    writeReg(FSKRegFdevMsb, 0x01); // +/- 25kHz
    writeReg(FSKRegFdevLsb, 0x99);

    // configure DIO mapping DIO0=PayloadReady DIO1=NOP DIO2=TimeOut
    writeReg(RegDioMapping1, MAP1_DIO0_FSK_READY | MAP1_DIO1_FSK_NOP | MAP1_DIO2_FSK_TIMEOUT);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO2);

    // enable antenna switch for RX
    hal_pin_rxtx(0);

    // now instruct the radio to receive
    hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
    writeReg(RegOpMode, OPMODE_FSK_RX); // no single rx mode available in FSK
}

static void startrx (u1_t rxmode) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );

    if (getSf(LMIC.rps) == FSK) { // FSK modem
        rxfsk(rxmode);
    } else { // LoRa modem
        rxlora(rxmode);
    }
    // the radio will go back to STANDBY mode as soon as the RX is finished
    // or timed out, and the corresponding IRQ will inform us about completion.
}

// reset radio
void radio_reset (void) {
    // drive RST pin
    hal_pin_rst(RST_PIN_RESET_STATE);

    // wait > 100us
    hal_waitUntil(os_getTime() + ms2osticks(1));

    // configure RST pin floating
    hal_pin_rst(2);

    // wait > 5ms
    hal_waitUntil(os_getTime() + ms2osticks(10));

    // check opmode
    ASSERT( readReg(RegOpMode) == OPMODE_FSK_STANDBY );
}

void radio_init (void) {
    hal_disableIRQs();

    // reset radio (FSK/STANDBY)
    radio_reset();

    // go to SLEEP mode
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);

    // sanity check, read version number
    ASSERT( readReg(RegVersion) == RADIO_VERSION );

    // seed 15-byte randomness via noise rssi in LoRa mode
    LMIC.rps = MAKERPS(SF7, BW125, CR_4_5, 0, 0);
    LMIC.freq = 434000000; // default LF freq
    rxlora(RXMODE_RSSI);
    while ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX); // continuous rx
    for (int i = 1; i < 16; i++) {
        for (int j = 0; j < 8; j++) {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while ((b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01));
            state.randbuf[i] = (state.randbuf[i] << 1) | b;
        }
    }
    state.randbuf[0] = 16; // set initial index
    hal_pin_rxtx(-1);
    writeReg(RegOpMode, OPMODE_LORA_SLEEP);

#ifdef BRD_sx1276_radio
    // receiver chain calibration

    // switch to FSK/STANDBY mode
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
    writeReg(RegOpMode, OPMODE_FSK_STANDBY);

    // cut the PA, just in case
    writeReg(RegPaConfig, 0x00);

    // set a frequency in LF band
    LMIC.freq = 434000000;
    configChannel();

    // rx chain calibration for LF band
    writeReg(FSKRegImageCal, RF_IMAGECAL_IMAGECAL_START);
    while (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING);

    // set a frequency in HF band
    LMIC.freq = 868300000;
    configChannel();

    // rx chain calibration for HF band
    writeReg(FSKRegImageCal, RF_IMAGECAL_IMAGECAL_START);
    while (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING);

    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
#endif

    // by default, DIO5 is configured as clock output. remap to NOP to reduce noise.
    writeReg(RegDioMapping2, MAP2_DIO5_LORA_NOP);

    hal_enableIRQs();
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
u1_t radio_rand1 (void) {
    u1_t i = state.randbuf[0];
    ASSERT( i != 0 );
    if (i == 16) {
        os_aes(AES_ENC, state.randbuf, 16); // encrypt seed with any key
        i = 0;
    }
    u1_t v = state.randbuf[i++];
    state.randbuf[0] = i;
    return v;
}

static void radio_irq_func (osjob_t* j) {
    // dispatch modem
    if (getSf(LMIC.rps) == FSK) { // FSK modem

	// not supported
	ASSERT(0);

    } else { // LORA modem
	u1_t irqflags = readReg(LORARegIrqFlags);

        if (irqflags & IRQ_LORA_TXDONE_MASK) { // TXDONE
	    BACKTRACE();

            // save exact tx time
            LMIC.txend = state.irqtime - LORA_TXDONE_FIXUP;

        } else if (irqflags & IRQ_LORA_RXDONE_MASK) { // RXDONE (rx or scan)
	    BACKTRACE();

            // read rx quality parameters
            LMIC.snr  = readReg(LORARegPktSnrValue);  // SNR [dB] * 4
	    LMIC.rssi = readReg(LORARegPktRssiValue); // final values -128..127 correspond to -196...+63 dBm (subtract RSSI_OFF)
	    if (LMIC.snr < 0) {
		LMIC.rssi = -RSSI_HF_CONST + LMIC.rssi + LMIC.snr/4 + RSSI_OFF;
	    } else {
		LMIC.rssi = -RSSI_HF_CONST + LMIC.rssi * 16/15 + RSSI_OFF;
	    }

            // get PDU length
            LMIC.dataLen = readReg(LORARegRxNbBytes);

            // save exact rx timestamps
            LMIC.rxtime = state.irqtime; // end of frame timestamp
            if (getBw(LMIC.rps) == BW125) {
                LMIC.rxtime -= LORA_RXDONE_FIXUP_125[getSf(LMIC.rps)];
            }
            else if (getBw(LMIC.rps) == BW500) {
                LMIC.rxtime -= LORA_RXDONE_FIXUP_500[getSf(LMIC.rps)];
            }
	    LMIC.rxtime0 = LMIC.rxtime - calcAirTime(LMIC.rps, LMIC.dataLen); // beginning of frame timestamp

	    // set FIFO read address pointer (to address of last packet received)
	    writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr));

	    // read FIFO
	    radio_readBuf(RegFifo, LMIC.frame, LMIC.dataLen);
#ifdef DEBUG_RX
	    debug_printf("RX[freq=%.1F,sf=%d,bw=%s,len=%d%s]: %h\r\n",
		    LMIC.freq, 6,
		    getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)),
		    LMIC.dataLen,
		    "",
		    LMIC.frame, LMIC.dataLen);
#endif
	} else if (irqflags & IRQ_LORA_RXTOUT_MASK) { // RXTOUT
	    BACKTRACE();

            // indicate timeout
            LMIC.dataLen = 0;

        } else {
	    // unexpected irq
	    ASSERT(0);
	}

	// mask all LoRa IRQs
	writeReg(LORARegIrqFlagsMask, 0xFF);

	// clear LoRa IRQ flags
	writeReg(LORARegIrqFlags, 0xFF);
    }

    // go from standby to sleep
    writeReg(RegOpMode, OPMODE_LORA_SLEEP);

    // run os job (use preset func ptr)
    // (an eventually scheduled timeout job will be replaced)
    os_setCallback(&LMIC.osjob, LMIC.osjob.func);

#ifdef CFG_testpin
    if (LMIC.testpinMode & 0x80) {
	// clear GPIO signal
	CFG_PIN_DEFAULT(CFG_testpin);
	// reset signaling request
	LMIC.testpinMode = 0;
    }
    if( LMIC.testpinMode == 2 ) {
	// enable for next IRQ which should be RX	
	LMIC.testpinMode = 1;
    }
#endif
}

#define IRQ_GUARD_TICKS sec2osticks(15)

// guard timeout in case interrupt is not asserted by radio
static void radio_irq_guard (osjob_t* j) {
    // radio is in weird state -- let's reboot
    ASSERT(0);
}

// called by hal exti IRQ handler (radio is in STANDBY mode now)
void radio_irq_handler (u1_t diomask) {
#ifdef CFG_testpin
    // signal (txdone) irq on GPIO line
    if (LMIC.testpinMode == 1) {
	CFG_PIN_VAL(CFG_testpin, GPIOCFG_MODE_OUT | GPIOCFG_OSPEED_40MHz | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_NONE, 1);
	LMIC.testpinMode |= 0x80;
    }
#endif
    // save current time
    state.irqtime = os_getTime();

    // check number of interrupt sources
    ASSERT( __builtin_popcount(diomask) == 1 );

    BACKTRACE();

    // disable rx/tx switch
    hal_pin_rxtx(-1);

    // disable IRQs in HAL
    hal_irqmask_set(0);

    // schedule irq job
    os_setCallback(&state.irqjob, radio_irq_func);
}

// stop radio, disarm interrupts, cancel jobs
static void radio_stop (void) {
    hal_disableIRQs();
    // put radio to sleep
    writeReg(RegOpMode, OPMODE_LORA_SLEEP); // LoRa/FSK bit is ignored when not in SLEEP mode
    // disable rx/tx switch
    hal_pin_rxtx(-1);
    // disable IRQs in HAL
    hal_irqmask_set(0);
    // cancel radio job
    os_clearCallback(&state.irqjob);
    hal_enableIRQs();
}

void os_radio (u1_t mode) {
    switch (mode) {
	case RADIO_RST:
	    radio_stop();
	    break;

	case RADIO_TX:
	    radio_stop();
	    // set timeout for tx operation
	    os_setTimedCallback(&state.irqjob, os_getTime() + IRQ_GUARD_TICKS, radio_irq_guard);
	    // transmit frame now
	    starttx(); // buf=LMIC.frame, len=LMIC.dataLen
	    break;

	case RADIO_RX:
	    radio_stop();
	    // set timeout for rx operation
	    os_setTimedCallback(&state.irqjob, LMIC.rxtime + IRQ_GUARD_TICKS, radio_irq_guard);
	    // receive frame now (exactly at rxtime)
	    startrx(RXMODE_SINGLE); // buf=LMIC.frame, time=LMIC.rxtime, timeout=LMIC.rxsyms
	    break;

	case RADIO_RXON:
	    radio_stop();
	    // start scanning for frame now
	    startrx(RXMODE_SCAN); // buf=LMIC.frame
	    break;

	case RADIO_TXCW:
	    radio_stop();
	    // transmit CW continuous -- end with os_radio(RADIO_RST)
	    txcw();
	    break;
    }
}
