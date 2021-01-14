/*
* QEMU e1000(e) emulation - shared code
*
* Copyright (c) 2008 Qumranet
*
* Based on work done by:
* Nir Peleg, Tutis Systems Ltd. for Qumranet Inc.
* Copyright (c) 2007 Dan Aloni
* Copyright (c) 2004 Antony T Curtis
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HW_NET_IGB_ENUMS_H
#define HW_NET_IGB_ENUMS_H

#include "igb_regs_new.h"
#include "igb_regs_tmp.h"

#define defreg(x)   x = (E1000_##x >> 2)

enum {
    defreg(CTRL),    defreg(EECD),    defreg(EERD),    defreg(GPRC),
    defreg(GPTC),    defreg(ICR),     defreg(ICS),     defreg(IMC),
    defreg(IMS),     defreg(LEDCTL),  defreg(MANC),    defreg(MDIC),
    defreg(MPC),     defreg(PBA),     defreg(RCTL),    defreg(RDBAH0),
    defreg(RDH0),    defreg(RDLEN0),  defreg(RDT0),
    defreg(STATUS),  defreg(SWSM),    defreg(TCTL),    defreg(TDBAH),
    defreg(TDBAL),   defreg(TDH),     defreg(TDLEN),   defreg(TDT),
    defreg(TDLEN1),  defreg(TDBAL1),  defreg(TDBAH1),  defreg(TDH1),
    defreg(TDT1),    defreg(TORH),    defreg(TORL),    defreg(TOTH),
    defreg(TOTL),    defreg(TPR),     defreg(TPT),     defreg(TXDCTL),
    defreg(WUFC),    defreg(RA),      defreg(MTA),     defreg(CRCERRS),
    defreg(VFTA),    defreg(VET),     defreg(RDTR),    defreg(RADV),
    defreg(TADV),    defreg(ITR),     defreg(SCC),     defreg(ECOL),
    defreg(MCC),     defreg(LATECOL), defreg(COLC),    defreg(DC),
    defreg(TNCRS),   defreg(SEQEC),   defreg(CEXTERR), defreg(RLEC),
    defreg(XONRXC),  defreg(XONTXC),  defreg(XOFFRXC), defreg(XOFFTXC),
    defreg(FCRUC),   defreg(AIT),     defreg(TDFH),    defreg(TDFT),
    defreg(TDFHS),   defreg(TDFTS),   defreg(TDFPC),   defreg(WUC),
    defreg(WUS),     defreg(POEMB),   defreg(PBS),     defreg(RDFH),
    defreg(RDFT),    defreg(RDFHS),   defreg(RDFTS),   defreg(RDFPC),
    defreg(PBM),     defreg(IPAV),    defreg(IP4AT),   defreg(IP6AT),
    defreg(WUPM),    defreg(FFLT),    defreg(FFMT),    defreg(FFVT),
    defreg(TARC0),   defreg(TARC1),   defreg(IAM),     defreg(EXTCNF_CTRL),
    defreg(GCR),     defreg(TIMINCA), defreg(EIAC),    defreg(CTRL_EXT),
    defreg(IVAR),    defreg(MFUTP01), defreg(MFUTP23), defreg(MANC2H),
    defreg(MFVAL),   defreg(MDEF),    defreg(FACTPS),  defreg(FTFT),
    defreg(RUC),     defreg(ROC),     defreg(RFC),     defreg(RJC),
    defreg(PRC64),   defreg(PRC127),  defreg(PRC255),  defreg(PRC511),
    defreg(PRC1023), defreg(PRC1522), defreg(PTC64),   defreg(PTC127),
    defreg(PTC255),  defreg(PTC511),  defreg(PTC1023), defreg(PTC1522),
    defreg(GORCL),   defreg(GORCH),   defreg(GOTCL),   defreg(GOTCH),
    defreg(RNBC),    defreg(BPRC),    defreg(MPRC),    defreg(RFCTL),
    defreg(PSRCTL),  defreg(MPTC),    defreg(BPTC),    defreg(TSCTFC),
    defreg(IAC),     defreg(MGTPRC),  defreg(MGTPDC),  defreg(MGTPTC),
    defreg(TSCTC),   defreg(RXCSUM),  defreg(FUNCTAG), defreg(GSCL_1),
    defreg(GSCL_2),  defreg(GSCL_3),  defreg(GSCL_4),  defreg(GSCN_0),
    defreg(GSCN_1),  defreg(GSCN_2),  defreg(GSCN_3),  defreg(GCR2),
    defreg(RAID),    defreg(RSRPD),   defreg(TIDV),    defreg(EITR),
    defreg(MRQC),    defreg(RETA),    defreg(RSSRK),   defreg(RDBAH1),
    defreg(RDLEN1),  defreg(RDH1),    defreg(RDT1),
    defreg(PBACLR),  defreg(FCAL),    defreg(FCAH),    defreg(FCT),
    defreg(FCRTH),   defreg(FCRTL),   defreg(FCTTV),   defreg(FCRTV),
    defreg(FLA),     defreg(EEWR),    defreg(FLOP),    defreg(FLOL),
    defreg(FLSWCTL), defreg(FLSWCNT), defreg(RXDCTL),  defreg(RXDCTL1),
    defreg(MAVTV0),  defreg(MAVTV1),  defreg(MAVTV2),  defreg(MAVTV3),
    defreg(TXSTMPL), defreg(TXSTMPH), defreg(SYSTIML), defreg(SYSTIMH),
    defreg(RXCFGL),  defreg(RXUDP),   defreg(TIMADJL), defreg(TIMADJH),
    defreg(RXSTMPH), defreg(RXSTMPL), defreg(RXSATRL), defreg(RXSATRH),
    defreg(FLASHT),  defreg(TIPG),    defreg(RDH),     defreg(RDT),
    defreg(RDLEN),   defreg(RDBAH),
    defreg(TXDCTL1),
    defreg(FLSWDATA),
    defreg(CTRL_DUP),
    defreg(EXTCNF_SIZE),
    defreg(EEMNGCTL),
    defreg(EEMNGDATA),
    defreg(FLMNGCTL),
    defreg(FLMNGDATA),
    defreg(FLMNGCNT),
    defreg(TSYNCRXCTL),
    defreg(TSYNCTXCTL),

    /* Aliases */
    defreg(RDH0_A),  defreg(RDT0_A),  defreg(RDTR_A),  defreg(RDFH_A),
    defreg(RDFT_A),  defreg(TDH_A),   defreg(TDT_A),   defreg(TIDV_A),
    defreg(TDFH_A),  defreg(TDFT_A),  defreg(RA_A),
    defreg(TDBAL_A), defreg(TDLEN_A), defreg(VFTA_A),  defreg(RDLEN0_A),
    defreg(FCRTL_A), defreg(FCRTH_A), defreg(RDBAH0_A),

    /* Additional regs used by IGB */
    defreg(FWSM),   defreg(SW_FW_SYNC), defreg(HTCBDPC), defreg(GPIE),
    defreg(EICR),   defreg(EICS),       defreg(EIMS),    defreg(EIAM),
    defreg(EIMC),   defreg(TXPBS),      defreg(TCTL_EXT),
    defreg(DTXCTL), defreg(RXPBS),      defreg(RQDPC),

    defreg(RDBAL0),  defreg(RDBAL1),  defreg(RDBAL2),  defreg(RDBAL3),
    defreg(RDBAL4),  defreg(RDBAL5),  defreg(RDBAL6),  defreg(RDBAL7),
    defreg(RDBAL8),  defreg(RDBAL9),  defreg(RDBAL10), defreg(RDBAL11),
    defreg(RDBAL12), defreg(RDBAL13), defreg(RDBAL14), defreg(RDBAL15),

    /*defreg(RDBAH0),  defreg(RDBAH1),*/  defreg(RDBAH2),  defreg(RDBAH3),
    defreg(RDBAH4),  defreg(RDBAH5),  defreg(RDBAH6),  defreg(RDBAH7),
    defreg(RDBAH8),  defreg(RDBAH9),  defreg(RDBAH10), defreg(RDBAH11),
    defreg(RDBAH12), defreg(RDBAH13), defreg(RDBAH14), defreg(RDBAH15),

    /*defreg(RDLEN0),  defreg(RDLEN1),*/  defreg(RDLEN2),  defreg(RDLEN3),
    defreg(RDLEN4),  defreg(RDLEN5),  defreg(RDLEN6),  defreg(RDLEN7),
    defreg(RDLEN8),  defreg(RDLEN9),  defreg(RDLEN10), defreg(RDLEN11),
    defreg(RDLEN12), defreg(RDLEN13), defreg(RDLEN14), defreg(RDLEN15),

    defreg(SRRCTL0),  defreg(SRRCTL1),  defreg(SRRCTL2),  defreg(SRRCTL3),
    defreg(SRRCTL4),  defreg(SRRCTL5),  defreg(SRRCTL6),  defreg(SRRCTL7),
    defreg(SRRCTL8),  defreg(SRRCTL9),  defreg(SRRCTL10), defreg(SRRCTL11),
    defreg(SRRCTL12), defreg(SRRCTL13), defreg(SRRCTL14), defreg(SRRCTL15),

    /*defreg(RDH0),  defreg(RDH1),*/  defreg(RDH2),  defreg(RDH3),
    defreg(RDH4),  defreg(RDH5),  defreg(RDH6),  defreg(RDH7),
    defreg(RDH8),  defreg(RDH9),  defreg(RDH10), defreg(RDH11),
    defreg(RDH12), defreg(RDH13), defreg(RDH14), defreg(RDH15),

    /*defreg(RDT0),  defreg(RDT1),*/  defreg(RDT2),  defreg(RDT3),
    defreg(RDT4),  defreg(RDT5),  defreg(RDT6),  defreg(RDT7),
    defreg(RDT8),  defreg(RDT9),  defreg(RDT10), defreg(RDT11),
    defreg(RDT12), defreg(RDT13), defreg(RDT14), defreg(RDT15),

    defreg(RXDCTL0),  /*defreg(RXDCTL1),*/  defreg(RXDCTL2),  defreg(RXDCTL3),
    defreg(RXDCTL4),  defreg(RXDCTL5),  defreg(RXDCTL6),  defreg(RXDCTL7),
    defreg(RXDCTL8),  defreg(RXDCTL9),  defreg(RXDCTL10), defreg(RXDCTL11),
    defreg(RXDCTL12), defreg(RXDCTL13), defreg(RXDCTL14), defreg(RXDCTL15),

    defreg(TDBAL0),  /*defreg(TDBAL1),*/  defreg(TDBAL2),  defreg(TDBAL3),
    defreg(TDBAL4),  defreg(TDBAL5),  defreg(TDBAL6),  defreg(TDBAL7),
    defreg(TDBAL8),  defreg(TDBAL9),  defreg(TDBAL10), defreg(TDBAL11),
    defreg(TDBAL12), defreg(TDBAL13), defreg(TDBAL14), defreg(TDBAL15),

    defreg(TDBAH0),  /*defreg(TDBAH1),*/  defreg(TDBAH2),  defreg(TDBAH3),
    defreg(TDBAH4),  defreg(TDBAH5),  defreg(TDBAH6),  defreg(TDBAH7),
    defreg(TDBAH8),  defreg(TDBAH9),  defreg(TDBAH10), defreg(TDBAH11),
    defreg(TDBAH12), defreg(TDBAH13), defreg(TDBAH14), defreg(TDBAH15),

    defreg(TDLEN0),  /*defreg(TDLEN1),*/  defreg(TDLEN2),  defreg(TDLEN3),
    defreg(TDLEN4),  defreg(TDLEN5),  defreg(TDLEN6),  defreg(TDLEN7),
    defreg(TDLEN8),  defreg(TDLEN9),  defreg(TDLEN10), defreg(TDLEN11),
    defreg(TDLEN12), defreg(TDLEN13), defreg(TDLEN14), defreg(TDLEN15),

    defreg(TDH0),  /*defreg(TDH1),*/  defreg(TDH2),  defreg(TDH3),
    defreg(TDH4),  defreg(TDH5),  defreg(TDH6),  defreg(TDH7),
    defreg(TDH8),  defreg(TDH9),  defreg(TDH10), defreg(TDH11),
    defreg(TDH12), defreg(TDH13), defreg(TDH14), defreg(TDH15),

    defreg(TDT0),  /*defreg(TDT1),*/  defreg(TDT2),  defreg(TDT3),
    defreg(TDT4),  defreg(TDT5),  defreg(TDT6),  defreg(TDT7),
    defreg(TDT8),  defreg(TDT9),  defreg(TDT10), defreg(TDT11),
    defreg(TDT12), defreg(TDT13), defreg(TDT14), defreg(TDT15),

    defreg(TXDCTL0),  /*defreg(TXDCTL1),*/  defreg(TXDCTL2),  defreg(TXDCTL3),
    defreg(TXDCTL4),  defreg(TXDCTL5),  defreg(TXDCTL6),  defreg(TXDCTL7),
    defreg(TXDCTL8),  defreg(TXDCTL9),  defreg(TXDCTL10), defreg(TXDCTL11),
    defreg(TXDCTL12), defreg(TXDCTL13), defreg(TXDCTL14), defreg(TXDCTL15),

    defreg(PFMAILBOX),  defreg(VFMAILBOX), defreg(MBVFICR), defreg(VMBMEM),
    defreg(MBVFIMR),    defreg(VFLRE),  defreg(VFRE),   defreg(VFTE),
    defreg(QDE),        defreg(DTXSWC), defreg(WVBR),   defreg(VMVIR),
    defreg(VMOLR),      defreg(RPLOLR), defreg(VLVF),

    defreg(VTCTRL0),  defreg(VTCTRL1),  defreg(VTCTRL2),  defreg(VTCTRL3),
    defreg(VTCTRL4),  defreg(VTCTRL5),  defreg(VTCTRL6),  defreg(VTCTRL7),
    defreg(VTEICS0),  defreg(VTEICS1),  defreg(VTEICS2),  defreg(VTEICS3),
    defreg(VTEICS4),  defreg(VTEICS5),  defreg(VTEICS6),  defreg(VTEICS7),
    defreg(VTEIMS0),  defreg(VTEIMS1),  defreg(VTEIMS2),  defreg(VTEIMS3),
    defreg(VTEIMS4),  defreg(VTEIMS5),  defreg(VTEIMS6),  defreg(VTEIMS7),
    defreg(VTEIMC0),  defreg(VTEIMC1),  defreg(VTEIMC2),  defreg(VTEIMC3),
    defreg(VTEIMC4),  defreg(VTEIMC5),  defreg(VTEIMC6),  defreg(VTEIMC7),
    defreg(VTEIAC0),  defreg(VTEIAC1),  defreg(VTEIAC2),  defreg(VTEIAC3),
    defreg(VTEIAC4),  defreg(VTEIAC5),  defreg(VTEIAC6),  defreg(VTEIAC7),
    defreg(VTEIAM0),  defreg(VTEIAM1),  defreg(VTEIAM2),  defreg(VTEIAM3),
    defreg(VTEIAM4),  defreg(VTEIAM5),  defreg(VTEIAM6),  defreg(VTEIAM7),
    defreg(VTEICR0),  defreg(VTEICR1),  defreg(VTEICR2),  defreg(VTEICR3),
    defreg(VTEICR4),  defreg(VTEICR5),  defreg(VTEICR6),  defreg(VTEICR7),
    defreg(VFGPRC0),  defreg(VFGPRC1),  defreg(VFGPRC2),  defreg(VFGPRC3),
    defreg(VFGPRC4),  defreg(VFGPRC5),  defreg(VFGPRC6),  defreg(VFGPRC7),
    defreg(VFGPTC0),  defreg(VFGPTC1),  defreg(VFGPTC2),  defreg(VFGPTC3),
    defreg(VFGPTC4),  defreg(VFGPTC5),  defreg(VFGPTC6),  defreg(VFGPTC7),
    defreg(VFGORC0),  defreg(VFGORC1),  defreg(VFGORC2),  defreg(VFGORC3),
    defreg(VFGORC4),  defreg(VFGORC5),  defreg(VFGORC6),  defreg(VFGORC7),
    defreg(VFGOTC0),  defreg(VFGOTC1),  defreg(VFGOTC2),  defreg(VFGOTC3),
    defreg(VFGOTC4),  defreg(VFGOTC5),  defreg(VFGOTC6),  defreg(VFGOTC7),
    defreg(VFMPRC0),  defreg(VFMPRC1),  defreg(VFMPRC2),  defreg(VFMPRC3),
    defreg(VFMPRC4),  defreg(VFMPRC5),  defreg(VFMPRC6),  defreg(VFMPRC7),
    defreg(VFGPRLBC0),  defreg(VFGPRLBC1),  defreg(VFGPRLBC2),
    defreg(VFGPRLBC3),  defreg(VFGPRLBC4),  defreg(VFGPRLBC5),
    defreg(VFGPRLBC6),  defreg(VFGPRLBC7),  defreg(VFGPTLBC0),
    defreg(VFGPTLBC1),  defreg(VFGPTLBC2),  defreg(VFGPTLBC3),
    defreg(VFGPTLBC4),  defreg(VFGPTLBC5),  defreg(VFGPTLBC6),
    defreg(VFGPTLBC7),  defreg(VFGORLBC0),  defreg(VFGORLBC1),
    defreg(VFGORLBC2),  defreg(VFGORLBC3),  defreg(VFGORLBC4),
    defreg(VFGORLBC5),  defreg(VFGORLBC6),  defreg(VFGORLBC7),
    defreg(VFGOTLBC0),  defreg(VFGOTLBC1),  defreg(VFGOTLBC2),
    defreg(VFGOTLBC3),  defreg(VFGOTLBC4),  defreg(VFGOTLBC5),
    defreg(VFGOTLBC6),  defreg(VFGOTLBC7),

    defreg(IVAR_MISC),  defreg(VTIVAR), defreg(VTIVAR_MISC),

    defreg(RDBAL0_ALT), defreg(RDBAL1_ALT), defreg(RDBAL2_ALT),
    defreg(RDBAL3_ALT),

};

#endif
