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

#ifndef HW_NET_E1000X_COMMON_H
#define HW_NET_E1000X_COMMON_H

#include "e1000_regs.h"

#define defreg(x)   x = (E1000_##x >> 2)
#define idefreg(x)  I_##x = (IGB_##x >> 2)
enum {
    defreg(CTRL),    defreg(EECD),    defreg(EERD),    defreg(GPRC),
    defreg(GPTC),    defreg(ICR),     defreg(ICS),     defreg(IMC),
    defreg(IMS),     defreg(LEDCTL),  defreg(MANC),    defreg(MDIC),
    defreg(MPC),     defreg(PBA),     defreg(RCTL),    defreg(RDBAH0),
    defreg(RDBAL0),  defreg(RDH0),    defreg(RDLEN0),  defreg(RDT0),
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
    defreg(RDBAL1),  defreg(RDLEN1),  defreg(RDH1),    defreg(RDT1),
    defreg(PBACLR),  defreg(FCAL),    defreg(FCAH),    defreg(FCT),
    defreg(FCRTH),   defreg(FCRTL),   defreg(FCTTV),   defreg(FCRTV),
    defreg(FLA),     defreg(EEWR),    defreg(FLOP),    defreg(FLOL),
    defreg(FLSWCTL), defreg(FLSWCNT), defreg(RXDCTL),  defreg(RXDCTL1),
    defreg(MAVTV0),  defreg(MAVTV1),  defreg(MAVTV2),  defreg(MAVTV3),
    defreg(TXSTMPL), defreg(TXSTMPH), defreg(SYSTIML), defreg(SYSTIMH),
    defreg(RXCFGL),  defreg(RXUDP),   defreg(TIMADJL), defreg(TIMADJH),
    defreg(RXSTMPH), defreg(RXSTMPL), defreg(RXSATRL), defreg(RXSATRH),
    defreg(FLASHT),  defreg(TIPG),    defreg(RDH),     defreg(RDT),
    defreg(RDLEN),   defreg(RDBAH),   defreg(RDBAL),
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
    defreg(TDFH_A),  defreg(TDFT_A),  defreg(RA_A),    defreg(RDBAL0_A),
    defreg(TDBAL_A), defreg(TDLEN_A), defreg(VFTA_A),  defreg(RDLEN0_A),
    defreg(FCRTL_A), defreg(FCRTH_A),

    /* Additional regs used by IGB */
    defreg(FWSM),   defreg(SW_FW_SYNC), defreg(HTCBDPC), defreg(GPIE),
    defreg(EICR),   defreg(EICS),       defreg(EIMS),    defreg(EIAM),
    defreg(EIMC),   defreg(TXPBS),      defreg(TCTL_EXT),
    defreg(DTXCTL), defreg(RXPBS),      defreg(RQDPC),

    /*defreg(RDBAL0),  defreg(RDBAL1),*/  defreg(RDBAL2),  defreg(RDBAL3),
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

    idefreg(IVAR),  idefreg(EIAC), idefreg(EITR)
};

static inline void
e1000x_inc_reg_if_not_full(uint32_t *mac, int index)
{
    if (mac[index] != 0xffffffff) {
        mac[index]++;
    }
}

static inline void
e1000x_grow_8reg_if_not_full(uint32_t *mac, int index, int size)
{
    uint64_t sum = mac[index] | (uint64_t)mac[index + 1] << 32;

    if (sum + size < sum) {
        sum = ~0ULL;
    } else {
        sum += size;
    }
    mac[index] = sum;
    mac[index + 1] = sum >> 32;
}

static inline int
e1000x_vlan_enabled(uint32_t *mac)
{
    return ((mac[CTRL] & E1000_CTRL_VME) != 0);
}

static inline int
e1000x_is_vlan_txd(uint32_t txd_lower)
{
    return ((txd_lower & E1000_TXD_CMD_VLE) != 0);
}

static inline int
e1000x_vlan_rx_filter_enabled(uint32_t *mac)
{
    return ((mac[RCTL] & E1000_RCTL_VFE) != 0);
}

static inline int
e1000x_fcs_len(uint32_t *mac)
{
    /* FCS aka Ethernet CRC-32. We don't get it from backends and can't
    * fill it in, just pad descriptor length by 4 bytes unless guest
    * told us to strip it off the packet. */
    return (mac[RCTL] & E1000_RCTL_SECRC) ? 0 : 4;
}

static inline void
e1000x_update_regs_on_link_down(uint32_t *mac, uint16_t *phy)
{
    mac[STATUS] &= ~E1000_STATUS_LU;
    phy[PHY_STATUS] &= ~MII_SR_LINK_STATUS;
    phy[PHY_STATUS] &= ~MII_SR_AUTONEG_COMPLETE;
    phy[PHY_LP_ABILITY] &= ~MII_LPAR_LPACK;
}

static inline void
e1000x_update_regs_on_link_up(uint32_t *mac, uint16_t *phy)
{
    mac[STATUS] |= E1000_STATUS_LU;
    phy[PHY_STATUS] |= MII_SR_LINK_STATUS;
}

void e1000x_update_rx_total_stats(uint32_t *mac,
                                  size_t data_size,
                                  size_t data_fcs_size);

void e1000x_core_prepare_eeprom(uint16_t       *eeprom,
                                const uint16_t *templ,
                                uint32_t        templ_size,
                                uint16_t        dev_id,
                                const uint8_t  *macaddr);

uint32_t e1000x_rxbufsize(uint32_t rctl);

bool e1000x_rx_ready(PCIDevice *d, uint32_t *mac);

bool e1000x_is_vlan_packet(const uint8_t *buf, uint16_t vet);

bool e1000x_rx_group_filter(uint32_t *mac, const uint8_t *buf);

bool e1000x_hw_rx_enabled(uint32_t *mac);

bool e1000x_is_oversized(uint32_t *mac, size_t size);

void e1000x_restart_autoneg(uint32_t *mac, uint16_t *phy, QEMUTimer *timer);

void e1000x_reset_mac_addr(NICState *nic, uint32_t *mac_regs,
                           uint8_t *mac_addr);

void e1000x_update_regs_on_autoneg_done(uint32_t *mac, uint16_t *phy);

void e1000x_increase_size_stats(uint32_t *mac, const int *size_regs, int size);

typedef struct e1000x_txd_props {
    uint8_t ipcss;
    uint8_t ipcso;
    uint16_t ipcse;
    uint8_t tucss;
    uint8_t tucso;
    uint16_t tucse;
    uint32_t paylen;
    uint8_t hdr_len;
    uint16_t mss;
    int8_t ip;
    int8_t tcp;
    bool tse;
} e1000x_txd_props;

void e1000x_read_tx_ctx_descr(struct e1000_context_desc *d,
                              e1000x_txd_props *props);

#endif
