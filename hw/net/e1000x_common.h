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

/* Offload Context Descriptor */
struct e1000_context_desc {
    union {
        uint32_t ip_config;
        struct {
            uint8_t ipcss;      /* IP checksum start */
            uint8_t ipcso;      /* IP checksum offset */
            uint16_t ipcse;     /* IP checksum end */
        } ip_fields;
    } lower_setup;
    union {
        uint32_t tcp_config;
        struct {
            uint8_t tucss;      /* TCP checksum start */
            uint8_t tucso;      /* TCP checksum offset */
            uint16_t tucse;     /* TCP checksum end */
        } tcp_fields;
    } upper_setup;
    uint32_t cmd_and_length;    /* */
    union {
        uint32_t data;
        struct {
            uint8_t status;     /* Descriptor status */
            uint8_t hdr_len;    /* Header length */
            uint16_t mss;       /* Maximum segment size */
        } fields;
    } tcp_seg_setup;
};

void e1000x_inc_reg_if_not_full(uint32_t *mac, int index);

void e1000x_grow_8reg_if_not_full(uint32_t *mac, int index, int size);

int e1000x_vlan_enabled(uint32_t *mac);

int e1000x_is_vlan_txd(uint32_t txd_lower);

int e1000x_vlan_rx_filter_enabled(uint32_t *mac);

int e1000x_fcs_len(uint32_t *mac);

void e1000x_update_regs_on_link_down(uint32_t *mac, uint16_t *phy);

void e1000x_update_regs_on_link_up(uint32_t *mac, uint16_t *phy);

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
