/*
* Core code for QEMU e1000e emulation
*
* Software developer's manuals:
* http://www.intel.com/content/dam/doc/datasheet/82574l-gbe-controller-datasheet.pdf
*
* Copyright (c) 2015 Ravello Systems LTD (http://ravellosystems.com)
* Developed by Daynix Computing LTD (http://www.daynix.com)
*
* Authors:
* Dmitry Fleytman <dmitry@daynix.com>
* Leonid Bloch <leonid@daynix.com>
* Yan Vugenfirer <yan@daynix.com>
*
* Based on work done by:
* Nir Peleg, Tutis Systems Ltd. for Qumranet Inc.
* Copyright (c) 2008 Qumranet
* Based on work done by:
* Copyright (c) 2007 Dan Aloni
* Copyright (c) 2004 Antony T Curtis
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "net/net.h"
#include "net/tap.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "sysemu/runstate.h"

#include "net_tx_pkt.h"
#include "net_rx_pkt.h"

#include "e1000x_common.h"
#include "igb_enums.h"
#include "igb_core.h"
#include "igb_regs_new.h"

#include "trace.h"

#define E1000E_MIN_XITR     (500) /* No more then 7813 interrupts per
                                     second according to spec 10.2.4.2 */
#define E1000E_MAX_TX_FRAGS (64)

static void igb_update_interrupt_state(E1000ECore *core);

static inline void
e1000e_set_interrupt_cause(E1000ECore *core, uint32_t val);

static inline void
e1000e_raise_legacy_irq(E1000ECore *core)
{
    trace_e1000e_irq_legacy_notify(true);
    e1000x_inc_reg_if_not_full(core->mac, IAC);
    pci_set_irq(core->owner, 1);
}

static inline void
e1000e_lower_legacy_irq(E1000ECore *core)
{
    trace_e1000e_irq_legacy_notify(false);
    pci_set_irq(core->owner, 0);
}

static inline void
e1000e_intrmgr_rearm_timer(E1000IntrDelayTimer *timer)
{
    int64_t delay_ns = (int64_t) timer->core->mac[timer->delay_reg] *
                                 timer->delay_resolution_ns;

    trace_e1000e_irq_rearm_timer(timer->delay_reg << 2, delay_ns);

    timer_mod(timer->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);

    timer->running = true;
}

static void
e1000e_intmgr_timer_resume(E1000IntrDelayTimer *timer)
{
    if (timer->running) {
        e1000e_intrmgr_rearm_timer(timer);
    }
}

static void
e1000e_intmgr_timer_pause(E1000IntrDelayTimer *timer)
{
    if (timer->running) {
        timer_del(timer->timer);
    }
}

static inline void
e1000e_intrmgr_stop_timer(E1000IntrDelayTimer *timer)
{
    if (timer->running) {
        timer_del(timer->timer);
        timer->running = false;
    }
}

static inline void
e1000e_intrmgr_fire_delayed_interrupts(E1000ECore *core)
{
    trace_e1000e_irq_fire_delayed_interrupts();
    e1000e_set_interrupt_cause(core, 0);
}

static void
e1000e_intrmgr_on_timer(void *opaque)
{
    E1000IntrDelayTimer *timer = opaque;

    trace_e1000e_irq_throttling_timer(timer->delay_reg << 2);

    timer->running = false;
    e1000e_intrmgr_fire_delayed_interrupts(timer->core);
}

static void
e1000e_intrmgr_on_msix_throttling_timer(void *opaque)
{
    E1000IntrDelayTimer *timer = opaque;
    int idx = timer - &timer->core->eitr[0];

    assert(msix_enabled(timer->core->owner));

    timer->running = false;

    if (!timer->core->eitr_intr_pending[idx]) {
        trace_e1000e_irq_throttling_no_pending_vec(idx);
        return;
    }

    trace_e1000e_irq_msix_notify_postponed_vec(idx);
    msix_notify(timer->core->owner, idx);
}

static void
e1000e_intrmgr_initialize_all_timers(E1000ECore *core, bool create)
{
    int i;

    core->radv.delay_reg = RADV;
    core->rdtr.delay_reg = RDTR;
    core->raid.delay_reg = RAID;
    core->tadv.delay_reg = TADV;
    core->tidv.delay_reg = TIDV;

    core->radv.delay_resolution_ns = E1000_INTR_DELAY_NS_RES;
    core->rdtr.delay_resolution_ns = E1000_INTR_DELAY_NS_RES;
    core->raid.delay_resolution_ns = E1000_INTR_DELAY_NS_RES;
    core->tadv.delay_resolution_ns = E1000_INTR_DELAY_NS_RES;
    core->tidv.delay_resolution_ns = E1000_INTR_DELAY_NS_RES;

    core->radv.core = core;
    core->rdtr.core = core;
    core->raid.core = core;
    core->tadv.core = core;
    core->tidv.core = core;

    for (i = 0; i < IGB_MSIX_VEC_NUM; i++) {
        core->eitr[i].core = core;
        core->eitr[i].delay_reg = EITR + i;
        core->eitr[i].delay_resolution_ns = E1000_INTR_DELAY_NS_RES;
    }

    if (!create) {
        return;
    }

    core->radv.timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, e1000e_intrmgr_on_timer, &core->radv);
    core->rdtr.timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, e1000e_intrmgr_on_timer, &core->rdtr);
    core->raid.timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, e1000e_intrmgr_on_timer, &core->raid);

    core->tadv.timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, e1000e_intrmgr_on_timer, &core->tadv);
    core->tidv.timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL, e1000e_intrmgr_on_timer, &core->tidv);

    for (i = 0; i < IGB_MSIX_VEC_NUM; i++) {
        core->eitr[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
            e1000e_intrmgr_on_msix_throttling_timer, &core->eitr[i]);
    }
}

static inline void
e1000e_intrmgr_stop_delay_timers(E1000ECore *core)
{
    e1000e_intrmgr_stop_timer(&core->radv);
    e1000e_intrmgr_stop_timer(&core->rdtr);
    e1000e_intrmgr_stop_timer(&core->raid);
    e1000e_intrmgr_stop_timer(&core->tidv);
    e1000e_intrmgr_stop_timer(&core->tadv);
}

static bool
e1000e_intrmgr_delay_rx_causes(E1000ECore *core, uint32_t *causes)
{
    uint32_t delayable_causes;
    uint32_t rdtr = core->mac[RDTR];
    uint32_t radv = core->mac[RADV];
    uint32_t raid = core->mac[RAID];

    if (msix_enabled(core->owner)) {
        return false;
    }

    delayable_causes = E1000_ICR_RXQ0 |
                       E1000_ICR_RXQ1 |
                       E1000_ICR_RXT0;

    /* Clean up all causes that may be delayed */
    core->delayed_causes |= *causes & delayable_causes;
    *causes &= ~delayable_causes;

    /* Check if delayed RX interrupts disabled by client
       or if there are causes that cannot be delayed */
    if ((rdtr == 0) || (*causes != 0)) {
        return false;
    }

    /* Check if delayed RX ACK interrupts disabled by client
       and there is an ACK packet received */
    if ((raid == 0) && (core->delayed_causes & E1000_ICR_ACK)) {
        return false;
    }

    /* All causes delayed */
    e1000e_intrmgr_rearm_timer(&core->rdtr);

    if (!core->radv.running && (radv != 0)) {
        e1000e_intrmgr_rearm_timer(&core->radv);
    }

    if (!core->raid.running && (core->delayed_causes & E1000_ICR_ACK)) {
        e1000e_intrmgr_rearm_timer(&core->raid);
    }

    return true;
}

static bool
e1000e_intrmgr_delay_tx_causes(E1000ECore *core, uint32_t *causes)
{
    static const uint32_t delayable_causes = E1000_ICR_TXQ0 |
                                             E1000_ICR_TXQ1 |
                                             E1000_ICR_TXQE |
                                             E1000_ICR_TXDW;

    if (msix_enabled(core->owner)) {
        return false;
    }

    /* Clean up all causes that may be delayed */
    core->delayed_causes |= *causes & delayable_causes;
    *causes &= ~delayable_causes;

    /* If there are causes that cannot be delayed */
    if (*causes != 0) {
        return false;
    }

    /* All causes delayed */
    e1000e_intrmgr_rearm_timer(&core->tidv);

    if (!core->tadv.running && (core->mac[TADV] != 0)) {
        e1000e_intrmgr_rearm_timer(&core->tadv);
    }

    return true;
}

static uint32_t
e1000e_intmgr_collect_delayed_causes(E1000ECore *core)
{
    uint32_t res;

    if (msix_enabled(core->owner)) {
        assert(core->delayed_causes == 0);
        return 0;
    }

    res = core->delayed_causes;
    core->delayed_causes = 0;

    e1000e_intrmgr_stop_delay_timers(core);

    return res;
}

static void
e1000e_intrmgr_resume(E1000ECore *core)
{
    int i;

    e1000e_intmgr_timer_resume(&core->radv);
    e1000e_intmgr_timer_resume(&core->rdtr);
    e1000e_intmgr_timer_resume(&core->raid);
    e1000e_intmgr_timer_resume(&core->tidv);
    e1000e_intmgr_timer_resume(&core->tadv);

    for (i = 0; i < IGB_MSIX_VEC_NUM; i++) {
        e1000e_intmgr_timer_resume(&core->eitr[i]);
    }
}

static void
e1000e_intrmgr_pause(E1000ECore *core)
{
    int i;

    e1000e_intmgr_timer_pause(&core->radv);
    e1000e_intmgr_timer_pause(&core->rdtr);
    e1000e_intmgr_timer_pause(&core->raid);
    e1000e_intmgr_timer_pause(&core->tidv);
    e1000e_intmgr_timer_pause(&core->tadv);

    for (i = 0; i < IGB_MSIX_VEC_NUM; i++) {
        e1000e_intmgr_timer_pause(&core->eitr[i]);
    }
}

static void
e1000e_intrmgr_reset(E1000ECore *core)
{
    int i;

    core->delayed_causes = 0;

    e1000e_intrmgr_stop_delay_timers(core);

    for (i = 0; i < E1000E_MSIX_VEC_NUM; i++) {
        e1000e_intrmgr_stop_timer(&core->eitr[i]);
    }
}

static void
e1000e_intrmgr_pci_unint(E1000ECore *core)
{
    int i;

    timer_del(core->radv.timer);
    timer_free(core->radv.timer);
    timer_del(core->rdtr.timer);
    timer_free(core->rdtr.timer);
    timer_del(core->raid.timer);
    timer_free(core->raid.timer);

    timer_del(core->tadv.timer);
    timer_free(core->tadv.timer);
    timer_del(core->tidv.timer);
    timer_free(core->tidv.timer);

    for (i = 0; i < IGB_MSIX_VEC_NUM; i++) {
        timer_del(core->eitr[i].timer);
        timer_free(core->eitr[i].timer);
    }
}

static void
e1000e_intrmgr_pci_realize(E1000ECore *core)
{
    e1000e_intrmgr_initialize_all_timers(core, true);
}

static inline bool
e1000e_rx_csum_enabled(E1000ECore *core)
{
    return (core->mac[RXCSUM] & E1000_RXCSUM_PCSD) ? false : true;
}

static inline bool
e1000e_rx_use_legacy_descriptor(E1000ECore *core)
{
    return false;
    return (core->mac[RFCTL] & E1000_RFCTL_EXTEN) ? false : true;
}

static inline bool
e1000e_rx_use_ps_descriptor(E1000ECore *core)
{
    return !e1000e_rx_use_legacy_descriptor(core) &&
           (core->mac[RCTL] & E1000_RCTL_DTYP_PS);
}

static inline bool
e1000e_rss_enabled(E1000ECore *core)
{
    return E1000_MRQC_ENABLED(core->mac[MRQC]) &&
           !e1000e_rx_csum_enabled(core) &&
           !e1000e_rx_use_legacy_descriptor(core);
}

typedef struct E1000E_RSSInfo_st {
    bool enabled;
    uint32_t hash;
    uint32_t queue;
    uint32_t type;
} E1000E_RSSInfo;

static uint32_t
e1000e_rss_get_hash_type(E1000ECore *core, struct NetRxPkt *pkt)
{
    bool isip4, isip6, isudp, istcp;

    assert(e1000e_rss_enabled(core));

    net_rx_pkt_get_protocols(pkt, &isip4, &isip6, &isudp, &istcp);

    if (isip4) {
        bool fragment = net_rx_pkt_get_ip4_info(pkt)->fragment;

        trace_e1000e_rx_rss_ip4(fragment, istcp, core->mac[MRQC],
                                E1000_MRQC_EN_TCPIPV4(core->mac[MRQC]),
                                E1000_MRQC_EN_IPV4(core->mac[MRQC]));

        if (!fragment && istcp && E1000_MRQC_EN_TCPIPV4(core->mac[MRQC])) {
            return E1000_MRQ_RSS_TYPE_IPV4TCP;
        }

        if (E1000_MRQC_EN_IPV4(core->mac[MRQC])) {
            return E1000_MRQ_RSS_TYPE_IPV4;
        }
    } else if (isip6) {
        eth_ip6_hdr_info *ip6info = net_rx_pkt_get_ip6_info(pkt);

        bool ex_dis = core->mac[RFCTL] & E1000_RFCTL_IPV6_EX_DIS;
        bool new_ex_dis = core->mac[RFCTL] & E1000_RFCTL_NEW_IPV6_EXT_DIS;

        /*
         * Following two traces must not be combined because resulting
         * event will have 11 arguments totally and some trace backends
         * (at least "ust") have limitation of maximum 10 arguments per
         * event. Events with more arguments fail to compile for
         * backends like these.
         */
        trace_e1000e_rx_rss_ip6_rfctl(core->mac[RFCTL]);
        trace_e1000e_rx_rss_ip6(ex_dis, new_ex_dis, istcp,
                                ip6info->has_ext_hdrs,
                                ip6info->rss_ex_dst_valid,
                                ip6info->rss_ex_src_valid,
                                core->mac[MRQC],
                                E1000_MRQC_EN_TCPIPV6(core->mac[MRQC]),
                                E1000_MRQC_EN_IPV6EX(core->mac[MRQC]),
                                E1000_MRQC_EN_IPV6(core->mac[MRQC]));

        if ((!ex_dis || !ip6info->has_ext_hdrs) &&
            (!new_ex_dis || !(ip6info->rss_ex_dst_valid ||
                              ip6info->rss_ex_src_valid))) {

            if (istcp && !ip6info->fragment &&
                E1000_MRQC_EN_TCPIPV6(core->mac[MRQC])) {
                return E1000_MRQ_RSS_TYPE_IPV6TCP;
            }

            if (E1000_MRQC_EN_IPV6EX(core->mac[MRQC])) {
                return E1000_MRQ_RSS_TYPE_IPV6EX;
            }

        }

        if (E1000_MRQC_EN_IPV6(core->mac[MRQC])) {
            return E1000_MRQ_RSS_TYPE_IPV6;
        }

    }

    return E1000_MRQ_RSS_TYPE_NONE;
}

static uint32_t
e1000e_rss_calc_hash(E1000ECore *core,
                     struct NetRxPkt *pkt,
                     E1000E_RSSInfo *info)
{
    NetRxPktRssType type;

    assert(e1000e_rss_enabled(core));

    switch (info->type) {
    case E1000_MRQ_RSS_TYPE_IPV4:
        type = NetPktRssIpV4;
        break;
    case E1000_MRQ_RSS_TYPE_IPV4TCP:
        type = NetPktRssIpV4Tcp;
        break;
    case E1000_MRQ_RSS_TYPE_IPV6TCP:
        type = NetPktRssIpV6TcpEx;
        break;
    case E1000_MRQ_RSS_TYPE_IPV6:
        type = NetPktRssIpV6;
        break;
    case E1000_MRQ_RSS_TYPE_IPV6EX:
        type = NetPktRssIpV6Ex;
        break;
    default:
        assert(false);
        return 0;
    }

    return net_rx_pkt_calc_rss_hash(pkt, type, (uint8_t *) &core->mac[RSSRK]);
}

static void
e1000e_rss_parse_packet(E1000ECore *core,
                        struct NetRxPkt *pkt,
                        E1000E_RSSInfo *info)
{
    trace_e1000e_rx_rss_started();

    if (!e1000e_rss_enabled(core)) {
        info->enabled = false;
        info->hash = 0;
        info->queue = 0;
        info->type = 0;
        trace_e1000e_rx_rss_disabled();
        return;
    }

    info->enabled = true;

    info->type = e1000e_rss_get_hash_type(core, pkt);

    trace_e1000e_rx_rss_type(info->type);

    if (info->type == E1000_MRQ_RSS_TYPE_NONE) {
        info->hash = 0;
        info->queue = 0;
        return;
    }

    info->hash = e1000e_rss_calc_hash(core, pkt, info);
    info->queue = E1000_RSS_QUEUE(&core->mac[RETA], info->hash);
}

static void
e1000e_setup_tx_offloads(E1000ECore *core, struct e1000e_tx *tx)
{
    if (tx->tse) {
        net_tx_pkt_build_vheader(tx->tx_pkt, true, true, tx->mss);
        net_tx_pkt_update_ip_checksums(tx->tx_pkt);
        e1000x_inc_reg_if_not_full(core->mac, TSCTC);
        return;
    }

    if (tx->txsm) {
        net_tx_pkt_build_vheader(tx->tx_pkt, false, true, 0);
    }

    if (tx->ixsm) {
        net_tx_pkt_update_ip_hdr_checksum(tx->tx_pkt);
    }
}

static bool
e1000e_tx_pkt_send(E1000ECore *core, struct e1000e_tx *tx, int queue_index)
{
    int target_queue = MIN(core->max_queue_num, queue_index);
    NetClientState *queue = qemu_get_subqueue(core->owner_nic, target_queue);

    e1000e_setup_tx_offloads(core, tx);

    net_tx_pkt_dump(tx->tx_pkt);

    if ((core->phy[0][PHY_CTRL] & MII_CR_LOOPBACK) ||
        ((core->mac[RCTL] & E1000_RCTL_LBM_MAC) == E1000_RCTL_LBM_MAC)) {
        return net_tx_pkt_send_loopback(tx->tx_pkt, queue);
    } else {
        return net_tx_pkt_send(tx->tx_pkt, queue);
    }
}

static void
e1000e_on_tx_done_update_stats(E1000ECore *core, struct NetTxPkt *tx_pkt)
{
    static const int PTCregs[6] = { PTC64, PTC127, PTC255, PTC511,
                                    PTC1023, PTC1522 };

    size_t tot_len = net_tx_pkt_get_total_len(tx_pkt);

    e1000x_increase_size_stats(core->mac, PTCregs, tot_len);
    e1000x_inc_reg_if_not_full(core->mac, TPT);
    e1000x_grow_8reg_if_not_full(core->mac, TOTL, tot_len);

    switch (net_tx_pkt_get_packet_type(tx_pkt)) {
    case ETH_PKT_BCAST:
        e1000x_inc_reg_if_not_full(core->mac, BPTC);
        break;
    case ETH_PKT_MCAST:
        e1000x_inc_reg_if_not_full(core->mac, MPTC);
        break;
    case ETH_PKT_UCAST:
        break;
    default:
        g_assert_not_reached();
    }

    core->mac[GPTC] = core->mac[TPT];
    core->mac[GOTCL] = core->mac[TOTL];
    core->mac[GOTCH] = core->mac[TOTH];
}

static void igb_process_tx_desc(E1000ECore *core, struct e1000e_tx *tx,
    union e1000_adv_tx_desc *tx_desc, int queue_index)
{
    struct e1000_adv_tx_context_desc *tx_ctx_desc;
    uint32_t cmd_type_len;
    uint32_t olinfo_status;
    uint64_t buffer_addr;
    uint16_t length;

    cmd_type_len = le32_to_cpu(tx_desc->read.cmd_type_len);

    if (cmd_type_len & E1000_ADVTXD_DCMD_DEXT) {
        if ((cmd_type_len & E1000_ADVTXD_DTYP_DATA) ==
            E1000_ADVTXD_DTYP_DATA) {
            /* Advanced Transmit Data Descriptor */
            olinfo_status = le32_to_cpu(tx_desc->read.olinfo_status);
            tx->tse = !!(cmd_type_len & E1000_ADVTXD_DCMD_TSE);
            tx->ixsm = !!(olinfo_status & E1000_ADVTXD_POTS_IXSM);
            tx->txsm = !!(olinfo_status & E1000_ADVTXD_POTS_TXSM);
        } else if ((cmd_type_len & E1000_ADVTXD_DTYP_CTXT) ==
                   E1000_ADVTXD_DTYP_CTXT) {
            /* Advanced Transmit Context Descriptor */
            tx_ctx_desc = (struct e1000_adv_tx_context_desc *)tx_desc;
            tx->vlan = le32_to_cpu(tx_ctx_desc->vlan_macip_lens) >> 16;
            tx->mss = le32_to_cpu(tx_ctx_desc->mss_l4len_idx) >> 16;
            return;
        } else {
            /* Unknown Descriptor Type */
            return;
        }
    } else {
        /* Legacy Descriptor */

        // TODO: Implement a support for legacy descriptors (7.2.2.1).
    }

    buffer_addr = le64_to_cpu(tx_desc->read.buffer_addr);
    length = cmd_type_len & 0xFFFF;

    if (!tx->skip_cp) {
        if (!net_tx_pkt_add_raw_fragment(tx->tx_pkt, buffer_addr, length)) {
            tx->skip_cp = true;
        }
    }

    if (cmd_type_len & E1000_TXD_CMD_EOP) {
        if (!tx->skip_cp && net_tx_pkt_parse(tx->tx_pkt)) {
            if (cmd_type_len & E1000_TXD_CMD_VLE) {
                net_tx_pkt_setup_vlan_header_ex(tx->tx_pkt, tx->vlan,
                    core->vet);
            }
            if (e1000e_tx_pkt_send(core, tx, queue_index)) {
                e1000e_on_tx_done_update_stats(core, tx->tx_pkt);
            }
        }

        tx->skip_cp = false;
        net_tx_pkt_reset(tx->tx_pkt);
    }
}

#define _IVAR_QUEUE_ENTRY(q, tx) ((q) < 8 ? (q)*4 + tx : ((q)-8)*4 + 2 + tx)

#define IVAR_RX_QUEUE_ENTRY(q)  _IVAR_QUEUE_ENTRY(q, 0)
#define IVAR_TX_QUEUE_ENTRY(q)  _IVAR_QUEUE_ENTRY(q, 1)

//#define IVAR_GET_ENTRY(i) ((core->mac[IVAR + (n)/4] >> (8 * ((n)%4))) & 0xFF)

#define IVAR_VALID_ENTRY(x) !!((x) & 0x80)

static uint32_t igb_tx_wb_interrupt_cause(E1000ECore *core, int queue_idx)
{
    uint32_t n, ent = 0;

    if (!msix_enabled(core->owner)) {
        return BIT(queue_idx);
    }

    n = IVAR_TX_QUEUE_ENTRY(queue_idx);
    ent = (core->mac[IVAR + n / 4] >> (8 * (n % 4))) & 0xff;

    return IVAR_VALID_ENTRY(ent) ? BIT(ent & 0x1f) : 0;
}

static uint32_t igb_rx_wb_interrupt_cause(E1000ECore *core, int queue_idx,
                                          bool min_threshold_hit)
{
    uint32_t n, ent = 0;

    if (!msix_enabled(core->owner)) {
        return BIT(queue_idx);
    }

    n = IVAR_RX_QUEUE_ENTRY(queue_idx);
    ent = (core->mac[IVAR + n / 4] >> (8 * (n % 4))) & 0xff;

    return IVAR_VALID_ENTRY(ent) ? BIT(ent & 0x1f) : 0;
}

#if 0
static inline uint32_t
e1000e_rx_wb_interrupt_cause(E1000ECore *core, int queue_idx,
                             bool min_threshold_hit)
{
    if (!msix_enabled(core->owner)) {
        return E1000_ICS_RXT0 | (min_threshold_hit ? E1000_ICS_RXDMT0 : 0);
    }

    return (queue_idx == 0) ? E1000_ICR_RXQ0 : E1000_ICR_RXQ1;
}
#endif

static uint32_t igb_txdesc_writeback(E1000ECore *core, dma_addr_t base,
    union e1000_adv_tx_desc *tx_desc, int queue_idx)
{
    uint32_t cmd_type_len;
    uint32_t status;

    cmd_type_len = le32_to_cpu(tx_desc->read.cmd_type_len);
    if (!(cmd_type_len & E1000_TXD_CMD_RS)) {
        return 0;
    }

    status = le32_to_cpu(tx_desc->wb.status) | E1000_TXD_STAT_DD;
    tx_desc->wb.status = cpu_to_le32(status);

    pci_dma_write(core->owner, base + offsetof(union e1000_adv_tx_desc, wb),
        &tx_desc->wb, sizeof(tx_desc->wb));

    return igb_tx_wb_interrupt_cause(core, queue_idx);
}

typedef struct E1000E_RingInfo_st {
    int dbah;
    int dbal;
    int dlen;
    int dh;
    int dt;
    int idx;
} E1000E_RingInfo;

static inline bool
e1000e_ring_empty(E1000ECore *core, const E1000E_RingInfo *r)
{
    return core->mac[r->dh] == core->mac[r->dt] ||
                core->mac[r->dt] >= core->mac[r->dlen] / E1000_RING_DESC_LEN;
}

static inline uint64_t
e1000e_ring_base(E1000ECore *core, const E1000E_RingInfo *r)
{
    uint64_t bah = core->mac[r->dbah];
    uint64_t bal = core->mac[r->dbal];

    return (bah << 32) + bal;
}

static inline uint64_t
e1000e_ring_head_descr(E1000ECore *core, const E1000E_RingInfo *r)
{
    return e1000e_ring_base(core, r) + E1000_RING_DESC_LEN * core->mac[r->dh];
}

static inline void
e1000e_ring_advance(E1000ECore *core, const E1000E_RingInfo *r, uint32_t count)
{
    core->mac[r->dh] += count;

    if (core->mac[r->dh] * E1000_RING_DESC_LEN >= core->mac[r->dlen]) {
        core->mac[r->dh] = 0;
    }
}

static inline uint32_t
e1000e_ring_free_descr_num(E1000ECore *core, const E1000E_RingInfo *r)
{
    trace_e1000e_ring_free_space(r->idx, core->mac[r->dlen],
                                 core->mac[r->dh],  core->mac[r->dt]);

    if (core->mac[r->dh] <= core->mac[r->dt]) {
        return core->mac[r->dt] - core->mac[r->dh];
    }

    if (core->mac[r->dh] > core->mac[r->dt]) {
        return core->mac[r->dlen] / E1000_RING_DESC_LEN +
               core->mac[r->dt] - core->mac[r->dh];
    }

    g_assert_not_reached();
    return 0;
}

static inline bool
e1000e_ring_enabled(E1000ECore *core, const E1000E_RingInfo *r)
{
    return core->mac[r->dlen] > 0;
}

static inline uint32_t
e1000e_ring_len(E1000ECore *core, const E1000E_RingInfo *r)
{
    return core->mac[r->dlen];
}

typedef struct E1000E_TxRing_st {
    const E1000E_RingInfo *i;
    struct e1000e_tx *tx;
} E1000E_TxRing;

static inline int
e1000e_mq_queue_idx(int base_reg_idx, int reg_idx)
{
    return (reg_idx - base_reg_idx) / 16;
}

static inline void igb_tx_ring_init(E1000ECore *core,
    E1000E_TxRing *txr, int idx)
{
    static const E1000E_RingInfo i[IGB_NUM_QUEUES] = {
        { TDBAH0, TDBAL0, TDLEN0, TDH0, TDT0, 0 },
        { TDBAH1, TDBAL1, TDLEN1, TDH1, TDT1, 1 },
        { TDBAH2, TDBAL2, TDLEN2, TDH2, TDT2, 2 },
        { TDBAH3, TDBAL3, TDLEN3, TDH3, TDT3, 3 },
        { TDBAH4, TDBAL4, TDLEN4, TDH4, TDT4, 4 },
        { TDBAH5, TDBAL5, TDLEN5, TDH5, TDT5, 5 },
        { TDBAH6, TDBAL6, TDLEN6, TDH6, TDT6, 6 },
        { TDBAH7, TDBAL7, TDLEN7, TDH7, TDT7, 7 },
        { TDBAH8, TDBAL8, TDLEN8, TDH8, TDT8, 8 },
        { TDBAH9, TDBAL9, TDLEN9, TDH9, TDT9, 9 },
        { TDBAH10, TDBAL10, TDLEN10, TDH10, TDT10, 10 },
        { TDBAH11, TDBAL11, TDLEN11, TDH11, TDT11, 11 },
        { TDBAH12, TDBAL12, TDLEN12, TDH12, TDT12, 12 },
        { TDBAH13, TDBAL13, TDLEN13, TDH13, TDT13, 13 },
        { TDBAH14, TDBAL14, TDLEN14, TDH14, TDT14, 14 },
        { TDBAH15, TDBAL15, TDLEN15, TDH15, TDT15, 15 }
    };

    assert(idx < ARRAY_SIZE(i));

    txr->i = &i[idx];
    txr->tx = &core->tx[idx];
}

typedef struct E1000E_RxRing_st {
    const E1000E_RingInfo *i;
} E1000E_RxRing;

static inline void igb_rx_ring_init(E1000ECore *core, E1000E_RxRing *rxr,
                                    int idx)
{
    static const E1000E_RingInfo i[IGB_NUM_QUEUES] = {
        { RDBAH0, RDBAL0, RDLEN0, RDH0, RDT0, 0 },
        { RDBAH1, RDBAL1, RDLEN1, RDH1, RDT1, 1 },
        { RDBAH2, RDBAL2, RDLEN2, RDH2, RDT2, 2 },
        { RDBAH3, RDBAL3, RDLEN3, RDH3, RDT3, 3 },
        { RDBAH4, RDBAL4, RDLEN4, RDH4, RDT4, 4 },
        { RDBAH5, RDBAL5, RDLEN5, RDH5, RDT5, 5 },
        { RDBAH6, RDBAL6, RDLEN6, RDH6, RDT6, 6 },
        { RDBAH7, RDBAL7, RDLEN7, RDH7, RDT7, 7 },
        { RDBAH8, RDBAL8, RDLEN8, RDH8, RDT8, 8 },
        { RDBAH9, RDBAL9, RDLEN9, RDH9, RDT9, 9 },
        { RDBAH10, RDBAL10, RDLEN10, RDH10, RDT10, 10 },
        { RDBAH11, RDBAL11, RDLEN11, RDH11, RDT11, 11 },
        { RDBAH12, RDBAL12, RDLEN12, RDH12, RDT12, 12 },
        { RDBAH13, RDBAL13, RDLEN13, RDH13, RDT13, 13 },
        { RDBAH14, RDBAL14, RDLEN14, RDH14, RDT14, 14 },
        { RDBAH15, RDBAL15, RDLEN15, RDH15, RDT15, 15 }
    };

    assert(idx < ARRAY_SIZE(i));

    rxr->i = &i[idx];
}

static void igb_start_xmit(E1000ECore *core, const E1000E_TxRing *txr)
{
    const E1000E_RingInfo *txi = txr->i;
    union e1000_adv_tx_desc tx_desc;
    dma_addr_t base;
    uint32_t cause = 0;

    // TODO: check if the queue itself is enabled too.
    if (!(core->mac[TCTL] & E1000_TCTL_EN)) {
        trace_e1000e_tx_disabled();
        return;
    }

    while (!e1000e_ring_empty(core, txi)) {
        base = e1000e_ring_head_descr(core, txi);
        pci_dma_read(core->owner, base, &tx_desc, sizeof(tx_desc));

        igb_process_tx_desc(core, txr->tx, &tx_desc, txi->idx);
        cause |= igb_txdesc_writeback(core, base, &tx_desc, txi->idx);

        e1000e_ring_advance(core, txi, 1);
    }

    if (!e1000e_intrmgr_delay_tx_causes(core, &cause)) {
        core->mac[EICR] |= cause;
        igb_update_interrupt_state(core);
    }
}

static bool
e1000e_has_rxbufs(E1000ECore *core, const E1000E_RingInfo *r,
                  size_t total_size)
{
    uint32_t bufs = e1000e_ring_free_descr_num(core, r);

    trace_e1000e_rx_has_buffers(r->idx, bufs, total_size,
                                core->rx_desc_buf_size);

    return total_size <= bufs / (core->rx_desc_len / E1000_MIN_RX_DESC_LEN) *
                         core->rx_desc_buf_size;
}

void igb_start_recv(E1000ECore *core)
{
    int i;

    trace_e1000e_rx_start_recv();

    for (i = 0; i <= core->max_queue_num; i++) {
        qemu_flush_queued_packets(qemu_get_subqueue(core->owner_nic, i));
    }
}

bool igb_can_receive(E1000ECore *core)
{
    int i;

    if (!e1000x_rx_ready(core->owner, core->mac)) {
        return false;
    }

    for (i = 0; i < IGB_NUM_QUEUES; i++) {
        E1000E_RxRing rxr;

        igb_rx_ring_init(core, &rxr, i);
        if (e1000e_ring_enabled(core, rxr.i) &&
            e1000e_has_rxbufs(core, rxr.i, 1)) {
            trace_e1000e_rx_can_recv();
            return true;
        }
    }

    trace_e1000e_rx_can_recv_rings_full();
    return false;
}

ssize_t igb_receive(E1000ECore *core, const uint8_t *buf, size_t size)
{
    const struct iovec iov = {
        .iov_base = (uint8_t *)buf,
        .iov_len = size
    };

    return igb_receive_iov(core, &iov, 1);
}

static inline bool
e1000e_rx_l3_cso_enabled(E1000ECore *core)
{
    return !!(core->mac[RXCSUM] & E1000_RXCSUM_IPOFLD);
}

static inline bool
e1000e_rx_l4_cso_enabled(E1000ECore *core)
{
    return !!(core->mac[RXCSUM] & E1000_RXCSUM_TUOFLD);
}

static bool igb_vf_receive_filter(E1000ECore *core, const uint8_t *buf)
{
    uint32_t ra[2], *rp;

    for (rp = core->mac + RA_VF; rp < core->mac + RA_VF + 16; rp += 2) {
        if (!(rp[1] & E1000_RAH_AV)) {
            continue;
        }
        ra[0] = cpu_to_le32(rp[0]);
        ra[1] = cpu_to_le16(rp[1] & 0xFFFF);

        if (!memcmp(buf, (uint8_t *)ra, 6)) {
            return true;
        }
    }

    return false;
}

static bool e1000e_receive_filter(E1000ECore *core, const uint8_t *buf)
{
    uint32_t rctl = core->mac[RCTL];

    if (e1000x_is_vlan_packet(buf, core->vet) &&
        e1000x_vlan_rx_filter_enabled(core->mac)) {
        uint16_t vid = lduw_be_p(buf + 14);
        uint32_t vfta = ldl_le_p((uint32_t *)(core->mac + VFTA) +
                                 ((vid >> 5) & 0x7f));
        if ((vfta & (1 << (vid & 0x1f))) == 0) {
            trace_e1000e_rx_flt_vlan_mismatch(vid);
            return false;
        } else {
            trace_e1000e_rx_flt_vlan_match(vid);
        }
    }

    switch (net_rx_pkt_get_packet_type(core->rx_pkt)) {
    case ETH_PKT_UCAST:
        if (rctl & E1000_RCTL_UPE) {
            return true; /* promiscuous ucast */
        }
        break;

    case ETH_PKT_BCAST:
        if (rctl & E1000_RCTL_BAM) {
            return true; /* broadcast enabled */
        }
        break;

    case ETH_PKT_MCAST:
        if (rctl & E1000_RCTL_MPE) {
            return true; /* promiscuous mcast */
        }
        break;

    default:
        g_assert_not_reached();
    }

    if (e1000x_rx_group_filter(core->mac, buf)) {
        return true;
    }

    return igb_vf_receive_filter(core, buf);
}

static inline void
e1000e_read_lgcy_rx_descr(E1000ECore *core, uint8_t *desc, hwaddr *buff_addr)
{
    struct e1000_rx_desc *d = (struct e1000_rx_desc *) desc;
    *buff_addr = le64_to_cpu(d->buffer_addr);
}

static inline void
e1000e_read_ext_rx_descr(E1000ECore *core, uint8_t *desc, hwaddr *buff_addr)
{
    union e1000_rx_desc_extended *d = (union e1000_rx_desc_extended *) desc;
    *buff_addr = le64_to_cpu(d->read.buffer_addr);
}

static inline void
e1000e_read_ps_rx_descr(E1000ECore *core, uint8_t *desc,
                        hwaddr (*buff_addr)[MAX_PS_BUFFERS])
{
    int i;
    union e1000_rx_desc_packet_split *d =
        (union e1000_rx_desc_packet_split *) desc;

    for (i = 0; i < MAX_PS_BUFFERS; i++) {
        (*buff_addr)[i] = le64_to_cpu(d->read.buffer_addr[i]);
    }

    trace_e1000e_rx_desc_ps_read((*buff_addr)[0], (*buff_addr)[1],
                                 (*buff_addr)[2], (*buff_addr)[3]);
}

static inline void
e1000e_read_rx_descr(E1000ECore *core, uint8_t *desc,
                     hwaddr (*buff_addr)[MAX_PS_BUFFERS])
{
    if (e1000e_rx_use_legacy_descriptor(core)) {
        e1000e_read_lgcy_rx_descr(core, desc, &(*buff_addr)[0]);
        (*buff_addr)[1] = (*buff_addr)[2] = (*buff_addr)[3] = 0;
    } else {
        if (core->mac[RCTL] & E1000_RCTL_DTYP_PS) {
            e1000e_read_ps_rx_descr(core, desc, buff_addr);
        } else {
            e1000e_read_ext_rx_descr(core, desc, &(*buff_addr)[0]);
            (*buff_addr)[1] = (*buff_addr)[2] = (*buff_addr)[3] = 0;
        }
    }
}

static void
e1000e_verify_csum_in_sw(E1000ECore *core,
                         struct NetRxPkt *pkt,
                         uint32_t *status_flags,
                         bool istcp, bool isudp)
{
    bool csum_valid;
    uint32_t csum_error;

    if (e1000e_rx_l3_cso_enabled(core)) {
        if (!net_rx_pkt_validate_l3_csum(pkt, &csum_valid)) {
            trace_e1000e_rx_metadata_l3_csum_validation_failed();
        } else {
            csum_error = csum_valid ? 0 : E1000_RXDEXT_STATERR_IPE;
            *status_flags |= E1000_RXD_STAT_IPCS | csum_error;
        }
    } else {
        trace_e1000e_rx_metadata_l3_cso_disabled();
    }

    if (!e1000e_rx_l4_cso_enabled(core)) {
        trace_e1000e_rx_metadata_l4_cso_disabled();
        return;
    }

    if (!net_rx_pkt_validate_l4_csum(pkt, &csum_valid)) {
        trace_e1000e_rx_metadata_l4_csum_validation_failed();
        return;
    }

    csum_error = csum_valid ? 0 : E1000_RXDEXT_STATERR_TCPE;

    if (istcp) {
        *status_flags |= E1000_RXD_STAT_TCPCS |
                         csum_error;
    } else if (isudp) {
        *status_flags |= E1000_RXD_STAT_TCPCS |
                         E1000_RXD_STAT_UDPCS |
                         csum_error;
    }
}

static inline bool
e1000e_is_tcp_ack(E1000ECore *core, struct NetRxPkt *rx_pkt)
{
    if (!net_rx_pkt_is_tcp_ack(rx_pkt)) {
        return false;
    }

    return true;
}

static void
e1000e_build_rx_metadata(E1000ECore *core,
                         struct NetRxPkt *pkt,
                         bool is_eop,
                         const E1000E_RSSInfo *rss_info,
                         uint32_t *rss, uint32_t *mrq,
                         uint32_t *status_flags,
                         uint16_t *ip_id,
                         uint16_t *vlan_tag)
{
    bool isip4, isip6, istcp, isudp;
    uint32_t pkt_type;

    *status_flags = E1000_RXD_STAT_DD;

    /* No additional metadata needed for non-EOP descriptors */
    if (!is_eop) {
        goto func_exit;
    }

    *status_flags |= E1000_RXD_STAT_EOP;

    net_rx_pkt_get_protocols(pkt, &isip4, &isip6, &isudp, &istcp);
    trace_e1000e_rx_metadata_protocols(isip4, isip6, isudp, istcp);

    /* VLAN state */
    if (net_rx_pkt_is_vlan_stripped(pkt)) {
        *status_flags |= E1000_RXD_STAT_VP;
        *vlan_tag = cpu_to_le16(net_rx_pkt_get_vlan_tag(pkt));
        trace_e1000e_rx_metadata_vlan(*vlan_tag);
    }

    /* Packet parsing results */
    if ((core->mac[RXCSUM] & E1000_RXCSUM_PCSD) != 0) {
        if (rss_info->enabled) {
            *rss = cpu_to_le32(rss_info->hash);
            *mrq = cpu_to_le32(rss_info->type | (rss_info->queue << 8));
            trace_e1000e_rx_metadata_rss(*rss, *mrq);
        }
    } else if (isip4) {
            *status_flags |= E1000_RXD_STAT_IPIDV;
            *ip_id = cpu_to_le16(net_rx_pkt_get_ip_id(pkt));
            trace_e1000e_rx_metadata_ip_id(*ip_id);
    }

    if (istcp && e1000e_is_tcp_ack(core, pkt)) {
        *status_flags |= E1000_RXD_STAT_ACK;
        trace_e1000e_rx_metadata_ack();
    }

    if (isip6 && (core->mac[RFCTL] & E1000_RFCTL_IPV6_DIS)) {
        trace_e1000e_rx_metadata_ipv6_filtering_disabled();
        pkt_type = E1000_RXD_PKT_MAC;
    } else if (istcp || isudp) {
        pkt_type = isip4 ? E1000_RXD_PKT_IP4_XDP : E1000_RXD_PKT_IP6_XDP;
    } else if (isip4 || isip6) {
        pkt_type = isip4 ? E1000_RXD_PKT_IP4 : E1000_RXD_PKT_IP6;
    } else {
        pkt_type = E1000_RXD_PKT_MAC;
    }

    *status_flags |= E1000_RXD_PKT_TYPE(pkt_type);
    trace_e1000e_rx_metadata_pkt_type(pkt_type);

    /* RX CSO information */
    if (isip6 && (core->mac[RFCTL] & E1000_RFCTL_IPV6_XSUM_DIS)) {
        trace_e1000e_rx_metadata_ipv6_sum_disabled();
        goto func_exit;
    }

    if (!net_rx_pkt_has_virt_hdr(pkt)) {
        trace_e1000e_rx_metadata_no_virthdr();
        e1000e_verify_csum_in_sw(core, pkt, status_flags, istcp, isudp);
        goto func_exit;
    }

    if (e1000e_rx_l3_cso_enabled(core)) {
        *status_flags |= isip4 ? E1000_RXD_STAT_IPCS : 0;
    } else {
        trace_e1000e_rx_metadata_l3_cso_disabled();
    }

    if (e1000e_rx_l4_cso_enabled(core)) {
        if (istcp) {
            *status_flags |= E1000_RXD_STAT_TCPCS;
        } else if (isudp) {
            *status_flags |= E1000_RXD_STAT_TCPCS | E1000_RXD_STAT_UDPCS;
        }
    } else {
        trace_e1000e_rx_metadata_l4_cso_disabled();
    }

    trace_e1000e_rx_metadata_status_flags(*status_flags);

func_exit:
    *status_flags = cpu_to_le32(*status_flags);
}

static inline void
e1000e_write_lgcy_rx_descr(E1000ECore *core, uint8_t *desc,
                           struct NetRxPkt *pkt,
                           const E1000E_RSSInfo *rss_info,
                           uint16_t length)
{
    uint32_t status_flags, rss, mrq;
    uint16_t ip_id;

    struct e1000_rx_desc *d = (struct e1000_rx_desc *) desc;

    assert(!rss_info->enabled);
    d->length = cpu_to_le16(length);
    d->csum = 0;

    e1000e_build_rx_metadata(core, pkt, pkt != NULL,
                             rss_info,
                             &rss, &mrq,
                             &status_flags, &ip_id,
                             &d->special);
    d->errors = (uint8_t) (le32_to_cpu(status_flags) >> 24);
    d->status = (uint8_t) le32_to_cpu(status_flags);
    d->special = 0;
}

static inline void
e1000e_write_ext_rx_descr(E1000ECore *core, uint8_t *desc,
                          struct NetRxPkt *pkt,
                          const E1000E_RSSInfo *rss_info,
                          uint16_t length)
{
    union e1000_rx_desc_extended *d = (union e1000_rx_desc_extended *) desc;

    memset(&d->wb, 0, sizeof(d->wb));
    d->wb.upper.length = cpu_to_le16(length);

    e1000e_build_rx_metadata(core, pkt, pkt != NULL,
                             rss_info,
                             &d->wb.lower.hi_dword.rss,
                             &d->wb.lower.mrq,
                             &d->wb.upper.status_error,
                             &d->wb.lower.hi_dword.csum_ip.ip_id,
                             &d->wb.upper.vlan);
}

static inline void
e1000e_write_ps_rx_descr(E1000ECore *core, uint8_t *desc,
                         struct NetRxPkt *pkt,
                         const E1000E_RSSInfo *rss_info,
                         size_t ps_hdr_len,
                         uint16_t(*written)[MAX_PS_BUFFERS])
{
    int i;
    union e1000_rx_desc_packet_split *d =
        (union e1000_rx_desc_packet_split *) desc;

    memset(&d->wb, 0, sizeof(d->wb));

    d->wb.middle.length0 = cpu_to_le16((*written)[0]);

    for (i = 0; i < PS_PAGE_BUFFERS; i++) {
        d->wb.upper.length[i] = cpu_to_le16((*written)[i + 1]);
    }

    e1000e_build_rx_metadata(core, pkt, pkt != NULL,
                             rss_info,
                             &d->wb.lower.hi_dword.rss,
                             &d->wb.lower.mrq,
                             &d->wb.middle.status_error,
                             &d->wb.lower.hi_dword.csum_ip.ip_id,
                             &d->wb.middle.vlan);

    d->wb.upper.header_status =
        cpu_to_le16(ps_hdr_len | (ps_hdr_len ? E1000_RXDPS_HDRSTAT_HDRSP : 0));

    trace_e1000e_rx_desc_ps_write((*written)[0], (*written)[1],
                                  (*written)[2], (*written)[3]);
}

static inline void
e1000e_write_rx_descr(E1000ECore *core, uint8_t *desc,
struct NetRxPkt *pkt, const E1000E_RSSInfo *rss_info,
    size_t ps_hdr_len, uint16_t(*written)[MAX_PS_BUFFERS])
{
    if (e1000e_rx_use_legacy_descriptor(core)) {
        assert(ps_hdr_len == 0);
        e1000e_write_lgcy_rx_descr(core, desc, pkt, rss_info, (*written)[0]);
    } else {
        if (core->mac[RCTL] & E1000_RCTL_DTYP_PS) {
            e1000e_write_ps_rx_descr(core, desc, pkt, rss_info,
                                      ps_hdr_len, written);
        } else {
            assert(ps_hdr_len == 0);
            e1000e_write_ext_rx_descr(core, desc, pkt, rss_info,
                                       (*written)[0]);
        }
    }
}

typedef struct e1000e_ba_state_st {
    uint16_t written[MAX_PS_BUFFERS];
    uint8_t cur_idx;
} e1000e_ba_state;

static inline void
e1000e_write_hdr_to_rx_buffers(E1000ECore *core,
                               hwaddr (*ba)[MAX_PS_BUFFERS],
                               e1000e_ba_state *bastate,
                               const char *data,
                               dma_addr_t data_len)
{
    assert(data_len <= core->rxbuf_sizes[0] - bastate->written[0]);

    pci_dma_write(core->owner, (*ba)[0] + bastate->written[0], data, data_len);
    bastate->written[0] += data_len;

    bastate->cur_idx = 1;
}

static void
e1000e_write_to_rx_buffers(E1000ECore *core,
                           hwaddr (*ba)[MAX_PS_BUFFERS],
                           e1000e_ba_state *bastate,
                           const char *data,
                           dma_addr_t data_len)
{
    while (data_len > 0) {
        uint32_t cur_buf_len = core->rxbuf_sizes[bastate->cur_idx];
        uint32_t cur_buf_bytes_left = cur_buf_len -
                                      bastate->written[bastate->cur_idx];
        uint32_t bytes_to_write = MIN(data_len, cur_buf_bytes_left);

        trace_e1000e_rx_desc_buff_write(bastate->cur_idx,
                                        (*ba)[bastate->cur_idx],
                                        bastate->written[bastate->cur_idx],
                                        data,
                                        bytes_to_write);

        pci_dma_write(core->owner,
            (*ba)[bastate->cur_idx] + bastate->written[bastate->cur_idx],
            data, bytes_to_write);

        bastate->written[bastate->cur_idx] += bytes_to_write;
        data += bytes_to_write;
        data_len -= bytes_to_write;

        if (bastate->written[bastate->cur_idx] == cur_buf_len) {
            bastate->cur_idx++;
        }

        assert(bastate->cur_idx < MAX_PS_BUFFERS);
    }
}

static void
e1000e_update_rx_stats(E1000ECore *core,
                       size_t data_size,
                       size_t data_fcs_size)
{
    e1000x_update_rx_total_stats(core->mac, data_size, data_fcs_size);

    switch (net_rx_pkt_get_packet_type(core->rx_pkt)) {
    case ETH_PKT_BCAST:
        e1000x_inc_reg_if_not_full(core->mac, BPRC);
        break;

    case ETH_PKT_MCAST:
        e1000x_inc_reg_if_not_full(core->mac, MPRC);
        break;

    default:
        break;
    }
}

static inline bool
e1000e_rx_descr_threshold_hit(E1000ECore *core, const E1000E_RingInfo *rxi)
{
    return e1000e_ring_free_descr_num(core, rxi) ==
           e1000e_ring_len(core, rxi) >> core->rxbuf_min_shift;
}

static bool
e1000e_do_ps(E1000ECore *core, struct NetRxPkt *pkt, size_t *hdr_len)
{
    bool isip4, isip6, isudp, istcp;
    bool fragment;

    if (!e1000e_rx_use_ps_descriptor(core)) {
        return false;
    }

    net_rx_pkt_get_protocols(pkt, &isip4, &isip6, &isudp, &istcp);

    if (isip4) {
        fragment = net_rx_pkt_get_ip4_info(pkt)->fragment;
    } else if (isip6) {
        fragment = net_rx_pkt_get_ip6_info(pkt)->fragment;
    } else {
        return false;
    }

    if (fragment && (core->mac[RFCTL] & E1000_RFCTL_IPFRSP_DIS)) {
        return false;
    }

    if (!fragment && (isudp || istcp)) {
        *hdr_len = net_rx_pkt_get_l5_hdr_offset(pkt);
    } else {
        *hdr_len = net_rx_pkt_get_l4_hdr_offset(pkt);
    }

    if ((*hdr_len > core->rxbuf_sizes[0]) ||
        (*hdr_len > net_rx_pkt_get_total_len(pkt))) {
        return false;
    }

    return true;
}

static void
e1000e_write_packet_to_guest(E1000ECore *core, struct NetRxPkt *pkt,
                             const E1000E_RxRing *rxr,
                             const E1000E_RSSInfo *rss_info)
{
    PCIDevice *d = core->owner;
    dma_addr_t base;
    uint8_t desc[E1000_MAX_RX_DESC_LEN];
    size_t desc_size;
    size_t desc_offset = 0;
    size_t iov_ofs = 0;

    struct iovec *iov = net_rx_pkt_get_iovec(pkt);
    size_t size = net_rx_pkt_get_total_len(pkt);
    size_t total_size = size + e1000x_fcs_len(core->mac);
    const E1000E_RingInfo *rxi;
    size_t ps_hdr_len = 0;
    bool do_ps = e1000e_do_ps(core, pkt, &ps_hdr_len);
    bool is_first = true;

    rxi = rxr->i;

    do {
        hwaddr ba[MAX_PS_BUFFERS];
        e1000e_ba_state bastate = { { 0 } };
        bool is_last = false;

        desc_size = total_size - desc_offset;

        if (desc_size > core->rx_desc_buf_size) {
            desc_size = core->rx_desc_buf_size;
        }

        if (e1000e_ring_empty(core, rxi)) {
            return;
        }

        base = e1000e_ring_head_descr(core, rxi);

        pci_dma_read(d, base, &desc, core->rx_desc_len);

        trace_e1000e_rx_descr(rxi->idx, base, core->rx_desc_len);

        e1000e_read_rx_descr(core, desc, &ba);

        if (ba[0]) {
            if (desc_offset < size) {
                static const uint32_t fcs_pad;
                size_t iov_copy;
                size_t copy_size = size - desc_offset;
                if (copy_size > core->rx_desc_buf_size) {
                    copy_size = core->rx_desc_buf_size;
                }

                /* For PS mode copy the packet header first */
                if (do_ps) {
                    if (is_first) {
                        size_t ps_hdr_copied = 0;
                        do {
                            iov_copy = MIN(ps_hdr_len - ps_hdr_copied,
                                           iov->iov_len - iov_ofs);

                            e1000e_write_hdr_to_rx_buffers(core, &ba, &bastate,
                                                      iov->iov_base, iov_copy);

                            copy_size -= iov_copy;
                            ps_hdr_copied += iov_copy;

                            iov_ofs += iov_copy;
                            if (iov_ofs == iov->iov_len) {
                                iov++;
                                iov_ofs = 0;
                            }
                        } while (ps_hdr_copied < ps_hdr_len);

                        is_first = false;
                    } else {
                        /* Leave buffer 0 of each descriptor except first */
                        /* empty as per spec 7.1.5.1                      */
                        e1000e_write_hdr_to_rx_buffers(core, &ba, &bastate,
                                                       NULL, 0);
                    }
                }

                /* Copy packet payload */
                while (copy_size) {
                    iov_copy = MIN(copy_size, iov->iov_len - iov_ofs);

                    e1000e_write_to_rx_buffers(core, &ba, &bastate,
                                            iov->iov_base + iov_ofs, iov_copy);

                    copy_size -= iov_copy;
                    iov_ofs += iov_copy;
                    if (iov_ofs == iov->iov_len) {
                        iov++;
                        iov_ofs = 0;
                    }
                }

                if (desc_offset + desc_size >= total_size) {
                    /* Simulate FCS checksum presence in the last descriptor */
                    e1000e_write_to_rx_buffers(core, &ba, &bastate,
                          (const char *) &fcs_pad, e1000x_fcs_len(core->mac));
                }
            }
        } else { /* as per intel docs; skip descriptors with null buf addr */
            trace_e1000e_rx_null_descriptor();
        }
        desc_offset += desc_size;
        if (desc_offset >= total_size) {
            is_last = true;
        }

        e1000e_write_rx_descr(core, desc, is_last ? core->rx_pkt : NULL,
                           rss_info, do_ps ? ps_hdr_len : 0, &bastate.written);

        pci_dma_write(d, base, &desc, core->rx_desc_len);

        e1000e_ring_advance(core, rxi,
                            core->rx_desc_len / E1000_MIN_RX_DESC_LEN);

    } while (desc_offset < total_size);

    e1000e_update_rx_stats(core, size, total_size);
}

static uint8_t get_vf_queue(uint8_t vf)
{
    const uint8_t bit_to_vf[] = {
        [0x01] = 0, [0x02] = 1, [0x04] = 2, [0x08] = 3,
        [0x10] = 4, [0x20] = 5, [0x40] = 6, [0x80] = 7
    };

    return bit_to_vf[vf];
}

ssize_t igb_receive_iov(E1000ECore *core, const struct iovec *iov, int iovcnt)
{
    static const uint64_t brd_addr = 0xFFFFFFFFFFFFL;
    static const int maximum_ethernet_hdr_len = (14 + 4);
    /* Min. octets in an ethernet frame sans FCS */
    static const int min_buf_size = 60;

    struct vf_select_table *vst;
    uint16_t queues = 0;
    uint32_t n = 0;
    uint8_t min_buf[min_buf_size];
    struct iovec min_iov;
    struct eth_header *ehdr;
    uint8_t *filter_buf;
    size_t size, orig_size;
    size_t iov_ofs = 0;
    E1000E_RxRing rxr;
    E1000E_RSSInfo rss_info;
    size_t total_size;
    ssize_t retval;
    bool rdmts_hit;
    bool is_brd;
    int i;

    trace_e1000e_rx_receive_iov(iovcnt);

    if (!e1000x_hw_rx_enabled(core->mac)) {
        return -1;
    }

    filter_buf = iov->iov_base + iov_ofs;
    orig_size = iov_size(iov, iovcnt);
    size = orig_size - iov_ofs;

    /* Pad to minimum Ethernet frame length */
    if (size < sizeof(min_buf)) {
        iov_to_buf(iov, iovcnt, iov_ofs, min_buf, size);
        memset(&min_buf[size], 0, sizeof(min_buf) - size);
        e1000x_inc_reg_if_not_full(core->mac, RUC);
        min_iov.iov_base = filter_buf = min_buf;
        min_iov.iov_len = size = sizeof(min_buf);
        iovcnt = 1;
        iov = &min_iov;
        iov_ofs = 0;
    } else if (iov->iov_len < maximum_ethernet_hdr_len) {
        /* This is very unlikely, but may happen. */
        iov_to_buf(iov, iovcnt, iov_ofs, min_buf, maximum_ethernet_hdr_len);
        filter_buf = min_buf;
    }

    /* Discard oversized packets if !LPE and !SBP. */
    if (e1000x_is_oversized(core->mac, size)) {
        return orig_size;
    }

    ehdr = PKT_GET_ETH_HDR(filter_buf);
    net_rx_pkt_set_packet_type(core->rx_pkt, get_eth_packet_type(ehdr));

    if (!e1000e_receive_filter(core, filter_buf)) {
        trace_e1000e_rx_flt_dropped();
        return orig_size;
    }

    net_rx_pkt_attach_iovec_ex(core->rx_pkt, iov, iovcnt, iov_ofs,
                               e1000x_vlan_enabled(core->mac), core->vet);

    if (!pcie_sriov_is_iov(core->owner)) {
        e1000e_rss_parse_packet(core, core->rx_pkt, &rss_info);
        queues |= BIT(rss_info.queue);
    } else {
        is_brd = !memcmp(ehdr->h_dest, &brd_addr, 6);

        for (i = ARRAY_SIZE(core->vf_select_table)-1; i >= 0; i--) {
            vst = &core->vf_select_table[i];
            if ((vst->vf != 0) &&
                (is_brd || !memcmp(ehdr->h_dest, &vst->macaddr, 6))) {
                queues |= BIT(get_vf_queue(vst->vf));
                /* Stop scan if an unicast address belong to a vf was found */
                if (!is_brd) {
                    break;
                }
            }
        }

        if (is_brd || (queues == 0)) {
            //e1000e_rss_parse_packet(core, core->rx_pkt, &rss_info);
            // TODO: fix RETA?
            rss_info.queue = core->owner->exp.sriov_pf.num_vfs;
            queues |= BIT(rss_info.queue);
        }
    }

    for (i = 0; i < E1000E_NUM_QUEUES; i++) {
        if (queues & BIT(i)) {
            rss_info.enabled = false;
            rss_info.hash = 0;
            rss_info.queue = i;
            rss_info.type = 0;

            igb_rx_ring_init(core, &rxr, i);

            trace_e1000e_rx_rss_dispatched_to_queue(rxr.i->idx);

            total_size = net_rx_pkt_get_total_len(core->rx_pkt) +
                e1000x_fcs_len(core->mac);

            if (e1000e_has_rxbufs(core, rxr.i, total_size)) {
                e1000e_write_packet_to_guest(core, core->rx_pkt, &rxr,
                    &rss_info);

                retval = orig_size;

                /* Check if receive descriptor minimum threshold hit */
                rdmts_hit = e1000e_rx_descr_threshold_hit(core, rxr.i);
                n |= igb_rx_wb_interrupt_cause(core, rxr.i->idx, rdmts_hit);

                trace_e1000e_rx_written_to_guest(n);
            } else {
                //n |= E1000_ICS_RXO;
                retval = 0;
                trace_e1000e_rx_not_written_to_guest(n);
            }
        }
    }

    if (!e1000e_intrmgr_delay_rx_causes(core, &n)) {
        trace_e1000e_rx_interrupt_set(n);
//        e1000e_set_interrupt_cause(core, n);
        core->mac[EICR] |= n;
        igb_update_interrupt_state(core);
    } else {
        trace_e1000e_rx_interrupt_delayed(n);
    }

    return retval;
}

static inline bool
e1000e_have_autoneg(E1000ECore *core)
{
    return core->phy[0][PHY_CTRL] & MII_CR_AUTO_NEG_EN;
}

static void e1000e_update_flowctl_status(E1000ECore *core)
{
    if (e1000e_have_autoneg(core) &&
        core->phy[0][PHY_STATUS] & MII_SR_AUTONEG_COMPLETE) {
        trace_e1000e_link_autoneg_flowctl(true);
        core->mac[CTRL] |= E1000_CTRL_TFCE | E1000_CTRL_RFCE;
    } else {
        trace_e1000e_link_autoneg_flowctl(false);
    }
}

static inline void
e1000e_link_down(E1000ECore *core)
{
    e1000x_update_regs_on_link_down(core->mac, core->phy[0]);
    e1000e_update_flowctl_status(core);
}

static inline void
e1000e_set_phy_ctrl(E1000ECore *core, int index, uint16_t val)
{
    /* bits 0-5 reserved; MII_CR_[RESTART_AUTO_NEG,RESET] are self clearing */
    core->phy[0][PHY_CTRL] = val & ~(0x3f |
                                     MII_CR_RESET |
                                     MII_CR_RESTART_AUTO_NEG);

    if ((val & MII_CR_RESTART_AUTO_NEG) &&
        e1000e_have_autoneg(core)) {
        e1000x_restart_autoneg(core->mac, core->phy[0], core->autoneg_timer);
    }
}

static void
e1000e_set_phy_oem_bits(E1000ECore *core, int index, uint16_t val)
{
    core->phy[0][PHY_OEM_BITS] = val & ~BIT(10);

    if (val & BIT(10)) {
        e1000x_restart_autoneg(core->mac, core->phy[0], core->autoneg_timer);
    }
}

static void
e1000e_set_phy_page(E1000ECore *core, int index, uint16_t val)
{
    core->phy[0][PHY_PAGE] = val & PHY_PAGE_RW_MASK;
}

void igb_core_set_link_status(E1000ECore *core)
{
    NetClientState *nc = qemu_get_queue(core->owner_nic);
    uint32_t old_status = core->mac[STATUS];

    trace_e1000e_link_status_changed(nc->link_down ? false : true);

    if (nc->link_down) {
        e1000x_update_regs_on_link_down(core->mac, core->phy[0]);
    } else {
        if (e1000e_have_autoneg(core) &&
            !(core->phy[0][PHY_STATUS] & MII_SR_AUTONEG_COMPLETE)) {
            e1000x_restart_autoneg(core->mac, core->phy[0],
                                   core->autoneg_timer);
        } else {
            e1000x_update_regs_on_link_up(core->mac, core->phy[0]);
            igb_start_recv(core);
        }
    }

    if (core->mac[STATUS] != old_status) {
        e1000e_set_interrupt_cause(core, E1000_ICR_LSC);
    }
}

static void igb_set_ctrl(E1000ECore *core, int index, uint32_t val)
{
    trace_e1000e_core_ctrl_write(index, val);

    /* RST is self clearing */
    core->mac[CTRL] = val & ~E1000_CTRL_RST;
    core->mac[CTRL_DUP] = core->mac[CTRL];

    trace_e1000e_link_set_params(
        !!(val & E1000_CTRL_ASDE),
        (val & E1000_CTRL_SPD_SEL) >> E1000_CTRL_SPD_SHIFT,
        !!(val & E1000_CTRL_FRCSPD),
        !!(val & E1000_CTRL_FRCDPX),
        !!(val & E1000_CTRL_RFCE),
        !!(val & E1000_CTRL_TFCE));

    if (val & E1000_CTRL_RST) {
        trace_e1000e_core_ctrl_sw_reset();
        igb_core_reset(core);
    }

    if (val & E1000_CTRL_PHY_RST) {
        trace_e1000e_core_ctrl_phy_reset();
        core->mac[STATUS] |= E1000_STATUS_PHYRA;
    }
}

static void
e1000e_set_rfctl(E1000ECore *core, int index, uint32_t val)
{
    trace_e1000e_rx_set_rfctl(val);

    if (!(val & E1000_RFCTL_ISCSI_DIS)) {
        trace_e1000e_wrn_iscsi_filtering_not_supported();
    }

    if (!(val & E1000_RFCTL_NFSW_DIS)) {
        trace_e1000e_wrn_nfsw_filtering_not_supported();
    }

    if (!(val & E1000_RFCTL_NFSR_DIS)) {
        trace_e1000e_wrn_nfsr_filtering_not_supported();
    }

    core->mac[RFCTL] = val;
}

static void update_vf_select_table(E1000ECore *core)
{
    struct vf_select_table *vst;
    uint64_t macaddr;
    uint32_t rah;
    int i;

    for (i = 0; i < ARRAY_SIZE(core->vf_select_table); i++) {
        rah = core->mac[RA_VF + i*2 + 1];
        if (rah & 0x80000000) { /* Address Valid */
            macaddr = cpu_to_le16(rah & 0xFFFF);
            macaddr = (macaddr << 32) | cpu_to_le32(core->mac[RA_VF + i*2]);

            vst = &core->vf_select_table[i];
            vst->macaddr = macaddr;
            vst->vf = (rah >> 18) & 0xFF;
        }
    }
}

static void igb_mac_set_recv_addr(E1000ECore *core, int index, uint32_t val)
{
    core->mac[index] = val;

    /* Update the VF-Select table only after a write to a High register */
    if ((index % 2) == 1) {
        update_vf_select_table(core);
    }
}

static void
e1000e_calc_per_desc_buf_size(E1000ECore *core)
{
    int i;
    core->rx_desc_buf_size = 0;

    for (i = 0; i < ARRAY_SIZE(core->rxbuf_sizes); i++) {
        core->rx_desc_buf_size += core->rxbuf_sizes[i];
    }
}

static void
e1000e_parse_rxbufsize(E1000ECore *core)
{
    uint32_t rctl = core->mac[RCTL];

    memset(core->rxbuf_sizes, 0, sizeof(core->rxbuf_sizes));

    if (rctl & E1000_RCTL_DTYP_MASK) {
        uint32_t bsize;

        bsize = core->mac[PSRCTL] & E1000_PSRCTL_BSIZE0_MASK;
        core->rxbuf_sizes[0] = (bsize >> E1000_PSRCTL_BSIZE0_SHIFT) * 128;

        bsize = core->mac[PSRCTL] & E1000_PSRCTL_BSIZE1_MASK;
        core->rxbuf_sizes[1] = (bsize >> E1000_PSRCTL_BSIZE1_SHIFT) * 1024;

        bsize = core->mac[PSRCTL] & E1000_PSRCTL_BSIZE2_MASK;
        core->rxbuf_sizes[2] = (bsize >> E1000_PSRCTL_BSIZE2_SHIFT) * 1024;

        bsize = core->mac[PSRCTL] & E1000_PSRCTL_BSIZE3_MASK;
        core->rxbuf_sizes[3] = (bsize >> E1000_PSRCTL_BSIZE3_SHIFT) * 1024;
    } else if (rctl & E1000_RCTL_FLXBUF_MASK) {
        int flxbuf = rctl & E1000_RCTL_FLXBUF_MASK;
        core->rxbuf_sizes[0] = (flxbuf >> E1000_RCTL_FLXBUF_SHIFT) * 1024;
    } else {
        core->rxbuf_sizes[0] = e1000x_rxbufsize(rctl);
    }

    trace_e1000e_rx_desc_buff_sizes(core->rxbuf_sizes[0], core->rxbuf_sizes[1],
                                    core->rxbuf_sizes[2], core->rxbuf_sizes[3]);

    e1000e_calc_per_desc_buf_size(core);
}

static void
e1000e_calc_rxdesclen(E1000ECore *core)
{
    if (e1000e_rx_use_legacy_descriptor(core)) {
        core->rx_desc_len = sizeof(struct e1000_rx_desc);
    } else {
        if (core->mac[RCTL] & E1000_RCTL_DTYP_PS) {
            core->rx_desc_len = sizeof(union e1000_rx_desc_packet_split);
        } else {
            core->rx_desc_len = sizeof(union e1000_rx_desc_extended);
        }
    }
    trace_e1000e_rx_desc_len(core->rx_desc_len);
}

static void
e1000e_set_rx_control(E1000ECore *core, int index, uint32_t val)
{
    core->mac[RCTL] = val;
    trace_e1000e_rx_set_rctl(core->mac[RCTL]);

    if (val & E1000_RCTL_EN) {
        e1000e_parse_rxbufsize(core);
        e1000e_calc_rxdesclen(core);
        core->rxbuf_min_shift = ((val / E1000_RCTL_RDMTS_QUAT) & 3) + 1 +
                                E1000_RING_DESC_LEN_SHIFT;

        igb_start_recv(core);
    }
}

static
void(*e1000e_phyreg_writeops[E1000E_PHY_PAGES][E1000E_PHY_PAGE_SIZE])
(E1000ECore *, int, uint16_t) = {
    [0] = {
        [PHY_CTRL]     = e1000e_set_phy_ctrl,
        [PHY_PAGE]     = e1000e_set_phy_page,
        [PHY_OEM_BITS] = e1000e_set_phy_oem_bits
    }
};

static inline void
e1000e_clear_ims_bits(E1000ECore *core, uint32_t bits)
{
    trace_e1000e_irq_clear_ims(bits, core->mac[IMS], core->mac[IMS] & ~bits);
    core->mac[IMS] &= ~bits;
}

static inline bool
e1000e_postpone_interrupt(bool *interrupt_pending,
                          E1000IntrDelayTimer *timer)
{
    if (timer->running) {
        trace_e1000e_irq_postponed_by_xitr(timer->delay_reg << 2);

        *interrupt_pending = true;
        return true;
    }

    if (timer->core->mac[timer->delay_reg] != 0) {
        e1000e_intrmgr_rearm_timer(timer);
    }

    return false;
}

static inline bool
e1000e_eitr_should_postpone(E1000ECore *core, int idx)
{
    return e1000e_postpone_interrupt(&core->eitr_intr_pending[idx],
                                     &core->eitr[idx]);
}

static inline void
e1000e_fix_icr_asserted(E1000ECore *core)
{
    core->mac[ICR] &= ~E1000_ICR_ASSERTED;
    if (core->mac[ICR]) {
        core->mac[ICR] |= E1000_ICR_ASSERTED;
    }

    trace_e1000e_irq_fix_icr_asserted(core->mac[ICR]);
}

static void igb_send_msi(E1000ECore *core, bool msix)
{
    PCIDevice *vf;
    uint32_t causes = core->mac[EICR] & core->mac[EIMS];
    uint32_t effective_eiac;
    uint16_t vfn;
    int vector;

    for (vector = 0; vector < IGB_MSIX_VEC_NUM; ++vector) {
        if ((causes & BIT(vector)) &&
            !e1000e_eitr_should_postpone(core, vector)) {

            trace_e1000e_irq_msix_notify_vec(vector);

            if (!pcie_sriov_is_iov(core->owner) || (vector < 4)) {
                msix_notify(core->owner, vector);
            } else {
                vfn = 7 - (vector-1)/3;
                vf = pcie_sriov_get_vf(core->owner, vfn);
                if (vf) { // TODO: Remove this. vf should not be null.
                    msix_notify(vf, (vector-1)%3);
                }
            }

            trace_e1000e_irq_icr_clear_eiac(core->mac[EICR],
                                            core->mac[EIAC]);

            effective_eiac = core->mac[EIAC] & BIT(vector);
            core->mac[EICR] &= ~effective_eiac;
        }
    }
}

static void igb_update_interrupt_state(E1000ECore *core)
{
    uint32_t icr;
    uint32_t causes;
    uint32_t int_alloc;
    bool interrupts_pending;
    bool is_msix = msix_enabled(core->owner);

    icr = core->mac[ICR] & core->mac[IMS];
    if (icr) {
        if (is_msix) {
            causes = 0;
            if (icr & IGB_INT_TCP_TIMER) {
                int_alloc = core->mac[IVAR_MISC] & 0xff;
                if (int_alloc & BIT(7)) {
                    causes |= BIT(int_alloc & 0x1f);
                }
            }
            /* Check if other bits (excluding the TCP Timer) are enabled. */
            if (icr & ~IGB_INT_TCP_TIMER) {
                int_alloc = (core->mac[IVAR_MISC] >> 8) & 0xff;
                if (int_alloc & BIT(7)) {
                    causes |= BIT(int_alloc & 0x1f);
                }
            }
            core->mac[EICR] |= causes;
        } else {
            core->mac[EICR] |= IGB_EINT_OTHER_CAUSE;
            trace_e1000e_irq_add_msi_other(core->mac[EICR]);
        }
    } else {
        if (!is_msix) {
            core->mac[EICR] &= ~IGB_EINT_OTHER_CAUSE;
            e1000e_fix_icr_asserted(core);
        }
    }

    interrupts_pending = !!(core->mac[EIMS] & core->mac[EICR]);

    trace_e1000e_irq_pending_interrupts(core->mac[EIMS] & core->mac[EICR],
                                        core->mac[EICR], core->mac[EIMS]);

    if (is_msix || msi_enabled(core->owner)) {
        if (interrupts_pending) {
            igb_send_msi(core, is_msix);
        }
    } else {
        if (interrupts_pending) {
            if (!e1000e_eitr_should_postpone(core, 0)) {
                e1000e_raise_legacy_irq(core);
            }
        } else {
            e1000e_lower_legacy_irq(core);
        }
    }
}

static void
e1000e_set_interrupt_cause(E1000ECore *core, uint32_t val)
{
    trace_e1000e_irq_set_cause_entry(val, core->mac[ICR]);

    // TODO: Does the IGB have Interrupts Delay?

    val |= e1000e_intmgr_collect_delayed_causes(core);
    core->mac[ICR] |= val;

    trace_e1000e_irq_set_cause_exit(val, core->mac[ICR]);

    igb_update_interrupt_state(core);
}

static void igb_set_eics(E1000ECore *core, int index, uint32_t val)
{
    bool msix = !!(core->mac[GPIE] & IGB_GPIE_MULTIPLE_MSIX);

    trace_igb_irq_write_eics(val, msix);

    core->mac[EICS] |=
        msix ? (val & IGB_EINT_MSIX_MASK) : (val & IGB_EINT_LEGACY_MASK);

    // TODO: Move to igb_update_interrupt_state if EICS is modified in other
    // places.
    core->mac[EICR] = core->mac[EICS];

    igb_update_interrupt_state(core);
}

static void igb_set_eims(E1000ECore *core, int index, uint32_t val)
{
    bool msix = !!(core->mac[GPIE] & IGB_GPIE_MULTIPLE_MSIX);

    trace_igb_irq_write_eims(val, msix);

    core->mac[EIMS] |=
        msix ? (val & IGB_EINT_MSIX_MASK) : (val & IGB_EINT_LEGACY_MASK);

    igb_update_interrupt_state(core);
}

static void igb_vf_reset(E1000ECore *core, uint16_t vfn)
{
    // TODO: Reset of the queue enable and the interrupt registers of the VF.

    core->mac[VFMAILBOX + vfn] &= ~E1000_VFMAILBOX_RSTI;
    core->mac[VFMAILBOX + vfn] = E1000_VFMAILBOX_RSTD;
}

static void mailbox_interrupt_to_vf(E1000ECore *core, uint16_t vfn)
{
    uint32_t ent = core->mac[VTIVAR_MISC + vfn] & 0xFF;

    if (IVAR_VALID_ENTRY(ent)) {
        core->mac[EICR] |= (ent & 0x3) << (22 - vfn*3);
        igb_update_interrupt_state(core);
    }
}

static void mailbox_interrupt_to_pf(E1000ECore *core)
{
    e1000e_set_interrupt_cause(core, IGB_INT_VMMB);
}

static void igb_set_pfmailbox(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = index - PFMAILBOX;

    trace_igb_set_pfmailbox(vfn, val);

    if (val & E1000_PFMAILBOX_STS) {
        core->mac[VFMAILBOX + vfn] |= E1000_VFMAILBOX_PF_STS;
        mailbox_interrupt_to_vf(core, vfn);
    }

    if (val & E1000_PFMAILBOX_ACK) {
        core->mac[VFMAILBOX + vfn] |= E1000_VFMAILBOX_PF_ACK;
        mailbox_interrupt_to_vf(core, vfn);
    }

    /* Buffer Taken by PF (can be set only if the VFU is cleared). */
    if (val & E1000_PFMAILBOX_PFU) {
        if (!(core->mac[index] & E1000_PFMAILBOX_VFU)) {
            core->mac[index] |= E1000_PFMAILBOX_PFU;
            core->mac[VFMAILBOX + vfn] |= E1000_VFMAILBOX_PFU;
        }
    } else {
        core->mac[index] &= ~E1000_PFMAILBOX_PFU;
        core->mac[VFMAILBOX + vfn] &= ~E1000_VFMAILBOX_PFU;
    }

    if (val & E1000_PFMAILBOX_RVFU) {
        core->mac[VFMAILBOX + vfn] &= ~E1000_VFMAILBOX_VFU;
        core->mac[MBVFICR] &= ~((BIT(vfn) << 16) | BIT(vfn));
    }
}

static void igb_set_vfmailbox(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = index - VFMAILBOX;

    trace_igb_set_vfmailbox(vfn, val);

    if (val & E1000_VFMAILBOX_REQ) {
        core->mac[MBVFICR] |= BIT(vfn);
        mailbox_interrupt_to_pf(core);
    }

    if (val & E1000_VFMAILBOX_ACK) {
        core->mac[MBVFICR] |= (BIT(vfn) << 16);
        mailbox_interrupt_to_pf(core);
    }

    /* Buffer Taken by VF (can be set only if the PFU is cleared). */
    if (val & E1000_VFMAILBOX_VFU) {
        if (!(core->mac[index] & E1000_VFMAILBOX_PFU)) {
            core->mac[index] |= E1000_VFMAILBOX_VFU;
            core->mac[PFMAILBOX + vfn] |= E1000_PFMAILBOX_VFU;
        }
    } else {
        core->mac[index] &= ~E1000_VFMAILBOX_VFU;
        core->mac[PFMAILBOX + vfn] &= ~E1000_PFMAILBOX_VFU;
    }
}

static void igb_set_mbvficr(E1000ECore *core, int index, uint32_t val)
{
    core->mac[MBVFICR] &= ~(val & 0xFF00FF);
}

static void igb_set_vflre(E1000ECore *core, int index, uint32_t val)
{
    core->mac[VFLRE] &= ~(val & 0xFF);
}

static void igb_set_eimc(E1000ECore *core, int index, uint32_t val)
{
    bool msix = !!(core->mac[GPIE] & IGB_GPIE_MULTIPLE_MSIX);

    /* Interrupts are disabled via a write to EIMC and reflected in EIMS. */
    core->mac[EIMS] &=
        msix ? ~(val & IGB_EINT_MSIX_MASK) : ~(val & IGB_EINT_LEGACY_MASK);

    trace_igb_irq_write_eimc(val, core->mac[EIMS], msix);
    igb_update_interrupt_state(core);
}

static void igb_set_eiac(E1000ECore *core, int index, uint32_t val)
{
    bool msix = !!(core->mac[GPIE] & IGB_GPIE_MULTIPLE_MSIX);

    if (msix)
    {
        trace_igb_irq_write_eiac(val);

        /* TODO: When using IOV, the bits that correspond to MSI-X vectors
           that are assigned to a VF are read-only. */
        core->mac[EIAC] |= (val & IGB_EINT_MSIX_MASK);
    }
}

static void igb_set_eiam(E1000ECore *core, int index, uint32_t val)
{
    bool msix = !!(core->mac[GPIE] & IGB_GPIE_MULTIPLE_MSIX);

    /* TODO: When using IOV, the bits that correspond to MSI-X vectors that
       are assigned to a VF are read-only. */
    core->mac[EIAM] |=
        msix ? ~(val & IGB_EINT_MSIX_MASK) : ~(val & IGB_EINT_LEGACY_MASK);

    trace_igb_irq_write_eiam(val, msix);
}

static void igb_set_eicr(E1000ECore *core, int index, uint32_t val)
{
    bool msix = !!(core->mac[GPIE] & IGB_GPIE_MULTIPLE_MSIX);

    /* TODO: In IOV mode, only bit zero of this vector is available for the PF
       function. */
    core->mac[EICR] &=
        msix ? ~(val & IGB_EINT_MSIX_MASK) : ~(val & IGB_EINT_LEGACY_MASK);

    trace_igb_irq_write_eicr(val, msix);
    igb_update_interrupt_state(core);
}

static void igb_set_vtctrl(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn;

    if (val & E1000_CTRL_RST) {
        vfn = (index - VTCTRL0) / 0x40;
        igb_vf_reset(core, vfn);
    }
}

static void igb_set_vteics(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = (index - VTEICS0) / 0x40;

    core->mac[index] = val;
    igb_set_eics(core, EICS, (val & 0x7) << (22 - vfn*3));
}

static void igb_set_vteims(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = (index - VTEIMS0) / 0x40;

    core->mac[index] = val;
    igb_set_eims(core, EIMS, (val & 0x7) << (22 - vfn*3));
}

static void igb_set_vteimc(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = (index - VTEIMC0) / 0x40;

    core->mac[index] = val;
    igb_set_eimc(core, EIMC, (val & 0x7) << (22 - vfn*3));
}

static void igb_set_vteiac(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = (index - VTEIAC0) / 0x40;

    core->mac[index] = val;
    igb_set_eiac(core, EIAC, (val & 0x7) << (22 - vfn*3));
}

static void igb_set_vteiam(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = (index - VTEIAM0) / 0x40;

    core->mac[index] = val;
    igb_set_eiam(core, EIAM, (val & 0x7) << (22 - vfn*3));
}

static void igb_set_vteicr(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = (index - VTEICR0) / 0x40;

    core->mac[index] = val;
    igb_set_eicr(core, EICR, (val & 0x7) << (22 - vfn*3));
}

static void igb_set_vtivar(E1000ECore *core, int index, uint32_t val)
{
    uint16_t vfn = (index - VTIVAR);
    uint16_t qn = vfn;
    uint8_t ent;
    int n;

    core->mac[index] = val;

    /* Get assigned vector associated with queue Rx#0. */
    ent = val & 0xFF;
    if (IVAR_VALID_ENTRY(ent)) {
        n = IVAR_RX_QUEUE_ENTRY(qn);
        ent = 0x80 | (24 - vfn*3 - (2-(ent & 0x7)));
        core->mac[IVAR + n/4] |= ent << 8*(n%4);
    }

    /* Get assigned vector associated with queue Tx#0 */
    ent = (val >> 8) & 0xFF;
    if (IVAR_VALID_ENTRY(ent)) {
        n = IVAR_TX_QUEUE_ENTRY(qn);
        ent = 0x80 | (24 - vfn*3 - (2-(ent & 0x7)));
        core->mac[IVAR + n/4] |= ent << 8*(n%4);
    }

    /* Ignoring assigned vectors associated with queues Rx#1 and Tx#1 for
     * now. */
}

static inline void
e1000e_autoneg_timer(void *opaque)
{
    E1000ECore *core = opaque;
    if (!qemu_get_queue(core->owner_nic)->link_down) {
        e1000x_update_regs_on_autoneg_done(core->mac, core->phy[0]);
        igb_start_recv(core);

        e1000e_update_flowctl_status(core);
        /* signal link status change to the guest */
        e1000e_set_interrupt_cause(core, E1000_ICR_LSC);
    }
}

static inline uint16_t
e1000e_get_reg_index_with_offset(const uint16_t *mac_reg_access, hwaddr addr)
{
    uint16_t index = (addr & 0x1ffff) >> 2;
    return index + (mac_reg_access[index] & 0xfffe);
}

static const char e1000e_phy_regcap[E1000E_PHY_PAGES][0x20] = {
    [0] = {
        [PHY_CTRL]          = PHY_ANYPAGE | PHY_RW,
        [PHY_STATUS]        = PHY_ANYPAGE | PHY_R,
        [PHY_ID1]           = PHY_ANYPAGE | PHY_R,
        [PHY_ID2]           = PHY_ANYPAGE | PHY_R,
        [PHY_AUTONEG_ADV]   = PHY_ANYPAGE | PHY_RW,
        [PHY_LP_ABILITY]    = PHY_ANYPAGE | PHY_R,
        [PHY_AUTONEG_EXP]   = PHY_ANYPAGE | PHY_R,
        [PHY_NEXT_PAGE_TX]  = PHY_ANYPAGE | PHY_RW,
        [PHY_LP_NEXT_PAGE]  = PHY_ANYPAGE | PHY_R,
        [PHY_1000T_CTRL]    = PHY_ANYPAGE | PHY_RW,
        [PHY_1000T_STATUS]  = PHY_ANYPAGE | PHY_R,
        [PHY_EXT_STATUS]    = PHY_ANYPAGE | PHY_R,
        [PHY_PAGE]          = PHY_ANYPAGE | PHY_RW,

        [PHY_COPPER_CTRL1]      = PHY_RW,
        [PHY_COPPER_STAT1]      = PHY_R,
        [PHY_COPPER_CTRL3]      = PHY_RW,
        [PHY_RX_ERR_CNTR]       = PHY_R,
        [PHY_OEM_BITS]          = PHY_RW,
        [PHY_BIAS_1]            = PHY_RW,
        [PHY_BIAS_2]            = PHY_RW,
        [PHY_PAGE_SELECT]       = PHY_RW,
        [PHY_COPPER_INT_ENABLE] = PHY_RW,
        [PHY_COPPER_STAT2]      = PHY_R,
        [PHY_COPPER_CTRL2]      = PHY_RW
    },
    [2] = {
        [PHY_MAC_CTRL1]         = PHY_RW,
        [PHY_MAC_INT_ENABLE]    = PHY_RW,
        [PHY_MAC_STAT]          = PHY_R,
        [PHY_MAC_CTRL2]         = PHY_RW
    },
    [3] = {
        [PHY_LED_03_FUNC_CTRL1] = PHY_RW,
        [PHY_LED_03_POL_CTRL]   = PHY_RW,
        [PHY_LED_TIMER_CTRL]    = PHY_RW,
        [PHY_LED_45_CTRL]       = PHY_RW
    },
    [5] = {
        [PHY_1000T_SKEW]        = PHY_R,
        [PHY_1000T_SWAP]        = PHY_R
    },
    [6] = {
        [PHY_CRC_COUNTERS]      = PHY_R
    }
};

static bool
e1000e_phy_reg_check_cap(E1000ECore *core, uint32_t addr,
                         char cap, uint8_t *page)
{
    *page =
        (e1000e_phy_regcap[0][addr] & PHY_ANYPAGE) ? 0
                                                    : core->phy[0][PHY_PAGE];

    if (*page >= E1000E_PHY_PAGES) {
        return false;
    }

    return e1000e_phy_regcap[*page][addr] & cap;
}

static void
e1000e_phy_reg_write(E1000ECore *core, uint8_t page,
                     uint32_t addr, uint16_t data)
{
    assert(page < E1000E_PHY_PAGES);
    assert(addr < E1000E_PHY_PAGE_SIZE);

    if (e1000e_phyreg_writeops[page][addr]) {
        e1000e_phyreg_writeops[page][addr](core, addr, data);
    } else {
        core->phy[page][addr] = data;
    }
}

static void
e1000e_set_mdic(E1000ECore *core, int index, uint32_t val)
{
    uint32_t data = val & E1000_MDIC_DATA_MASK;
    uint32_t addr = ((val & E1000_MDIC_REG_MASK) >> E1000_MDIC_REG_SHIFT);
    uint8_t page;

    if ((val & E1000_MDIC_PHY_MASK) >> E1000_MDIC_PHY_SHIFT != 1) { /* phy # */
        val = core->mac[MDIC] | E1000_MDIC_ERROR;
    } else if (val & E1000_MDIC_OP_READ) {
        if (!e1000e_phy_reg_check_cap(core, addr, PHY_R, &page)) {
            trace_e1000e_core_mdic_read_unhandled(page, addr);
            val |= E1000_MDIC_ERROR;
        } else {
            val = (val ^ data) | core->phy[page][addr];
            trace_e1000e_core_mdic_read(page, addr, val);
        }
    } else if (val & E1000_MDIC_OP_WRITE) {
        if (!e1000e_phy_reg_check_cap(core, addr, PHY_W, &page)) {
            trace_e1000e_core_mdic_write_unhandled(page, addr);
            val |= E1000_MDIC_ERROR;
        } else {
            trace_e1000e_core_mdic_write(page, addr, data);
            e1000e_phy_reg_write(core, page, addr, data);
        }
    }
    core->mac[MDIC] = val | E1000_MDIC_READY;

    if (val & E1000_MDIC_INT_EN) {
        e1000e_set_interrupt_cause(core, E1000_ICR_MDAC);
    }
}

static void
e1000e_set_rdt(E1000ECore *core, int index, uint32_t val)
{
    core->mac[index] = val & 0xffff;
    trace_e1000e_rx_set_rdt(e1000e_mq_queue_idx(RDT0, index), val);
    igb_start_recv(core);
}

static void
e1000e_set_status(E1000ECore *core, int index, uint32_t val)
{
    if ((val & E1000_STATUS_PHYRA) == 0) {
        core->mac[index] &= ~E1000_STATUS_PHYRA;
    }
}

static void
e1000e_set_ctrlext(E1000ECore *core, int index, uint32_t val)
{
    trace_e1000e_link_set_ext_params(!!(val & E1000_CTRL_EXT_ASDCHK),
                                     !!(val & E1000_CTRL_EXT_SPD_BYPS));

    // TODO: PFRSTD

    /* Zero self-clearing bits */
    val &= ~(E1000_CTRL_EXT_ASDCHK | E1000_CTRL_EXT_EE_RST);
    core->mac[CTRL_EXT] = val;
}

static void
e1000e_set_pbaclr(E1000ECore *core, int index, uint32_t val)
{
    int i;

    core->mac[PBACLR] = val & E1000_PBACLR_VALID_MASK;

    if (!msix_enabled(core->owner)) {
        return;
    }

    for (i = 0; i < E1000E_MSIX_VEC_NUM; i++) {
        if (core->mac[PBACLR] & BIT(i)) {
            msix_clr_pending(core->owner, i);
        }
    }
}

static void
e1000e_set_fcrth(E1000ECore *core, int index, uint32_t val)
{
    core->mac[FCRTH] = val & 0xFFF8;
}

static void
e1000e_set_fcrtl(E1000ECore *core, int index, uint32_t val)
{
    core->mac[FCRTL] = val & 0x8000FFF8;
}

static inline void
e1000e_set_16bit(E1000ECore *core, int index, uint32_t val)
{
    core->mac[index] = val & 0xffff;
}

static void
e1000e_set_vet(E1000ECore *core, int index, uint32_t val)
{
    core->mac[VET] = val & 0xffff;
    core->vet = le16_to_cpu(core->mac[VET]);
    trace_e1000e_vlan_vet(core->vet);
}

static void
e1000e_set_dlen(E1000ECore *core, int index, uint32_t val)
{
    core->mac[index] = val & E1000_XDLEN_MASK;
}

static void igb_set_dbal(E1000ECore *core, int index, uint32_t val)
{
    core->mac[index] = val & IGB_XDBAL_MASK;
}

static void igb_set_tdt(E1000ECore *core, int index, uint32_t val)
{
    E1000E_TxRing txr;
    int qn = e1000e_mq_queue_idx(TDT0, index);

    core->mac[index] = val & IGB_TDT_MASK;
    igb_tx_ring_init(core, &txr, qn);
    igb_start_xmit(core, &txr);
}

static void
e1000e_set_ics(E1000ECore *core, int index, uint32_t val)
{
    trace_e1000e_irq_write_ics(val);
    e1000e_set_interrupt_cause(core, val);
}

static void write_iam_content_to_ims(E1000ECore *core)
{
    // TODO: Read and understand 8.8.11 and NSICR in 8.8.15 before removing
    // this return!
    return;

    /* If GPIE.NSICR = 0, then the copy of IAM to IMS will occur only if at
       least one bit is set in the IMS and there is a true interrupt as
       reflected in ICR.INTA. */
    if ((core->mac[GPIE] & IGB_GPIE_NSICR) ||
        (core->mac[IMS] && (core->mac[ICR] & IGB_INT_INTA)))
    {
        core->mac[IMS] = core->mac[IAM];
    }
}

static void igb_set_icr(E1000ECore *core, int index, uint32_t val)
{
    uint32_t icr = core->mac[ICR] & ~val;

    trace_igb_irq_icr_write(val, core->mac[ICR], icr);
    core->mac[ICR] = icr;
    write_iam_content_to_ims(core);
    igb_update_interrupt_state(core);
}

static void
e1000e_set_imc(E1000ECore *core, int index, uint32_t val)
{
    trace_e1000e_irq_ims_clear_set_imc(val);
    e1000e_clear_ims_bits(core, val);
    igb_update_interrupt_state(core);
}

static void igb_set_ims(E1000ECore *core, int index, uint32_t val)
{
    static const uint32_t ims_valid_mask =
        IGB_INT_TXDW     | IGB_INT_LSC      | IGB_INT_RXDMT0   |
        IGB_INT_MACSEC   | IGB_INT_RX0      | IGB_INT_RXDW     |
        IGB_INT_VMMB     | IGB_INT_GPI_SDP0 | IGB_INT_GPI_SDP1 |
        IGB_INT_GPI_SDP2 | IGB_INT_GPI_SDP3 | IGB_INT_PTRAP    |
        IGB_INT_MNG      | IGB_INT_OMED     | IGB_INT_FER      |
        IGB_INT_NFER     | IGB_INT_CSRTO    | IGB_INT_SCE      |
        IGB_INT_SW_WD    | IGB_INT_OUTSYNC  | IGB_INT_TCP_TIMER;

    uint32_t valid_val = val & ims_valid_mask;

    trace_e1000e_irq_set_ims(val, core->mac[IMS], core->mac[IMS] | valid_val);
    core->mac[IMS] |= valid_val;
    igb_update_interrupt_state(core);
}

static void
e1000e_set_rdtr(E1000ECore *core, int index, uint32_t val)
{
    e1000e_set_16bit(core, index, val);

    if ((val & E1000_RDTR_FPD) && (core->rdtr.running)) {
        trace_e1000e_irq_rdtr_fpd_running();
        e1000e_intrmgr_fire_delayed_interrupts(core);
    } else {
        trace_e1000e_irq_rdtr_fpd_not_running();
    }
}

static void
e1000e_set_tidv(E1000ECore *core, int index, uint32_t val)
{
    e1000e_set_16bit(core, index, val);

    if ((val & E1000_TIDV_FPD) && (core->tidv.running)) {
        trace_e1000e_irq_tidv_fpd_running();
        e1000e_intrmgr_fire_delayed_interrupts(core);
    } else {
        trace_e1000e_irq_tidv_fpd_not_running();
    }
}

static uint32_t
e1000e_mac_readreg(E1000ECore *core, int index)
{
    return core->mac[index];
}

static uint32_t
e1000e_mac_ics_read(E1000ECore *core, int index)
{
    trace_e1000e_irq_read_ics(core->mac[ICS]);
    return core->mac[ICS];
}

static uint32_t
e1000e_mac_ims_read(E1000ECore *core, int index)
{
    trace_e1000e_irq_read_ims(core->mac[IMS]);
    return core->mac[IMS];
}

#define E1000E_LOW_BITS_READ_FUNC(num)                      \
    static uint32_t                                         \
    e1000e_mac_low##num##_read(E1000ECore *core, int index) \
    {                                                       \
        return core->mac[index] & (BIT(num) - 1);           \
    }                                                       \

#define E1000E_LOW_BITS_READ(num)                           \
    e1000e_mac_low##num##_read

E1000E_LOW_BITS_READ_FUNC(4);
E1000E_LOW_BITS_READ_FUNC(6);
E1000E_LOW_BITS_READ_FUNC(11);
E1000E_LOW_BITS_READ_FUNC(13);
E1000E_LOW_BITS_READ_FUNC(16);

static uint32_t
e1000e_mac_swsm_read(E1000ECore *core, int index)
{
    uint32_t val = core->mac[SWSM];
    core->mac[SWSM] = val | 1;
    return val;
}

static uint32_t igb_mac_eitr_read(E1000ECore *core, int index)
{
    uint32_t val = core->eitr_guest_value[index - EITR];

    /* CNT_INGR (bit 31) is always read as zero. */
    val &= (BIT(31) - 1);

    return val;
}

static uint32_t igb_mac_pfmailbox_read(E1000ECore *core, int index)
{
    uint32_t val = core->mac[index];

    /* STS and ACK (bit 0 and 1) are always read as zero. */
    val &= 0xFC;

    return val;
}

static uint32_t igb_mac_vfmailbox_read(E1000ECore *core, int index)
{
    uint32_t val = core->mac[index];

    /* REQ and ACK (bit 0 and 1) are always read as zero. */
    val &= 0xFC;

    /* PFSTS, PFACK and RSTD (bits 4, 5 and 7) are clear after read bits. */
    core->mac[index] &= 0x4F;

    return val;
}

static uint32_t igb_mac_icr_read(E1000ECore *core, int index)
{
    uint32_t ret = core->mac[ICR];

    trace_igb_irq_icr_read(ret);

    if (core->mac[GPIE] & IGB_GPIE_NSICR) {
        trace_igb_irq_icr_clear_gpie_nsicr();
        core->mac[ICR] = 0;
    } else {
        if (core->mac[IMS] == 0) {
            trace_igb_irq_icr_clear_zero_ims();
            core->mac[ICR] = 0;
        }
    }

    write_iam_content_to_ims(core);
    igb_update_interrupt_state(core);
    return ret;
}

static uint32_t
e1000e_mac_read_clr4(E1000ECore *core, int index)
{
    uint32_t ret = core->mac[index];

    core->mac[index] = 0;
    return ret;
}

static uint32_t
e1000e_mac_read_clr8(E1000ECore *core, int index)
{
    uint32_t ret = core->mac[index];

    core->mac[index] = 0;
    core->mac[index - 1] = 0;
    return ret;
}

static uint32_t
e1000e_get_ctrl(E1000ECore *core, int index)
{
    uint32_t val = core->mac[CTRL];

    trace_e1000e_link_read_params(
        !!(val & E1000_CTRL_ASDE),
        (val & E1000_CTRL_SPD_SEL) >> E1000_CTRL_SPD_SHIFT,
        !!(val & E1000_CTRL_FRCSPD),
        !!(val & E1000_CTRL_FRCDPX),
        !!(val & E1000_CTRL_RFCE),
        !!(val & E1000_CTRL_TFCE));

    return val;
}

static uint32_t
e1000e_get_status(E1000ECore *core, int index)
{
    uint32_t res = core->mac[STATUS];

    if (!(core->mac[CTRL] & E1000_CTRL_GIO_MASTER_DISABLE)) {
        res |= E1000_STATUS_GIO_MASTER_ENABLE;
    }

    if (core->mac[CTRL] & E1000_CTRL_FRCDPX) {
        res |= (core->mac[CTRL] & E1000_CTRL_FD) ? E1000_STATUS_FD : 0;
    } else {
        res |= E1000_STATUS_FD;
    }

    if ((core->mac[CTRL] & E1000_CTRL_FRCSPD) ||
        (core->mac[CTRL_EXT] & E1000_CTRL_EXT_SPD_BYPS)) {
        switch (core->mac[CTRL] & E1000_CTRL_SPD_SEL) {
        case E1000_CTRL_SPD_10:
            res |= E1000_STATUS_SPEED_10;
            break;
        case E1000_CTRL_SPD_100:
            res |= E1000_STATUS_SPEED_100;
            break;
        case E1000_CTRL_SPD_1000:
        default:
            res |= E1000_STATUS_SPEED_1000;
            break;
        }
    } else {
        res |= E1000_STATUS_SPEED_1000;
    }

    trace_e1000e_link_status(
        !!(res & E1000_STATUS_LU),
        !!(res & E1000_STATUS_FD),
        (res & E1000_STATUS_SPEED_MASK) >> E1000_STATUS_SPEED_SHIFT,
        (res & E1000_STATUS_ASDV) >> E1000_STATUS_ASDV_SHIFT);

    return res;
}

static uint32_t
e1000e_get_tarc(E1000ECore *core, int index)
{
    return core->mac[index] & ((BIT(11) - 1) |
                                BIT(27)      |
                                BIT(28)      |
                                BIT(29)      |
                                BIT(30));
}

static void
e1000e_mac_writereg(E1000ECore *core, int index, uint32_t val)
{
    core->mac[index] = val;
}

static void igb_mac_set_macaddr(E1000ECore *core, int index, uint32_t val)
{
    uint32_t macaddr[2];

    core->mac[index] = val;

    macaddr[0] = cpu_to_le32(core->mac[RA]);
    macaddr[1] = cpu_to_le32(core->mac[RA + 1]);

    qemu_format_nic_info_str(qemu_get_queue(core->owner_nic),
        (uint8_t *) macaddr);

    trace_e1000e_mac_set_sw(MAC_ARG(macaddr));
}

static void
e1000e_set_eecd(E1000ECore *core, int index, uint32_t val)
{
    static const uint32_t ro_bits = E1000_EECD_PRES          |
                                    E1000_EECD_AUTO_RD       |
                                    E1000_EECD_SIZE_EX_MASK;

    core->mac[EECD] = (core->mac[EECD] & ro_bits) | (val & ~ro_bits);
}

static void
e1000e_set_eerd(E1000ECore *core, int index, uint32_t val)
{
    uint32_t addr = (val >> E1000_EERW_ADDR_SHIFT) & E1000_EERW_ADDR_MASK;
    uint32_t flags = 0;
    uint32_t data = 0;

    if ((addr < E1000E_EEPROM_SIZE) && (val & E1000_EERW_START)) {
        data = core->eeprom[addr];
        flags = E1000_EERW_DONE;
    }

    core->mac[EERD] = flags                           |
                      (addr << E1000_EERW_ADDR_SHIFT) |
                      (data << E1000_EERW_DATA_SHIFT);
}

static void
e1000e_set_eewr(E1000ECore *core, int index, uint32_t val)
{
    uint32_t addr = (val >> E1000_EERW_ADDR_SHIFT) & E1000_EERW_ADDR_MASK;
    uint32_t data = (val >> E1000_EERW_DATA_SHIFT) & E1000_EERW_DATA_MASK;
    uint32_t flags = 0;

    if ((addr < E1000E_EEPROM_SIZE) && (val & E1000_EERW_START)) {
        core->eeprom[addr] = data;
        flags = E1000_EERW_DONE;
    }

    core->mac[EERD] = flags                           |
                      (addr << E1000_EERW_ADDR_SHIFT) |
                      (data << E1000_EERW_DATA_SHIFT);
}

static void igb_set_eitr(E1000ECore *core, int index, uint32_t val)
{
    uint32_t interval = val & 0x7FFE;
    uint32_t eitr_num = index - EITR;

    trace_igb_irq_eitr_set(eitr_num, val);

    core->eitr_guest_value[eitr_num] = val;
    core->mac[index] = interval;
}

static void
e1000e_set_psrctl(E1000ECore *core, int index, uint32_t val)
{
    if (core->mac[RCTL] & E1000_RCTL_DTYP_MASK) {

        if ((val & E1000_PSRCTL_BSIZE0_MASK) == 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "e1000e: PSRCTL.BSIZE0 cannot be zero");
            return;
        }

        if ((val & E1000_PSRCTL_BSIZE1_MASK) == 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "e1000e: PSRCTL.BSIZE1 cannot be zero");
            return;
        }
    }

    core->mac[PSRCTL] = val;
}

static void
e1000e_update_rx_offloads(E1000ECore *core)
{
    int cso_state = e1000e_rx_l4_cso_enabled(core);

    trace_e1000e_rx_set_cso(cso_state);
}

static void
e1000e_set_rxcsum(E1000ECore *core, int index, uint32_t val)
{
    core->mac[RXCSUM] = val;
    e1000e_update_rx_offloads(core);
}

static void
e1000e_set_gcr(E1000ECore *core, int index, uint32_t val)
{
    uint32_t ro_bits = core->mac[GCR] & E1000_GCR_RO_BITS;
    core->mac[GCR] = (val & ~E1000_GCR_RO_BITS) | ro_bits;
}

#define e1000e_getreg(x)    [x] = e1000e_mac_readreg
typedef uint32_t (*readops)(E1000ECore *, int);
static const readops e1000e_macreg_readops[] = {
    e1000e_getreg(PBA),
    e1000e_getreg(WUFC),
    e1000e_getreg(MANC),
    e1000e_getreg(TOTL),
    e1000e_getreg(RDT0),
    e1000e_getreg(RDT1),
    e1000e_getreg(RDT2),
    e1000e_getreg(RDT3),
    e1000e_getreg(RDT4),
    e1000e_getreg(RDT5),
    e1000e_getreg(RDT6),
    e1000e_getreg(RDT7),
    e1000e_getreg(RDT8),
    e1000e_getreg(RDT9),
    e1000e_getreg(RDT10),
    e1000e_getreg(RDT11),
    e1000e_getreg(RDT12),
    e1000e_getreg(RDT13),
    e1000e_getreg(RDT14),
    e1000e_getreg(RDT15),
    e1000e_getreg(RDBAH0),
    e1000e_getreg(RDBAH1),
    e1000e_getreg(RDBAH2),
    e1000e_getreg(RDBAH3),
    e1000e_getreg(RDBAH4),
    e1000e_getreg(RDBAH5),
    e1000e_getreg(RDBAH6),
    e1000e_getreg(RDBAH7),
    e1000e_getreg(RDBAH8),
    e1000e_getreg(RDBAH9),
    e1000e_getreg(RDBAH10),
    e1000e_getreg(RDBAH11),
    e1000e_getreg(RDBAH12),
    e1000e_getreg(RDBAH13),
    e1000e_getreg(RDBAH14),
    e1000e_getreg(RDBAH15),
    e1000e_getreg(TDBAL0),
    e1000e_getreg(TDBAL1),
    e1000e_getreg(TDBAL2),
    e1000e_getreg(TDBAL3),
    e1000e_getreg(TDBAL4),
    e1000e_getreg(TDBAL5),
    e1000e_getreg(TDBAL6),
    e1000e_getreg(TDBAL7),
    e1000e_getreg(TDBAL8),
    e1000e_getreg(TDBAL9),
    e1000e_getreg(TDBAL10),
    e1000e_getreg(TDBAL11),
    e1000e_getreg(TDBAL12),
    e1000e_getreg(TDBAL13),
    e1000e_getreg(TDBAL14),
    e1000e_getreg(TDBAL15),
    e1000e_getreg(RDLEN0),
    e1000e_getreg(RDLEN1),
    e1000e_getreg(RDLEN2),
    e1000e_getreg(RDLEN3),
    e1000e_getreg(RDLEN4),
    e1000e_getreg(RDLEN5),
    e1000e_getreg(RDLEN6),
    e1000e_getreg(RDLEN7),
    e1000e_getreg(RDLEN8),
    e1000e_getreg(RDLEN9),
    e1000e_getreg(RDLEN10),
    e1000e_getreg(RDLEN11),
    e1000e_getreg(RDLEN12),
    e1000e_getreg(RDLEN13),
    e1000e_getreg(RDLEN14),
    e1000e_getreg(RDLEN15),
    e1000e_getreg(SRRCTL0),
    e1000e_getreg(SRRCTL1),
    e1000e_getreg(SRRCTL2),
    e1000e_getreg(SRRCTL3),
    e1000e_getreg(SRRCTL4),
    e1000e_getreg(SRRCTL5),
    e1000e_getreg(SRRCTL6),
    e1000e_getreg(SRRCTL7),
    e1000e_getreg(SRRCTL8),
    e1000e_getreg(SRRCTL9),
    e1000e_getreg(SRRCTL10),
    e1000e_getreg(SRRCTL11),
    e1000e_getreg(SRRCTL12),
    e1000e_getreg(SRRCTL13),
    e1000e_getreg(SRRCTL14),
    e1000e_getreg(SRRCTL15),
    e1000e_getreg(LATECOL),
    e1000e_getreg(SEQEC),
    e1000e_getreg(XONTXC),
    e1000e_getreg(WUS),
    e1000e_getreg(GORCL),
    e1000e_getreg(MGTPRC),
    e1000e_getreg(EERD),
    e1000e_getreg(EIAC),
    e1000e_getreg(PSRCTL),
    e1000e_getreg(MANC2H),
    e1000e_getreg(RXCSUM),
    e1000e_getreg(GSCL_3),
    e1000e_getreg(GSCN_2),
    e1000e_getreg(FCAH),
    e1000e_getreg(FCRTH),
    e1000e_getreg(FLOP),
    e1000e_getreg(FLASHT),
    e1000e_getreg(RXSTMPH),
    e1000e_getreg(TXSTMPL),
    e1000e_getreg(TIMADJL),
    e1000e_getreg(RDH0),
    e1000e_getreg(RDH1),
    e1000e_getreg(RDH2),
    e1000e_getreg(RDH3),
    e1000e_getreg(RDH4),
    e1000e_getreg(RDH5),
    e1000e_getreg(RDH6),
    e1000e_getreg(RDH7),
    e1000e_getreg(RDH8),
    e1000e_getreg(RDH9),
    e1000e_getreg(RDH10),
    e1000e_getreg(RDH11),
    e1000e_getreg(RDH12),
    e1000e_getreg(RDH13),
    e1000e_getreg(RDH14),
    e1000e_getreg(RDH15),
    e1000e_getreg(TDT0),
    e1000e_getreg(TDT1),
    e1000e_getreg(TDT2),
    e1000e_getreg(TDT3),
    e1000e_getreg(TDT4),
    e1000e_getreg(TDT5),
    e1000e_getreg(TDT6),
    e1000e_getreg(TDT7),
    e1000e_getreg(TDT8),
    e1000e_getreg(TDT9),
    e1000e_getreg(TDT10),
    e1000e_getreg(TDT11),
    e1000e_getreg(TDT12),
    e1000e_getreg(TDT13),
    e1000e_getreg(TDT14),
    e1000e_getreg(TDT15),
    e1000e_getreg(TNCRS),
    e1000e_getreg(RJC),
    e1000e_getreg(IAM),
    e1000e_getreg(GSCL_2),
    e1000e_getreg(FLSWDATA),
    e1000e_getreg(RXSATRH),
    e1000e_getreg(TIPG),
    e1000e_getreg(FLMNGCTL),
    e1000e_getreg(FLMNGCNT),
    e1000e_getreg(TSYNCTXCTL),
    e1000e_getreg(EXTCNF_SIZE),
    e1000e_getreg(EXTCNF_CTRL),
    e1000e_getreg(EEMNGDATA),
    e1000e_getreg(CTRL_EXT),
    e1000e_getreg(SYSTIMH),
    e1000e_getreg(EEMNGCTL),
    e1000e_getreg(FLMNGDATA),
    e1000e_getreg(TSYNCRXCTL),
    e1000e_getreg(LEDCTL),
    e1000e_getreg(TCTL),
    e1000e_getreg(TCTL_EXT),
    e1000e_getreg(DTXCTL),
    e1000e_getreg(RXPBS),
    e1000e_getreg(TDH0),
    e1000e_getreg(TDH1),
    e1000e_getreg(TDH2),
    e1000e_getreg(TDH3),
    e1000e_getreg(TDH4),
    e1000e_getreg(TDH5),
    e1000e_getreg(TDH6),
    e1000e_getreg(TDH7),
    e1000e_getreg(TDH8),
    e1000e_getreg(TDH9),
    e1000e_getreg(TDH10),
    e1000e_getreg(TDH11),
    e1000e_getreg(TDH12),
    e1000e_getreg(TDH13),
    e1000e_getreg(TDH14),
    e1000e_getreg(TDH15),
    e1000e_getreg(RADV),
    e1000e_getreg(ECOL),
    e1000e_getreg(DC),
    e1000e_getreg(RLEC),
    e1000e_getreg(XOFFTXC),
    e1000e_getreg(RFC),
    e1000e_getreg(RNBC),
    e1000e_getreg(MGTPTC),
    e1000e_getreg(TIMINCA),
    e1000e_getreg(RXCFGL),
    e1000e_getreg(MFUTP01),
    e1000e_getreg(FACTPS),
    e1000e_getreg(GSCL_1),
    e1000e_getreg(GSCN_0),
    e1000e_getreg(GCR2),
    e1000e_getreg(PBACLR),
    e1000e_getreg(FCTTV),
    e1000e_getreg(EEWR),
    e1000e_getreg(FLSWCTL),
    e1000e_getreg(RXSATRL),
    e1000e_getreg(SYSTIML),
    e1000e_getreg(RXUDP),
    e1000e_getreg(TORL),
    e1000e_getreg(TDLEN0),
    e1000e_getreg(TDLEN1),
    e1000e_getreg(TDLEN2),
    e1000e_getreg(TDLEN3),
    e1000e_getreg(TDLEN4),
    e1000e_getreg(TDLEN5),
    e1000e_getreg(TDLEN6),
    e1000e_getreg(TDLEN7),
    e1000e_getreg(TDLEN8),
    e1000e_getreg(TDLEN9),
    e1000e_getreg(TDLEN10),
    e1000e_getreg(TDLEN11),
    e1000e_getreg(TDLEN12),
    e1000e_getreg(TDLEN13),
    e1000e_getreg(TDLEN14),
    e1000e_getreg(TDLEN15),
    e1000e_getreg(MCC),
    e1000e_getreg(WUC),
    e1000e_getreg(EECD),
    e1000e_getreg(MFUTP23),
    e1000e_getreg(RAID),
    e1000e_getreg(FCRTV),
    e1000e_getreg(TXDCTL0),
    e1000e_getreg(TXDCTL1),
    e1000e_getreg(TXDCTL2),
    e1000e_getreg(TXDCTL3),
    e1000e_getreg(TXDCTL4),
    e1000e_getreg(TXDCTL5),
    e1000e_getreg(TXDCTL6),
    e1000e_getreg(TXDCTL7),
    e1000e_getreg(TXDCTL8),
    e1000e_getreg(TXDCTL9),
    e1000e_getreg(TXDCTL10),
    e1000e_getreg(TXDCTL11),
    e1000e_getreg(TXDCTL12),
    e1000e_getreg(TXDCTL13),
    e1000e_getreg(TXDCTL14),
    e1000e_getreg(TXDCTL15),
    e1000e_getreg(TXCTL0),
    e1000e_getreg(TXCTL1),
    e1000e_getreg(TXCTL2),
    e1000e_getreg(TXCTL3),
    e1000e_getreg(TXCTL4),
    e1000e_getreg(TXCTL5),
    e1000e_getreg(TXCTL6),
    e1000e_getreg(TXCTL7),
    e1000e_getreg(TXCTL8),
    e1000e_getreg(TXCTL9),
    e1000e_getreg(TXCTL10),
    e1000e_getreg(TXCTL11),
    e1000e_getreg(TXCTL12),
    e1000e_getreg(TXCTL13),
    e1000e_getreg(TXCTL14),
    e1000e_getreg(TXCTL15),
    e1000e_getreg(VTCTRL0),
    e1000e_getreg(VTCTRL1),
    e1000e_getreg(VTCTRL2),
    e1000e_getreg(VTCTRL3),
    e1000e_getreg(VTCTRL4),
    e1000e_getreg(VTCTRL5),
    e1000e_getreg(VTCTRL6),
    e1000e_getreg(VTCTRL7),
    e1000e_getreg(VTEIMS0),
    e1000e_getreg(VTEIMS1),
    e1000e_getreg(VTEIMS2),
    e1000e_getreg(VTEIMS3),
    e1000e_getreg(VTEIMS4),
    e1000e_getreg(VTEIMS5),
    e1000e_getreg(VTEIMS6),
    e1000e_getreg(VTEIMS7),
    e1000e_getreg(VTEIAC0),
    e1000e_getreg(VTEIAC1),
    e1000e_getreg(VTEIAC2),
    e1000e_getreg(VTEIAC3),
    e1000e_getreg(VTEIAC4),
    e1000e_getreg(VTEIAC5),
    e1000e_getreg(VTEIAC6),
    e1000e_getreg(VTEIAC7),
    e1000e_getreg(VTEIAM0),
    e1000e_getreg(VTEIAM1),
    e1000e_getreg(VTEIAM2),
    e1000e_getreg(VTEIAM3),
    e1000e_getreg(VTEIAM4),
    e1000e_getreg(VTEIAM5),
    e1000e_getreg(VTEIAM6),
    e1000e_getreg(VTEIAM7),
    e1000e_getreg(VFGPRC0),
    e1000e_getreg(VFGPRC1),
    e1000e_getreg(VFGPRC2),
    e1000e_getreg(VFGPRC3),
    e1000e_getreg(VFGPRC4),
    e1000e_getreg(VFGPRC5),
    e1000e_getreg(VFGPRC6),
    e1000e_getreg(VFGPRC7),
    e1000e_getreg(VFGPTC0),
    e1000e_getreg(VFGPTC1),
    e1000e_getreg(VFGPTC2),
    e1000e_getreg(VFGPTC3),
    e1000e_getreg(VFGPTC4),
    e1000e_getreg(VFGPTC5),
    e1000e_getreg(VFGPTC6),
    e1000e_getreg(VFGPTC7),
    e1000e_getreg(VFGORC0),
    e1000e_getreg(VFGORC1),
    e1000e_getreg(VFGORC2),
    e1000e_getreg(VFGORC3),
    e1000e_getreg(VFGORC4),
    e1000e_getreg(VFGORC5),
    e1000e_getreg(VFGORC6),
    e1000e_getreg(VFGORC7),
    e1000e_getreg(VFGOTC0),
    e1000e_getreg(VFGOTC1),
    e1000e_getreg(VFGOTC2),
    e1000e_getreg(VFGOTC3),
    e1000e_getreg(VFGOTC4),
    e1000e_getreg(VFGOTC5),
    e1000e_getreg(VFGOTC6),
    e1000e_getreg(VFGOTC7),
    e1000e_getreg(VFMPRC0),
    e1000e_getreg(VFMPRC1),
    e1000e_getreg(VFMPRC2),
    e1000e_getreg(VFMPRC3),
    e1000e_getreg(VFMPRC4),
    e1000e_getreg(VFMPRC5),
    e1000e_getreg(VFMPRC6),
    e1000e_getreg(VFMPRC7),
    e1000e_getreg(VFGPRLBC0),
    e1000e_getreg(VFGPRLBC1),
    e1000e_getreg(VFGPRLBC2),
    e1000e_getreg(VFGPRLBC3),
    e1000e_getreg(VFGPRLBC4),
    e1000e_getreg(VFGPRLBC5),
    e1000e_getreg(VFGPRLBC6),
    e1000e_getreg(VFGPRLBC7),
    e1000e_getreg(VFGPTLBC0),
    e1000e_getreg(VFGPTLBC1),
    e1000e_getreg(VFGPTLBC2),
    e1000e_getreg(VFGPTLBC3),
    e1000e_getreg(VFGPTLBC4),
    e1000e_getreg(VFGPTLBC5),
    e1000e_getreg(VFGPTLBC6),
    e1000e_getreg(VFGPTLBC7),
    e1000e_getreg(VFGORLBC0),
    e1000e_getreg(VFGORLBC1),
    e1000e_getreg(VFGORLBC2),
    e1000e_getreg(VFGORLBC3),
    e1000e_getreg(VFGORLBC4),
    e1000e_getreg(VFGORLBC5),
    e1000e_getreg(VFGORLBC6),
    e1000e_getreg(VFGORLBC7),
    e1000e_getreg(VFGOTLBC0),
    e1000e_getreg(VFGOTLBC1),
    e1000e_getreg(VFGOTLBC2),
    e1000e_getreg(VFGOTLBC3),
    e1000e_getreg(VFGOTLBC4),
    e1000e_getreg(VFGOTLBC5),
    e1000e_getreg(VFGOTLBC6),
    e1000e_getreg(VFGOTLBC7),
    e1000e_getreg(RCTL),
    e1000e_getreg(MDIC),
    e1000e_getreg(FCRUC),
    e1000e_getreg(VET),
    e1000e_getreg(RDBAL0),
    e1000e_getreg(RDBAL1),
    e1000e_getreg(RDBAL2),
    e1000e_getreg(RDBAL3),
    e1000e_getreg(RDBAL4),
    e1000e_getreg(RDBAL5),
    e1000e_getreg(RDBAL6),
    e1000e_getreg(RDBAL7),
    e1000e_getreg(RDBAL8),
    e1000e_getreg(RDBAL9),
    e1000e_getreg(RDBAL10),
    e1000e_getreg(RDBAL11),
    e1000e_getreg(RDBAL12),
    e1000e_getreg(RDBAL13),
    e1000e_getreg(RDBAL14),
    e1000e_getreg(RDBAL15),
    e1000e_getreg(TDBAH0),
    e1000e_getreg(TDBAH1),
    e1000e_getreg(TDBAH2),
    e1000e_getreg(TDBAH3),
    e1000e_getreg(TDBAH4),
    e1000e_getreg(TDBAH5),
    e1000e_getreg(TDBAH6),
    e1000e_getreg(TDBAH7),
    e1000e_getreg(TDBAH8),
    e1000e_getreg(TDBAH9),
    e1000e_getreg(TDBAH10),
    e1000e_getreg(TDBAH11),
    e1000e_getreg(TDBAH12),
    e1000e_getreg(TDBAH13),
    e1000e_getreg(TDBAH14),
    e1000e_getreg(TDBAH15),
    e1000e_getreg(RDTR),
    e1000e_getreg(SCC),
    e1000e_getreg(COLC),
    e1000e_getreg(CEXTERR),
    e1000e_getreg(XOFFRXC),
    e1000e_getreg(IPAV),
    e1000e_getreg(GOTCL),
    e1000e_getreg(MGTPDC),
    e1000e_getreg(GCR),
    e1000e_getreg(POEMB),
    e1000e_getreg(MFVAL),
    e1000e_getreg(FUNCTAG),
    e1000e_getreg(GSCL_4),
    e1000e_getreg(GSCN_3),
    e1000e_getreg(MRQC),
    e1000e_getreg(FCT),
    e1000e_getreg(FLA),
    e1000e_getreg(FLOL),
    e1000e_getreg(RXDCTL0),
    e1000e_getreg(RXDCTL1),
    e1000e_getreg(RXDCTL2),
    e1000e_getreg(RXDCTL3),
    e1000e_getreg(RXDCTL4),
    e1000e_getreg(RXDCTL5),
    e1000e_getreg(RXDCTL6),
    e1000e_getreg(RXDCTL7),
    e1000e_getreg(RXDCTL8),
    e1000e_getreg(RXDCTL9),
    e1000e_getreg(RXDCTL10),
    e1000e_getreg(RXDCTL11),
    e1000e_getreg(RXDCTL12),
    e1000e_getreg(RXDCTL13),
    e1000e_getreg(RXDCTL14),
    e1000e_getreg(RXDCTL15),
    e1000e_getreg(RXSTMPL),
    e1000e_getreg(TXSTMPH),
    e1000e_getreg(TIMADJH),
    e1000e_getreg(FCRTL),
    e1000e_getreg(TADV),
    e1000e_getreg(XONRXC),
    e1000e_getreg(TSCTFC),
    e1000e_getreg(RFCTL),
    e1000e_getreg(GSCN_1),
    e1000e_getreg(FCAL),
    e1000e_getreg(FLSWCNT),
    e1000e_getreg(GPIE),
    e1000e_getreg(TXPBS),
    e1000e_getreg(RLPML),

    [TOTH]    = e1000e_mac_read_clr8,
    [GOTCH]   = e1000e_mac_read_clr8,
    [PRC64]   = e1000e_mac_read_clr4,
    [PRC255]  = e1000e_mac_read_clr4,
    [PRC1023] = e1000e_mac_read_clr4,
    [PTC64]   = e1000e_mac_read_clr4,
    [PTC255]  = e1000e_mac_read_clr4,
    [PTC1023] = e1000e_mac_read_clr4,
    [GPRC]    = e1000e_mac_read_clr4,
    [TPT]     = e1000e_mac_read_clr4,
    [RUC]     = e1000e_mac_read_clr4,
    [BPRC]    = e1000e_mac_read_clr4,
    [MPTC]    = e1000e_mac_read_clr4,
    [IAC]     = e1000e_mac_read_clr4,
    [ICR]     = igb_mac_icr_read,
    [RDFH]    = E1000E_LOW_BITS_READ(13),
    [RDFHS]   = E1000E_LOW_BITS_READ(13),
    [RDFPC]   = E1000E_LOW_BITS_READ(13),
    [TDFH]    = E1000E_LOW_BITS_READ(13),
    [TDFHS]   = E1000E_LOW_BITS_READ(13),
    [STATUS]  = e1000e_get_status,
    [TARC0]   = e1000e_get_tarc,
    [PBS]     = E1000E_LOW_BITS_READ(6),
    [ICS]     = e1000e_mac_ics_read,
    /* 8.8.10: Reading the IMC register returns the value of the IMS register.
    */
    [IMC]     = e1000e_mac_ims_read,
    [AIT]     = E1000E_LOW_BITS_READ(16),
    [TORH]    = e1000e_mac_read_clr8,
    [GORCH]   = e1000e_mac_read_clr8,
    [PRC127]  = e1000e_mac_read_clr4,
    [PRC511]  = e1000e_mac_read_clr4,
    [PRC1522] = e1000e_mac_read_clr4,
    [PTC127]  = e1000e_mac_read_clr4,
    [PTC511]  = e1000e_mac_read_clr4,
    [PTC1522] = e1000e_mac_read_clr4,
    [GPTC]    = e1000e_mac_read_clr4,
    [TPR]     = e1000e_mac_read_clr4,
    [ROC]     = e1000e_mac_read_clr4,
    [MPRC]    = e1000e_mac_read_clr4,
    [BPTC]    = e1000e_mac_read_clr4,
    [TSCTC]   = e1000e_mac_read_clr4,
    [RDFT]    = E1000E_LOW_BITS_READ(13),
    [RDFTS]   = E1000E_LOW_BITS_READ(13),
    [TDFPC]   = E1000E_LOW_BITS_READ(13),
    [TDFT]    = E1000E_LOW_BITS_READ(13),
    [TDFTS]   = E1000E_LOW_BITS_READ(13),
    [CTRL]    = e1000e_get_ctrl,
    [TARC1]   = e1000e_get_tarc,
    [SWSM]    = e1000e_mac_swsm_read,
    [IMS]     = e1000e_mac_ims_read,

    /* TBD: These are E1000E specific: */
    [CRCERRS ... MPC]      = e1000e_mac_readreg,
    [IP6AT ... IP6AT + 3]  = e1000e_mac_readreg,
    [IP4AT ... IP4AT + 6]  = e1000e_mac_readreg,
    [RA ... RA + 31]       = e1000e_mac_readreg,
    [RA_VF ... RA_VF + 31] = e1000e_mac_readreg,
    [WUPM ... WUPM + 31]   = e1000e_mac_readreg,
    [MTA ... MTA + 127]    = e1000e_mac_readreg,
    [VFTA ... VFTA + 127]  = e1000e_mac_readreg,
    [FFMT ... FFMT + 254]  = E1000E_LOW_BITS_READ(4),
    [FFVT ... FFVT + 254]  = e1000e_mac_readreg,
    [MDEF ... MDEF + 7]    = e1000e_mac_readreg,
    [FFLT ... FFLT + 10]   = E1000E_LOW_BITS_READ(11),
    [FTFT ... FTFT + 254]  = e1000e_mac_readreg,
    [PBM ... PBM + 10239]  = e1000e_mac_readreg,
    [RETA ... RETA + 31]   = e1000e_mac_readreg,
    [RSSRK ... RSSRK + 9] = e1000e_mac_readreg,
    [MAVTV0 ... MAVTV3]    = e1000e_mac_readreg,
    [EITR ... EITR + IGB_MSIX_VEC_NUM - 1] = igb_mac_eitr_read,
    [VTEICR0] = e1000e_mac_read_clr4,
    [VTEICR1] = e1000e_mac_read_clr4,
    [VTEICR2] = e1000e_mac_read_clr4,
    [VTEICR3] = e1000e_mac_read_clr4,
    [VTEICR4] = e1000e_mac_read_clr4,
    [VTEICR5] = e1000e_mac_read_clr4,
    [VTEICR6] = e1000e_mac_read_clr4,
    [VTEICR7] = e1000e_mac_read_clr4,

    /* IGB specific - should go in a disjoint struct
     * but put here now just to make diffs easier:
     */
    [FWSM]       = e1000e_mac_readreg,
    [SW_FW_SYNC] = e1000e_mac_readreg,
    [HTCBDPC]    = e1000e_mac_read_clr4,
    [EICR]       = e1000e_mac_read_clr4,
    [EIMS]       = e1000e_mac_readreg,
    [EIAM]       = e1000e_mac_readreg,
    [IVAR ... IVAR + 7] = e1000e_mac_readreg,
    e1000e_getreg(IVAR_MISC),
    [PFMAILBOX ... PFMAILBOX + 7] = igb_mac_pfmailbox_read,
    [VFMAILBOX ... VFMAILBOX + 7] = igb_mac_vfmailbox_read,
    e1000e_getreg(MBVFICR),
    [VMBMEM ... VMBMEM + 127] = e1000e_mac_readreg,
    e1000e_getreg(MBVFIMR),
    e1000e_getreg(VFLRE),
    e1000e_getreg(VFRE),
    e1000e_getreg(VFTE),
    e1000e_getreg(QDE),
    e1000e_getreg(DTXSWC),
    e1000e_getreg(RPLOLR),
    [VLVF ... VLVF + 31] = e1000e_mac_readreg,
    [VMVIR ... VMVIR + 7] = e1000e_mac_readreg,
    [VMOLR ... VMOLR + 7] = e1000e_mac_readreg,
    [WVBR] = e1000e_mac_read_clr4,
    [RQDPC ... RQDPC + IGB_NUM_QUEUES - 1] = e1000e_mac_read_clr4,
    [VTIVAR ... VTIVAR + 7] = e1000e_mac_readreg,
    [VTIVAR_MISC ... VTIVAR_MISC + 7] = e1000e_mac_readreg,
};
enum { E1000E_NREADOPS = ARRAY_SIZE(e1000e_macreg_readops) };

#define e1000e_putreg(x)    [x] = e1000e_mac_writereg
typedef void (*writeops)(E1000ECore *, int, uint32_t);
static const writeops e1000e_macreg_writeops[] = {
    e1000e_putreg(PBA),
    e1000e_putreg(SWSM),
    e1000e_putreg(WUFC),
    e1000e_putreg(RDBAH0),
    e1000e_putreg(RDBAH1),
    e1000e_putreg(RDBAH2),
    e1000e_putreg(RDBAH3),
    e1000e_putreg(RDBAH4),
    e1000e_putreg(RDBAH5),
    e1000e_putreg(RDBAH6),
    e1000e_putreg(RDBAH7),
    e1000e_putreg(RDBAH8),
    e1000e_putreg(RDBAH9),
    e1000e_putreg(RDBAH10),
    e1000e_putreg(RDBAH11),
    e1000e_putreg(RDBAH12),
    e1000e_putreg(RDBAH13),
    e1000e_putreg(RDBAH14),
    e1000e_putreg(RDBAH15),
    e1000e_putreg(SRRCTL0),
    e1000e_putreg(SRRCTL1),
    e1000e_putreg(SRRCTL2),
    e1000e_putreg(SRRCTL3),
    e1000e_putreg(SRRCTL4),
    e1000e_putreg(SRRCTL5),
    e1000e_putreg(SRRCTL6),
    e1000e_putreg(SRRCTL7),
    e1000e_putreg(SRRCTL8),
    e1000e_putreg(SRRCTL9),
    e1000e_putreg(SRRCTL10),
    e1000e_putreg(SRRCTL11),
    e1000e_putreg(SRRCTL12),
    e1000e_putreg(SRRCTL13),
    e1000e_putreg(SRRCTL14),
    e1000e_putreg(SRRCTL15),
    e1000e_putreg(RXDCTL0),
    e1000e_putreg(RXDCTL1),
    e1000e_putreg(RXDCTL2),
    e1000e_putreg(RXDCTL3),
    e1000e_putreg(RXDCTL4),
    e1000e_putreg(RXDCTL5),
    e1000e_putreg(RXDCTL6),
    e1000e_putreg(RXDCTL7),
    e1000e_putreg(RXDCTL8),
    e1000e_putreg(RXDCTL9),
    e1000e_putreg(RXDCTL10),
    e1000e_putreg(RXDCTL11),
    e1000e_putreg(RXDCTL12),
    e1000e_putreg(RXDCTL13),
    e1000e_putreg(RXDCTL14),
    e1000e_putreg(RXDCTL15),
    e1000e_putreg(LEDCTL),
    e1000e_putreg(TCTL),
    e1000e_putreg(TCTL_EXT),
    e1000e_putreg(DTXCTL),
    e1000e_putreg(RXPBS),
    e1000e_putreg(RQDPC),
    e1000e_putreg(FCAL),
    e1000e_putreg(FCRUC),
    e1000e_putreg(AIT),
    e1000e_putreg(TDFH),
    e1000e_putreg(TDFT),
    e1000e_putreg(TDFHS),
    e1000e_putreg(TDFTS),
    e1000e_putreg(TDFPC),
    e1000e_putreg(WUC),
    e1000e_putreg(WUS),
    e1000e_putreg(RDFH),
    e1000e_putreg(RDFT),
    e1000e_putreg(RDFHS),
    e1000e_putreg(RDFTS),
    e1000e_putreg(RDFPC),
    e1000e_putreg(IPAV),
    e1000e_putreg(TDBAH0),
    e1000e_putreg(TDBAH1),
    e1000e_putreg(TDBAH2),
    e1000e_putreg(TDBAH3),
    e1000e_putreg(TDBAH4),
    e1000e_putreg(TDBAH5),
    e1000e_putreg(TDBAH6),
    e1000e_putreg(TDBAH7),
    e1000e_putreg(TDBAH8),
    e1000e_putreg(TDBAH9),
    e1000e_putreg(TDBAH10),
    e1000e_putreg(TDBAH11),
    e1000e_putreg(TDBAH12),
    e1000e_putreg(TDBAH13),
    e1000e_putreg(TDBAH14),
    e1000e_putreg(TDBAH15),
    e1000e_putreg(TIMINCA),
    e1000e_putreg(IAM),
    e1000e_putreg(TARC0),
    e1000e_putreg(TARC1),
    e1000e_putreg(FLSWDATA),
    e1000e_putreg(POEMB),
    e1000e_putreg(PBS),
    e1000e_putreg(MFUTP01),
    e1000e_putreg(MFUTP23),
    e1000e_putreg(MANC),
    e1000e_putreg(MANC2H),
    e1000e_putreg(MFVAL),
    e1000e_putreg(EXTCNF_CTRL),
    e1000e_putreg(FACTPS),
    e1000e_putreg(FUNCTAG),
    e1000e_putreg(GSCL_1),
    e1000e_putreg(GSCL_2),
    e1000e_putreg(GSCL_3),
    e1000e_putreg(GSCL_4),
    e1000e_putreg(GSCN_0),
    e1000e_putreg(GSCN_1),
    e1000e_putreg(GSCN_2),
    e1000e_putreg(GSCN_3),
    e1000e_putreg(GCR2),
    e1000e_putreg(MRQC),
    e1000e_putreg(FLOP),
    e1000e_putreg(FLOL),
    e1000e_putreg(FLSWCTL),
    e1000e_putreg(FLSWCNT),
    e1000e_putreg(FLA),
    e1000e_putreg(TXDCTL0),
    e1000e_putreg(TXDCTL1),
    e1000e_putreg(TXDCTL2),
    e1000e_putreg(TXDCTL3),
    e1000e_putreg(TXDCTL4),
    e1000e_putreg(TXDCTL5),
    e1000e_putreg(TXDCTL6),
    e1000e_putreg(TXDCTL7),
    e1000e_putreg(TXDCTL8),
    e1000e_putreg(TXDCTL9),
    e1000e_putreg(TXDCTL10),
    e1000e_putreg(TXDCTL11),
    e1000e_putreg(TXDCTL12),
    e1000e_putreg(TXDCTL13),
    e1000e_putreg(TXDCTL14),
    e1000e_putreg(TXDCTL15),
    e1000e_putreg(TXCTL0),
    e1000e_putreg(TXCTL1),
    e1000e_putreg(TXCTL2),
    e1000e_putreg(TXCTL3),
    e1000e_putreg(TXCTL4),
    e1000e_putreg(TXCTL5),
    e1000e_putreg(TXCTL6),
    e1000e_putreg(TXCTL7),
    e1000e_putreg(TXCTL8),
    e1000e_putreg(TXCTL9),
    e1000e_putreg(TXCTL10),
    e1000e_putreg(TXCTL11),
    e1000e_putreg(TXCTL12),
    e1000e_putreg(TXCTL13),
    e1000e_putreg(TXCTL14),
    e1000e_putreg(TXCTL15),
    e1000e_putreg(TIPG),
    e1000e_putreg(RXSTMPH),
    e1000e_putreg(RXSTMPL),
    e1000e_putreg(RXSATRL),
    e1000e_putreg(RXSATRH),
    e1000e_putreg(TXSTMPL),
    e1000e_putreg(TXSTMPH),
    e1000e_putreg(SYSTIML),
    e1000e_putreg(SYSTIMH),
    e1000e_putreg(TIMADJL),
    e1000e_putreg(TIMADJH),
    e1000e_putreg(RXUDP),
    e1000e_putreg(RXCFGL),
    e1000e_putreg(TSYNCRXCTL),
    e1000e_putreg(TSYNCTXCTL),
    e1000e_putreg(EXTCNF_SIZE),
    e1000e_putreg(EEMNGCTL),
    e1000e_putreg(GPIE),
    e1000e_putreg(TXPBS),
    e1000e_putreg(RLPML),

    [TDH0]     = e1000e_set_16bit,
    [TDH1]     = e1000e_set_16bit,
    [TDH2]     = e1000e_set_16bit,
    [TDH3]     = e1000e_set_16bit,
    [TDH4]     = e1000e_set_16bit,
    [TDH5]     = e1000e_set_16bit,
    [TDH6]     = e1000e_set_16bit,
    [TDH7]     = e1000e_set_16bit,
    [TDH8]     = e1000e_set_16bit,
    [TDH9]     = e1000e_set_16bit,
    [TDH10]    = e1000e_set_16bit,
    [TDH11]    = e1000e_set_16bit,
    [TDH12]    = e1000e_set_16bit,
    [TDH13]    = e1000e_set_16bit,
    [TDH14]    = e1000e_set_16bit,
    [TDH15]    = e1000e_set_16bit,
    [TDT0]     = igb_set_tdt,
    [TDT1]     = igb_set_tdt,
    [TDT2]     = igb_set_tdt,
    [TDT3]     = igb_set_tdt,
    [TDT4]     = igb_set_tdt,
    [TDT5]     = igb_set_tdt,
    [TDT6]     = igb_set_tdt,
    [TDT7]     = igb_set_tdt,
    [TDT8]     = igb_set_tdt,
    [TDT9]     = igb_set_tdt,
    [TDT10]    = igb_set_tdt,
    [TDT11]    = igb_set_tdt,
    [TDT12]    = igb_set_tdt,
    [TDT13]    = igb_set_tdt,
    [TDT14]    = igb_set_tdt,
    [TDT15]    = igb_set_tdt,
    [MDIC]     = e1000e_set_mdic,
    [ICS]      = e1000e_set_ics,
    [RDH0]     = e1000e_set_16bit,
    [RDH1]     = e1000e_set_16bit,
    [RDH2]     = e1000e_set_16bit,
    [RDH3]     = e1000e_set_16bit,
    [RDH4]     = e1000e_set_16bit,
    [RDH5]     = e1000e_set_16bit,
    [RDH6]     = e1000e_set_16bit,
    [RDH7]     = e1000e_set_16bit,
    [RDH8]     = e1000e_set_16bit,
    [RDH9]     = e1000e_set_16bit,
    [RDH10]    = e1000e_set_16bit,
    [RDH11]    = e1000e_set_16bit,
    [RDH12]    = e1000e_set_16bit,
    [RDH13]    = e1000e_set_16bit,
    [RDH14]    = e1000e_set_16bit,
    [RDH15]    = e1000e_set_16bit,
    [RDT0]     = e1000e_set_rdt,
    [RDT1]     = e1000e_set_rdt,
    [RDT2]     = e1000e_set_rdt,
    [RDT3]     = e1000e_set_rdt,
    [RDT4]     = e1000e_set_rdt,
    [RDT5]     = e1000e_set_rdt,
    [RDT6]     = e1000e_set_rdt,
    [RDT7]     = e1000e_set_rdt,
    [RDT8]     = e1000e_set_rdt,
    [RDT9]     = e1000e_set_rdt,
    [RDT10]    = e1000e_set_rdt,
    [RDT11]    = e1000e_set_rdt,
    [RDT12]    = e1000e_set_rdt,
    [RDT13]    = e1000e_set_rdt,
    [RDT14]    = e1000e_set_rdt,
    [RDT15]    = e1000e_set_rdt,
    [IMC]      = e1000e_set_imc,
    [IMS]      = igb_set_ims,
    [ICR]      = igb_set_icr,
    [EECD]     = e1000e_set_eecd,
    [RCTL]     = e1000e_set_rx_control,
    [CTRL]     = igb_set_ctrl,
    [RDTR]     = e1000e_set_rdtr,
    [RADV]     = e1000e_set_16bit,
    [TADV]     = e1000e_set_16bit,
    [EERD]     = e1000e_set_eerd,
    [GCR]      = e1000e_set_gcr,
    [PSRCTL]   = e1000e_set_psrctl,
    [RXCSUM]   = e1000e_set_rxcsum,
    [RAID]     = e1000e_set_16bit,
    [TIDV]     = e1000e_set_tidv,
    [TDLEN0]   = e1000e_set_dlen,
    [TDLEN1]   = e1000e_set_dlen,
    [TDLEN2]   = e1000e_set_dlen,
    [TDLEN3]   = e1000e_set_dlen,
    [TDLEN4]   = e1000e_set_dlen,
    [TDLEN5]   = e1000e_set_dlen,
    [TDLEN6]   = e1000e_set_dlen,
    [TDLEN7]   = e1000e_set_dlen,
    [TDLEN8]   = e1000e_set_dlen,
    [TDLEN9]   = e1000e_set_dlen,
    [TDLEN10]  = e1000e_set_dlen,
    [TDLEN11]  = e1000e_set_dlen,
    [TDLEN12]  = e1000e_set_dlen,
    [TDLEN13]  = e1000e_set_dlen,
    [TDLEN14]  = e1000e_set_dlen,
    [TDLEN15]  = e1000e_set_dlen,
    [RDLEN0]   = e1000e_set_dlen,
    [RDLEN1]   = e1000e_set_dlen,
    [RDLEN2]   = e1000e_set_dlen,
    [RDLEN3]   = e1000e_set_dlen,
    [RDLEN4]   = e1000e_set_dlen,
    [RDLEN5]   = e1000e_set_dlen,
    [RDLEN6]   = e1000e_set_dlen,
    [RDLEN7]   = e1000e_set_dlen,
    [RDLEN8]   = e1000e_set_dlen,
    [RDLEN9]   = e1000e_set_dlen,
    [RDLEN10]  = e1000e_set_dlen,
    [RDLEN11]  = e1000e_set_dlen,
    [RDLEN12]  = e1000e_set_dlen,
    [RDLEN13]  = e1000e_set_dlen,
    [RDLEN14]  = e1000e_set_dlen,
    [RDLEN15]  = e1000e_set_dlen,
    [TDBAL0]   = igb_set_dbal,
    [TDBAL1]   = igb_set_dbal,
    [TDBAL2]   = igb_set_dbal,
    [TDBAL3]   = igb_set_dbal,
    [TDBAL4]   = igb_set_dbal,
    [TDBAL5]   = igb_set_dbal,
    [TDBAL6]   = igb_set_dbal,
    [TDBAL7]   = igb_set_dbal,
    [TDBAL8]   = igb_set_dbal,
    [TDBAL9]   = igb_set_dbal,
    [TDBAL10]  = igb_set_dbal,
    [TDBAL11]  = igb_set_dbal,
    [TDBAL12]  = igb_set_dbal,
    [TDBAL13]  = igb_set_dbal,
    [TDBAL14]  = igb_set_dbal,
    [TDBAL15]  = igb_set_dbal,
    [RDBAL0]   = igb_set_dbal,
    [RDBAL1]   = igb_set_dbal,
    [RDBAL2]   = igb_set_dbal,
    [RDBAL3]   = igb_set_dbal,
    [RDBAL4]   = igb_set_dbal,
    [RDBAL5]   = igb_set_dbal,
    [RDBAL6]   = igb_set_dbal,
    [RDBAL7]   = igb_set_dbal,
    [RDBAL8]   = igb_set_dbal,
    [RDBAL9]   = igb_set_dbal,
    [RDBAL10]  = igb_set_dbal,
    [RDBAL11]  = igb_set_dbal,
    [RDBAL12]  = igb_set_dbal,
    [RDBAL13]  = igb_set_dbal,
    [RDBAL14]  = igb_set_dbal,
    [RDBAL15]  = igb_set_dbal,
    [STATUS]   = e1000e_set_status,
    [PBACLR]   = e1000e_set_pbaclr,
    [CTRL_EXT] = e1000e_set_ctrlext,
    [FCAH]     = e1000e_set_16bit,
    [FCT]      = e1000e_set_16bit,
    [FCTTV]    = e1000e_set_16bit,
    [FCRTV]    = e1000e_set_16bit,
    [FCRTH]    = e1000e_set_fcrth,
    [FCRTL]    = e1000e_set_fcrtl,
    [VET]      = e1000e_set_vet,
    [FLASHT]   = e1000e_set_16bit,
    [EEWR]     = e1000e_set_eewr,
    [CTRL_DUP] = igb_set_ctrl,
    [RFCTL]    = e1000e_set_rfctl,

    [IP6AT ... IP6AT + 3]    = e1000e_mac_writereg,
    [IP4AT ... IP4AT + 6]    = e1000e_mac_writereg,
    [RA]                     = e1000e_mac_writereg,
    [RA + 1]                 = igb_mac_set_macaddr,
    [RA + 2 ... RA + 31]     = e1000e_mac_writereg,
    [RA_VF ... RA_VF + 31]   = igb_mac_set_recv_addr,
    [WUPM ... WUPM + 31]     = e1000e_mac_writereg,
    [MTA ... MTA + 127]      = e1000e_mac_writereg,
    [VFTA ... VFTA + 127]    = e1000e_mac_writereg,
    [FFMT ... FFMT + 254]    = e1000e_mac_writereg,
    [FFVT ... FFVT + 254]    = e1000e_mac_writereg,
    [PBM ... PBM + 10239]    = e1000e_mac_writereg,
    [MDEF ... MDEF + 7]      = e1000e_mac_writereg,
    [FFLT ... FFLT + 10]     = e1000e_mac_writereg,
    [FTFT ... FTFT + 254]    = e1000e_mac_writereg,
    [RETA ... RETA + 31]     = e1000e_mac_writereg,
    [RSSRK ... RSSRK + 9]   = e1000e_mac_writereg,
    [MAVTV0 ... MAVTV3]      = e1000e_mac_writereg,
    [EITR ... EITR + IGB_MSIX_VEC_NUM - 1] = igb_set_eitr,

    /* IGB specific - should go in a disjoint struct
     * but put here now just to make changes comprehensible:
     */
    [FWSM]     = e1000e_mac_writereg,
    [SW_FW_SYNC] = e1000e_mac_writereg,
    [EICR] = igb_set_eicr,
    [EICS] = igb_set_eics,
    [EIAC] = igb_set_eiac,
    [EIAM] = igb_set_eiam,
    [EIMC] = igb_set_eimc,
    [EIMS] = igb_set_eims,
    [IVAR ... IVAR + 7] = e1000e_mac_writereg,
    e1000e_putreg(IVAR_MISC),
    [PFMAILBOX ... PFMAILBOX + 7] = igb_set_pfmailbox,
    [VFMAILBOX ... VFMAILBOX + 7] = igb_set_vfmailbox,
    [MBVFICR] = igb_set_mbvficr,
    [VMBMEM ... VMBMEM + 127] = e1000e_mac_writereg,
    e1000e_putreg(MBVFIMR),
    [VFLRE] = igb_set_vflre,
    e1000e_putreg(VFRE),
    e1000e_putreg(VFTE),
    e1000e_putreg(QDE),
    e1000e_putreg(DTXSWC),
    e1000e_putreg(RPLOLR),
    [VLVF ... VLVF + 31] = e1000e_mac_writereg,
    [VMVIR ... VMVIR + 7] = e1000e_mac_writereg,
    [VMOLR ... VMOLR + 7] = e1000e_mac_writereg,
    [UTA ... UTA + 127] = e1000e_mac_writereg,
    [VTCTRL0] = igb_set_vtctrl,
    [VTCTRL1] = igb_set_vtctrl,
    [VTCTRL2] = igb_set_vtctrl,
    [VTCTRL3] = igb_set_vtctrl,
    [VTCTRL4] = igb_set_vtctrl,
    [VTCTRL5] = igb_set_vtctrl,
    [VTCTRL6] = igb_set_vtctrl,
    [VTCTRL7] = igb_set_vtctrl,
    [VTEICS0] = igb_set_vteics,
    [VTEICS1] = igb_set_vteics,
    [VTEICS2] = igb_set_vteics,
    [VTEICS3] = igb_set_vteics,
    [VTEICS4] = igb_set_vteics,
    [VTEICS5] = igb_set_vteics,
    [VTEICS6] = igb_set_vteics,
    [VTEICS7] = igb_set_vteics,
    [VTEIMS0] = igb_set_vteims,
    [VTEIMS1] = igb_set_vteims,
    [VTEIMS2] = igb_set_vteims,
    [VTEIMS3] = igb_set_vteims,
    [VTEIMS4] = igb_set_vteims,
    [VTEIMS5] = igb_set_vteims,
    [VTEIMS6] = igb_set_vteims,
    [VTEIMS7] = igb_set_vteims,
    [VTEIMC0] = igb_set_vteimc,
    [VTEIMC1] = igb_set_vteimc,
    [VTEIMC2] = igb_set_vteimc,
    [VTEIMC3] = igb_set_vteimc,
    [VTEIMC4] = igb_set_vteimc,
    [VTEIMC5] = igb_set_vteimc,
    [VTEIMC6] = igb_set_vteimc,
    [VTEIMC7] = igb_set_vteimc,
    [VTEIAC0] = igb_set_vteiac,
    [VTEIAC1] = igb_set_vteiac,
    [VTEIAC2] = igb_set_vteiac,
    [VTEIAC3] = igb_set_vteiac,
    [VTEIAC4] = igb_set_vteiac,
    [VTEIAC5] = igb_set_vteiac,
    [VTEIAC6] = igb_set_vteiac,
    [VTEIAC7] = igb_set_vteiac,
    [VTEIAM0] = igb_set_vteiam,
    [VTEIAM1] = igb_set_vteiam,
    [VTEIAM2] = igb_set_vteiam,
    [VTEIAM3] = igb_set_vteiam,
    [VTEIAM4] = igb_set_vteiam,
    [VTEIAM5] = igb_set_vteiam,
    [VTEIAM6] = igb_set_vteiam,
    [VTEIAM7] = igb_set_vteiam,
    [VTEICR0] = igb_set_vteicr,
    [VTEICR1] = igb_set_vteicr,
    [VTEICR2] = igb_set_vteicr,
    [VTEICR3] = igb_set_vteicr,
    [VTEICR4] = igb_set_vteicr,
    [VTEICR5] = igb_set_vteicr,
    [VTEICR6] = igb_set_vteicr,
    [VTEICR7] = igb_set_vteicr,
    [VTIVAR ... VTIVAR + 7] = igb_set_vtivar,
    [VTIVAR_MISC ... VTIVAR_MISC + 7] = e1000e_mac_writereg
};
enum { E1000E_NWRITEOPS = ARRAY_SIZE(e1000e_macreg_writeops) };

enum { MAC_ACCESS_PARTIAL = 1 };

/* The array below combines alias offsets of the index values for the
 * MAC registers that have aliases, with the indication of not fully
 * implemented registers (lowest bit). This combination is possible
 * because all of the offsets are even. */
static const uint16_t mac_reg_access[E1000E_MAC_SIZE] = {
    /* Alias index offsets */
    [FCRTL_A] = 0x07fe, [FCRTH_A] = 0x0802,
    //[RDH0_A]  = 0x09bc, [RDT0_A]  = 0x09bc, [RDTR_A] = 0x09c6,
    [RDFH_A]  = 0xe904, [RDFT_A]  = 0xe904,
    //[TDH_A]   = 0x0cf8, [TDT_A]   = 0x0cf8,
    [TIDV_A] = 0x0cf8,
    [TDFH_A]  = 0xed00, [TDFT_A]  = 0xed00,
    [RA_ALT ... RA_ALT + 31]      = 0x14f0,
    [VFTA_A ... VFTA_A + 127] = 0x1400,
    //[RDBAH0_A ... RDLEN0_A] = 0x09bc,
    //[TDBAL_A ... TDLEN_A]   = 0x0cf8,

    //[CTRL_ALT] = -0x0001,
    //[ICR_ALT] = 0x0510,
    //[ICS_ALT] = 0x050F,
    //[IMS_ALT] = 0x050E,
    //[IMC_ALT] = 0x050D,
    //[IAM_ALT] = 0x050C,
    //[FCRTL_ALT] = 0x07FE,
    [RDBAL0_ALT] = 0x2600,
    [RDBAH0_ALT] = 0x2600,
    [RDLEN0_ALT] = 0x2600,
    [SRRCTL0_ALT] = 0x2600,
    [RDH0_ALT] = 0x2600,
    [RDT0_ALT] = 0x2600,
    [RXDCTL0_ALT] = 0x2600,
    [RXCTL0_ALT] = 0x2600,
    [RQDPC0_ALT] = 0x2600,
    [RDBAL1_ALT] = 0x25D0,
    [RDBAL2_ALT] = 0x25A0,
    [RDBAL3_ALT] = 0x2570,
    [RDBAH1_ALT] = 0x25D0,
    [RDBAH2_ALT] = 0x25A0,
    [RDBAH3_ALT] = 0x2570,
    [RDLEN1_ALT] = 0x25D0,
    [RDLEN2_ALT] = 0x25A0,
    [RDLEN3_ALT] = 0x2570,
    [SRRCTL1_ALT] = 0x25D0,
    [SRRCTL2_ALT] = 0x25A0,
    [SRRCTL3_ALT] = 0x2570,
    [RDH1_ALT] = 0x25D0,
    [RDH2_ALT] = 0x25A0,
    [RDH3_ALT] = 0x2570,
    [RDT1_ALT] = 0x25D0,
    [RDT2_ALT] = 0x25A0,
    [RDT3_ALT] = 0x2570,
    [RXDCTL1_ALT] = 0x25D0,
    [RXDCTL2_ALT] = 0x25A0,
    [RXDCTL3_ALT] = 0x2570,
    [RXCTL1_ALT] = 0x25D0,
    [RXCTL2_ALT] = 0x25A0,
    [RXCTL3_ALT] = 0x2570,
    [RQDPC1_ALT] = 0x25D0,
    [RQDPC2_ALT] = 0x25A0,
    [RQDPC3_ALT] = 0x2570,
    //[MTA_ALT] = 0x1400,
    //[VFTA_ALT] = 0x1400,
    [TDBAL0_ALT] = 0x2A00,
    [TDBAH0_ALT] = 0x2A00,
    [TDLEN0_ALT] = 0x2A00,
    [TDH0_ALT] = 0x2A00,
    [TDT0_ALT] = 0x2A00,
    [TXDCTL0_ALT] = 0x2A00,
    [TXCTL0_ALT] = 0x2A00,
    [TDWBAL0_ALT] = 0x2A00,
    [TDWBAH0_ALT] = 0x2A00,
    [TDBAL1_ALT] = 0x29D0,
    [TDBAL2_ALT] = 0x29A0,
    [TDBAL3_ALT] = 0x2970,
    [TDBAH1_ALT] = 0x29D0,
    [TDBAH2_ALT] = 0x29A0,
    [TDBAH3_ALT] = 0x2970,
    [TDLEN1_ALT] = 0x29D0,
    [TDLEN2_ALT] = 0x29A0,
    [TDLEN3_ALT] = 0x2970,
    [TDH1_ALT] = 0x29D0,
    [TDH2_ALT] = 0x29A0,
    [TDH3_ALT] = 0x2970,
    [TDT1_ALT] = 0x29D0,
    [TDT2_ALT] = 0x29A0,
    [TDT3_ALT] = 0x2970,
    [TXDCTL1_ALT] = 0x29D0,
    [TXDCTL2_ALT] = 0x29A0,
    [TXDCTL3_ALT] = 0x2970,
    [TXCTL1_ALT] = 0x29D0,
    [TXCTL2_ALT] = 0x29A0,
    [TXCTL3_ALT] = 0x29D0,
    [TDWBAL1_ALT] = 0x29D0,
    [TDWBAL2_ALT] = 0x29A0,
    [TDWBAL3_ALT] = 0x2970,
    [TDWBAH1_ALT] = 0x29D0,
    [TDWBAH2_ALT] = 0x29A0,
    [TDWBAH3_ALT] = 0x2970,

    /* Access options */
    [RDFH]  = MAC_ACCESS_PARTIAL,    [RDFT]  = MAC_ACCESS_PARTIAL,
    [RDFHS] = MAC_ACCESS_PARTIAL,    [RDFTS] = MAC_ACCESS_PARTIAL,
    [RDFPC] = MAC_ACCESS_PARTIAL,
    [TDFH]  = MAC_ACCESS_PARTIAL,    [TDFT]  = MAC_ACCESS_PARTIAL,
    [TDFHS] = MAC_ACCESS_PARTIAL,    [TDFTS] = MAC_ACCESS_PARTIAL,
    [TDFPC] = MAC_ACCESS_PARTIAL,    [EECD]  = MAC_ACCESS_PARTIAL,
    [PBM]   = MAC_ACCESS_PARTIAL,    [FLA]   = MAC_ACCESS_PARTIAL,
    [FCAL]  = MAC_ACCESS_PARTIAL,    [FCAH]  = MAC_ACCESS_PARTIAL,
    [FCT]   = MAC_ACCESS_PARTIAL,    [FCTTV] = MAC_ACCESS_PARTIAL,
    [FCRTV] = MAC_ACCESS_PARTIAL,    [FCRTL] = MAC_ACCESS_PARTIAL,
    [FCRTH] = MAC_ACCESS_PARTIAL,    [TXDCTL] = MAC_ACCESS_PARTIAL,
    [TXDCTL1] = MAC_ACCESS_PARTIAL,
    [MAVTV0 ... MAVTV3] = MAC_ACCESS_PARTIAL
};

void igb_core_write(E1000ECore *core, hwaddr addr, uint64_t val, unsigned size)
{
    uint16_t index = e1000e_get_reg_index_with_offset(mac_reg_access, addr);

    if (index < E1000E_NWRITEOPS && e1000e_macreg_writeops[index]) {
        if (mac_reg_access[index] & MAC_ACCESS_PARTIAL) {
            trace_e1000e_wrn_regs_write_trivial(index << 2);
        }
        trace_e1000e_core_write(index << 2, size, val);
        e1000e_macreg_writeops[index](core, index, val);
    } else if (index < E1000E_NREADOPS && e1000e_macreg_readops[index]) {
        trace_e1000e_wrn_regs_write_ro(index << 2, size, val);
    } else {
        trace_e1000e_wrn_regs_write_unknown(index << 2, size, val);
    }
}

uint64_t igb_core_read(E1000ECore *core, hwaddr addr, unsigned size)
{
    uint64_t val;
    uint16_t index = e1000e_get_reg_index_with_offset(mac_reg_access, addr);

    if (index < E1000E_NREADOPS && e1000e_macreg_readops[index]) {
        if (mac_reg_access[index] & MAC_ACCESS_PARTIAL) {
            trace_e1000e_wrn_regs_read_trivial(index << 2);
        }
        val = e1000e_macreg_readops[index](core, index);
        trace_e1000e_core_read(index << 2, size, val);
        return val;
    } else {
        trace_e1000e_wrn_regs_read_unknown(index << 2, size);
    }
    return 0;
}

static inline void
e1000e_autoneg_pause(E1000ECore *core)
{
    timer_del(core->autoneg_timer);
}

static void
e1000e_autoneg_resume(E1000ECore *core)
{
    if (e1000e_have_autoneg(core) &&
        !(core->phy[0][PHY_STATUS] & MII_SR_AUTONEG_COMPLETE)) {
        qemu_get_queue(core->owner_nic)->link_down = false;
        timer_mod(core->autoneg_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 500);
    }
}

static void
e1000e_vm_state_change(void *opaque, int running, RunState state)
{
    E1000ECore *core = opaque;

    if (running) {
        trace_e1000e_vm_state_running();
        e1000e_intrmgr_resume(core);
        e1000e_autoneg_resume(core);
    } else {
        trace_e1000e_vm_state_stopped();
        e1000e_autoneg_pause(core);
        e1000e_intrmgr_pause(core);
    }
}

void igb_core_pci_realize(E1000ECore     *core,
                          const uint16_t *eeprom_templ,
                          uint32_t        eeprom_size,
                          const uint8_t  *macaddr)
{
    int i;

    core->autoneg_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                       e1000e_autoneg_timer, core);
    e1000e_intrmgr_pci_realize(core);

    core->vmstate =
        qemu_add_vm_change_state_handler(e1000e_vm_state_change, core);

    for (i = 0; i < E1000E_NUM_QUEUES; i++) {
        net_tx_pkt_init(&core->tx[i].tx_pkt, core->owner,
                        E1000E_MAX_TX_FRAGS, false);
    }

    net_rx_pkt_init(&core->rx_pkt, false);

    e1000x_core_prepare_eeprom(core->eeprom,
                               eeprom_templ,
                               eeprom_size,
                               PCI_DEVICE_GET_CLASS(core->owner)->device_id,
                               macaddr);
    e1000e_update_rx_offloads(core);
}

void igb_core_pci_uninit(E1000ECore *core)
{
    int i;

    timer_del(core->autoneg_timer);
    timer_free(core->autoneg_timer);

    e1000e_intrmgr_pci_unint(core);

    qemu_del_vm_change_state_handler(core->vmstate);

    for (i = 0; i < E1000E_NUM_QUEUES; i++) {
        net_tx_pkt_reset(core->tx[i].tx_pkt);
        net_tx_pkt_uninit(core->tx[i].tx_pkt);
    }

    net_rx_pkt_uninit(core->rx_pkt);
}

static const uint16_t
e1000e_phy_reg_init[E1000E_PHY_PAGES][E1000E_PHY_PAGE_SIZE] = {
    [0] = {
        [PHY_CTRL] =   MII_CR_SPEED_SELECT_MSB  |
                       MII_CR_FULL_DUPLEX       |
                       MII_CR_AUTO_NEG_EN,

        [PHY_STATUS] = MII_SR_EXTENDED_CAPS     |
                       MII_SR_LINK_STATUS       |
                       MII_SR_AUTONEG_CAPS      |
                       MII_SR_PREAMBLE_SUPPRESS |
                       MII_SR_EXTENDED_STATUS   |
                       MII_SR_10T_HD_CAPS       |
                       MII_SR_10T_FD_CAPS       |
                       MII_SR_100X_HD_CAPS      |
                       MII_SR_100X_FD_CAPS,

        [PHY_ID1]               = 0x2a8,
        [PHY_ID2]               = 0x391,
        [PHY_AUTONEG_ADV]       = 0xde1,
        [PHY_LP_ABILITY]        = 0x7e0,
        [PHY_AUTONEG_EXP]       = BIT(2),
        [PHY_NEXT_PAGE_TX]      = BIT(0) | BIT(13),
        [PHY_1000T_CTRL]        = BIT(8) | BIT(9) | BIT(10) | BIT(11),
        [PHY_1000T_STATUS]      = 0x3c00,
        [PHY_EXT_STATUS]        = BIT(12) | BIT(13),

        [PHY_COPPER_CTRL1]      = BIT(5) | BIT(6) | BIT(8) | BIT(9) |
                                  BIT(12) | BIT(13),
        [PHY_COPPER_STAT1]      = BIT(3) | BIT(10) | BIT(11) | BIT(13) | BIT(15)
    },
    [2] = {
        [PHY_MAC_CTRL1]         = BIT(3) | BIT(7),
        [PHY_MAC_CTRL2]         = BIT(1) | BIT(2) | BIT(6) | BIT(12)
    },
    [3] = {
        [PHY_LED_TIMER_CTRL]    = BIT(0) | BIT(2) | BIT(14)
    }
};

static const uint32_t e1000e_mac_reg_init[] = {
    [PBA]           = 0x00140014,
    [LEDCTL]        = BIT(1) | BIT(8) | BIT(9) | BIT(15) | BIT(17) | BIT(18),
    [EXTCNF_CTRL]   = BIT(3),
    [EEMNGCTL]      = E1000_EEPROM_CFG_DONE | E1000_EEPROM_CFG_DONE_PORT_1 |
                      BIT(31),
    [FLASHT]        = 0x2,
    [FLSWCTL]       = BIT(30) | BIT(31),
    [FLOL]          = BIT(0),
    [RXDCTL0]       = BIT(25) | BIT(16),
    [RXDCTL1]       = BIT(25) | BIT(16),
    [RXDCTL2]       = BIT(25) | BIT(16),
    [RXDCTL3]       = BIT(25) | BIT(16),
    [RXDCTL4]       = BIT(25) | BIT(16),
    [RXDCTL5]       = BIT(25) | BIT(16),
    [RXDCTL6]       = BIT(25) | BIT(16),
    [RXDCTL7]       = BIT(25) | BIT(16),
    [RXDCTL8]       = BIT(25) | BIT(16),
    [RXDCTL9]       = BIT(25) | BIT(16),
    [RXDCTL10]      = BIT(25) | BIT(16),
    [RXDCTL11]      = BIT(25) | BIT(16),
    [RXDCTL12]      = BIT(25) | BIT(16),
    [RXDCTL13]      = BIT(25) | BIT(16),
    [RXDCTL14]      = BIT(25) | BIT(16),
    [RXDCTL15]      = BIT(25) | BIT(16),
    [TIPG]          = 0x8 | (0x4 << 10) | (0x6 << 20),
    [RXCFGL]        = 0x88F7,
    [RXUDP]         = 0x319,
    [CTRL]          = E1000_CTRL_FD | E1000_CTRL_LRST | E1000_CTRL_SPD_1000 |
                      E1000_CTRL_ADVD3WUC,
    [STATUS]        = E1000_STATUS_ASDV_1000 | E1000_STATUS_LU,
    [PSRCTL]        = (2 << E1000_PSRCTL_BSIZE0_SHIFT) |
                      (4 << E1000_PSRCTL_BSIZE1_SHIFT) |
                      (4 << E1000_PSRCTL_BSIZE2_SHIFT),
    [TARC0]         = 0x3 | E1000_TARC_ENABLE,
    [TARC1]         = 0x3 | E1000_TARC_ENABLE,
    [EECD]          = E1000_EECD_AUTO_RD | E1000_EECD_PRES,
    [EERD]          = E1000_EERW_DONE,
    [EEWR]          = E1000_EERW_DONE,
    [GCR]           = E1000_L0S_ADJUST |
                      E1000_L1_ENTRY_LATENCY_MSB |
                      E1000_L1_ENTRY_LATENCY_LSB,
    [TDFH]          = 0x600,
    [TDFT]          = 0x600,
    [TDFHS]         = 0x600,
    [TDFTS]         = 0x600,
    [POEMB]         = 0x30D,
    [PBS]           = 0x028,
    [MANC]          = E1000_MANC_DIS_IP_CHK_ARP,
    [FACTPS]        = E1000_FACTPS_LAN0_ON | 0x20000000,
    [SWSM]          = 0,
    [RXCSUM]        = E1000_RXCSUM_IPOFLD | E1000_RXCSUM_TUOFLD,
    [TXPBS]         = 0x28,
    [RXPBS]         = 0x40,
    [TCTL]          = (0x1 << 3) | (0xF << 4) | (0x40 << 12) | (0x1 << 26) | (0xA << 28),
    [TCTL_EXT]      = 0x40 | (0x42 << 10),
    [DTXCTL]        = (0x1 << 2) | (0x1 << 6),

    [VFMAILBOX ... VFMAILBOX + 7] = BIT(6),
    [MBVFIMR]       = 0xFF,
    [VFRE]          = 0xFF,
    [VFTE]          = 0xFF,
    [VMOLR ... VMOLR + 7] = 0x80002600,
    [RPLOLR]        = 0x80000000,
    [RLPML]         = 0x2600,
    [TXCTL0]       = BIT(13) | BIT(9),
    [TXCTL1]       = BIT(13) | BIT(9),
    [TXCTL2]       = BIT(13) | BIT(9),
    [TXCTL3]       = BIT(13) | BIT(9),
    [TXCTL4]       = BIT(13) | BIT(9),
    [TXCTL5]       = BIT(13) | BIT(9),
    [TXCTL6]       = BIT(13) | BIT(9),
    [TXCTL7]       = BIT(13) | BIT(9),
    [TXCTL8]       = BIT(13) | BIT(9),
    [TXCTL9]       = BIT(13) | BIT(9),
    [TXCTL10]      = BIT(13) | BIT(9),
    [TXCTL11]      = BIT(13) | BIT(9),
    [TXCTL12]      = BIT(13) | BIT(9),
    [TXCTL13]      = BIT(13) | BIT(9),
    [TXCTL14]      = BIT(13) | BIT(9),
    [TXCTL15]      = BIT(13) | BIT(9),
};

void igb_core_reset(E1000ECore *core)
{
    struct e1000e_tx *tx;
    int i;

    timer_del(core->autoneg_timer);

    e1000e_intrmgr_reset(core);

    memset(core->phy, 0, sizeof core->phy);
    memmove(core->phy, e1000e_phy_reg_init, sizeof e1000e_phy_reg_init);
    memset(core->mac, 0, sizeof core->mac);
    memmove(core->mac, e1000e_mac_reg_init, sizeof e1000e_mac_reg_init);

    core->rxbuf_min_shift = 1 + E1000_RING_DESC_LEN_SHIFT;

    if (qemu_get_queue(core->owner_nic)->link_down) {
        e1000e_link_down(core);
    }

    e1000x_reset_mac_addr(core->owner_nic, core->mac, core->permanent_mac);

    for (i = 0; i < ARRAY_SIZE(core->tx); i++) {
        tx = &core->tx[i];
        net_tx_pkt_reset(tx->tx_pkt);
        tx->vlan = 0;
        tx->mss = 0;
        tx->tse = false;
        tx->ixsm = false;
        tx->txsm = false;
        tx->skip_cp = false;
    }
}

void igb_core_pre_save(E1000ECore *core)
{
    int i;
    NetClientState *nc = qemu_get_queue(core->owner_nic);

    /*
    * If link is down and auto-negotiation is supported and ongoing,
    * complete auto-negotiation immediately. This allows us to look
    * at MII_SR_AUTONEG_COMPLETE to infer link status on load.
    */
    if (nc->link_down && e1000e_have_autoneg(core)) {
        core->phy[0][PHY_STATUS] |= MII_SR_AUTONEG_COMPLETE;
        e1000e_update_flowctl_status(core);
    }

    for (i = 0; i < ARRAY_SIZE(core->tx); i++) {
        if (net_tx_pkt_has_fragments(core->tx[i].tx_pkt)) {
            core->tx[i].skip_cp = true;
        }
    }
}

int igb_core_post_load(E1000ECore *core)
{
    NetClientState *nc = qemu_get_queue(core->owner_nic);

    /* nc.link_down can't be migrated, so infer link_down according
     * to link status bit in core.mac[STATUS].
     */
    nc->link_down = (core->mac[STATUS] & E1000_STATUS_LU) == 0;

    return 0;
}
