/*
* QEMU Intel 82576 SR/IOV Ethernet Controller Emulation
*
* Copyright (c) 2020-2021 Red Hat, Inc.
*
* Intel Datasheet: http://www.intel.com/
* content/dam/www/public/us/en/documents/datasheets/82576eg-gbe-datasheet.pdf
*
* Written by:
* Gal Hammmer <ghammer@redhat.com>
* Marcel Apfelbaum <marcel@redhat.com>
*
* Based on work written by Knut Omang.
* Based on e1000e implementation.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "qemu/osdep.h"
#include "qemu/range.h"
#include "sysemu/sysemu.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "igb_regs.h"
#include "e1000x_common.h"
#include "igb_enums.h"
#include "igb_core.h"
#include "igb_common.h"
#include "trace.h"
#include "qapi/error.h"

#define TYPE_IGB "e1000e"
#define IGB(obj) OBJECT_CHECK(IgbState, (obj), TYPE_IGB)

#define IGB_TOTAL_VFS       (8)
#define IGB_MSIX_VECTORS    (10)

#define IGB_CAP_SRIOV_OFFSET    (0x160)
#define IGB_VF_OFFSET           (0x80)
#define IGB_VF_STRIDE           (2)

#define IGB_MMIO_BAR_IDX    (0)
#define IGB_FLASH_BAR_IDX   (1)
#define IGB_IO_BAR_IDX      (2)
#define IGB_MSIX_BAR_IDX    (3)

#define IGB_MMIO_SIZE   (128 * 1024)
#define IGB_FLASH_SIZE  (128 * 1024)
#define IGB_IO_SIZE     (32)
#define IGB_MSIX_SIZE   (16 * 1024)

typedef struct IgbState {
    PCIDevice parent_obj;
    NICState *nic;
    NICConf conf;

    MemoryRegion mmio;
    MemoryRegion flash;
    MemoryRegion io;
    MemoryRegion msix;

    uint32_t ioaddr;

    E1000ECore core;
} IgbState;

static void igb_write_config(PCIDevice *dev, uint32_t addr,
    uint32_t val, int len)
{
    IgbState *s = IGB(dev);

    trace_igb_write_config(addr, val, len);
    pci_default_write_config(dev, addr, val, len);

    if (range_covers_byte(addr, len, PCI_COMMAND) &&
        (dev->config[PCI_COMMAND] & PCI_COMMAND_MASTER)) {
        igb_start_recv(&s->core);
    }
}

uint64_t igb_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbState *s = IGB(opaque);
    return igb_core_read(&s->core, addr, size);
}

void igb_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IgbState *s = IGB(opaque);
    igb_core_write(&s->core, addr, val, size);
}

static const MemoryRegionOps mmio_ops = {
    .read = igb_mmio_read,
    .write = igb_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static bool igb_io_get_reg_index(IgbState *s, uint32_t *idx)
{
    if (s->ioaddr < 0x1FFFF) {
        *idx = s->ioaddr;
        return true;
    }

    if (s->ioaddr < 0x7FFFF) {
        trace_igb_wrn_io_addr_undefined(s->ioaddr);
        return false;
    }

    if (s->ioaddr < 0xFFFFF) {
        trace_igb_wrn_io_addr_flash(s->ioaddr);
        return false;
    }

    trace_igb_wrn_io_addr_unknown(s->ioaddr);
    return false;
}

static uint64_t igb_io_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbState *s = IGB(opaque);
    uint32_t idx = 0;
    uint64_t val;

    switch (addr) {
    case E1000_IOADDR:
        trace_igb_io_read_addr(s->ioaddr);
        return s->ioaddr;
    case E1000_IODATA:
        if (igb_io_get_reg_index(s, &idx)) {
            val = igb_core_read(&s->core, idx, sizeof(val));
            trace_igb_io_read_data(idx, val);
            return val;
        }
        return 0;
    default:
        trace_igb_wrn_io_read_unknown(addr);
        return 0;
    }
}

static void igb_io_write(void *opaque, hwaddr addr, uint64_t val,
    unsigned size)
{
    IgbState *s = IGB(opaque);
    uint32_t idx = 0;

    switch (addr) {
    case E1000_IOADDR:
        trace_igb_io_write_addr(val);
        s->ioaddr = (uint32_t) val;
        return;
    case E1000_IODATA:
        if (igb_io_get_reg_index(s, &idx)) {
            trace_igb_io_write_data(idx, val);
            igb_core_write(&s->core, idx, val, sizeof(val));
        }
        return;
    default:
        trace_igb_wrn_io_write_unknown(addr);
        return;
    }
}

static const MemoryRegionOps io_ops = {
    .read = igb_io_read,
    .write = igb_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static bool igb_nc_can_receive(NetClientState *nc)
{
    IgbState *s = qemu_get_nic_opaque(nc);
    return igb_can_receive(&s->core);
}

static ssize_t igb_nc_receive_iov(NetClientState *nc, const struct iovec *iov,
    int iovcnt)
{
    IgbState *s = qemu_get_nic_opaque(nc);
    return igb_receive_iov(&s->core, iov, iovcnt);
}

static ssize_t igb_nc_receive(NetClientState *nc, const uint8_t *buf,
    size_t size)
{
    const struct iovec iov = {
        .iov_base = (uint8_t *)buf,
        .iov_len = size
    };

    return igb_nc_receive_iov(nc, &iov, 1);
}

static void igb_set_link_status(NetClientState *nc)
{
    IgbState *s = qemu_get_nic_opaque(nc);
    igb_core_set_link_status(&s->core);
}

static NetClientInfo net_igb_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = igb_nc_can_receive,
    .receive = igb_nc_receive,
    .receive_iov = igb_nc_receive_iov,
    .link_status_changed = igb_set_link_status,
};

static int igb_add_pm_capability(PCIDevice *dev, uint8_t offset, uint16_t pmc)
{
    Error *err = NULL;
    int ret;

    ret = pci_add_capability(dev, PCI_CAP_ID_PM, offset, PCI_PM_SIZEOF, &err);
    if (err) {
        error_report_err(err);
        return ret;
    }

    pci_set_word(dev->config + offset + PCI_PM_PMC, PCI_PM_CAP_VER_1_1 | pmc);

    pci_set_word(dev->wmask + offset + PCI_PM_CTRL, PCI_PM_CTRL_STATE_MASK |
        PCI_PM_CTRL_PME_ENABLE | PCI_PM_CTRL_DATA_SEL_MASK);

    pci_set_word(dev->w1cmask + offset + PCI_PM_CTRL, PCI_PM_CTRL_PME_STATUS);

    return ret;
}

static void igb_init_net_peer(IgbState *s, PCIDevice *dev, uint8_t *macaddr)
{
    DeviceState *ds = DEVICE(dev);

    s->nic = qemu_new_nic(&net_igb_info, &s->conf,
        object_get_typename(OBJECT(s)), ds->id, s);

    s->core.max_queue_num = s->conf.peers.queues - 1;

    trace_igb_mac_set_permanent(MAC_ARG(macaddr));
    memcpy(s->core.permanent_mac, macaddr, sizeof(s->core.permanent_mac));

    qemu_format_nic_info_str(qemu_get_queue(s->nic), macaddr);
}

/* EEPROM (NVM) contents documented in section 6.1, table 6-1:
 * and in 6.10 Software accessed words.
 *
 * TBD: Need to walk through this, names in comments are ok up to 0x4F
 */
static const uint16_t igb_eeprom_template[80] = {
  /*        Address        |    Compat.    | ImRev |Compat.|OEM sp.*/
    0x0000, 0x0000, 0x0000, 0x0d14, 0xffff, 0x2010, 0xffff, 0xffff,
  /*      PBA      |ICtrl1 | SSID  | SVID  | DevID |-------|ICtrl2 */
    0x1040, 0xffff, 0x046b, 0x484c, 0x108e, 0x10c9, 0x0000, 0xf14b,
  /* SwPin0| DevID | EESZ  |-------|ICtrl3 |PCI-tc | MSIX  | APtr  */
    0xe30c, 0x10c9, 0x6000, 0x0000, 0x8c01, 0x0014, 0x4a40, 0x0060,
  /* PCIe Init. Conf 1,2,3 |PCICtrl| LD1,3 |DDevID |DevRev | LD0,2 */
    0x6cf6, 0xd7b0, 0x0a7e, 0x8403, 0x4784, 0x10a6, 0x0001, 0x4602,
  /* SwPin1| FunC  |LAN-PWR|ManHwC |ICtrl3 | IOVct |VDevID |-------*/
    0xe30c, 0x2020, 0x1ae5, 0x004a, 0x8401, 0x00f7, 0x10ca, 0x0000,
  /*---------------| LD1,3 | LD0,2 | ROEnd | ROSta | Wdog  | VPD   */
    0x0000, 0x0000, 0x4784, 0x4602, 0x0000, 0x0000, 0x0000, 0xffff,
  /* PCSet0| Ccfg0 |PXEver |IBAcap |PCSet1 | Ccfg1 |iSCVer | ??    */
    0x0100, 0x4000, 0x131f, 0x4013, 0x0100, 0x4000, 0xffff, 0xffff,
  /* PCSet2| Ccfg2 |PCSet3 | Ccfg3 | ??    |AltMacP| ??    |CHKSUM */
    0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x00e0, 0xffff, 0xb73b,
  /* ArbEn |-------| ImuID | ImuID |-------------------------------*/
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  /*----------------------- Reserved ------------------------------*/
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  /* Word 0x50 - 0x5XX (sec.6.5) */
};

static void pci_igb_realize(PCIDevice *dev, Error **err)
{
    IgbState *s = IGB(dev);
    uint8_t *macaddr;
    int ret;
    int i;

    trace_igb_cb_pci_realize();

    dev->config_write = igb_write_config;

    dev->config[PCI_CACHE_LINE_SIZE] = 0x10;
    dev->config[PCI_INTERRUPT_PIN] = 1;

    /* BAR0: MMIO */
    memory_region_init_io(&s->mmio, OBJECT(dev), &mmio_ops, s, "igb-mmio",
        IGB_MMIO_SIZE);
    pci_register_bar(dev, IGB_MMIO_BAR_IDX, PCI_BASE_ADDRESS_SPACE_MEMORY,
        &s->mmio);

    /* BAR1: flash memory (dummy) */
    memory_region_init(&s->flash, OBJECT(dev), "igb-flash", IGB_FLASH_SIZE);
    pci_register_bar(dev, IGB_FLASH_BAR_IDX, PCI_BASE_ADDRESS_SPACE_MEMORY,
        &s->flash);

    /* BAR2: I/O ports */
    memory_region_init_io(&s->io, OBJECT(dev), &io_ops, s, "igb-io",
        IGB_IO_SIZE);
    pci_register_bar(dev, IGB_IO_BAR_IDX, PCI_BASE_ADDRESS_SPACE_IO, &s->io);

    /* BAR3: MSIX table */
    memory_region_init(&s->msix, OBJECT(dev), "igb-msix", IGB_MSIX_SIZE);
    pci_register_bar(dev, IGB_MSIX_BAR_IDX, PCI_BASE_ADDRESS_MEM_TYPE_64,
        &s->msix);

    /* Add PCI capabilities in reverse order */
    ret = pcie_endpoint_cap_init(dev, 0xa0);
    if (ret < 0) {
        goto err_pcie_cap;
    }

    ret = msix_init(dev, IGB_MSIX_VECTORS,  &s->msix, IGB_MSIX_BAR_IDX, 0,
        &s->msix, IGB_MSIX_BAR_IDX, 0x2000, 0x70, err);
    if (ret) {
        goto err_msix;
    }

    ret = msi_init(dev, 0x50, 1, true, true, err);
    if (ret < 0) {
        goto err_msi;
    }

    for (i = 0; i < IGB_MSIX_VECTORS; i++) {
        ret = msix_vector_use(dev, i);
        if (ret) {
            goto err_pcie_cap;
        }
    }

    if (igb_add_pm_capability(dev, 0x40, PCI_PM_CAP_DSI) < 0) {
        hw_error("Failed to initialize PM capability");
    }

    /* PCIe extended capabilities (in order) */
    ret = pcie_aer_init(dev, 1, 0x100, 0x40, err);
    if (ret < 0) {
        goto err_aer;
    }

    pcie_ari_init(dev, 0x150, 1);

    pcie_sriov_pf_init(dev, IGB_CAP_SRIOV_OFFSET, "igbvf",
        IGB_82576_VF_DEV_ID, IGB_TOTAL_VFS, IGB_TOTAL_VFS, IGB_VF_OFFSET,
        IGB_VF_STRIDE);

    pcie_sriov_pf_init_vf_bar(dev, 0,
        PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
        16 * 1024);
    pcie_sriov_pf_init_vf_bar(dev, 3,
        PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
        16 * 1024);

    /* Create networking backend */
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    macaddr = s->conf.macaddr.a;

    igb_init_net_peer(s, dev, macaddr);

    /* Initialize core */
    s->core.owner = &s->parent_obj;
    s->core.owner_nic = s->nic;

    igb_core_pci_realize(&s->core, igb_eeprom_template,
        sizeof(igb_eeprom_template), macaddr);

    return;

err_aer:
    msi_uninit(dev);
err_msi:
    msix_unuse_all_vectors(dev);
    msix_uninit(dev, &s->msix, &s->msix);
err_msix:
    pcie_cap_exit(dev);
err_pcie_cap:
    return;
}

static void pci_igb_uninit(PCIDevice *dev)
{
    IgbState *s = IGB(dev);

    trace_igb_cb_pci_uninit();

    pcie_sriov_pf_exit(dev);
    pcie_cap_exit(dev);
    qemu_del_nic(s->nic);
    msix_unuse_all_vectors(dev);
    msix_uninit(dev, &s->msix, &s->msix);
    msi_uninit(dev);
}

static void igb_reset(DeviceState *dev)
{
    PCIDevice *d = PCI_DEVICE(dev);
    IgbState *s = IGB(dev);

    trace_igb_cb_qdev_reset();
    pcie_sriov_pf_disable_vfs(d);
    igb_core_reset(&s->core);
}

static int igb_pre_save(void *opaque)
{
    IgbState *s = IGB(opaque);

    trace_igb_cb_pre_save();

    igb_core_pre_save(&s->core);
    return 0;
}

static int igb_post_load(void *opaque, int version_id)
{
    IgbState *s = IGB(opaque);

    trace_igb_cb_post_load();
    return igb_core_post_load(&s->core);
}

static const VMStateDescription igb_vmstate_tx = {
    .name = "igb-tx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(vlan, struct e1000e_tx),
        VMSTATE_UINT16(mss, struct e1000e_tx),
        VMSTATE_BOOL(tse, struct e1000e_tx),
        VMSTATE_BOOL(ixsm, struct e1000e_tx),
        VMSTATE_BOOL(txsm, struct e1000e_tx),
        VMSTATE_BOOL(first, struct e1000e_tx),
        VMSTATE_BOOL(skip_cp, struct e1000e_tx),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription igb_vmstate_intr_timer = {
    .name = "e1000e-intr-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, E1000IntrDelayTimer),
        VMSTATE_BOOL(running, E1000IntrDelayTimer),
        VMSTATE_END_OF_LIST()
    }
};

#define VMSTATE_E1000E_INTR_DELAY_TIMER(_f, _s)                     \
    VMSTATE_STRUCT(_f, _s, 0,                                       \
                   igb_vmstate_intr_timer, E1000IntrDelayTimer)

#define VMSTATE_E1000E_INTR_DELAY_TIMER_ARRAY(_f, _s, _num)         \
    VMSTATE_STRUCT_ARRAY(_f, _s, _num, 0,                           \
                         igb_vmstate_intr_timer, E1000IntrDelayTimer)

static const VMStateDescription igb_vmstate = {
    .name = "e1000e",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = igb_pre_save,
    .post_load = igb_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, IgbState),
        VMSTATE_MSIX(parent_obj, IgbState),

        VMSTATE_UINT32(ioaddr, IgbState),
        VMSTATE_UINT32(core.rxbuf_min_shift, IgbState),
        VMSTATE_UINT8(core.rx_desc_len, IgbState),
        VMSTATE_UINT32_ARRAY(core.rxbuf_sizes, IgbState,
                             E1000_PSRCTL_BUFFS_PER_DESC),
        VMSTATE_UINT32(core.rx_desc_buf_size, IgbState),
        VMSTATE_UINT16_ARRAY(core.eeprom, IgbState, E1000E_EEPROM_SIZE),
        VMSTATE_UINT16_2DARRAY(core.phy, IgbState,
                               E1000E_PHY_PAGES, E1000E_PHY_PAGE_SIZE),
        VMSTATE_UINT32_ARRAY(core.mac, IgbState, E1000E_MAC_SIZE),
        VMSTATE_UINT8_ARRAY(core.permanent_mac, IgbState, ETH_ALEN),

        VMSTATE_UINT32(core.delayed_causes, IgbState),

        VMSTATE_E1000E_INTR_DELAY_TIMER_ARRAY(core.eitr, IgbState,
                                              IGB_MSIX_VEC_NUM),
        VMSTATE_BOOL_ARRAY(core.eitr_intr_pending, IgbState,
                           IGB_MSIX_VEC_NUM),

        VMSTATE_UINT32_ARRAY(core.eitr_guest_value, IgbState,
                             IGB_MSIX_VEC_NUM),

        VMSTATE_UINT16(core.vet, IgbState),

        VMSTATE_STRUCT_ARRAY(core.tx, IgbState, E1000E_NUM_QUEUES, 0,
                             igb_vmstate_tx, struct e1000e_tx),
        VMSTATE_END_OF_LIST()
    }
};

static Property igb_properties[] = {
    DEFINE_NIC_PROPERTIES(IgbState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void igb_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);

    c->realize = pci_igb_realize;
    c->exit = pci_igb_uninit;
    c->vendor_id = PCI_VENDOR_ID_INTEL;
    c->device_id = E1000_DEV_ID_82576;
    c->revision = 1;
    c->romfile = NULL;
    c->class_id = PCI_CLASS_NETWORK_ETHERNET;

    dc->desc = "Intel 82576 Gigabit Ethernet Controller";
    dc->reset = igb_reset;
    dc->vmsd = &igb_vmstate;

    device_class_set_props(dc, igb_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static void igb_instance_init(Object* obj)
{
    IgbState *s = IGB(obj);

    device_add_bootindex_property(obj, &s->conf.bootindex, "bootindex",
        "/ethernet-phy@0", DEVICE(obj));
}

static const TypeInfo igb_info = {
    .name = TYPE_IGB,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IgbState),
    .class_init = igb_class_init,
    .instance_init = igb_instance_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void igb_register_types(void)
{
    type_register_static(&igb_info);
}

type_init(igb_register_types)
