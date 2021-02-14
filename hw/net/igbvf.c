/*
* QEMU Intel 82576 SR/IOV Ethernet Controller Emulation
*
* Copyright (c) 2020-2021 Red Hat, Inc.
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
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/msix.h"
#include "igb_regs_new.h"
#include "igb_regs_tmp.h"
#include "igb_common.h"
#include "trace.h"
#include "qapi/error.h"

#define TYPE_IGBVF "igbvf"
#define IGBVF(obj) OBJECT_CHECK(IgbVfState, (obj), TYPE_IGBVF)

#define PCI_DEVICE_ID_INTEL_82576_VF    0x10CA

#define IGBVF_MSIX_VECTORS  (3)

#define IGBVF_MMIO_BAR_IDX  (0)
#define IGBVF_MSIX_BAR_IDX  (3)

#define IGBVF_MMIO_SIZE     (16 * 1024)
#define IGBVF_MSIX_SIZE     (16 * 1024)

typedef struct IgbVfState {
    PCIDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion msix;

} IgbVfState;

static hwaddr vf_to_pf_addr(hwaddr addr, uint16_t vfn)
{
    switch (addr)
    {
        case E1000_CTRL:
        case 0x0004: /* E1000_CTRL_ALT */
            return 0x10000 + vfn * 0x100;
        case E1000_STATUS:
            return 0x0008;
        case 0x1048: /* E1000_VTFRTIMER */
            return 0x1048;
        case E1000_EICS:
            return 0x10020 + vfn * 0x100;
        case E1000_EIMS:
            return 0x10024 + vfn * 0x100;
        case E1000_EIMC:
            return 0x10028 + vfn * 0x100;
        case E1000_EIAC:
            return 0x1002C + vfn * 0x100;
        case E1000_EIAM:
            return 0x10030 + vfn * 0x100;
        case E1000_EICR:
            return 0x10080 + vfn * 0x100;
        case 0x1680:
        case 0x1684:
        case 0x1688: /* E1000_EITR 0-2 */
            return 0x16E0 - (0x1688 - addr) - vfn * 0xC;
        case E1000_IVAR:
            return 0x11700 + vfn * 4;
        case E1000_IVAR_MISC:
            return 0x11720 + vfn * 4;
        case 0x0F04: /* E1000_PBACL */
            return 0x5B68;
        case 0x0F0C: /* E1000_PSRTYPE */
            return 0x5480 + vfn * 4;
        case E1000_VFMAILBOX:
            return 0x0C40 + vfn * 4;
        case 0x0800 ... 0x083F: /* VMBMEM */
            return addr + vfn * 0x40;
        case E1000_RDBAL0_ALT:
            return 0xC000 + vfn * 0x40;
        case E1000_RDBAH0_ALT:
            return 0xC004 + vfn * 0x40;
        case E1000_RDLEN0_ALT:
            return 0xC008 + vfn * 0x40;
        case E1000_SRRCTL0_ALT:
            return 0xC00C + vfn * 0x40;
        case E1000_RDH0_ALT:
            return 0xC010 + vfn * 0x40;
        case E1000_RXCTL0_ALT:
            return 0xC014 + vfn * 0x40;
        case E1000_RDT0_ALT:
            return 0xC018 + vfn * 0x40;
        case E1000_RXDCTL0_ALT:
            return 0xC028 + vfn * 0x40;
        case E1000_RQDPC0_ALT:
            return 0xC030 + vfn * 0x40;
        case E1000_TDBAL0_ALT:
            return 0xE000 + vfn * 0x40;
        case E1000_TDBAH0_ALT:
            return 0xE004 + vfn * 0x40;
        case E1000_TDLEN0_ALT:
            return 0xE008 + vfn * 0x40;
        case E1000_TDH0_ALT:
            return 0xE010 + vfn * 0x40;
        case E1000_TXCTL0_ALT:
            return 0xE014 + vfn * 0x40;
        case E1000_TDT0_ALT:
            return 0xE018 + vfn * 0x40;
        case E1000_TXDCTL0_ALT:
            return 0xE028 + vfn * 0x40;
        case E1000_TDWBAL0_ALT:
            return 0xE038 + vfn * 0x40;
        case E1000_TDWBAH0_ALT:
            return 0xE03C + vfn * 0x40;
        case E1000_VFGPRC:
            return 0x10010 + vfn * 0x100;
        case E1000_VFGPTC:
            return 0x10014 + vfn * 0x100;
        case E1000_VFGORC:
            return 0x10018 + vfn * 0x100;
        case E1000_VFGOTC:
            return 0x10034 + vfn * 0x100;
        case E1000_VFMPRC:
            return 0x1003C + vfn * 0x100;
        case E1000_VFGPRLBC:
            return 0x10040 + vfn * 0x100;
        case E1000_VFGPTLBC:
            return 0x10044 + vfn * 0x100;
        case E1000_VFGORLBC:
            return 0x10048 + vfn * 0x100;
        case E1000_VFGOTLBC:
            return 0x10050 + vfn * 0x100;
        case 0x34E8: /* E1000_PBTWAC */
            return 0x34E8;
        case 0x24E8: /* E1000_PBRWAC */
            return 0x24E8;
    }

    g_assert_not_reached();

    return addr;
}

static void igbvf_write_config(PCIDevice *dev, uint32_t addr, uint32_t val,
    int len)
{
    trace_igbvf_write_config(addr, val, len);
    pci_default_write_config(dev, addr, val, len);
}

static uint64_t igbvf_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIDevice *vf = PCI_DEVICE(opaque);
    PCIDevice *pf = pcie_sriov_get_pf(vf);

    addr = vf_to_pf_addr(addr, pcie_sriov_vf_number(vf));
    return igb_mmio_read(pf, addr, size);
}

static void igbvf_mmio_write(void *opaque, hwaddr addr, uint64_t val,
    unsigned size)
{
    PCIDevice *vf = PCI_DEVICE(opaque);
    PCIDevice *pf = pcie_sriov_get_pf(vf);

    addr = vf_to_pf_addr(addr, pcie_sriov_vf_number(vf));
    igb_mmio_write(pf, addr, val, size);
}

static const MemoryRegionOps mmio_ops = {
    .read = igbvf_mmio_read,
    .write = igbvf_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void igbvf_pci_realize(PCIDevice *dev, Error **errp)
{
    IgbVfState *s = IGBVF(dev);
    int i;

    dev->config_write = igbvf_write_config;

    memory_region_init_io(&s->mmio, OBJECT(dev), &mmio_ops, s, "igbvf-mmio",
        IGBVF_MMIO_SIZE);
    pcie_sriov_vf_register_bar(dev, IGBVF_MMIO_BAR_IDX, &s->mmio);

    memory_region_init(&s->msix, OBJECT(dev), "igbvf-msix", IGBVF_MSIX_SIZE);
    pcie_sriov_vf_register_bar(dev, IGBVF_MSIX_BAR_IDX, &s->msix);

    (void)msix_init(dev, IGBVF_MSIX_VECTORS, &s->msix, IGBVF_MSIX_BAR_IDX, 0,
        &s->msix, IGBVF_MSIX_BAR_IDX, 0x2000, 0x70, errp);

    for (i = 0; i < IGBVF_MSIX_VECTORS; i++) {
        if (msix_vector_use(dev, i) < 0) {
            msix_unuse_all_vectors(dev);
            msix_uninit(dev, &s->msix, &s->msix);
        }
    }

    if (pcie_endpoint_cap_init(dev, 0xa0) < 0) {
        hw_error("Failed to initialize PCIe capability");
    }

    if (pcie_aer_init(dev, 1, 0x100, 0x40, errp) < 0) {
        hw_error("Failed to initialize AER capability");
    }

    pcie_ari_init(dev, 0x150, 1);
}

static void igbvf_pci_uninit(PCIDevice *dev)
{
    IgbVfState *s = IGBVF(dev);

    pcie_aer_exit(dev);
    pcie_cap_exit(dev);
    msix_unuse_all_vectors(dev);
    msix_uninit(dev, &s->msix, &s->msix);
}

static void igbvf_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);

    c->realize = igbvf_pci_realize;
    c->exit = igbvf_pci_uninit;
    c->vendor_id = PCI_VENDOR_ID_INTEL;
    c->device_id = PCI_DEVICE_ID_INTEL_82576_VF;
    c->revision = 1;
    c->romfile = NULL;
    c->class_id = PCI_CLASS_NETWORK_ETHERNET;

    dc->desc = "Intel 82576 Virtual Function";

    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo igbvf_info = {
    .name = TYPE_IGBVF,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IgbVfState),
    .class_init = igbvf_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void igb_register_types(void)
{
    type_register_static(&igbvf_info);
}

type_init(igb_register_types)
