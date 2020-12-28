/*
* TBD
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
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/msix.h"
#include "igb_common.h"
#include "trace.h"
#include "qapi/error.h"

#define TYPE_IGBVF "igbvf"
#define IGBVF(obj) OBJECT_CHECK(IgbVfState, (obj), TYPE_IGBVF)

#define PCI_DEVICE_ID_INTEL_82576_VF    0x10CA

#define IGB_MSIX_VECTORS_VF (3)

#define IGBVF_MMIO_BAR_IDX  (0)
#define IGBVF_MSIX_BAR_IDX  (3)

#define IGBVF_MMIO_SIZE     (16 * 1024)
#define IGBVF_MSIX_SIZE     (16 * 1024)

typedef struct IgbVfState {
    PCIDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion msix;

} IgbVfState;

static void igbvf_write_config(PCIDevice *d, uint32_t address, uint32_t val,
                               int len)
{
    IgbVfState *s = IGBVF(d);
    (void)s;

    trace_igbvf_write_config(address, val, len);
    pci_default_write_config(d, address, val, len);
}

static uint64_t igbvf_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    IgbVfState *s = opaque;
    PCIDevice *pf = pcie_sriov_get_pf(&s->parent_obj);

    return igb_mmio_read(pf, addr, size);
}

static void igbvf_mmio_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    IgbVfState *s = opaque;
    PCIDevice *pf = pcie_sriov_get_pf(&s->parent_obj);

    return igb_mmio_write(pf, addr, val, size);
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

static void igbvf_pci_realize(PCIDevice *d, Error **errp)
{
    IgbVfState *s = IGBVF(d);
    int ret;
    int v;

    /* TBD: pci_e1000_realize(d, errp); */
    if (*errp)
        return;

    d->config_write = igbvf_write_config;

    memory_region_init_io(&s->mmio, OBJECT(d), &mmio_ops, s, "igbvf-mmio",
                          IGBVF_MMIO_SIZE);
    pcie_sriov_vf_register_bar(d, IGBVF_MMIO_BAR_IDX, &s->mmio);

    memory_region_init(&s->msix, OBJECT(d), "igbvf-msix", IGBVF_MSIX_SIZE);
    pcie_sriov_vf_register_bar(d, IGBVF_MSIX_BAR_IDX, &s->msix);

    ret = msix_init(d, IGB_MSIX_VECTORS_VF, &s->msix,
                    IGBVF_MSIX_BAR_IDX, 0, &s->msix,
                    IGBVF_MSIX_BAR_IDX, 0x2000, 0x70, errp);
    if (ret) {
        goto err_msix;
    }

    for (v = 0; v < IGB_MSIX_VECTORS_VF; v++) {
        ret = msix_vector_use(d, v);
        if (ret) {
            goto err_pcie_cap;
        }
    }

    ret = pcie_endpoint_cap_init(d, 0xa0);
    if (ret < 0) {
        goto err_pcie_cap;
    }

    ret = pcie_aer_init(d, 1, 0x100, 0x40, errp);
    if (ret < 0) {
        goto err_aer;
    }

    pcie_ari_init(d, 0x150, 1);
    return;

 err_aer:
    pcie_cap_exit(d);
 err_pcie_cap:
    msix_unuse_all_vectors(d);
    msix_uninit(d, &s->msix, &s->msix);
 err_msix:
    return;
    /* TBD: pci_e1000_uninit(d); */
}

static void igbvf_pci_reset(DeviceState *dev)
{
    PCIDevice *d = PCI_DEVICE(dev);
    IgbVfState *s = IGBVF(dev);

    (void)s;
    (void)d;
    trace_igb_cb_qdev_reset();

    /* TBD */
}

static void igbvf_pci_uninit(PCIDevice *d)
{
    IgbVfState *igb = IGBVF(d);
    MemoryRegion *mr = &igb->msix;

    pcie_cap_exit(d);
    msix_uninit(d, mr, mr);
    /* TBD: pci_e1000_uninit(d); */
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

    dc->desc = "Intel 82576 GbE Controller Virtual Function";
    dc->reset = igbvf_pci_reset;
    //dc->vmsd = &igb_vmstate;

    //device_class_set_props(dc, igb_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static void igbvf_instance_init(Object * obj)
{

}

static const TypeInfo igbvf_info = {
    .name = TYPE_IGBVF,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(IgbVfState),
    .class_init = igbvf_class_init,
    .instance_init = igbvf_instance_init,
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
