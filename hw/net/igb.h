#ifndef HW_NET_IGB_H
#define HW_NET_IGB_H

#include "qemu/osdep.h"
#include "hw/pci/pci.h"

uint64_t igb_read(PCIDevice *dev, hwaddr addr, unsigned size);

void igb_write(PCIDevice *dev, hwaddr addr, uint64_t val, unsigned size);

#endif // HW_NET_IGB_H
