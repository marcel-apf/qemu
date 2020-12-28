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

#ifndef HW_NET_IGB_COMMON_H
#define HW_NET_IGB_COMMON_H

uint64_t igb_mmio_read(void *opaque, hwaddr addr, unsigned size);
void igb_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);

#endif // HW_NET_IGB_COMMON_H
