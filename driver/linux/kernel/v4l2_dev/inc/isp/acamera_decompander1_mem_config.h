/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2019 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#ifndef __ACAMERA_DECOMPANDER1_MEM_CONFIG_H__
#define __ACAMERA_DECOMPANDER1_MEM_CONFIG_H__


#include "system_sw_io.h"

#include "system_hw_io.h"

// ------------------------------------------------------------------------------ //
// Instance 'decompander1_ping_mem' of module 'decompander1_ping_mem'
// ------------------------------------------------------------------------------ //

#define ACAMERA_DECOMPANDER1_MEM_BASE_ADDR (0xabf0L)
#define ACAMERA_DECOMPANDER1_MEM_SIZE (0x800)

#define ACAMERA_DECOMPANDER1_MEM_ARRAY_DATA_DEFAULT (0x0)
#define ACAMERA_DECOMPANDER1_MEM_ARRAY_DATA_DATASIZE (32)
#define ACAMERA_DECOMPANDER1_MEM_ARRAY_DATA_OFFSET (0xabf0L)

// args: index (0-256), data (32-bit)
static __inline void acamera_decompander1_mem_array_data_write( uintptr_t base, uint32_t index, uint32_t data) {
    system_sw_write_32(base + 0xabf0L + (index << 2), data);
}
static __inline uint32_t acamera_decompander1_mem_array_data_read( uintptr_t base, uint32_t index) {
    return system_sw_read_32(base + 0xabf0L + (index << 2));
}
// ------------------------------------------------------------------------------ //
#endif //__ACAMERA_DECOMPANDER1_MEM_CONFIG_H__
