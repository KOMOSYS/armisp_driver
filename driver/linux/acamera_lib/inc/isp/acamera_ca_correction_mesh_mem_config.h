//----------------------------------------------------------------------------
//   The confidential and proprietary information contained in this file may
//   only be used by a person authorised under and to the extent permitted
//   by a subsisting licensing agreement from ARM Limited or its affiliates.
//
//          (C) COPYRIGHT [2019] ARM Limited or its affiliates.
//              ALL RIGHTS RESERVED
//
//   This entire notice must be reproduced on all copies of this file
//   and copies of this file may only be made by a person if such person is
//   permitted to do so under the terms of a subsisting license agreement
//   from ARM Limited or its affiliates.
//----------------------------------------------------------------------------

#ifndef __ACAMERA_CA_CORRECTION_MESH_MEM_CONFIG_H__
#define __ACAMERA_CA_CORRECTION_MESH_MEM_CONFIG_H__


#include "system_sw_io.h"

#include "system_hw_io.h"

// ------------------------------------------------------------------------------ //
// Instance 'ca_correction_mesh_ping_mem' of module 'ca_correction_mesh_ping_mem'
// ------------------------------------------------------------------------------ //

#define ACAMERA_CA_CORRECTION_MESH_MEM_BASE_ADDR (0xf074L)
#define ACAMERA_CA_CORRECTION_MESH_MEM_SIZE (0x4000)

#define ACAMERA_CA_CORRECTION_MESH_MEM_ARRAY_DATA_DEFAULT (0x0)
#define ACAMERA_CA_CORRECTION_MESH_MEM_ARRAY_DATA_DATASIZE (32)
#define ACAMERA_CA_CORRECTION_MESH_MEM_ARRAY_DATA_OFFSET (0xf074L)

// args: index (0-4095), data (32-bit)
static __inline void acamera_ca_correction_mesh_mem_array_data_write( uintptr_t base, uint32_t index, uint32_t data) {
    system_sw_write_32(base + 0xf074L + (index << 2), data);
}
static __inline uint32_t acamera_ca_correction_mesh_mem_array_data_read( uintptr_t base, uint32_t index) {
    return system_sw_read_32(base + 0xf074L + (index << 2));
}
// ------------------------------------------------------------------------------ //
#endif //__ACAMERA_CA_CORRECTION_MESH_MEM_CONFIG_H__
