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

#ifndef __ACAMERA_SBUS_I2C_H__
#define __ACAMERA_SBUS_I2C_H__

#include "acamera_sbus_api.h"

void acamera_sbus_i2c_init( acamera_sbus_t *p_bus );

void acamera_sbus_i2c_deinit( acamera_sbus_t *p_bus );

void i2c_init_access( void );

#endif /* __ACAMERA_SBUS_I2C_H__ */
