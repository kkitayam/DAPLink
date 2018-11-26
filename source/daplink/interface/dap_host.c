/**
 * @file    dap_host.c
 * @brief   Implementation of dap_host.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "target_reset.h"
#include "target_config.h"
#include "DAP_config.h"
#include "DAP.h"
#include "dap_host.h"

uint8_t dap_init(void)
{
#if (DAP_SWD == 1)
    return swd_init();
#else
    return jtag_init();
#endif
}

uint8_t dap_off(void)
{
#if (DAP_SWD == 1)
    return swd_off();
#else
    return jtag_off();
#endif
}

uint8_t dap_init_debug(void)
{
#if (DAP_SWD == 1)
    return swd_init_debug();
#else
    return jtag_init_debug();
#endif
}

uint8_t dap_read_dp(uint8_t adr, uint32_t *val)
{
#if (DAP_SWD == 1)
    return swd_read_dp(adr, val);
#else
    return jtag_read_dp(adr, val);
#endif
}

uint8_t dap_write_dp(uint8_t adr, uint32_t val)
{
#if (DAP_SWD == 1)
    return swd_write_dp(adr, val);
#else
    return jtag_write_dp(adr, val);
#endif
}

uint8_t dap_read_ap(uint32_t adr, uint32_t *val)
{
#if (DAP_SWD == 1)
    return swd_read_ap(adr, val);
#else
    return jtag_read_ap(adr, val);
#endif
}

uint8_t dap_write_ap(uint32_t adr, uint32_t val)
{
#if (DAP_SWD == 1)
    return swd_write_ap(adr, val);
#else
    return jtag_write_ap(adr, val);
#endif
}

uint8_t dap_read_memory(uint32_t address, uint8_t *data, uint32_t size)
{
#if (DAP_SWD == 1)
    return swd_read_memory(address, data, size);
#else
    return jtag_read_memory(address, data, size);
#endif
}

uint8_t dap_write_memory(uint32_t address, uint8_t *data, uint32_t size)
{
#if (DAP_SWD == 1)
    return swd_write_memory(address, data, size);
#else
    return jtag_write_memory(address, data, size);
#endif
}

uint8_t dap_flash_syscall_exec(const program_syscall_t *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
#if (DAP_SWD == 1)
    return swd_flash_syscall_exec(sysCallParam, entry, arg1, arg2, arg3, arg4);
#else
    return jtag_flash_syscall_exec(sysCallParam, entry, arg1, arg2, arg3, arg4);
#endif
}

uint8_t dap_set_target_state_hw(TARGET_RESET_STATE state)
{
#if (DAP_SWD == 1)
    return swd_set_target_state_hw(state);
#else
    return jtag_set_target_state_hw(state);
#endif
}

uint8_t dap_set_target_state_sw(TARGET_RESET_STATE state)
{
#if (DAP_SWD == 1)
    return swd_set_target_state_sw(state);
#else
    return jtag_set_target_state_sw(state);
#endif
}

void dap_set_reset_connect(DAP_CONNECT_TYPE type)
{
#if (DAP_SWD == 1)
    return dap_set_reset_connect(type);
#endif
}

void dap_set_soft_reset(uint32_t soft_reset_type)
{
#if (DAP_SWD == 1)
    return swd_set_soft_reset(soft_reset_type);
#else
    return jtag_set_soft_reset(soft_reset_type);
#endif
}
