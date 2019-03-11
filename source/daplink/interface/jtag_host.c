/**
 * @file    jtag_host.c
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

#include "cmsis_os2.h"
#include "target_reset.h"
#include "target_config.h"
#include "dap_host.h"
#include "DAP_config.h"
#include "DAP.h"
#include "target_family.h"

#if (DAP_JTAG == 1)

#if defined(TARGET_MCU_CORTEX_A) && defined(DEBUG_REGSITER_BASE_OVERRIDE)
#undef  DEBUG_REGSITER_BASE
#define DEBUG_REGSITER_BASE    (DEBUG_REGSITER_BASE_OVERRIDE)
#endif

#define DEBUG_JTAG_HOST 0

#if DEBUG_JTAG_HOST
#include "daplink_debug.h"
#define jtag_host_printf    debug_msg
#else
#define jtag_host_printf(...)
#endif

#if (DAP_SWD == 1) && (DAP_JTAG == 1)
#define DAP_SWJ    1
#else
#define DAP_SWJ    0
#endif


#define XSTR(s)   STR(s)
#define STR(s)    #s
#define HEX(v)    0x##v
#define CHEX(v)   \x##v
/* Reverse a byte array. */
#define GET_REV_STR(_1,_2,_3,_4,_5,_6,_7,_8,NAME,...) NAME
#define REV_STR1(n0)                             XSTR(CHEX(n0))
#define REV_STR2(n0,n1)                          REV_STR1(n1) XSTR(CHEX(n0))
#define REV_STR3(n0,n1,n2)                       REV_STR2(n2,n1) XSTR(CHEX(n0))
#define REV_STR4(n0,n1,n2,n3)                    REV_STR3(n3,n2,n1) XSTR(CHEX(n0))
#define REV_STR5(n0,n1,n2,n3,n4)                 REV_STR4(n4,n3,n2,n1) XSTR(CHEX(n0))
#define REV_STR6(n0,n1,n2,n3,n4,n5)              REV_STR5(n5,n4,n3,n2,n1) XSTR(CHEX(n0))
#define REV_STR7(n0,n1,n2,n3,n4,n5,n6)           REV_STR6(n6,n5,n4,n3,n2,n1) XSTR(CHEX(n0))
#define REV_STR8(n0,n1,n2,n3,n4,n5,n6,n7)        REV_STR7(n7,n6,n5,n4,n3,n2,n1) XSTR(CHEX(n0))
#define REV_STR(...) GET_REV_STR(__VA_ARGS__, REV_STR8, REV_STR7, REV_STR6, REV_STR5, REV_STR4, REV_STR3, REV_STR2, REV_STR1)(__VA_ARGS__)

#define MAX_TRANSFER_RETRY 1000
#define MAX_TIMEOUT   100000  // Timeout for syscalls on target

#if defined(TARGET_MCU_CORTEX_A)
#define AHB_AP             (0x00000000)  /* setting of SELECT access memmory */
#define APB_AP             (0x01000000)  /* setting of SELECT access Debug Register */
#define MSK_APSEL          (0xFF000000)

// AP CSW register, base value
#define CSW_VALUE (0x80000000 | CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_PADDRINC)
#define CMD_MRC                (0xEE100E15)  /* 1110 1110 0001 0000 RRRR 1110 0001 0101 */
#define CMD_MCR                (0xEE000E15)  /* 1110 1110 0000 0000 RRRR 1110 0001 0101 */
#define CMD_MSR                (0xE12CF000)  /* 1110 0001 0010 1100 1111 0000 0000 RRRR */
#define CMD_MRS                (0xE14F0000)  /* 1110 0001 0100 1111 RRRR 0000 0000 0000 */
#define CMD_MOV                (0xE1A00000)  /* 1110 0001 1010 0000 DDDD 0000 0000 RRRR */ /* D = distination */

#define DBGDSCR_HALTED         (0x00000001)

static uint32_t current_ap = AHB_AP;
#else

// Default NVIC and Core debug base addresses
#define NVIC_Addr    (0xe000e000)
#define DBG_Addr     (0xe000edf0)

// AP CSW register, base value
#define CSW_VALUE (CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)

#define DCRDR 0xE000EDF8
#define DCRSR 0xE000EDF4
#define DHCSR 0xE000EDF0
#define REGWnR (1 << 16)

#define SCB_AIRCR_PRIGROUP_Pos              8                                             /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */

static DAP_CONNECT_TYPE reset_connect = CONNECT_NORMAL;
static uint32_t soft_reset = SYSRESETREQ;

#endif

typedef struct {
    uint32_t r[16];
    uint32_t xpsr;
} DEBUG_STATE;

static union {
    uint8_t  b[8];
    uint32_t w[2];
} req_xfer = {ID_DAP_Transfer, 0, 1, };
static uint8_t resp[8];

// Write 32-bit word aligned values to target memory using address auto-increment.
// size is in bytes.
static uint8_t jtag_write_block(uint32_t address, uint8_t *data, uint32_t size)
{
    uint32_t size_in_words;
    uint32_t i;
    uint32_t *work_write_data;

    if (size == 0) {
        return 0;
    }

    size_in_words = size / 4;

    // CSW register
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }

    if (!jtag_write_ap(AP_TAR, address)) {
        return 0;
    }
    // DRW write
    work_write_data = (uint32_t *)data;
    for (i = 0; i < size_in_words; i++) {
        if (!jtag_write_ap(AP_DRW, *work_write_data)) {
            return 0;
        }
        work_write_data++;
    }
    return 1;
}

// Read target memory.
static uint8_t jtag_read_data(uint32_t addr, uint32_t *val)
{
    if (!jtag_write_ap(AP_TAR, addr)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    if (!jtag_read_ap(AP_DRW, val)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    return 1;
}

// Write target memory.
static uint8_t jtag_write_data(uint32_t address, uint32_t data)
{
    if (!jtag_write_ap(AP_TAR, address)) {
        return 0;
    }
    if (!jtag_write_ap(AP_DRW, data)) {
        return 0;
    }
    return 1;
}

// Read 32-bit word from target memory.
static uint8_t jtag_read_word(uint32_t addr, uint32_t *val)
{
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    if (!jtag_read_data(addr, val)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    return 1;
}

// Write 32-bit word to target memory.
static uint8_t jtag_write_word(uint32_t addr, uint32_t val)
{
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32)) {
        return 0;
    }
    if (!jtag_write_data(addr, val)) {
        return 0;
    }
    return 1;
}

#ifdef TARGET_MCU_CORTEX_A
static uint8_t jtag_switch_ap(uint32_t target)
{
    uint32_t ret;

    if (target != current_ap) {
        ret = jtag_write_dp(DP_SELECT, target);
        if (!ret) {
            return 0;
        }
        current_ap = target;
    }
    return 1;
}

static uint8_t jtag_enable_debug(void)
{
    uint32_t val;

    if (!jtag_switch_ap(APB_AP)) {
        return 0;
    }
    if (!jtag_read_word(DBGDSCR, &val)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    /* DBGDSCR ITRen = 1(ARM instruction enable) */
    /* and ExtDCCmode = 01(stall mode) */
    val = val | 0x00106000;
    if (!jtag_write_word(DBGDSCR, val)) {
        return 0;
    }
    return 1;
}

static uint8_t jtag_read_core_register(uint32_t n, uint32_t *val)
{
    if (!jtag_switch_ap(APB_AP)) {
        return 0;
    }

    if (!jtag_write_word(DBGITR, CMD_MCR | (n << 12))) {
        return 0;
    }

    if (!jtag_read_word(DBGDTRTX, val)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    return 1;
}

static uint8_t jtag_write_core_register(uint32_t n, uint32_t val)
{
    if (!jtag_switch_ap(APB_AP)) {
        return 0;
    }

    if (!jtag_write_word(DBGDTRRX, val)) {
        return 0;
    }

    /* Write MRC */
    if (!jtag_write_word(DBGITR, (CMD_MRC | (n << 12)))) {
        return 0;
    }

    return 1;
}

static uint8_t jtag_restart_req(void)
{
    uint32_t val, i, timeout = MAX_TIMEOUT;

    if (!jtag_switch_ap(APB_AP)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    /* Clear ITRen */
    if (!jtag_read_word(DBGDSCR, &val)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    val = val & ~0x00002000;
    if (!jtag_write_word(DBGDSCR, val)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    for (i = 0; i < timeout; i++) {
        /* read DBGDSCR */
        if (!jtag_read_word(DBGDSCR, &val)) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }
        /* wait Clear UND_I, ADABORT_I, SDABORT_I[bit:8-6] and InstrCompl_I[bit24] set to 1 */
        if ((val & 0x010001C0) == 0x01000000) {
            break;
        } else if (i == (timeout -1)) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }
        osDelay(1);
    }
    /* DBGDRCR Restart req */
    if (!jtag_write_word(DBGDRCR, 0x00000006 )) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    for (i = 0; i < timeout; i++) {
        /* read DBGDSCR */
        if (!jtag_read_word(DBGDSCR, &val)) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }
        if ((val & 0x00000002) == 0x00000002) {
            /* restarted */
            return 1;
        }
        osDelay(1);
    }
    jtag_host_printf("E %d\n", __LINE__);
    return 0;
}

static uint8_t jtag_halt_req(void)
{
    if (!jtag_switch_ap(APB_AP)) {
        return 0;
    }
    /* DBGDRCR halt req*/
    if (!jtag_write_word(DBGDRCR, 0x00000001)) {
        return 0;
    }
    return 1;
}

static uint8_t jtag_wait_until_halted(void)
{
    uint32_t val, i, timeout = MAX_TIMEOUT;

    if (!jtag_switch_ap(APB_AP)) {
        return 0;
    }

    for (i = 0; i < timeout; i++) {
        /* read DBGDSCR */
        if (!jtag_read_word(DBGDSCR, &val)) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }

        if ((val & DBGDSCR_HALTED) == DBGDSCR_HALTED) {
            return 1;
        }
        osDelay(1);
    }

    return 0;
}

// Execute system call.
static uint8_t jtag_write_debug_state(DEBUG_STATE *state)
{
    uint32_t i, status;

    if (!jtag_write_dp(DP_SELECT, 0)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    // R0, R1, R2, R3
    for (i = 0; i < 4; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }
    }

    // R9
    if (!jtag_write_core_register(9, state->r[9])) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    // R13, R14
    for (i = 13; i < 15; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }
    }

    // xPSR
    /* xPSR write */
    /* write PSR (write r6) */
    if (!jtag_write_core_register(6, state->xpsr)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    /* MSR (PSR <- r6) */
    if (!jtag_write_word(DBGITR, CMD_MSR | (6))) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    /* R15(PC) */
    /* MRC R7 */
    if (!jtag_write_core_register(7, state->r[15])) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    /* MOV R15, R7 */
    if (!jtag_write_word(DBGITR, CMD_MOV | (15 << 12) | (7))) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    if (!jtag_restart_req()) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    // check status
    if (!jtag_read_dp(DP_CTRL_STAT, &status)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    if (status & (STICKYERR | WDATAERR)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    return 1;
}

#else

#define jtag_switch_ap(x)              (1)

static uint8_t jtag_enable_debug(void)
{
    if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
        return 0;
    }
    return 1;
}

static uint8_t jtag_read_core_register(uint32_t n, uint32_t *val)
{
    int i = 0, timeout = 100;

    if (!jtag_write_word(DCRSR, n)) {
        return 0;
    }

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++) {
        if (!jtag_read_word(DHCSR, val)) {
            return 0;
        }

        if (*val & S_REGRDY) {
            break;
        }
        osDelay(1);
    }

    if (i == timeout) {
        return 0;
    }

    if (!jtag_read_word(DCRDR, val)) {
        return 0;
    }

    return 1;
}

static uint8_t jtag_write_core_register(uint32_t n, uint32_t val)
{
    int i = 0, timeout = 100;

    if (!jtag_write_word(DCRDR, val)) {
        return 0;
    }

    if (!jtag_write_word(DCRSR, n | REGWnR)) {
        return 0;
    }

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++) {
        if (!jtag_read_word(DHCSR, &val)) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }

        if (val & S_REGRDY) {
            return 1;
        }
        osDelay(1);
    }

    return 0;
}

static uint8_t jtag_wait_until_halted(void)
{
    // Wait for target to stop
    uint32_t val, i, timeout = MAX_TIMEOUT;

    for (i = 0; i < timeout; i++) {
        if (!jtag_read_word(DBG_HCSR, &val)) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }

        if (val & S_HALT) {
            return 1;
        }
        osDelay(1);
    }

    return 0;
}

// Execute system call.
static uint8_t jtag_write_debug_state(DEBUG_STATE *state)
{
    uint32_t i, status;

    if (!jtag_write_dp(DP_SELECT, 0)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    // R0, R1, R2, R3
    for (i = 0; i < 4; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }
    }

    // R9
    if (!jtag_write_core_register(9, state->r[9])) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    // R13, R14, R15
    for (i = 13; i < 16; i++) {
        if (!jtag_write_core_register(i, state->r[i])) {
            jtag_host_printf("E %d\n", __LINE__);
            return 0;
        }
    }

    // xPSR
    if (!jtag_write_core_register(16, state->xpsr)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    // check status
    if (!jtag_read_dp(DP_CTRL_STAT, &status)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    if (status & (STICKYERR | WDATAERR)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    return 1;
}

#endif

static int jtag_debug_port_setup(void)
{
    int type;

    /* request DAP_Connect() */
    DAP_ProcessCommand("\x02\x00", resp);
    type = resp[1];
    /* request DAP_JTAG_Configure() to setup IR Length */
    DAP_ProcessCommand("\x15\x01\x04", resp);

    if (type == 2) { /* JTAG */
#if (DAP_SWJ != 0)
        SWJ_Sequence(51, REV_STR(00,07,FF,FF,FF,FF,FF,FF));
        SWJ_Sequence(16, REV_STR(E7,3C));
        SWJ_Sequence(6, REV_STR(3F));
#endif
        JTAG_Sequence(6 | JTAG_SEQUENCE_TMS, REV_STR(3F), &resp[0]);
        JTAG_Sequence(1, REV_STR(01), &resp[1]);
        return 1;
    }
    return 0;
}

static int jtag_debug_port_start(void)
{
    uint32_t val;
    int i;
    int timeout = 100;

    // Switch to DP Register Bank 0
    if (!jtag_write_dp(DP_SELECT, 0x00000000)) {
        return 0;
    }
    // Read DP CTRL/STAT Register and check if CSYSPWRUPACK and CDBGPWRUPACK bits are set
    if (!jtag_read_dp(DP_CTRL_STAT, &val)) {
        return 0;
    }
    if ((CSYSPWRUPREQ | CDBGPWRUPREQ) != (val & (CSYSPWRUPREQ | CDBGPWRUPREQ))) {
        /* power up */
        if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
            return 0;
        }
        for (i = 0; i < timeout; i++) {
            if (!jtag_read_dp(DP_CTRL_STAT, &val)) {
                return 0;
            }
            if ((val & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK)) {
                // Break from loop if powerup is complete
                break;
            }
        }
    }
    if (i == timeout) {
        return 0;
    }
    if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | CDBGRSTREQ)) {
        return 0;
    }
    /* Wait for Debug Reset to be acknowledged, don't issue error on timeout to deal with improperly connected Debug Reset */
    for (i = 0; i < timeout; i++) {
        if (!jtag_read_dp(DP_CTRL_STAT, &val)) {
            return 0;
        }
        if ((val & (CDBGPWRUPACK | CSYSPWRUPACK | CDBGRSTACK)) == (CDBGPWRUPACK | CSYSPWRUPACK | CDBGRSTACK)) {
            // Break from loop if dbgreset is complete
            break;
        }
    }

    // Init AP Transfer Mode, Transaction Counter, and Lane Mask (Normal Transfer Mode, Include all Byte Lanes)
    // Additionally clear STICKYORUN, STICKYCMP, and STICKYERR bits by writing '1'
    if (!jtag_write_dp(DP_CTRL_STAT, 0x50000F32)) {
        return 0;
    }

    // call a target dependant function:
    // some target can enter in a lock state
    // this function can unlock these targets
    if (g_target_family && g_target_family->target_unlock_sequence) {
        g_target_family->target_unlock_sequence();
    }

    if (!jtag_write_dp(DP_SELECT, 0)) {
        return 0;
    }

    return 1;
}

static int jtag_debug_port_stop(void)
{
    // Switch to DP Register Bank 0
    if (!jtag_write_dp(DP_SELECT,0x00000000)) {
        return 0;
    }
    // Power Down Debug port
    if (!jtag_write_dp(DP_CTRL_STAT,0x00000000)) {
        return 0;
    }
    return 1;
}


/****************************************************************
 * public functions
 ****************************************************************/
uint8_t jtag_init(void)
{
    if (!jtag_debug_port_setup()) {
        return 0;
    }
    return 1;
}

uint8_t jtag_off(void)
{
    if (jtag_debug_port_stop()) {
        return 0;
    }
    return 1;
}

uint8_t jtag_init_debug(void)
{
    if (!jtag_init()) {
        return 0;
    }
    if (!jtag_debug_port_start()) {
        return 0;
    }
    return 1;
}

uint8_t jtag_read_dp(uint8_t adr, uint32_t *val)
{
    uint32_t num;

    req_xfer.b[3] = (adr & (DAP_TRANSFER_A2|DAP_TRANSFER_A3)) | DAP_TRANSFER_RnW;
    num = DAP_ProcessCommand(req_xfer.b, resp);
    if (((num & 0xFFFF) != 7) ||
            (resp[0] != ID_DAP_Transfer) || (resp[1] != 1) || (resp[2] != 1)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
    *val = (resp[6]<<24) | (resp[5]<<16) | (resp[4]<<8) | resp[3];
    return 1;
}

uint8_t jtag_write_dp(uint8_t adr, uint32_t val)
{
    uint32_t num;

#ifdef TARGET_MCU_CORTEX_A
    if (DP_SELECT == adr) {
        current_ap = val & MSK_APSEL;
    }
#endif

    req_xfer.b[3] = adr & (DAP_TRANSFER_A2|DAP_TRANSFER_A3);
    req_xfer.w[1] = val;
    num = DAP_ProcessCommand(req_xfer.b, resp);
    if (((num & 0xFFFF) != 3) ||
            (resp[0] != ID_DAP_Transfer) || (resp[1] != 1) || (resp[2] != 1)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    return 1;
}

uint8_t jtag_read_ap(uint32_t adr, uint32_t *val)
{
    uint32_t num;

    req_xfer.b[3] = (adr & (DAP_TRANSFER_A2|DAP_TRANSFER_A3)) | DAP_TRANSFER_RnW | DAP_TRANSFER_APnDP;
    num = DAP_ProcessCommand(req_xfer.b, resp);
    if (((num & 0xFFFF) != 7) ||
            (resp[0] != ID_DAP_Transfer) || (resp[1] != 1) || (resp[2] != 1)) {
        jtag_host_printf("E %d %x %x %x %x\n", __LINE__, num, resp[0], resp[1], resp[2]);
        return 0;
    }
    *val = (resp[6]<<24) | (resp[5]<<16) | (resp[4]<<8) | resp[3];
    return 1;
}

uint8_t jtag_write_ap(uint32_t adr, uint32_t val)
{
    uint32_t num;

    req_xfer.b[3] = adr & (DAP_TRANSFER_A2|DAP_TRANSFER_A3) | DAP_TRANSFER_APnDP;
    req_xfer.w[1] = val;
    num = DAP_ProcessCommand(req_xfer.b, resp);
    if (((num & 0xFFFF) != 3) ||
            (resp[0] != ID_DAP_Transfer) || (resp[1] != 1) || (resp[2] != 1)) {
        jtag_host_printf("E %d %x %x %x %x\n", __LINE__, num, resp[0], resp[1], resp[2]);
        return 0;
    }
    return 1;
}

uint8_t jtag_read_memory(uint32_t address, uint8_t *data, uint32_t size)
{
    uint32_t read_size;
    uint32_t* read_data;

    jtag_host_printf("read_memory %x %d\n", address, size);
#ifdef TARGET_MCU_CORTEX_A
    if (!jtag_switch_ap(AHB_AP)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
#endif

    read_size = (size / 4);
    read_data = (uint32_t*)data;
    /* Write bytes until end */
    while ((read_size > 0)) {
        if (!jtag_read_data(address, read_data)) {
            return 0;
        }
        address += 4;
        read_data++;
        read_size--;
    }
    return 1;
}

uint8_t jtag_write_memory(uint32_t address, uint8_t *data, uint32_t size)
{
    uint32_t n;

    jtag_host_printf("write_memory %x %d\n", address, size);
#ifdef TARGET_MCU_CORTEX_A
    if (!jtag_switch_ap(AHB_AP)) {
        return 0;
    }
#endif

    while (size > 3) {
        // Limit to auto increment page size
        n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
        if (size < n) {
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        }

        if (!jtag_write_block(address, data, n)) {
            return 0;
        }

        address += n;
        data += n;
        size -= n;
    }
    /* Auto increment is end */
    /* Return the CSW reg value to SIZE8 */
    if (!jtag_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE8)) {
        return 0;
    }

    return 1;
}


uint8_t jtag_flash_syscall_exec(const program_syscall_t *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
    DEBUG_STATE state = {{0}, 0};

    jtag_host_printf("syscall %x %x %x %x %x\n", entry, arg1, arg2, arg3, arg4);

    // Call flash algorithm function on target and wait for result.
    state.r[0]     = arg1;                   // R0: Argument 1
    state.r[1]     = arg2;                   // R1: Argument 2
    state.r[2]     = arg3;                   // R2: Argument 3
    state.r[3]     = arg4;                   // R3: Argument 4
    state.r[9]     = sysCallParam->static_base;    // SB: Static Base
    state.r[13]    = sysCallParam->stack_pointer;  // SP: Stack Pointer
    state.r[14]    = sysCallParam->breakpoint;     // LR: Exit Point
    state.r[15]    = entry;                        // PC: Entry Point
#ifdef TARGET_MCU_CORTEX_A
    state.xpsr     = 0x00000000;          // xPSR: T = 0, ISR = 0
#else
    state.xpsr     = 0x01000000;          // xPSR: T = 1, ISR = 0
#endif

    if (!jtag_write_debug_state(&state)) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    if (!jtag_wait_until_halted()) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

#ifdef TARGET_MCU_CORTEX_A
    if (!jtag_enable_debug()) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }
#endif

    if (!jtag_read_core_register(0, &state.r[0])) {
        jtag_host_printf("E %d\n", __LINE__);
        return 0;
    }

    // Flash functions return 0 if successful.
    if (state.r[0] != 0) {
        jtag_host_printf("E %d %d\n",__LINE__, state.r[0]);
        return 0;
    }
    return 1;
}

uint8_t jtag_set_target_state_hw(TARGET_RESET_STATE state)
{
    jtag_host_printf("hw %d\n", state);
    /* Calling jtag_init prior to enterring RUN state causes operations to fail. */
    if (state != RUN) {
        if (!jtag_debug_port_setup()) {
            return 0;
        }
        if (!jtag_debug_port_start()) {
            return 0;
        }
    }
    switch (state) {
        case RESET_HOLD:
            dap_set_target_reset(1);
            break;

        case RESET_RUN:
            dap_set_target_reset(1);
            osDelay(2);
            dap_set_target_reset(0);
            osDelay(2);
            if (!jtag_debug_port_stop()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }
            break;
        case RESET_PROGRAM:
#ifdef TARGET_MCU_CORTEX_A
            // Reset
            dap_set_target_reset(1);
            osDelay(2);
            dap_set_target_reset(0);
            osDelay(2);
#endif
            if (!jtag_init_debug()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }

#if !defined(TARGET_MCU_CORTEX_A)
            if (reset_connect == CONNECT_UNDER_RESET) {
                // Assert reset
                dap_set_target_reset(1);
                osDelay(2);
            }
#endif

            if (!jtag_enable_debug()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }

#ifdef TARGET_MCU_CORTEX_A
            if (!jtag_halt_req()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }
#else
            // Enable halt on reset
            if (!jtag_write_word(DBG_EMCR, VC_CORERESET)) {
                return 0;
            }
            if (reset_connect == CONNECT_NORMAL) {
                // Assert reset
                dap_set_target_reset(1);
                osDelay(2);
            }
            // Deassert reset
            dap_set_target_reset(0);
#endif
            osDelay(2);
            if (!jtag_wait_until_halted()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }

#if !defined(TARGET_MCU_CORTEX_A)
            // Disable halt on reset
            if (!jtag_write_word(DBG_EMCR, 0)) {
                return 0;
            }
#endif
            break;
        default:
            return 0;
    }
    return 1;
}

uint8_t jtag_set_target_state_sw(TARGET_RESET_STATE state)
{
#ifdef TARGET_MCU_CORTEX_A
    jtag_host_printf("sw %d\n", state);
    return 0;
#else
    uint32_t val;
    int8_t ap_retries = 2;
    jtag_host_printf("sw %d\n", state);
    if (state != RUN) {
        if (!jtag_debug_port_setup()) {
            return 0;
        }
        if (!jtag_debug_port_start()) {
            return 0;
        }
    }
    switch (state) {
        case RESET_HOLD:
            jtag_set_target_reset(1);
            break;
        case RESET_RUN:
            jtag_set_target_reset(1);
            osDelay(2);
            jtag_set_target_reset(0);
            osDelay(2);
            jtag_off();
            break;
        case RESET_PROGRAM:
            if (!jtag_init_debug()) {
                return 0;
            }
            // Enable debug and halt the core (DHCSR <- 0xA05F0003)
            while (jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT) == 0) {
                if ( --ap_retries <=0 ) {
                    return 0;
                }
                // Target is in invalid state?
                jtag_set_target_reset(1);
                osDelay(2);
                jtag_set_target_reset(0);
                osDelay(2);
            }
            // Wait until core is halted
            if (!jtag_wait_until_halted()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }

            // Enable halt on reset
            if (!jtag_write_word(DBG_EMCR, VC_CORERESET)) {
                return 0;
            }

            // Perform a soft reset
            if (!jtag_read_word(NVIC_AIRCR, &val)) {
                return 0;
            }

            if (!jtag_write_word(NVIC_AIRCR, VECTKEY | (val & SCB_AIRCR_PRIGROUP_Msk) | soft_reset)) {
                return 0;
            }

            osDelay(2);

            if (!jtag_wait_until_halted()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }

            // Disable halt on reset
            if (!jtag_write_word(DBG_EMCR, 0)) {
                return 0;
            }
            break;

        case NO_DEBUG:
            if (!jtag_write_word(DBG_HCSR, DBGKEY)) {
                return 0;
            }
            break;

        case DEBUG:
            if (!jtag_write_dp(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR)) {
                return 0;
            }

            // Ensure CTRL/STAT register selected in DPBANKSEL
            if (!jtag_write_dp(DP_SELECT, 0)) {
                return 0;
            }

            // Power up
            if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
                return 0;
            }

            // Enable debug
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN)) {
                return 0;
            }

            break;

        case HALT:
            if (!jtag_init_debug()) {
                return 0;
            }

            // Enable debug and halt the core (DHCSR <- 0xA05F0003)
            if (!jtag_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT)) {
                return 0;
            }

            // Wait until core is halted
            if (!jtag_wait_until_halted()) {
                jtag_host_printf("E %d\n", __LINE__);
                return 0;
            }
            break;
        case RUN:
            if (!jtag_write_word(DBG_HCSR, DBGKEY)) {
                return 0;
            }
            jtag_off();
            break;
        default:
            return 0;
    }

    return 1;
#endif
}

void jtag_set_reset_connect(DAP_CONNECT_TYPE type)
{
#if !defined(TARGET_MCU_CORTEX_A)
    reset_connect = type;
#endif
}

void jtag_set_soft_reset(uint32_t soft_reset_type)
{
#if !defined(TARGET_MCU_CORTEX_A)
    soft_reset = soft_reset_type;
#endif
}

#endif
