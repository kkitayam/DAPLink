/**
 * @file    lpc54114xpresso.c
 * @brief   board ID for the NXP LPC54114Xpresso board
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
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

#include "target_family.h"
#include "target_board.h"

const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .board_id = "1054",
    .family_id = kStub_HWReset_FamilyID,
    .daplink_url_name =   "PRODINFOHTM",
    .daplink_drive_name = "LPC54114   ",
    .daplink_target_url = "https://os.mbed.com/platforms/LPCXpresso54114/",
    .target_cfg = &target_device,
    .board_vendor = "NXP",
    .board_name = "LPC54114Xpresso",
};
