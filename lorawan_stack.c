/* * OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
 * lowpower wireless sensor communication
 *
 * Copyright 2017 University of Antwerp
 * Copyright (c) 2017 STMicroelectronics International N.V. (see below)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This code is based on code provided by STMicroelectronics, with the following license and disclaimer:
 *
 * @file    lora.c
 * @author  MCD Application Team
 * @version V1.1.2
 * @date    08-September-2017
 * @brief   lora API to drive the lora state Machine
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "lorawan_stack.h"
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "MODULE_LORAWAN_defs.h"
#include "d7ap_fs.h"
#include "debug.h"
#include "errors.h"
#include "hw.h"
#include "modem_region.h"
#include "scheduler.h"
#include "timer.h"

#if defined(MODULE_LORAWAN_LOG_ENABLED)
#define DPRINT(...) log_print_stack_string(LOG_STACK_ALP, __VA_ARGS__)
#define DPRINT_DATA(p, n) log_print_data(p, n)
#else
#define DPRINT(...)
#define DPRINT_DATA(p, n)
#endif

#define LORAWAN_CLASS CLASS_A
#define JOINREQ_NBTRIALS_LONG 48
#define JOINREQ_NBTRIALS_SHORT 3
#define LORAWAN_APP_DATA_BUFF_SIZE 242

#define SHORT_JOIN_ATTEMPTS_LIMIT 3
#define APP_LAYER_RETRANSMISSIONS_LIMIT 1
#define MAC_LAYER_RETRANSMISSIONS_LIMIT 4

const modem_region_t region = MODULE_LORAWAN_REGION; // TODO: make AS923_x configurable

typedef enum { STATE_NOT_JOINED, STATE_JOINED, STATE_JOIN_FAILED, STATE_JOINING } join_state_t;

static join_state_t join_state = STATE_NOT_JOINED;
static LoRaMacPrimitives_t loraMacPrimitives;
static LoRaMacCallback_t loraMacCallbacks;
static LoRaMacStatus_t loraMacStatus;
static uint8_t devEui[8] = { 0 }; // used for OTAA
static uint8_t appEui[8] = { 0 }; // used for OTAA
static uint8_t appKey[16] = { 0 };

bool adr_enabled = false;
uint8_t datarate = 0;

static bool use_confirmed_tx = false;

static uint8_t payload_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lorawan_AppData_t app_data = { payload_data_buffer, 0, 0 };

static lorawan_rx_callback_t rx_callback = NULL; // called when transmitting is done
static lorawan_tx_completed_callback_t tx_callback = NULL;
static lorawan_status_callback_t stack_status_callback = NULL;

static bool inited = false;
static bool first_init = true;
static bool lorawan_transmitting = false;

static uint8_t joinRequestTrials = 0;

static float antenna_gain_f = 0.0;

static uint8_t app_retransmissions_counter = 0;

/**
 * @brief Called everytime a LoRaWAN retransmission is executed.
 * This will be executed when joining or when nacks are received when an ack was requested
 * @param join_attempt_number
 * 
 * Maintain the same Data Rate and TX power while using the first used FCnt
 * b/c of a known issue with retransmissions of confirmed frames
 * after that, up tx power/reduce data rate in steps e.g.
 * 
 * DR_5/TX low /Fcnt 1, DR_5/TX low /Fcnt 1, DR_5/TX low/Fcnt 1, DR_5/TX low/Fcnt 1,
 * DR_5/Tx low/Fcnt 2, DR_5/Tx high/Fcnt 2, DR_4/Tx high/Fcnt 2, DR_3/Tx high/Fcnt 2
 */
static void network_retry_transmission(uint8_t attempt)
{
    uint8_t total_attempts = (app_retransmissions_counter * MAC_LAYER_RETRANSMISSIONS_LIMIT) + attempt;

    if(join_state == STATE_JOINED) {
        if(total_attempts > MAC_LAYER_RETRANSMISSIONS_LIMIT + 1) {
            LoRaMacIncreaseTxPowerOrDecreaseDataRate();
        }
    }
    
    if (stack_status_callback != NULL)
        stack_status_callback(LORAWAN_STACK_RETRY_TRANSMISSION, total_attempts);
}

static void subband_changed(uint8_t new_subband)
{
    int res = d7ap_fs_write_file(USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 1, &new_subband, 1, ROOT_AUTH);
    if (res != SUCCESS) {
        log_print_error_string(
            "failed to write subband to USER_FILE_LORAWAN_JOINTYPE_FILE_ID file, err code: %u.", res);
    }
}

/**
 * @brief sets the DR to be used for the next join-request during a long join procedure
 * For EU868, IN865 and CN470: any datarate can be used so the passed value gets used by the stack
 * For AS915, US915 and AS923: explicite DRs must be used so the values in these functions get safely ignored.
 */
uint8_t JoinAlternateDrLong(uint8_t joinRequestCounter)
{
    uint8_t dr = 0;

    if ((joinRequestCounter % 48) == 0) {
        dr = DR_0;
    } else if ((joinRequestCounter % 32) == 0) {
        dr = DR_1;
    } else if ((joinRequestCounter % 24) == 0) {
        dr = DR_2;
    } else if ((joinRequestCounter % 16) == 0) {
        dr = DR_3;
    } else if ((joinRequestCounter % 8) == 0) {
        dr = DR_4;
    } else {
        dr = DR_5;
    }

    return dr;
}

/**
 * @brief sets the DR to be used for the next join-request during a short join procedure
 */
uint8_t JoinAlternateDrShort(uint8_t joinRequestCounter)
{
    uint8_t dr = 0;

    if (joinRequestCounter == 1) {
        dr = DR_5;
    } else if (joinRequestCounter == 2) {
        dr = DR_3;
    } else if (joinRequestCounter == 3) {
        dr = DR_0;
    } else {
        log_print_error_string("Error: performing more than 3 join requests in short join loop.");
    }

    return dr;
}

/**
 * @brief LoRaWAN state machine. Sets parameters in the LoRaWAN stack and handles callbacks
 */
static void run_fsm()
{
    switch (join_state) {
    case STATE_JOINING: {
        joinRequestTrials++;

        uint32_t length = 1;
        bool short_join_enabled = false;
        int res = d7ap_fs_read_file(
            USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 0, (uint8_t*)&short_join_enabled, &length, ROOT_AUTH);
        if (res != SUCCESS) {
            log_print_error_string(
                "failed to read short_join_enabled from JOINTYPE file, use long join, err code: %u.", res);
            short_join_enabled = false;
        }

        //if device managed to join before but its not using a recognised subband, continue to do a longer join
        if(region == MODEM_REGION_US915 || region == MODEM_REGION_AU915 || region == MODEM_REGION_CN470) {
            if(lorawan_get_subband() == 0) {
                short_join_enabled = false;   
            }
        }

        uint8_t nbTrials = JOINREQ_NBTRIALS_LONG;

        if (short_join_enabled) {
            nbTrials = JOINREQ_NBTRIALS_SHORT;
            DPRINT("short join, attempt: %u.", joinRequestTrials);
        } else {
            DPRINT("long join, attempt: %u.", joinRequestTrials);
        }

        if (joinRequestTrials <= nbTrials) {

            if (joinRequestTrials > 1) {
                DPRINT("Nack so trying to join again: %d", joinRequestTrials);
                network_retry_transmission(joinRequestTrials); // note that it is possible that there will be a delay
                                                               // (it is only calculated in ScheduleTx)
                // if there is a delay, then the MacDutyDelay function will be called, which calls the
                // duty_cycle_delay_cb function.
            }

            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;

            if (short_join_enabled) {
                mlmeReq.Req.Join.Datarate = JoinAlternateDrShort(joinRequestTrials);
                mlmeReq.subband = lorawan_get_subband();
                DPRINT("join using subband %u", mlmeReq.subband);
            } else {
                mlmeReq.Req.Join.Datarate = JoinAlternateDrLong(joinRequestTrials);
                mlmeReq.subband = 0;
            }

            LoRaMacStatus_t status = LoRaMacMlmeRequest(&mlmeReq);
            if (status != LORAMAC_STATUS_OK) {
                if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED) {
                    DPRINT("Duty cycle limit hit during join procedure; return join failed");
                } else if (status == LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND) {
                    DPRINT("LBT check failed, reattempt join after 5s");
                    timer_post_task_delay(&run_fsm, TIMER_TICKS_PER_SEC * 5);
                    return;
                } 
                else {
                    log_print_error_string("Unexpected error: %u, return failure. Cancel join procedure.", status);
                }
                join_state = STATE_JOIN_FAILED;
                if (stack_status_callback)
                    stack_status_callback(LORAWAN_STACK_JOIN_FAILED, nbTrials);
            }
        } else {
            DPRINT("Error while trying to join: NBTrial Joins failed in succession: %u", joinRequestTrials);
            join_state = STATE_JOIN_FAILED;
            if (short_join_enabled) {
                length = 1;
                uint8_t short_joins_failed = 0;
                res = d7ap_fs_read_file(USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 2, &short_joins_failed, &length, ROOT_AUTH);
                if (res != SUCCESS) {
                    log_print_error_string("failed to read short_joins_failed from JOINTYPE file, set to limit to "
                                           "force end of short join procedure, err code: %u.",
                        res);
                    short_joins_failed = SHORT_JOIN_ATTEMPTS_LIMIT;
                }
                short_joins_failed++;
                DPRINT("short joins, limit: %u, %u", short_joins_failed, SHORT_JOIN_ATTEMPTS_LIMIT);
                if (short_joins_failed >= SHORT_JOIN_ATTEMPTS_LIMIT) {
                    DPRINT("%u short join attempts failed in succession, reverting to long join process.",
                        short_joins_failed);
                    // write file clear to revert to long join process
                    lorawan_join_type_params_t disable_short_join
                        = { .short_join_enabled = false, .subband = 0, .short_joins_failed_counter = 0 };
                    res = d7ap_fs_write_file(USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 0, (const uint8_t*)&disable_short_join,
                        USER_FILE_LORAWAN_JOINTYPE_SIZE, ROOT_AUTH);
                    if (res != SUCCESS) {
                        log_print_error_string(
                            "failed to write USER_FILE_LORAWAN_JOINTYPE_FILE_ID file to clear contents, err code: %u.",
                            res);
                    }
                } else {
                    res = d7ap_fs_write_file(USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 2, &short_joins_failed, 1, ROOT_AUTH);
                    if (res != SUCCESS) {
                        log_print_error_string("failed to write short_joins_failed to "
                                               "USER_FILE_LORAWAN_JOINTYPE_FILE_ID file, err code: %u.",
                            res);
                    }
                }
            }
            if (stack_status_callback)
                stack_status_callback(LORAWAN_STACK_JOIN_FAILED, nbTrials);
        }
        // sched_post_task_prio(&run_fsm, MIN_PRIORITY);
        break;
    }
    case STATE_JOINED: {
        DPRINT("JOINED");
        sched_cancel_task(&run_fsm);
        // if(stack_status_callback)
        //  stack_status_callback(LORAWAN_STACK_JOINED, 0);
        break;
    }
    case STATE_JOIN_FAILED: {
        DPRINT("JOIN FAILED");
        // if(stack_status_callback)
        //  stack_status_callback(LORAWAN_STACK_JOIN_FAILED, 0);
        break;
    }
    case STATE_NOT_JOINED: {
        join_state = STATE_JOINING;
        DPRINT("Keys changed");
        sched_post_task(&run_fsm);
        break;
    }
    default: {
        assert(false);
        break;
    }
    }
}

/**
 * @brief Gets the current delay caused by the duty cycle restriction
 * This will update automatically with each function call
 * @return delay in seconds
 */
uint16_t lorawan_get_duty_cycle_delay() { return lorawanGetDutyCycleWaitTime(); }

/**
 * @brief Called from LoRaWAN stack and calls registered callback.
 * This will be called everytime a message is delayed because of duty cycle limitations.
 * Join-requests that cannot be sent because of the duty cycle do not cause this to fire;
 * instead the MlmeRequest returns DUTYCYCLE_RESTRICTED and the Join procedure returns failed.
 * @param delay
 * @param attempt: the attempt number. Indicated how many NACKS have occured
 */
static void duty_cycle_delay_cb(uint32_t delay, uint8_t attempt)
{
    uint8_t total_attempts = (app_retransmissions_counter * MAC_LAYER_RETRANSMISSIONS_LIMIT) + attempt;
    if (stack_status_callback != NULL)
        stack_status_callback(LORAWAN_STACK_DUTY_CYCLE_DELAY, total_attempts);
}

/**
 * @brief Called from LoRaWAN stack and calls registered callbacks.
 * Provides us with MAC Common Part Sublayer data after a LoRaWAN transmit
 * @param McpsConfirm
 */
static void mcps_confirm(McpsConfirm_t* McpsConfirm)
{
    DPRINT("mcps_confirm: %i", McpsConfirm->AckReceived);
    lorawan_stack_status_t status = LORAWAN_STACK_ERROR_NACK;
    bool retransmit = false;
    if (McpsConfirm != NULL) {
        if(McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
            if(McpsConfirm->McpsRequest == MCPS_UNCONFIRMED
            || (McpsConfirm->McpsRequest == MCPS_CONFIRMED && McpsConfirm->AckReceived == 1) ) {
                status = LORAWAN_STACK_ERROR_OK;
            }
        } else {
            if(McpsConfirm->McpsRequest == MCPS_CONFIRMED && McpsConfirm->AckReceived != 1) {
                if(app_retransmissions_counter < APP_LAYER_RETRANSMISSIONS_LIMIT) {
                    //perform an app-layer retransmission (i.e. increment the FCnt and retransmit up to MAC_LAYER_RETRANSMISSIONS_LIMIT times more)
                    app_retransmissions_counter++;
                    retransmit = true;
                } 
            } 
        }
    } else {
        status = LORAWAN_STACK_ERROR_UNKNOWN;
    }

    if(!retransmit) {
        tx_callback(status, McpsConfirm->NbRetries + (app_retransmissions_counter*MAC_LAYER_RETRANSMISSIONS_LIMIT));
        lorawan_transmitting = false;
    } else {
        lorawan_stack_send_attempt(true); //triggers the app-layer retranmission
    }
}

/**
 * @brief Called from LoRaWAN stack and calls registered callbacks.
 * Provides us with MAC Common Part Sublayer data after a LoRaWAN received
 * @param mcpsIndication
 */
static void mcps_indication(McpsIndication_t* mcpsIndication)
{
    if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
        DPRINT("mcps_indication status: %i", mcpsIndication->Status);
        if (join_state == STATE_JOINING) {
            DPRINT("mcpsIndication fired occurred while join procedure ongoing, retry a join");
            sched_post_task(&run_fsm);
        }
        return;
    }
    if (mcpsIndication->RxData == true) {
        DPRINT("received %i bytes for port %i", mcpsIndication->BufferSize, mcpsIndication->Port);
        app_data.Port = mcpsIndication->Port;
        app_data.BuffSize = mcpsIndication->BufferSize;
        memcpy1(app_data.Buff, mcpsIndication->Buffer, app_data.BuffSize);
        rx_callback(&app_data);
    }
}

/**
 * @brief Called from LoRaWAN stack and calls registered callbacks.
 * Provides us with MAC layer management entity data after a LoRaWAN join
 * @param mlmeConfirm
 */
static void mlme_confirm(MlmeConfirm_t* mlmeConfirm)
{
    switch (mlmeConfirm->MlmeRequest) {
    case MLME_JOIN: {
        if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
            DPRINT("join succeeded");
            joinRequestTrials = 0;

            join_state = STATE_JOINED;

            uint8_t short_join_enabled = true;
            int res = d7ap_fs_write_file(USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 0, &short_join_enabled,
                1, ROOT_AUTH);
            if (res != SUCCESS) {
                log_print_error_string("failed to enable short joins on USER_FILE_LORAWAN_JOINTYPE_FILE_ID file; long "
                                       "joins will continue to happen instead, err code: %u.",
                    res);
            }
            // note: subband gets written either from Join-Accept, or from an ADR command

            MibRequestConfirm_t mibReq;
            mibReq.Type = MIB_ANTENNA_GAIN; // MAC parameters are reset in LoRaMac-Node on every join request sent, so
                                            // this should be set after a successful join
            mibReq.Param.AntennaGain = antenna_gain_f;
            LoRaMacMibSetRequestConfirm(&mibReq);

            //if region is US915 or AU915, force use of DR_1 in following uplink
            //b/c there is a chance LNS does not allocate a 500kHz channel
            if(region == MODEM_REGION_US915 || region == MODEM_REGION_AU915) {
                datarate = MODULE_LORAWAN_MINIMUM_DATARATE;
                mibReq.Type = MIB_CHANNELS_DATARATE;
                mibReq.Param.ChannelsDatarate = MODULE_LORAWAN_MINIMUM_DATARATE;
                LoRaMacStatus_t err = LoRaMacMibSetRequestConfirm(&mibReq);
                if (err != LORAMAC_STATUS_OK) {
                    DPRINT("couldn't set DR properly");
                }
            }

            if (stack_status_callback)
                stack_status_callback(LORAWAN_STACK_JOINED, mlmeConfirm->NbRetries);
        } else if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT) {
            DPRINT("join failed because of RX2 timeout, going to attempt to retry");
            sched_post_task(&run_fsm);
        } else {
            DPRINT("Error while trying to join: %i", mlmeConfirm->Status);
            DPRINT("attempting to join again");
            sched_post_task(&run_fsm);
        }
        break;
    }
    default:
        DPRINT("mlme_confirm called for not implemented mlme request %i", mlmeConfirm->MlmeRequest);
        break;
    }
}

/**
 * @brief Called from LoRaWAN stack and calls registered callbacks.
 * @param mlmeIndication
 */
static void mlme_indication(MlmeIndication_t* mlmeIndication)
{
    if (mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
        DPRINT("mlme_indication status: %i", mlmeIndication->Status);
        return;
    }
}

/**
 * @brief Checks if we have joined a LoRaWAN network
 * @return bool representing the joined status
 */
static bool is_joined()
{
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_NETWORK_ACTIVATION;
    LoRaMacMibGetRequestConfirm(&mibReq);

    // MIB_NETWORK_JOINED has become MIB_NETWORK_ACTIVATION, indicates whether activation type used was OTAA or ABP. If
    // non-zero, the device has joined.
    return (mibReq.Param.NetworkActivation != 0);
}

/**
 * @brief updates the otaa keys
 * @param file_id
 */
static void lorawan_otaa_register_keys(uint8_t file_id)
{
    DPRINT("OTAA Register Keys:");
    bool keys_changed = false;
    bool was_joining = (join_state == STATE_JOINING);
    uint8_t keys[USER_FILE_LORAWAN_KEYS_SIZE];
    uint32_t length = USER_FILE_LORAWAN_KEYS_SIZE;
    d7ap_fs_read_file(USER_FILE_LORAWAN_KEYS_FILE_ID, 0, keys, &length, ROOT_AUTH);
    if (memcmp(appEui, keys, 8) != 0) {
        join_state = STATE_NOT_JOINED;
        memcpy(appEui, keys, 8);
        keys_changed = true;
    }

    if (memcmp(appKey, &keys[8], 16) != 0) {
        join_state = STATE_NOT_JOINED;
        memcpy(appKey, &keys[8], 16);
        DPRINT("AppKey:");
        DPRINT_DATA(appKey, 16);
        keys_changed = true;
    }
    if (keys_changed && was_joining) {
        lorawan_stack_deinit();
        lorawan_stack_init_otaa();
        if (stack_status_callback)
            stack_status_callback(LORAWAN_STACK_JOIN_FAILED, 1);
    }
}

/**
 * @brief init lorawan, this has to be used only once
 */
static void set_initial_keys()
{
    first_init = false;

    uint32_t length = D7A_FILE_UID_SIZE;
    d7ap_fs_read_file(D7A_FILE_UID_FILE_ID, 0, devEui, &length, ROOT_AUTH);

    d7ap_fs_register_file_modified_callback(USER_FILE_LORAWAN_KEYS_FILE_ID, &lorawan_otaa_register_keys);
    lorawan_otaa_register_keys(USER_FILE_LORAWAN_KEYS_FILE_ID);
}

/**
 * @brief set the antenna gain based on the contents of a file
 * this function gets called after every write of the file so the file can be effectively used to set the max tx power
 */
static void lorawan_set_antenna_gain(uint8_t file_id)
{
    int8_t antenna_gain;
    uint32_t length = USER_FILE_LORAWAN_ANTENNA_GAIN_SIZE;
    int res = d7ap_fs_read_file(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID, 0, (uint8_t*)&antenna_gain, &length, ROOT_AUTH);

    if (res == -ENOENT) {
        // file does not exist yet (older filesystem version), create it
        antenna_gain = MODULE_LORAWAN_DEFAULT_ANTENNA_GAIN;

        uint8_t antenna_gain_file[1] = {
            (uint8_t)antenna_gain,
        };

        d7ap_fs_file_header_t file_header
            = { .file_permissions = (file_permission_t) { .guest_read = true, .user_read = true },
                  .file_properties.storage_class = FS_STORAGE_PERMANENT,
                  .length = USER_FILE_LORAWAN_ANTENNA_GAIN_SIZE,
                  .allocated_length = USER_FILE_LORAWAN_ANTENNA_GAIN_SIZE };

        // initialize file on fs
        int ret = d7ap_fs_init_file(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID, &file_header, antenna_gain_file);
    }

    antenna_gain_f = (float)antenna_gain;

    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_ANTENNA_GAIN;
    mibReq.Param.AntennaGain = antenna_gain_f;
    LoRaMacMibSetRequestConfirm(&mibReq);
}

static void subband_file_init()
{
    uint32_t length = USER_FILE_LORAWAN_JOINTYPE_SIZE;
    lorawan_join_type_params_t disable_short_join
        = { .short_join_enabled = false, .subband = 0, .short_joins_failed_counter = 0 };
    int res
        = d7ap_fs_read_file(USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 0, (uint8_t*)&disable_short_join, &length, ROOT_AUTH);

    if (res == -ENOENT) {
        DPRINT("initialise subband file");
        // if the file doesn't already exist (first init after reboot), create it
        d7ap_fs_file_header_t volatile_file_header
            = { .file_permissions = (file_permission_t) { .guest_write = true, .user_write = true },
                  .file_properties.storage_class = FS_STORAGE_VOLATILE,
                  .length = USER_FILE_LORAWAN_JOINTYPE_SIZE,
                  .allocated_length = USER_FILE_LORAWAN_JOINTYPE_SIZE };

        res = d7ap_fs_init_file(
            USER_FILE_LORAWAN_JOINTYPE_FILE_ID, &volatile_file_header, (uint8_t*)&disable_short_join);
        assert(res == SUCCESS);
    } else if (res == SUCCESS) {
        DPRINT("read subband file successfully, it already existed");
    } else {
        log_print_error_string("Unexpected error when creating subband file: %u.", res);
    }
}

/**
 * @brief Register the different callbacks
 * @param lorawan_rx_cb: LoRaWAN received data
 * @param lorawan_tx_cb: LoRaWAN transmitted data
 * @param join_completed_cb: LoRaWAN network joined
 * @param lorawan_duty_cycle_delay_cb: LoRaWAN delayed because of duty cycle
 * @param lorawan_join_attempt_cb: attempt to join network
 */
void lorawan_register_cbs(lorawan_rx_callback_t lorawan_rx_cb, lorawan_tx_completed_callback_t lorawan_tx_cb,
    lorawan_status_callback_t lorawan_status_cb)
{
    rx_callback = lorawan_rx_cb;
    tx_callback = lorawan_tx_cb;
    stack_status_callback = lorawan_status_cb;
}

/**
 * @brief Check if there are changes to the network config. Updates some parameters
 * that can be adjusted on the fly.
 * @param lorawan_session_config
 * @return bool represents if the LoRaWAN network is still joined
 */
lorawan_stack_status_t lorawan_otaa_is_joined(lorawan_session_config_otaa_t* lorawan_session_config)
{
    if (inited == false) {
        log_print_error_string("TX not possible, not inited"); // Should not happen when using alp layer
        return LORAWAN_STACK_ERROR_NOT_INITED;
    }
    DPRINT("Checking for change in config");
    if (join_state == STATE_JOINING)
        return LORAWAN_STACK_ALREADY_JOINING;

    bool joined = (join_state == STATE_JOINED);
    sched_cancel_task(&run_fsm);
    datarate = lorawan_session_config->data_rate;

    if (adr_enabled != lorawan_session_config->adr_enabled) {
        adr_enabled = lorawan_session_config->adr_enabled;
        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_ADR;
        mibReq.Param.AdrEnable = adr_enabled;
        LoRaMacMibSetRequestConfirm(&mibReq);
    }

    if (!joined) {
        LoRaMacStatus_t status = LORAMAC_STATUS_OK;
        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_NETWORK_ACTIVATION;
        mibReq.Param.NetworkActivation = ACTIVATION_TYPE_NONE;
        status = LoRaMacMibSetRequestConfirm(&mibReq);
        // note: for otaa, activation type gets set to ACTIVATION_TYPE_OTAA by the MAC layer on a successful join.
        if (status != LORAMAC_STATUS_OK) {
            assert(false);
        }

        DPRINT("Change found - Join using OTAA");
        DPRINT("DevEui:");
        DPRINT_DATA(devEui, 8);
        DPRINT("AppEui:");
        DPRINT_DATA(appEui, 8);
        DPRINT("AppKey:");
        DPRINT_DATA(appKey, 16);
        DPRINT("Adaptive Data Rate: %d, Data rate: %d", adr_enabled, datarate);

        if (sched_post_task(&run_fsm) == SUCCESS) {
            join_state = STATE_JOINING;
            joinRequestTrials = 0;
        }
    }
    return joined ? LORAWAN_STACK_ERROR_OK : LORAWAN_STACK_ERROR_NOT_JOINED;
}

static LoRaMacRegion_t lorawan_get_region()
{
    switch (region) {
    case MODEM_REGION_AS923_1_DUTY_CYCLE: {
        return LORAMAC_REGION_AS923;
    }
    case MODEM_REGION_AU915: {
        return LORAMAC_REGION_AU915;
    }
    case MODEM_REGION_EU868: {
        return LORAMAC_REGION_EU868;
    }
    case MODEM_REGION_IN865: {
        return LORAMAC_REGION_IN865;
    }
    case MODEM_REGION_US915: {
        return LORAMAC_REGION_US915;
    }
    case MODEM_REGION_CN470: {
        return LORAMAC_REGION_CN470;
    }
    case MODEM_REGION_KR920: {
        return LORAMAC_REGION_KR920;
    }
    case MODEM_REGION_CN779:
    case MODEM_REGION_EU433:
    case MODEM_REGION_RU864:
    case MODEM_REGION_AS923_1_DUTY_CYCLE_DWELL_TIME:
    case MODEM_REGION_AS923_1_NO_RESTRICTIONS:
    case MODEM_REGION_AS923_2:
    case MODEM_REGION_AS923_3:
    case MODEM_REGION_AS923_4: {
        log_print_error_string("Error: Unsupported region: %u", region);
        assert(false);
        break;
    }
    default: {
        log_print_error_string("Error: return default");
        break;
    }
    }
    return 0;
}

/**
 * @brief Inits the LoRaWAN stack using over the air activation
 * @param lorawan_session_config
 */
error_t lorawan_stack_init_otaa()
{
    if (inited)
        return EALREADY;
    if (first_init) {
        set_initial_keys();
        lorawan_set_antenna_gain(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID);
        subband_file_init();
    }

    HW_Init(); // TODO refactor*/
    join_state = STATE_NOT_JOINED;
    lorawan_transmitting = false;
    sched_register_task(&run_fsm);

    loraMacPrimitives.MacMcpsConfirm = &mcps_confirm;
    loraMacPrimitives.MacMcpsIndication = &mcps_indication;
    loraMacPrimitives.MacMlmeConfirm = &mlme_confirm;
    loraMacPrimitives.MacMlmeIndication = &mlme_indication;
    loraMacPrimitives.MacDutyDelay = &duty_cycle_delay_cb;
    loraMacPrimitives.MacRetryTransmission = &network_retry_transmission;
    loraMacPrimitives.MacSubbandChanged = &subband_changed;

    // these callbacks are used by the LoRaMac to get the DevEui and AppEui when needed
    // in older versions the keys were provided with each join request. In current LoRaMac-node these are saved in an
    // emulated secure element but in order to avoid duplication, we instead save them here and provide LoRaMac
    // callbacks to access them
    loraMacCallbacks.GetDevEui = &lorawan_get_deveui;
    loraMacCallbacks.GetAppEui = &lorawan_get_appeui;

    loraMacStatus = LoRaMacInitialization(&loraMacPrimitives, &loraMacCallbacks, lorawan_get_region());
    if (loraMacStatus == LORAMAC_STATUS_OK) {
        DPRINT("init OK");
    } else {
        DPRINT("init failed %d", loraMacStatus);
        return -FAIL;
    }

    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_DEVICE_CLASS;
    mibReq.Param.Class = LORAWAN_CLASS;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_NWK_KEY; // note: the naming conventions in LoRaMac-node follows the LoRaWAN 1.1 naming
                               // convention, but the security used in the version we fork is still 1.0.3
    mibReq.Param.NwkKey = appKey;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_DEFAULT_ANTENNA_GAIN;
    mibReq.Param.DefaultAntennaGain = MODULE_LORAWAN_DEFAULT_ANTENNA_GAIN;
    LoRaMacMibSetRequestConfirm(&mibReq);

#if defined(REGION_EU868)
    LoRaMacTestSetDutyCycleOn(true);

#if (USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1)
    LoRaMacChannelAdd(3, (ChannelParams_t)LC4);
    LoRaMacChannelAdd(4, (ChannelParams_t)LC5);
    LoRaMacChannelAdd(5, (ChannelParams_t)LC6);
    LoRaMacChannelAdd(6, (ChannelParams_t)LC7);
    LoRaMacChannelAdd(7, (ChannelParams_t)LC8);
    LoRaMacChannelAdd(8, (ChannelParams_t)LC9);
    LoRaMacChannelAdd(9, (ChannelParams_t)LC10);

    mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
    mibReq.Param.Rx2DefaultChannel = (Rx2ChannelParams_t) { 869525000, DR_3 };
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_RX2_CHANNEL;
    mibReq.Param.Rx2Channel = (Rx2ChannelParams_t) { 869525000, DR_3 };
    LoRaMacMibSetRequestConfirm(&mibReq);
#endif

#endif

    LoRaMacStart(); // start up the LoRaMac (change from default state, which is LORAMAC_STOPPED)

    sched_register_task(&LoRaMacProcess);

    d7ap_fs_register_file_modified_callback(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID, &lorawan_set_antenna_gain);

    inited = true;

    return SUCCESS;
}

/**
 * @brief Deinitialize the LoRaWAN stack
 * @param lorawan_session_config
 */
void lorawan_stack_deinit()
{
    if (!inited)
        return;
    inited = false;
    DPRINT("Deiniting LoRaWAN stack");
    sched_cancel_task(&run_fsm);
    LoRaMacDeInitialization();
    join_state = STATE_NOT_JOINED;
    lorawan_transmitting = false;
    HW_DeInit();
    adr_enabled = false;
    d7ap_fs_unregister_file_modified_callback(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID);
}

lorawan_stack_status_t lorawan_stack_send_attempt(bool request_ack)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    if (LoRaMacQueryTxPossible(app_data.BuffSize, &txInfo) != LORAMAC_STATUS_OK) {
        if (app_data.BuffSize > txInfo.CurrentPossiblePayloadSize) {
            // payload size is too big for frame, cannot send
            DPRINT("TX not possible, max payloadsize %i, trying to transmit %i", txInfo.CurrentPossiblePayloadSize,
                app_data.BuffSize);
            return LORAWAN_STACK_ERROR_TX_NOT_POSSIBLE;
        } else {
            // payload size + MAC commands is too big,
            // Send empty frame in order to flush MAC commands
            DPRINT("TX not possible, max payloadsize %i, trying to transmit %i and %i bytes of MAC commands",
                txInfo.CurrentPossiblePayloadSize, app_data.BuffSize,
                txInfo.CurrentPossiblePayloadSize - txInfo.MaxPossibleApplicationDataSize);
            DPRINT("Flush MAC commands");
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fBuffer = NULL;
            mcpsReq.Req.Unconfirmed.fBufferSize = 0;
            mcpsReq.Req.Unconfirmed.Datarate = datarate;
            LoRaMacMcpsRequest(&mcpsReq);
            return LORAWAN_STACK_ALREADY_TRANSMITTING;
        }
    }

    if (!request_ack) {
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fPort = app_data.Port;
        mcpsReq.Req.Unconfirmed.fBuffer = app_data.Buff;
        mcpsReq.Req.Unconfirmed.fBufferSize = app_data.BuffSize;
        mcpsReq.Req.Unconfirmed.Datarate = datarate;
    } else {
        mcpsReq.Type = MCPS_CONFIRMED;
        mcpsReq.Req.Confirmed.fPort = app_data.Port;
        mcpsReq.Req.Confirmed.fBuffer = app_data.Buff;
        mcpsReq.Req.Confirmed.fBufferSize = app_data.BuffSize;
        mcpsReq.Req.Confirmed.NbTrials = MAC_LAYER_RETRANSMISSIONS_LIMIT;
        mcpsReq.Req.Confirmed.Datarate = datarate;
    }

    LoRaMacStatus_t status = LoRaMacMcpsRequest(&mcpsReq);
    if (status != LORAMAC_STATUS_OK) {
        //  state = STATE_SLEEP;
        DPRINT("failed sending data (status %i)", status);
        return LORAWAN_STACK_ERROR_UNKNOWN;
    }

    return LORAWAN_STACK_ERROR_OK;
}

/**
 * @brief Sends data using LoRaWAN
 * @param payload
 * @param length
 * @param app_port
 * @param request_ack
 * @return lorawan stack status
 */
lorawan_stack_status_t lorawan_stack_send(uint8_t* payload, uint8_t length, uint8_t app_port, bool request_ack)
{
    DPRINT("LoRaWAN stack send");
    if (inited == false) {
        log_print_error_string("TX not possible, not inited"); // Should not happen when using alp layer
        return LORAWAN_STACK_ERROR_NOT_INITED;
    }
    if (!is_joined()) {
        log_print_error_string("TX not possible, not joined"); // Should not happen when using alp layer
        return LORAWAN_STACK_ERROR_NOT_JOINED;
    }

    if (lorawan_transmitting) {
        DPRINT("TX not possible, already transmitting");
        return LORAWAN_STACK_ALREADY_TRANSMITTING;
    }

    if (length > LORAWAN_APP_DATA_BUFF_SIZE)
        return LORAWAN_STACK_ERROR_TX_NOT_POSSIBLE;

    memcpy1(app_data.Buff, payload, length);
    app_data.BuffSize = length;
    app_data.Port = app_port;

    lorawan_stack_status_t status =  lorawan_stack_send_attempt(request_ack);

    if(status == LORAWAN_STACK_ERROR_OK) {
        lorawan_transmitting = true;
        app_retransmissions_counter = 0;
    }
    return status;
}


/**
 * @brief returns saved devEui
 *
 * @return lorawan devEui
 */
uint8_t* lorawan_get_deveui(void) { return devEui; }

/**
 * @brief returns saved AppEui
 *
 * @return lorawan appEui (in LoRaWAN 1.1, name is changed to   joinEui)
 */
uint8_t* lorawan_get_appeui(void) { return appEui; }

/**
 * @brief returns saved subband
 *
 * @return the saved subband
 */
static uint8_t lorawan_get_subband(void)
{
    uint32_t length = 1;
    uint8_t subband = 0;
    int res = d7ap_fs_read_file(USER_FILE_LORAWAN_JOINTYPE_FILE_ID, 1, (uint8_t*)&subband, &length, ROOT_AUTH);
    if (res != SUCCESS) {
        log_print_error_string("failed to read subband from JOINTYPE file, assume it is 0, err code: %u.", res);
        subband = 0;
    }
    return subband;
}
