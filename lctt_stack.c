/* OSS-7 - An opensource implementation of the DASH7 Alliance Protocol for ultra
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
 * LCTT (LoRaWAN Certification Test Tool) Stack Implementation
 * This file contains the LCTT-specific implementation when MODULE_LORAWAN_LCTT_TEST_MODE=y
 */

#include "lorawan_stack.h"
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "MODULE_LORAWAN_defs.h"
#include "d7ap_fs.h"
#include "debug.h"
#include "errors.h"
#include "hw.h"
#include "hw_msp.h"
#include "modem_region.h"
#include "scheduler.h"
#include "timer.h"

#include "LmHandler.h"
#include "LmhpClockSync.h"
#include "LmhpCompliance.h"

#if defined(MODULE_LORAWAN_LOG_ENABLED)
#define DPRINT(...) log_print_stack_string(LOG_STACK_ALP, __VA_ARGS__)
#define DPRINT_DATA(p, n) log_print_data(p, n)
#else
#define DPRINT(...)
#define DPRINT_DATA(p, n)
#endif

#define LORAWAN_PUBLIC_NETWORK_ENABLED 1
#define LORAWAN_CLASS CLASS_A
#define LORAWAN_APP_DATA_BUFF_SIZE 242

const modem_region_t region = MODULE_LORAWAN_REGION; // TODO: make AS923_x configurable

static uint8_t devEui[8] = { 0 }; // used for OTAA
static uint8_t appEui[8] = { 0 }; // used for OTAA
static uint8_t appKey[16] = { 0 };

bool adr_enabled = false;
uint8_t datarate = 0;

static uint8_t payload_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lorawan_AppData_t app_data = { payload_data_buffer, 0, 0 };

static lorawan_rx_callback_t rx_callback = NULL; // called when transmitting is done
static lorawan_tx_completed_callback_t tx_callback = NULL;
static lorawan_status_callback_t stack_status_callback = NULL;

static bool inited = false;
static bool first_init = true;

static bool yet_to_send_first_frame = true;

static LmHandlerParams_t LmHandlerParams = { .Region = MODULE_LORAWAN_REGION,
    .AdrEnable = 0, // by default off, gets set during init
    .TxDatarate = 0, // by default 0, gets set during init
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK_ENABLED,
    .DutyCycleEnabled = 0, // TODO: ensure off is ok for test procedure
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFF_SIZE,
    .DataBuffer = payload_data_buffer };

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxCon = LORAMAC_HANDLER_CONFIRMED_MSG;

/*
 * Indicates if the system time has been synchronized
 */
static volatile bool IsClockSynched = false;

/**
 * handles join procedure in lctt mode
 */
static void lorawan_stack_lctt_on_join_request(LmHandlerJoinParams_t* params)
{
    if (params->Status == LORAMAC_HANDLER_ERROR) {
        DPRINT("join failed, try join again");
        LmHandlerJoin();
    } else {
        DPRINT("join success!");
        LmHandlerRequestClass(LORAWAN_CLASS);
    }
}

#define APP_TX_DUTYCYCLE 240 // the application data transmission duty cycle (s)
#define APP_TX_DUTYCYCLE_RND 5 // a random delay for application data transmission duty cycle (s)

static volatile uint8_t IsTxFramePending = 0;
static volatile uint32_t TxPeriodicity = TIMER_TICKS_PER_SEC * APP_TX_DUTYCYCLE;

/*!
 * schedules transmission of an uplink frame in lctt mode
 */
static void lorawan_stack_lctt_on_tx_timer_event(void* context)
{
    // DPRINT("Timer ticked, schedule a packet");
    timer_cancel_task(&lorawan_stack_lctt_on_tx_timer_event);
    IsTxFramePending = 1;

    // Schedule next transmission
    timer_post_task_delay(&lorawan_stack_lctt_on_tx_timer_event, TxPeriodicity);
}

/**
 * transmit an uplink frame in lctt mode
 */
static void lorawan_stack_lctt_uplink_process(void)
{
    LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

    if (LmHandlerIsBusy() == true) {
        return;
    }

    uint8_t isPending = 0;
    BACKUP_PRIMASK();
    DISABLE_IRQ();
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    RESTORE_PRIMASK();
    if (isPending == 1) {
        if (IsClockSynched == false) {
            status = LmhpClockSyncAppTimeReq();
        } else {
            DPRINT("send a random one byte payload");
            payload_data_buffer[0] = randr(0, 255);
            // Send random one byte payload
            LmHandlerAppData_t appData = {
                .Buffer = payload_data_buffer,
                .BufferSize = 1,
                .Port = 1,
            };
            status = LmHandlerSend(&appData, IsTxCon); // needed? or only at the start?
        }
    }
}

// the callbacks that return void are used in other apps just to display data
static uint8_t lorawan_stack_lctt_get_battery_level(void) { return 0; }
static float lorawan_stack_lctt_get_temperature(void) { return 0.0; }
static void lorawan_stack_lctt_on_mac_process_notify(void) { return; }
static void lorawan_stack_lctt_on_nvm_data_change(LmHandlerNvmContextStates_t state) { return; }
static void lorawan_stack_lctt_on_network_parameters_change(CommissioningParams_t* params) { return; }
static void lorawan_stack_lctt_on_mac_mcps_request(LoRaMacStatus_t status, McpsReq_t* mcpsReq, TimerTime_t nextTxDelay)
{
    return;
}
static void lorawan_stack_lctt_on_mac_mlme_request(LoRaMacStatus_t status, MlmeReq_t* mlmeReq, TimerTime_t nextTxDelay)
{
    return;
}
static void lorawan_stack_lctt_on_tx_data(LmHandlerTxParams_t* params) { return; }
static void lorawan_stack_lctt_on_rx_data(LmHandlerAppData_t* appData, LmHandlerRxParams_t* params) { 
    DPRINT("Packet received!");
    if(params->Status == LORAMAC_EVENT_INFO_STATUS_OK && appData == NULL) {
        DPRINT("Port: %u", appData->Port);
        yet_to_send_first_frame = false;
    }
}
static void lorawan_stack_lctt_on_class_change(DeviceClass_t deviceClass) { return; } // only supporting Class A
static void lorawan_stack_lctt_on_beacon_status_change(LoRaMAcHandlerBeaconParams_t* params) { return; }
static void lorawan_stack_lctt_on_sys_time_update(bool isSynchronized, int32_t timeCorrection)
{
    DPRINT("Clock synced!");
    IsClockSynched = isSynchronized;
}

static LmHandlerCallbacks_t LmHandlerCallbacks = { .GetBatteryLevel = lorawan_stack_lctt_get_battery_level,
    .GetTemperature = lorawan_stack_lctt_get_temperature,
    .GetRandomSeed = HW_GetRandomSeed,
    .OnMacProcess = lorawan_stack_lctt_on_mac_process_notify,
    .OnNvmContextChange = lorawan_stack_lctt_on_nvm_data_change,
    .OnNetworkParametersChange = lorawan_stack_lctt_on_network_parameters_change,
    .OnMacMcpsRequest = lorawan_stack_lctt_on_mac_mcps_request,
    .OnMacMlmeRequest = lorawan_stack_lctt_on_mac_mlme_request,
    .OnJoinRequest = lorawan_stack_lctt_on_join_request,
    .OnTxData = lorawan_stack_lctt_on_tx_data,
    .OnRxData = lorawan_stack_lctt_on_rx_data,
    .OnClassChange = lorawan_stack_lctt_on_class_change,
    .OnBeaconStatusChange = lorawan_stack_lctt_on_beacon_status_change,
    .OnSysTimeUpdate = lorawan_stack_lctt_on_sys_time_update };

static LmhpComplianceParams_t LmhpComplianceParams = { .AdrEnabled = false, // gets set during init
    .DutyCycleEnabled = false,
    .StopPeripherals = NULL,
    .StartPeripherals = NULL };

static void lorawan_otaa_lctt_process_loop()
{
    // Processes the LoRaMac events
    LmHandlerProcess();

    // Process application uplinks management
    if (yet_to_send_first_frame)
        lorawan_stack_lctt_uplink_process();

    // TODO: in current implementation there is no low power mode
    sched_post_task(&lorawan_otaa_lctt_process_loop);
}

/**
 * @brief init lorawan, this has to be used only once
 */
static void set_initial_keys()
{
    first_init = false;

    uint32_t length = D7A_FILE_UID_SIZE;
    d7ap_fs_read_file(D7A_FILE_UID_FILE_ID, 0, devEui, &length, ROOT_AUTH);

    // In LCTT mode, use hardcoded keys for easier test configuration
    uint8_t hardcodedDevEui[8] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80 };
    memcpy(devEui, hardcodedDevEui, sizeof(devEui));

    // Register callback for key changes (even though LCTT uses hardcoded keys)
    uint8_t keys[USER_FILE_LORAWAN_KEYS_SIZE];
    uint32_t key_length = USER_FILE_LORAWAN_KEYS_SIZE;
    d7ap_fs_read_file(USER_FILE_LORAWAN_KEYS_FILE_ID, 0, keys, &key_length, ROOT_AUTH);
    memcpy(appEui, keys, 8);
    memcpy(appKey, &keys[8], 16);
}

/**
 * @brief Check if there are changes to the network config. Updates some parameters
 * that can be adjusted on the fly.
 * @param lorawan_session_config
 * @return bool represents if the LoRaWAN network is still joined
 */
lorawan_stack_status_t lorawan_otaa_is_joined(lorawan_session_config_otaa_t* lorawan_session_config)
{
    adr_enabled = lorawan_session_config->adr_enabled;
    LmHandlerParams.AdrEnable = lorawan_session_config->adr_enabled;

    datarate = lorawan_session_config->data_rate;
    LmHandlerParams.TxDatarate = lorawan_session_config->data_rate;

    LmhpComplianceParams.AdrEnabled = lorawan_session_config->adr_enabled;

    DPRINT("Starting in LoRaWAN Precertification Test Tool Mode!");
    DPRINT("DevEui:");
    DPRINT_DATA(devEui, 8);
    DPRINT("AppEui:");
    DPRINT_DATA(appEui, 8);
    DPRINT("AppKey:");
    DPRINT_DATA(appKey, 16);
    DPRINT("Adaptive Data Rate: %d, Data rate: %d", adr_enabled, datarate);

    LmHandlerJoin(); // if this fails, the onJoinRequest callback will be called, and the join will be retried

    // start tx process
    sched_register_task(&lorawan_stack_lctt_on_tx_timer_event);
    sched_post_task(&lorawan_stack_lctt_on_tx_timer_event);

    sched_register_task(&lorawan_otaa_lctt_process_loop);
    sched_post_task(&lorawan_otaa_lctt_process_loop);

    return !LmHandlerJoinStatus(); // when JoinStatus is 1, device is joined. interface expects a 0 to indicate no
                                   // error.
}

/**
 * @brief Inits the LoRaWAN stack using over the air activation (LCTT mode)
 */
error_t lorawan_stack_init_otaa()
{
    if (inited)
        return EALREADY;
    if (first_init) {
        set_initial_keys();
    }

    HW_Init();

    sched_register_task(&LoRaMacProcess);

    uint8_t hardcodedDevEui[8] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80 };
    memcpy(devEui, hardcodedDevEui, sizeof(devEui)); // in LCTT mode, use set DEVEUI to ease configuration of test tools

    if (LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams, appKey) != LORAMAC_HANDLER_SUCCESS) {
        DPRINT("LoRaMac wasn't properly initialized");
        return -FAIL;
    }

    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_DEFAULT_ANTENNA_GAIN;
    mibReq.Param.DefaultAntennaGain = MODULE_LORAWAN_DEFAULT_ANTENNA_GAIN;
    DPRINT("init, set default antenna gain to be %u", MODULE_LORAWAN_DEFAULT_ANTENNA_GAIN);
    LoRaMacMibSetRequestConfirm(&mibReq);
    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError(50);

    // this should not be NULL?
    LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE,
        &LmhpComplianceParams); 

    LmHandlerPackageRegister(PACKAGE_ID_CLOCK_SYNC, NULL); 

    IsClockSynched = true;

    inited = true;

    LoRaMacTestSetDutyCycleOn(false);

    DPRINT("Inited in LCTT test mode");

    return SUCCESS;
}

/**
 * @brief Deinitialize the LoRaWAN stack (LCTT mode)
 */
void lorawan_stack_deinit()
{
    if (!inited)
        return;
    inited = false;
    DPRINT("Deiniting LoRaWAN stack (LCTT mode)");
    sched_cancel_task(&lorawan_otaa_lctt_process_loop);
    LoRaMacDeInitialization();
    HW_DeInit();
    adr_enabled = false;
}

/**
 * @brief Sends data using LoRaWAN (LCTT mode)
 * @param payload
 * @param length
 * @param app_port
 * @param request_ack
 * @return lorawan stack status
 */
lorawan_stack_status_t lorawan_stack_send(uint8_t* payload, uint8_t length, uint8_t app_port, bool request_ack)
{
    if (inited == false) {
        DPRINT("TX not possible, not inited"); 
        return LORAWAN_STACK_ERROR_NOT_INITED;
    }
    
    if (!LmHandlerJoinStatus()) {
        DPRINT("TX not possible, not joined"); 
        return LORAWAN_STACK_ERROR_NOT_JOINED;
    }

    if (LmHandlerIsBusy()) {
        DPRINT("TX not possible, already transmitting");
        return LORAWAN_STACK_ALREADY_TRANSMITTING;
    }

    if (length > LORAWAN_APP_DATA_BUFF_SIZE)
        return LORAWAN_STACK_ERROR_TX_NOT_POSSIBLE;

    // Prepare the application data
    LmHandlerAppData_t appData = {
        .Buffer = payload,
        .BufferSize = length,
        .Port = app_port,
    };

    LmHandlerErrorStatus_t status = LmHandlerSend(&appData, request_ack ? LORAMAC_HANDLER_CONFIRMED_MSG : LORAMAC_HANDLER_UNCONFIRMED_MSG);
    
    if (status != LORAMAC_HANDLER_SUCCESS) {
        DPRINT("failed sending data (status %i)", status);
        return LORAWAN_STACK_ERROR_UNKNOWN;
    }

    return LORAWAN_STACK_ERROR_OK;
}

/**
 * @brief Register the different callbacks (LCTT mode)
 */
void lorawan_register_cbs(lorawan_rx_callback_t lorawan_rx_cb, lorawan_tx_completed_callback_t lorawan_tx_cb,
    lorawan_status_callback_t lorawan_status_cb)
{
    rx_callback = lorawan_rx_cb;
    tx_callback = lorawan_tx_cb;
    stack_status_callback = lorawan_status_cb;
}

/**
 * @brief returns saved devEui (LCTT mode)
 */
uint8_t* lorawan_get_deveui(void) { return devEui; }

/**
 * @brief returns saved AppEui (LCTT mode)
 */
uint8_t* lorawan_get_appeui(void) { return appEui; }

/**
 * @brief Gets the current delay caused by the duty cycle restriction (LCTT mode)
 */
uint16_t lorawan_get_duty_cycle_delay() { return 0; } // In LCTT mode, duty cycle is typically disabled