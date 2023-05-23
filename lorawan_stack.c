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
#include "hw.h"
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "debug.h"
#include "scheduler.h"
#include "MODULE_LORAWAN_defs.h"
#include "d7ap_fs.h"
#include "errors.h"

#if defined(MODULE_LORAWAN_LOG_ENABLED) 
#define DPRINT(...) log_print_stack_string(LOG_STACK_ALP, __VA_ARGS__)
#define DPRINT_DATA(p, n) log_print_data(p, n)
#else
#define DPRINT(...)
#define DPRINT_DATA(p, n)
#endif

// TODO configurable
#define LORAWAN_PUBLIC_NETWORK_ENABLED              1 // TODO configurable?
#define LORAWAN_CLASS                               CLASS_A // TODO configurable?
#define JOINREQ_NBTRIALS                            48 // (>=48 according to spec)
#define LORAWAN_APP_DATA_BUFF_SIZE                  222 // TODO = max?

#define EU868 LORAMAC_REGION_EU868
#define US915 LORAMAC_REGION_US915
#define AS923 LORAMAC_REGION_AS923

const LoRaMacRegion_t region = MODULE_LORAWAN_REGION;

typedef enum
{
  STATE_NOT_JOINED,
  STATE_JOINED,
  STATE_JOIN_FAILED,
  STATE_JOINING
} join_state_t;

static join_state_t join_state = STATE_NOT_JOINED;
static LoRaMacPrimitives_t loraMacPrimitives;
static LoRaMacCallback_t loraMacCallbacks;
static LoRaMacStatus_t loraMacStatus;
static uint8_t devEui[8] = { 0 };     //used for OTAA
static uint8_t appEui[8] = { 0 };     //used for OTAA
static uint8_t appKey[16] = { 0 };    

static uint8_t app_port;
static bool request_ack;
bool adr_enabled = false;
uint8_t datarate = 0; 

static bool use_confirmed_tx = false;

static uint8_t payload_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lorawan_AppData_t app_data = { payload_data_buffer, 0 ,0 };

static lorawan_rx_callback_t rx_callback = NULL; //called when transmitting is done
static lorawan_tx_completed_callback_t tx_callback = NULL;
static lorawan_status_callback_t stack_status_callback = NULL;

static bool inited = false;
static bool first_init = true;
static bool lorawan_transmitting = false;

static uint8_t joinRequestTrials = 0;

static float antenna_gain_f = 0.0;

/**
 * @brief Called everytime a LoRaWAN retransmission is executed.
 * This will be executed when joining or when nacks are received when an ack was requested
 * @param join_attempt_number
 */
static void network_retry_transmission(uint8_t attempt)
{
  if(stack_status_callback != NULL)
    stack_status_callback(LORAWAN_STACK_RETRY_TRANSMISSION, attempt);
}

/**
 * @brief LoRaWAN state machine. Sets parameters in the LoRaWAN stack and handles callbacks
 */
static void run_fsm()
{
  switch(join_state)
  {
    case STATE_JOINING:
    {
      joinRequestTrials++;

      if(joinRequestTrials < JOINREQ_NBTRIALS) {

          if(joinRequestTrials > 1)
          {
            DPRINT("Nack so trying to join again: %d", joinRequestTrials);
            network_retry_transmission(joinRequestTrials); //note that it is possible that there will be a delay (it is only calculated in ScheduleTx) 
            // if there is a delay, then the MacDutyDelay function will be called, which calls the duty_cycle_delay_cb function.
          }

          MlmeReq_t mlmeReq;
          mlmeReq.Type = MLME_JOIN;
          mlmeReq.Req.Join.Datarate = datarate; 
          LoRaMacMlmeRequest(&mlmeReq);
      } else {
          DPRINT("Error while trying to join: NBTrial Joins failed in succession");
          join_state = STATE_JOIN_FAILED;
          if(stack_status_callback)
            stack_status_callback(LORAWAN_STACK_JOIN_FAILED, JOINREQ_NBTRIALS);
      }
      //sched_post_task_prio(&run_fsm, MIN_PRIORITY);
      break;
    }
    case STATE_JOINED:
    {
      DPRINT("JOINED");
      sched_cancel_task(&run_fsm);
      //if(stack_status_callback)
      //  stack_status_callback(LORAWAN_STACK_JOINED, 0);
      break;
    }
    case STATE_JOIN_FAILED:
    {
      DPRINT("JOIN FAILED");
      //if(stack_status_callback)
      //  stack_status_callback(LORAWAN_STACK_JOIN_FAILED, 0);
      break;
    }
     case STATE_NOT_JOINED:
    {
      join_state = STATE_JOINING;
      DPRINT("Keys changed");
      sched_post_task(&run_fsm);
      break;
    }
    default:
    {
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
uint16_t lorawan_get_duty_cycle_delay()
{
  return lorawanGetDutyCycleWaitTime();
}

/**
 * @brief Called from LoRaWAN stack and calls registered callback. 
 * This will be called everytime a message is delayed because of duty cycle limitations.
 * @param delay
 * @param attempt: the attempt number. Indicated how many NACKS have occured
 */
static void duty_cycle_delay_cb(uint32_t delay, uint8_t attempt)
{
  if(stack_status_callback != NULL)
    stack_status_callback(LORAWAN_STACK_DUTY_CYCLE_DELAY, attempt);
}

/**
 * @brief Called from LoRaWAN stack and calls registered callbacks. 
 * Provides us with MAC Common Part Sublayer data after a LoRaWAN transmit
 * @param McpsConfirm
 */
static void mcps_confirm(McpsConfirm_t *McpsConfirm)
{
  DPRINT("mcps_confirm: %i",McpsConfirm->AckReceived);
  lorawan_stack_status_t status = LORAWAN_STACK_ERROR_NACK;
  if(McpsConfirm!=NULL) {
    if(((McpsConfirm->McpsRequest == MCPS_CONFIRMED && McpsConfirm->AckReceived == 1) || (McpsConfirm->McpsRequest == MCPS_UNCONFIRMED)) && McpsConfirm->Status==LORAMAC_EVENT_INFO_STATUS_OK)
      status = LORAWAN_STACK_ERROR_OK;
  }
  else
    status = LORAWAN_STACK_ERROR_UNKNOWN;
  tx_callback(status, McpsConfirm->NbRetries);
  lorawan_transmitting = false;
}

/**
 * @brief Called from LoRaWAN stack and calls registered callbacks. 
 * Provides us with MAC Common Part Sublayer data after a LoRaWAN received 
 * @param mcpsIndication
 */
static void mcps_indication(McpsIndication_t *mcpsIndication)
{
  if(mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK)
  {
    DPRINT("mcps_indication status: %i", mcpsIndication->Status);
    return;
  }
  if( mcpsIndication->RxData == true )
  {
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
static void mlme_confirm(MlmeConfirm_t *mlmeConfirm)
{
  switch(mlmeConfirm->MlmeRequest)
  {
    case MLME_JOIN:
    {
      if(mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
      {
        DPRINT("join succeeded");
        joinRequestTrials = 0;

        join_state = STATE_JOINED;

        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_ANTENNA_GAIN; //MAC parameters are reset in LoRaMac-Node on every join request sent, so this should be set after a successful join
        mibReq.Param.AntennaGain = antenna_gain_f;
        LoRaMacMibSetRequestConfirm( &mibReq );

        if(stack_status_callback)
            stack_status_callback(LORAWAN_STACK_JOINED, mlmeConfirm->NbRetries);
      }
      else if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT) 
      {
        DPRINT("join failed because of RX2 timeout, going to attempt to retry");
        sched_post_task(&run_fsm); 
      }
      else
      {
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
static void mlme_indication(MlmeIndication_t *mlmeIndication)
{
  if(mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK)
  {
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

  // MIB_NETWORK_JOINED has become MIB_NETWORK_ACTIVATION, indicates whether activation type used was OTAA or ABP. If non-zero, the device has joined.
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
    if(keys_changed && was_joining)
    {
        lorawan_stack_deinit();
        lorawan_stack_init_otaa();
        if(stack_status_callback)
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
  int res = d7ap_fs_read_file(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID, 0, (uint8_t*) &antenna_gain, &length, ROOT_AUTH);

  if(res == -ENOENT) {
      // file does not exist yet (older filesystem version), create it
      if(region == EU868) {
        antenna_gain = MODULE_LORAWAN_EU_DEFAULT_ANTENNA_GAIN;  //dBm #in EU868 the MAX_EIRP is 16dBm so this corresponds to a max TX power of 15dBm 
      } else if(region == US915) {
        antenna_gain = MODULE_LORAWAN_US_DEFAULT_ANTENNA_GAIN; //dBm #in US915 the MAX_EIRP is 30dBm so this corresponds to a max TX power of 15dBm
      } else if(region == AS923) {
        antenna_gain = MODULE_LORAWAN_AS_DEFAULT_ANTENNA_GAIN;  //dBm #in AS915 the MAX_EIRP is 16dBm so this corresponds to a max TX power of 15dBm
      }

      uint8_t antenna_gain_file[1] = {
          (uint8_t) antenna_gain,
      };
        
      d7ap_fs_file_header_t volatile_file_header = {
        .file_permissions = (file_permission_t){ .guest_read = true, .user_read = true },
        .file_properties.storage_class = FS_STORAGE_PERMANENT,
        .length                        = USER_FILE_LORAWAN_ANTENNA_GAIN_SIZE,
        .allocated_length              = USER_FILE_LORAWAN_ANTENNA_GAIN_SIZE
      };

      // initialize file on fs
      int ret = d7ap_fs_init_file(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID, &volatile_file_header, antenna_gain_file);
  }

  antenna_gain_f = (float) antenna_gain;

  MibRequestConfirm_t mibReq;
  mibReq.Type = MIB_ANTENNA_GAIN;
  mibReq.Param.AntennaGain = antenna_gain_f;
  LoRaMacMibSetRequestConfirm( &mibReq );
}

/**
 * @brief Register the different callbacks
 * @param lorawan_rx_cb: LoRaWAN received data
 * @param lorawan_tx_cb: LoRaWAN transmitted data
 * @param join_completed_cb: LoRaWAN network joined
 * @param lorawan_duty_cycle_delay_cb: LoRaWAN delayed because of duty cycle
 * @param lorawan_join_attempt_cb: attempt to join network
 */
void lorawan_register_cbs(lorawan_rx_callback_t  lorawan_rx_cb, lorawan_tx_completed_callback_t lorawan_tx_cb, lorawan_status_callback_t lorawan_status_cb )
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
    if(inited == false)
  {
    log_print_error_string("TX not possible, not inited"); //Should not happen when using alp layer
    return LORAWAN_STACK_ERROR_NOT_INITED;
  }
  DPRINT("Checking for change in config");
  if (join_state==STATE_JOINING)
    return LORAWAN_STACK_ALREADY_JOINING;

  bool joined = (join_state == STATE_JOINED);
  sched_cancel_task(&run_fsm);
  app_port=lorawan_session_config->application_port;
  request_ack=lorawan_session_config->request_ack;
  datarate = lorawan_session_config->data_rate;
  
  if( adr_enabled != lorawan_session_config->adr_enabled)
  {
    adr_enabled = lorawan_session_config->adr_enabled;
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = adr_enabled;
    LoRaMacMibSetRequestConfirm( &mibReq );
  }
 
  if(!joined)
  {
    LoRaMacStatus_t status=LORAMAC_STATUS_OK;
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_NETWORK_ACTIVATION;
    mibReq.Param.NetworkActivation = ACTIVATION_TYPE_NONE;
    status=LoRaMacMibSetRequestConfirm( &mibReq );
    //note: for otaa, activation type gets set to ACTIVATION_TYPE_OTAA by the MAC layer on a successful join.
    if(status!=LORAMAC_STATUS_OK) {
      assert(false);}

    DPRINT("Change found - Init using OTAA");
    DPRINT("DevEui:");
    DPRINT_DATA(devEui, 8);
    DPRINT("AppEui:");
    DPRINT_DATA(appEui, 8);
    DPRINT("AppKey:"); 
    DPRINT_DATA(appKey, 16);
    DPRINT("Adaptive Data Rate: %d, Data rate: %d", adr_enabled, datarate);

   
    join_state = STATE_JOINING;
    sched_post_task(&run_fsm);

    joinRequestTrials = 0;
  }
    return joined ? LORAWAN_STACK_ERROR_OK : LORAWAN_STACK_ERROR_NOT_JOINED;
}

/**
 * @brief Inits the LoRaWAN stack using over the air activation
 * @param lorawan_session_config
 */
error_t lorawan_stack_init_otaa() { 
  if(inited)
    return EALREADY;
  if(first_init) {
    set_initial_keys();
    lorawan_set_antenna_gain(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID);
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

  //these callbacks are used by the LoRaMac to get the DevEui and AppEui when needed
  //in older versions the keys were provided with each join request. In current LoRaMac-node these are saved in an emulated secure element
  //but in order to avoid duplication, we instead save them here and provide LoRaMac callbacks to access them
  loraMacCallbacks.GetDevEui = &lorawan_get_deveui;
  loraMacCallbacks.GetAppEui = &lorawan_get_appeui;

  loraMacStatus = LoRaMacInitialization(&loraMacPrimitives, &loraMacCallbacks, region);
  if(loraMacStatus == LORAMAC_STATUS_OK) {
    DPRINT("init OK");
  } else {
    DPRINT("init failed %d", loraMacStatus);
    return -FAIL;
  }

  MibRequestConfirm_t mibReq;

  mibReq.Type = MIB_PUBLIC_NETWORK;
  mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK_ENABLED;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_DEVICE_CLASS;
  mibReq.Param.Class = LORAWAN_CLASS;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_NWK_KEY; //note: the naming conventions in LoRaMac-node follows the LoRaWAN 1.1 naming convention, but the security used in the version we fork is still 1.0.3
  mibReq.Param.NwkKey = appKey; 
  LoRaMacMibSetRequestConfirm( &mibReq );

  if(region == EU868) {
    mibReq.Type = MIB_DEFAULT_ANTENNA_GAIN;
    mibReq.Param.DefaultAntennaGain = MODULE_LORAWAN_EU_DEFAULT_ANTENNA_GAIN;
    LoRaMacMibSetRequestConfirm( &mibReq );
  } else if(region == US915) {
    mibReq.Type = MIB_DEFAULT_ANTENNA_GAIN;
    mibReq.Param.DefaultAntennaGain = MODULE_LORAWAN_US_DEFAULT_ANTENNA_GAIN;
    LoRaMacMibSetRequestConfirm( &mibReq );
  } else if(region == AS923) {
    mibReq.Type = MIB_DEFAULT_ANTENNA_GAIN;
    mibReq.Param.DefaultAntennaGain = MODULE_LORAWAN_AS_DEFAULT_ANTENNA_GAIN;
    LoRaMacMibSetRequestConfirm( &mibReq );
  }
  

#if defined( REGION_EU868 )
  LoRaMacTestSetDutyCycleOn(true);

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
  LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
  LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
  LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
  LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
  LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
  LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
  LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );

  mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
  mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_RX2_CHANNEL;
  mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
  LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif

  LoRaMacStart( ); // start up the LoRaMac (change from default state, which is LORAMAC_STOPPED)
  
  sched_register_task(&LoRaMacProcess);

  d7ap_fs_register_file_modified_callback(USER_FILE_LORAWAN_ANTENNA_GAIN_FILE_ID, &lorawan_set_antenna_gain);
  
  inited = true;

  return SUCCESS;
}

/**
 * @brief Deinitialize the LoRaWAN stack
 * @param lorawan_session_config
 */
void lorawan_stack_deinit(){
    if(!inited)
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

/**
 * @brief Sends data using LoRaWAN
 * @param payload
 * @param length
 * @param app_port
 * @param request_ack
 * @return lorawan stack status
 */
lorawan_stack_status_t lorawan_stack_send(uint8_t* payload, uint8_t length, uint8_t app_port, bool request_ack) {

  if(inited == false)
  {
    log_print_error_string("TX not possible, not inited"); //Should not happen when using alp layer
    return LORAWAN_STACK_ERROR_NOT_INITED;
  }
  if(!is_joined()) 
  {
    log_print_error_string("TX not possible, not joined"); //Should not happen when using alp layer
    return LORAWAN_STACK_ERROR_NOT_JOINED;
  }

  if(lorawan_transmitting)
  {
    DPRINT("TX not possible, already transmitting");
    return LORAWAN_STACK_ALREADY_TRANSMITTING;
  }


  if(length > LORAWAN_APP_DATA_BUFF_SIZE)
      return LORAWAN_STACK_ERROR_TX_NOT_POSSIBLE;
  

  memcpy1(app_data.Buff, payload, length);
  app_data.BuffSize = length;
  app_data.Port = app_port;

  McpsReq_t mcpsReq;
  LoRaMacTxInfo_t txInfo;
  if(LoRaMacQueryTxPossible(app_data.BuffSize, &txInfo) != LORAMAC_STATUS_OK )
  {
    // Send empty frame in order to flush MAC commands
    DPRINT("TX not possible, max payloadsize %i, trying to transmit %i", txInfo.MaxPossibleApplicationDataSize, txInfo.CurrentPossiblePayloadSize);
    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fBuffer = NULL;
    mcpsReq.Req.Unconfirmed.fBufferSize = 0;
    mcpsReq.Req.Unconfirmed.Datarate = datarate;
    LoRaMacMcpsRequest(&mcpsReq);
    return LORAWAN_STACK_ERROR_TX_NOT_POSSIBLE;
  }

  if(!request_ack)
  {
    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fPort = app_data.Port;
    mcpsReq.Req.Unconfirmed.fBuffer = app_data.Buff;
    mcpsReq.Req.Unconfirmed.fBufferSize = app_data.BuffSize;
    mcpsReq.Req.Unconfirmed.Datarate = datarate;
  }
  else
  {
    mcpsReq.Type = MCPS_CONFIRMED;
    mcpsReq.Req.Confirmed.fPort = app_data.Port;
    mcpsReq.Req.Confirmed.fBuffer = app_data.Buff;
    mcpsReq.Req.Confirmed.fBufferSize = app_data.BuffSize;
    mcpsReq.Req.Confirmed.NbTrials = 8;
    mcpsReq.Req.Confirmed.Datarate = datarate;
  }

  LoRaMacStatus_t status = LoRaMacMcpsRequest(&mcpsReq);
  if(status != LORAMAC_STATUS_OK) {
    //  state = STATE_SLEEP;
    DPRINT("failed sending data (status %i)", status);
    return LORAWAN_STACK_ERROR_UNKNOWN;
  }

  lorawan_transmitting = true;
  return LORAWAN_STACK_ERROR_OK;
}

/**
 * @brief returns saved devEui
 * 
 * @return lorawan devEui
 */
static uint8_t* lorawan_get_deveui( void ) {
    return devEui;
}

/**
 * @brief returns saved AppEui
 * 
 * @return lorawan appEui (in LoRaWAN 1.1, name is changed to   joinEui)
 */
static uint8_t* lorawan_get_appeui( void ) {
    return appEui;
}
