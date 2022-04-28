/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /*******************************************************************************
  * @file    mlm32l07x01.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    08-September-2017
  * @brief   driver LoRa module murata cmwx1zzabz-078
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/

#include "hw.h"
#include "radio.h"
#include "hwradio.h"
#include "mlm32l07x01.h"
#include "timeServer.h"
#include "hwgpio.h"
#include "platform.h"
#include "hwsystem.h"



const struct Radio_s Radio =
{
    SX127xIoInit,
    SX127xIoDeInit,
    SX127xInit, 
    SX127xGetStatus, 
    SX127xSetModem,
    hw_radio_set_center_freq,
    hw_lora_random,
    SX127xSetRxConfig,        
    SX127xSetTxConfig,       
    SX127xCheckRfFrequency,
    SX127xGetTimeOnAir, 
    SX127xSend,
    SX127xSetSleep,
    SX127xSetStby,
    SX127xSetRx, 
    hw_lora_set_tx_continuous_wave,
    SX127xReadRssi,
    SX127xSetMaxPayloadLength,
    hw_lora_set_public_network,
    SX127xGetWakeupTime
};

void   SX127xIoInit ( void ) {
    hw_radio_io_init(true);
}

void   SX127xIoDeInit ( void ) {
    hw_radio_io_deinit();

}

uint32_t SX127xInit( RadioEvents_t *events ) {

    hwradio_init_args_t init_args;
    init_args.tx_lora_packet_cb = events->TxDone;
    init_args.rx_lora_packet_cb = events->RxDone;
    init_args.rx_lora_error_cb = events->RxError;
    init_args.rx_lora_timeout_cb = events->RxTimeout;

    hw_radio_init(&init_args);

}

RadioState_t SX127xGetStatus( void ) {
    // there is an equivalent state enum in sx127x.c, but the states are not in the same order 
    // (radio.h is: IDLE, RX, TX, CAD, sx127x.c is: IDLE, TX, RX, STANDBY, where IDLE stands for sleep).
    // so an enum conversion is needed
    switch(hw_radio_get_opmode()) {
        case HW_STATE_SLEEP:
            return RF_IDLE;
            break;
        case HW_STATE_RX:
            return RF_RX_RUNNING;
            break;
        case HW_STATE_TX:
            return RF_TX_RUNNING;
            break;
        case HW_STATE_STANDBY:
            return RF_IDLE;
            break;
        default: //HW_STATE_IDLE
            return RF_IDLE;
            break;
    }    
}

void SX127xSetModem( RadioModems_t modem ) {
    hw_radio_switch_longRangeMode(modem == MODEM_LORA); 
}

void SX127xSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX127xSetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        {
            log_print_error_string("setRxConfig in FSK mode. Unexpected behaviour as in oss-7 FSK mode is not used in the LoRaWAN stack");
            //Note: this may have to change in the future as FSK mode is used as DR7 in CN470-510 and AS923
        }
        break;
    case MODEM_LORA:
        {
            hw_lora_set_rx_config(bandwidth,
                         datarate,  coderate,
                          preambleLen,
                        symbTimeout, fixLen,
                         payloadLen,
                         crcOn, freqHopOn, hopPeriod,
                         iqInverted, rxContinuous);
        }
        break;
    }
}

void SX127xSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    SX127xSetModem( modem );

    hw_radio_set_tx_power(power);
    
    switch( modem )
    {
    case MODEM_FSK:
        {
            log_print_error_string("setTxConfig in FSK mode. Unexpected behaviour as in oss-7 FSK mode is not used in the LoRaWAN stack");
            //Note: this may have to change in the future as FSK mode is used as DR7 in CN470-510 and AS923
        }
        break;
    case MODEM_LORA:
        {
            hw_lora_set_tx_config(bandwidth,  datarate,
                        coderate,  preambleLen,
                        fixLen, crcOn, freqHopOn,
                        hopPeriod, iqInverted, timeout);
        }
        break;
    }
}

bool SX127xCheckRfFrequency( uint32_t frequency ) {
    return true; // TBD: Implement check. Currently all frequencies are supported
}

static uint32_t SX127xGetGfskTimeOnAirNumerator( uint16_t preambleLen, bool fixLen,
                                                 uint8_t payloadLen, bool crcOn )
{
    const uint8_t syncWordLength = 3;
    return ( preambleLen << 3 ) + ( ( fixLen == false ) ? 8 : 0 ) + ( syncWordLength << 3 ) + ( ( payloadLen + ( 0 ) + ( ( crcOn == true ) ? 2 : 0 ) ) << 3 );
}

static uint32_t SX127xGetLoRaTimeOnAirNumerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    int32_t crDenom           = coderate + 4;
    bool    lowDatareOptimize = false;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or SF6
    if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        if( preambleLen < 12 ) preambleLen = 12;
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
        lowDatareOptimize = true;
    }

    int32_t ceilDenominator;
    int32_t ceilNumerator = ( payloadLen << 3 ) + ( crcOn ? 16 : 0 ) - ( 4 * datarate ) + ( fixLen ? 0 : 20 );

    if( datarate <= 6 )
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if( lowDatareOptimize == true )
        {
            ceilDenominator = 4 * ( datarate - 2 );
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if( ceilNumerator < 0 ) ceilNumerator = 0;

    // Perform integral ceil()
    int32_t intermediate = ( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

    if( datarate <= 6 ) intermediate += 2;

    return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

static uint32_t SX127xGetLoRaBandwidthInHz( uint32_t bw )
{
    uint32_t bandwidthInHz = 0;

    switch( bw )
    {
    case 0: // 125 kHz
        bandwidthInHz = 125000UL;
        break;
    case 1: // 250 kHz
        bandwidthInHz = 250000UL;
        break;
    case 2: // 500 kHz
        bandwidthInHz = 500000UL;
        break;
    }

    return bandwidthInHz;
}

uint32_t SX127xGetTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn ) {
    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch( modem )
    {
    case MODEM_FSK:
        {
            numerator   = 1000U * SX127xGetGfskTimeOnAirNumerator( preambleLen, fixLen, payloadLen, crcOn );
            denominator = datarate;
        }
        break;
    case MODEM_LORA:
        {
            numerator   = 1000U * SX127xGetLoRaTimeOnAirNumerator( bandwidth, datarate, coderate, preambleLen, fixLen,
                                                                   payloadLen, crcOn );
            denominator = SX127xGetLoRaBandwidthInHz( bandwidth );
        }
        break;
    }
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

void  SX127xSend( uint8_t *buffer, uint8_t size ) {
    uint16_t fit_size = (uint16_t) size;
    hw_radio_send_payload(buffer, size);
}


void SX127xSetSleep( void ) {
    hw_radio_set_idle(); 
}

void SX127xSetStby( void ) {
    hw_radio_set_opmode(HW_STATE_STANDBY); //TODO: unlike the sx1276 ver, this does not stop any timers...
}

void SX127xSetRx( uint32_t timeout ) {
    hw_radio_set_opmode(HW_STATE_RX);

    if(timeout != 0) {
        // in RX_SINGLE mode, if no preamble is detected then a hardware interrupt will fire before this timeout expires
        // in RX_CONT mode, the device will move back out of RX mode as a result of this timeout
        hw_radio_set_rx_timeout(timeout); //and set a fire to put it back to idle after a set amount of time
    }
    
}

int16_t SX127xReadRssi( RadioModems_t modem ) {
  return hw_radio_get_rssi(); // uses the current modem choice instead of swapping.
}

void SX127xSetMaxPayloadLength( RadioModems_t modem, uint8_t max ) {
    switch(modem) {
        case MODEM_FSK:
            hw_radio_set_payload_length(max);
            break;
        case MODEM_LORA:
            hw_lora_set_max_payload_length(max);
    }
}

uint32_t SX127xGetWakeupTime( void ) {
    return BOARD_TCXO_WAKEUP_TIME + RADIO_WAKEUP_TIME;
}


