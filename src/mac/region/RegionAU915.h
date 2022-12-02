/*!
 * \file      RegionAU915.h
 *
 * \brief     Region definition for AU915
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 *
 * \defgroup  REGIONAU915 Region AU915
 *            Implementation according to LoRaWAN Specification v1.0.2.
 * \{
 */
#ifndef __REGION_AU915_H__
#define __REGION_AU915_H__
#include "MODULE_LORAWAN_defs.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "region/Region.h"

/*!
 * LoRaMac maximum number of channels
 */
#define AU915_MAX_NB_CHANNELS                       72

/*!
 * Minimal datarate that can be used by the node
 */
#define AU915_TX_MIN_DATARATE                       DR_0

/*!
 * Maximal datarate that can be used by the node
 */
#define AU915_TX_MAX_DATARATE                       DR_13

/*!
 * Minimal datarate that can be used by the node
 */
#define AU915_RX_MIN_DATARATE                       DR_8

/*!
 * Maximal datarate that can be used by the node
 */
#define AU915_RX_MAX_DATARATE                       DR_13

/*!
 * Default datarate used by the node
 */
#define AU915_DEFAULT_DATARATE                      MODULE_LORAWAN_MINIMUM_DATARATE

/*!
 * The minimum datarate which is used when the
 * dwell time is limited.
 */
#define AU915_DWELL_LIMIT_DATARATE                  MODULE_LORAWAN_MINIMUM_DATARATE

/*!
 * Minimal Rx1 receive datarate offset
 */
#define AU915_MIN_RX1_DR_OFFSET                     0

/*!
 * Maximal Rx1 receive datarate offset
 */
#define AU915_MAX_RX1_DR_OFFSET                     5

/*!
 * Minimal Tx output power that can be used by the node
 */
#define AU915_MIN_TX_POWER                          TX_POWER_14

/*!
 * Maximal Tx output power that can be used by the node
 */
#define AU915_MAX_TX_POWER                          TX_POWER_0

/*!
 * Default Tx output power used by the node
 */
#define AU915_DEFAULT_TX_POWER                      TX_POWER_0

/*!
 * Default uplink dwell time configuration
 */
#define AU915_DEFAULT_UPLINK_DWELL_TIME             1

/*!
 * Default Max EIRP
 */
#define AU915_DEFAULT_MAX_EIRP                      30.0f

/*!
 * Default antenna gain
 */
#define AU915_DEFAULT_ANTENNA_GAIN                  2.15f

/*!
 * Enabled or disabled the duty cycle
 */
#define AU915_DUTY_CYCLE_ENABLED                    0

/*!
 * Maximum RX window duration
 */
#define AU915_MAX_RX_WINDOW                         3000

/*!
 * Second reception window channel frequency definition.
 */
#define AU915_RX_WND_2_FREQ                         923300000

/*!
 * Second reception window channel datarate definition.
 */
#define AU915_RX_WND_2_DR                           DR_8

/*
 * CLASS B
 */
/*!
 * Beacon frequency
 */
#define AU915_BEACON_CHANNEL_FREQ                   923300000

/*!
 * Beacon frequency channel stepwidth
 */
#define AU915_BEACON_CHANNEL_STEPWIDTH              600000

/*!
 * Ping slot channel frequency
 */
#define AU915_PING_SLOT_CHANNEL_FREQ                923300000

/*!
 * Number of possible beacon channels
 */
#define AU915_BEACON_NB_CHANNELS                    8

/*!
 * Payload size of a beacon frame
 */
#define AU915_BEACON_SIZE                           23

/*!
 * Size of RFU 1 field
 */
#define AU915_RFU1_SIZE                             4

/*!
 * Size of RFU 2 field
 */
#define AU915_RFU2_SIZE                             3

/*!
 * Datarate of the beacon channel
 */
#define AU915_BEACON_CHANNEL_DR                     DR_8

/*!
 * Bandwith of the beacon channel
 */
#define AU915_BEACON_CHANNEL_BW                     2

/*!
 * Ping slot channel datarate
 */
#define AU915_PING_SLOT_CHANNEL_DR                  DR_8

/*!
 * LoRaMac maximum number of bands
 */
#define AU915_MAX_NB_BANDS                          1

/*!
 * Band 0 definition
 * Band = { DutyCycle, TxMaxPower, LastBandUpdateTime, LastMaxCreditAssignTime, TimeCredits, MaxTimeCredits, ReadyForTransmission }
 */
#define AU915_BAND0                                 { 1, AU915_MAX_TX_POWER, 0, 0, 0, 0, 0 } //  100.0 %

/*!
 * Defines the first channel for RX window 1 for US band
 */
#define AU915_FIRST_RX1_CHANNEL                     ( (uint32_t) 923300000 )

/*!
 * Defines the last channel for RX window 1 for US band
 */
#define AU915_LAST_RX1_CHANNEL                      ( (uint32_t) 927500000 )

/*!
 * Defines the step width of the channels for RX window 1
 */
#define AU915_STEPWIDTH_RX1_CHANNEL                 ( (uint32_t) 600000 )

/*!
 * Data rates table definition
 */
static const uint8_t DataratesAU915[]  = { 12, 11, 10, 9, 8, 7, 8, 0, 12, 11, 10, 9, 8, 7, 0, 0 };

/*!
 * Bandwidths table definition in Hz
 */
static const uint32_t BandwidthsAU915[] = { 125000, 125000, 125000, 125000, 125000, 125000, 500000, 0, 500000, 500000, 500000, 500000, 500000, 500000, 0, 0 };

/*!
 * Up/Down link data rates offset definition
 */
static const int8_t DatarateOffsetsAU915[7][6] =
{
    { DR_8 , DR_8 , DR_8 , DR_8 , DR_8 , DR_8  }, // DR_0
    { DR_9 , DR_8 , DR_8 , DR_8 , DR_8 , DR_8  }, // DR_1
    { DR_10, DR_9 , DR_8 , DR_8 , DR_8 , DR_8  }, // DR_2
    { DR_11, DR_10, DR_9 , DR_8 , DR_8 , DR_8  }, // DR_3
    { DR_12, DR_11, DR_10, DR_9 , DR_8 , DR_8  }, // DR_4
    { DR_13, DR_12, DR_11, DR_10, DR_9 , DR_8  }, // DR_5
    { DR_13, DR_13, DR_12, DR_11, DR_10, DR_9  }, // DR_6
};

/*!
 * Maximum payload with respect to the datarate index.
 * The table is valid for the dwell time configuration of 0 for uplinks.
 */
static const uint8_t MaxPayloadOfDatarateDwell0AU915[] = { 51, 51, 51, 115, 242, 242, 242, 0, 53, 129, 242, 242, 242, 242 };

/*!
 * Maximum payload with respect to the datarate index.
 * The table is valid for the dwell time configuration of 1 for uplinks.
 */
static const uint8_t MaxPayloadOfDatarateDwell1AU915[] = { 0, 0, 11, 53, 125, 242, 242, 0, 53, 129, 242, 242, 242, 242 };

/*!
 * \brief The function gets a value of a specific phy attribute.
 *
 * \param [IN] getPhy Pointer to the function parameters.
 *
 * \retval Returns a structure containing the PHY parameter.
 */
PhyParam_t RegionAU915GetPhyParam( GetPhyParams_t* getPhy );

/*!
 * \brief Updates the last TX done parameters of the current channel.
 *
 * \param [IN] txDone Pointer to the function parameters.
 */
void RegionAU915SetBandTxDone( SetBandTxDoneParams_t* txDone );

/*!
 * \brief Initializes the channels masks and the channels.
 *
 * \param [IN] type Sets the initialization type.
 */
void RegionAU915InitDefaults( InitDefaultsParams_t* params );

/*!
 * \brief Verifies a parameter.
 *
 * \param [IN] verify Pointer to the function parameters.
 *
 * \param [IN] type Sets the initialization type.
 *
 * \retval Returns true, if the parameter is valid.
 */
bool RegionAU915Verify( VerifyParams_t* verify, PhyAttribute_t phyAttribute );

/*!
 * \brief The function parses the input buffer and sets up the channels of the
 *        CF list.
 *
 * \param [IN] applyCFList Pointer to the function parameters.
 */
void RegionAU915ApplyCFList( ApplyCFListParams_t* applyCFList );

/*!
 * \brief Sets a channels mask.
 *
 * \param [IN] chanMaskSet Pointer to the function parameters.
 *
 * \retval Returns true, if the channels mask could be set.
 */
bool RegionAU915ChanMaskSet( ChanMaskSetParams_t* chanMaskSet );

/*!
 * Computes the Rx window timeout and offset.
 *
 * \param [IN] datarate     Rx window datarate index to be used
 *
 * \param [IN] minRxSymbols Minimum required number of symbols to detect an Rx frame.
 *
 * \param [IN] rxError      System maximum timing error of the receiver. In milliseconds
 *                          The receiver will turn on in a [-rxError : +rxError] ms
 *                          interval around RxOffset
 *
 * \param [OUT]rxConfigParams Returns updated WindowTimeout and WindowOffset fields.
 */
void RegionAU915ComputeRxWindowParameters( int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams );

/*!
 * \brief Configuration of the RX windows.
 *
 * \param [IN] rxConfig Pointer to the function parameters.
 *
 * \param [OUT] datarate The datarate index which was set.
 *
 * \retval Returns true, if the configuration was applied successfully.
 */
bool RegionAU915RxConfig( RxConfigParams_t* rxConfig, int8_t* datarate );

/*!
 * \brief TX configuration.
 *
 * \param [IN] txConfig Pointer to the function parameters.
 *
 * \param [OUT] txPower The tx power index which was set.
 *
 * \param [OUT] txTimeOnAir The time-on-air of the frame.
 *
 * \retval Returns true, if the configuration was applied successfully.
 */
bool RegionAU915TxConfig( TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir );

/*!
 * \brief The function processes a Link ADR Request.
 *
 * \param [IN] linkAdrReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
uint8_t RegionAU915LinkAdrReq( LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed );

/*!
 * \brief The function processes a RX Parameter Setup Request.
 *
 * \param [IN] rxParamSetupReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
uint8_t RegionAU915RxParamSetupReq( RxParamSetupReqParams_t* rxParamSetupReq );

/*!
 * \brief The function processes a Channel Request.
 *
 * \param [IN] newChannelReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
int8_t RegionAU915NewChannelReq( NewChannelReqParams_t* newChannelReq );

/*!
 * \brief The function processes a TX ParamSetup Request.
 *
 * \param [IN] txParamSetupReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 *         Returns -1, if the functionality is not implemented. In this case, the end node
 *         shall not process the command.
 */
int8_t RegionAU915TxParamSetupReq( TxParamSetupReqParams_t* txParamSetupReq );

/*!
 * \brief The function processes a DlChannel Request.
 *
 * \param [IN] dlChannelReq Pointer to the function parameters.
 *
 * \retval Returns the status of the operation, according to the LoRaMAC specification.
 */
int8_t RegionAU915DlChannelReq( DlChannelReqParams_t* dlChannelReq );

/*!
 * \brief Alternates the datarate of the channel for the join request.
 *
 * \param [IN] currentDr Current datarate.
 *
 * \retval Datarate to apply.
 */
int8_t RegionAU915AlternateDr( int8_t currentDr, AlternateDrType_t type );

/*!
 * \brief Searches and set the next random available channel
 *
 * \param [OUT] channel Next channel to use for TX.
 *
 * \param [OUT] time Time to wait for the next transmission according to the duty
 *              cycle.
 *
 * \param [OUT] aggregatedTimeOff Updates the aggregated time off.
 *
 * \retval Function status [1: OK, 0: Unable to find a channel on the current datarate]
 */
LoRaMacStatus_t RegionAU915NextChannel( NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff );

/*!
 * \brief Adds a channel.
 *
 * \param [IN] channelAdd Pointer to the function parameters.
 *
 * \retval Status of the operation.
 */
LoRaMacStatus_t RegionAU915ChannelAdd( ChannelAddParams_t* channelAdd );

/*!
 * \brief Removes a channel.
 *
 * \param [IN] channelRemove Pointer to the function parameters.
 *
 * \retval Returns true, if the channel was removed successfully.
 */
bool RegionAU915ChannelsRemove( ChannelRemoveParams_t* channelRemove  );

/*!
 * \brief Computes new datarate according to the given offset
 *
 * \param [IN] downlinkDwellTime Downlink dwell time configuration. 0: No limit, 1: 400ms
 *
 * \param [IN] dr Current datarate
 *
 * \param [IN] drOffset Offset to be applied
 *
 * \retval newDr Computed datarate.
 */
uint8_t RegionAU915ApplyDrOffset( uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset );

/*!
 * \brief Sets the radio into beacon reception mode
 *
 * \param [IN] rxBeaconSetup Pointer to the function parameters
 */
 void RegionAU915RxBeaconSetup( RxBeaconSetup_t* rxBeaconSetup, uint8_t* outDr );

/*! \} defgroup REGIONAU915 */

#ifdef __cplusplus
}
#endif

#endif // __REGION_AU915_H__
