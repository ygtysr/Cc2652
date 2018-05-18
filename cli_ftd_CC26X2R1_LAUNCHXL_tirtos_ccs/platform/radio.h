/******************************************************************************

 @file radio.h

 @brief TIRTOS platform specific radio functions for OpenThread

 Group: CMCU, LPRF
 Target Device: CC2652

 ******************************************************************************
 
 Copyright (c) 2017-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc26x2_sdk_2_10_00_44
 Release Date: 2018-04-09 12:59:57
 *****************************************************************************/

// name collision with openthread/radio.h
#ifndef PLATFORM_RADIO_H_
#define PLATFORM_RADIO_H_

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_ieee_cmd.h)

#include <ti/drivers/rf/RF.h>

/**
 * Size of the receive buffers in the receive queue.
 */
#define RX_BUF_SIZE 144

/**
 * Value to pass to `RF_cancelCmd` to signify aborting the command.
 *
 * documented in `source/ti/drivers/rf/RF.h`
 */
#define RF_DRIVER_ABORT 0

/**
 * Return value used when searching the source match array.
 *
 * Returned if an address could not be found or if an empty element could not
 * be found.
 */
#define PLATFORM_RADIO_SRC_MATCH_NONE 0xFF

/**
 * Number of extended addresses in @ref sSrcMatchExtData.
 */
#define PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM 10

/**
 * Number of short addresses in @ref sSrcMatchShortData.
 */
#define PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM 10

/**
 * size of length field in receive struct.
 *
 * defined in Table 23-10 of the cc13xx and cc26xx TRM.
 */
#define DATA_ENTRY_LENSZ_BYTE 1

/**
 * Event flags marked in @ref rfEvents.
 */
#define RF_EVENT_TX_DONE      Event_Id_00
#define RF_EVENT_ED_SCAN_DONE Event_Id_01
#define RF_EVENT_RX_DONE      Event_Id_02
#define RF_EVENT_RX_ACK_DONE  Event_Id_03
#define RF_EVENT_SLEEP_YIELD  Event_Id_04

/**
 * (IEEE 802.15.4-2006) PSDU.FCF.frameType.
 */
#define IEEE802154_FRAME_TYPE_MASK        (0x7)

/**
 * (IEEE 802.15.4-2006) frame type: ACK.
 */
#define IEEE802154_FRAME_TYPE_ACK         (0x2)

/**
 * (IEEE 802.15.4-2006) PSDU.FCF.bFramePending.
 */
#define IEEE802154_FRAME_PENDING          (1<<4)

/**
 * (IEEE 802.15.4-2006) Length of an ack frame.
 */
#define IEEE802154_ACK_LENGTH             (5)

/**
 * (IEEE 802.15.4-2006) PSDU.FCF.bAR.
 */
#define IEEE802154_ACK_REQUEST            (1<<5)

/**
 * (IEEE 802.15.4-2006) PSDU.sequenceNumber.
 */
#define IEEE802154_DSN_OFFSET             (2)

/**
 * (IEEE 802.15.4-2006) macMinBE.
 */
#define IEEE802154_MAC_MIN_BE             (3)

/**
 * (IEEE 802.15.4-2006) macMaxBE.
 */
#define IEEE802154_MAC_MAX_BE             (5)

/**
 * (IEEE 802.15.4-2006) macMaxCSMABackoffs.
 */
#define IEEE802154_MAC_MAX_CSMA_BACKOFFS  (4)

/**
 * (IEEE 802.15.4-2006) macMaxFrameRetries.
 */
#define IEEE802154_MAC_MAX_FRAMES_RETRIES (3)

/**
 * (IEEE 802.15.4-2006 7.4.1) MAC constants.
 */
#define IEEE802154_A_UINT_BACKOFF_PERIOD  (20)

/**
 * (IEEE 802.15.4-2006 6.4.1) PHY constants.
 */
#define IEEE802154_A_TURNAROUND_TIME      (12)

/**
 * (IEEE 802.15.4-2006 6.4.2) PHY PIB attribute, specifically the O-QPSK PHY.
 */
#define IEEE802154_PHY_SHR_DURATION       (10)

/**
 * (IEEE 802.15.4-2006 6.4.2) PHY PIB attribute, specifically the O-QPSK PHY.
 */
#define IEEE802154_PHY_SYMBOLS_PER_OCTET  (2)

/**
 * (IEEE 802.15.4-2006 7.4.2) macAckWaitDuration PIB attribute.
 */
#define IEEE802154_MAC_ACK_WAIT_DURATION                                       \
        (IEEE802154_A_UINT_BACKOFF_PERIOD + IEEE802154_A_TURNAROUND_TIME       \
         + IEEE802154_PHY_SHR_DURATION                                         \
         + ( 6 * IEEE802154_PHY_SYMBOLS_PER_OCTET))

/**
 * (IEEE 802.15.4-2006 6.5.3.2) O-QPSK symbol rate.
 */
#define IEEE802154_SYMBOLS_PER_SEC        (62500)

/**
 * Frequency of the Radio Timer module.
 *
 * 4MHz clock.
 */
#define PLATFORM_RADIO_RAT_TICKS_PER_SEC  (4000000)

/**
 * Invalid RSSI value returned from an ED scan.
 */
#define PLATFORM_RADIO_INVALID_RSSI       (127)

/**
 * Structure for source matching extended addresses.
 *
 * Defined in Table 23-73 of the cc13xx and cc26xx TRM.
 */
typedef struct __RFC_STRUCT ext_src_match_data_s ext_src_match_data_t;
struct __RFC_STRUCT ext_src_match_data_s
{
    uint32_t srcMatchEn [((PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM + 31) / 32)];
    uint32_t srcPendEn  [((PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM + 31) / 32)];
    uint64_t extAddrEnt [PLATFORM_RADIO_EXTADD_SRC_MATCH_NUM];
} __RFC_STRUCT_ATTR;

/**
 * Structure for source matching short addresses.
 *
 * Defined in Table 23-74 of the cc13xx and cc26xx TRM.
 */
typedef struct __RFC_STRUCT short_src_match_data_s short_src_match_data_t;
struct __RFC_STRUCT short_src_match_data_s
{
    uint32_t             srcMatchEn   [((PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM + 31) / 32)];
    uint32_t             srcPendEn    [((PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM + 31) / 32)];
    rfc_shortAddrEntry_t shortAddrEnt [PLATFORM_RADIO_SHORTADD_SRC_MATCH_NUM];
} __RFC_STRUCT_ATTR;

/**
 * Address type for @ref rfCoreModifySourceMatchEntry().
 */
typedef enum platformRadio_address
{
    platformRadio_address_short = 0,
    platformRadio_address_ext   = 1,
} platformRadio_address;

/**
 * This enum represents the state of a radio.
 *
 * Initially, a radio is in the Disabled state.
 *
 * The following are valid radio state transitions for the platform
 *
 * ```
 *  +----------+  Enable()  +-------+  Receive()   +---------+   Transmit()  +----------+
 *  |          |----------->|       |------------->|         |-------------->|          |
 *  | Disabled |            | Sleep |              | Receive |               | Transmit |
 *  |          |<-----------|       |<-------------|         |<--------------|          |
 *  +----------+  Disable() |       |   Sleep()    +---------+ Receive() or  +----------+
 *    ^                     |       |                          Transmit complete
 *    |                     |       | EnergyScan()  +--------+
 *    |                     |       |-------------->|        |
 *  Init()                  |       |               | EdScan |
 *                          |       |<--------------|        |
 *                          +-------+ scan complete +--------+
 * ```
 *
 * These states slightly differ from the states in \ref include/platform/radio.h.
 * The additional states the phy can be in are due to the asynchronous nature
 * of the CM0 radio core.
 *
 * | state            | description                                        |
 * |------------------|----------------------------------------------------|
 * | Disabled         | The rfcore powerdomain is off and the RFCPE is off |
 * | Sleep            | The RFCORE PD is on, and the RFCPE is in IEEE mode |
 * | Receive          | The RFCPE is running a CMD_IEEE_RX                 |
 * | EdScan           | The RFCPE is running a CMD_IEEE_ED_SCAN            |
 * | Transmit         | The RFCPE is running a transmit command string     |
 *
 */
typedef enum platformRadio_phyState
{
    platformRadio_phyState_Disabled = 0,
    platformRadio_phyState_Sleep,
    platformRadio_phyState_Receive,
    platformRadio_phyState_EdScan,
    platformRadio_phyState_Transmit,
} platformRadio_phyState;

/**
 * TX Power dBm lookup table - values from SmartRF Studio.
 */

RF_TxPowerTable_Entry txPowerTable[] =
{
    { -21, RF_TxPowerTable_DEFAULT_PA_ENTRY(7, 3, 0, 3)    },
    { -18, RF_TxPowerTable_DEFAULT_PA_ENTRY(9, 3, 0, 3)    },
    { -15, RF_TxPowerTable_DEFAULT_PA_ENTRY(12, 2, 0, 100) },
    { -12, RF_TxPowerTable_DEFAULT_PA_ENTRY(40, 2, 0, 8)   },
    { -10, RF_TxPowerTable_DEFAULT_PA_ENTRY(12, 2, 0, 11)  },
    { -9,  RF_TxPowerTable_DEFAULT_PA_ENTRY(13, 2, 0, 5)   },
    { -6,  RF_TxPowerTable_DEFAULT_PA_ENTRY(13, 1, 0, 16)  },
    { -5,  RF_TxPowerTable_DEFAULT_PA_ENTRY(14, 1, 0, 17)  },
    { -3,  RF_TxPowerTable_DEFAULT_PA_ENTRY(17, 1, 0, 20)  },
    { 0,   RF_TxPowerTable_DEFAULT_PA_ENTRY(25, 1, 0, 26)  },
    { 1,   RF_TxPowerTable_DEFAULT_PA_ENTRY(28, 1, 0, 28)  },
    { 2,   RF_TxPowerTable_DEFAULT_PA_ENTRY(13, 0, 0, 34)  },
    { 3,   RF_TxPowerTable_DEFAULT_PA_ENTRY(17, 0, 0, 42)  },
    { 4,   RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 0, 0, 54)  },
    { 5,   RF_TxPowerTable_DEFAULT_PA_ENTRY(30, 0, 0, 74)  },
    RF_TxPowerTable_TERMINATION_ENTRY
};

#endif /* PLATFORM_RADIO_H_ */

