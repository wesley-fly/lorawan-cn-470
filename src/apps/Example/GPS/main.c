/*!
 * \file      main.c
 *
 * \brief     LoRaMac classA device implementation
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
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

/*! \file classA/SensorNode/main.c */
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "utilities.h"
#include "board.h"
//#include "gpio.h"
#include "gps.h"
#include "cxd5603.h"
#include "LoRaMac.h"
#include "Commissioning.h"
#include "LoRaMacCrypto.h"

//#define ACTIVE_REGION LORAMAC_REGION_EU868
#ifdef REGION_AS923
#define ACTIVE_REGION LORAMAC_REGION_AS923
#elif REGION_AU915
#define ACTIVE_REGION LORAMAC_REGION_AU915
#elif REGION_CN470
#define ACTIVE_REGION LORAMAC_REGION_CN470
#elif REGION_CN779
#define ACTIVE_REGION LORAMAC_REGION_CN779
#elif REGION_EU433
#define ACTIVE_REGION LORAMAC_REGION_EU433
#elif REGION_EU868
#define ACTIVE_REGION LORAMAC_REGION_EU868
#elif REGION_KR920
#define ACTIVE_REGION LORAMAC_REGION_KR920
#elif REGION_IN865
#define ACTIVE_REGION LORAMAC_REGION_IN865
#elif REGION_US915
#define ACTIVE_REGION LORAMAC_REGION_US915
#elif REGION_US915_HYBRID
#define ACTIVE_REGION LORAMAC_REGION_US915_HYBRID
#else
#warning "No active region defined."
#endif

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        false

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 16;
static uint8_t AppDataSizeBackup = 16;
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
*/
static bool AppLedStateOn = false;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

#if defined( GPS_CXD5603_ENABLE )
/* GPS Parameter */
int32_t GPS_Latitude=0,GPS_Longitude=0;
uint8_t GPS_LatPole,GPS_LongPole;
#define FIFO_TX_SIZE                            512
#define FIFO_RX_SIZE                            512
uint8_t UartTxBuffer[FIFO_TX_SIZE];
uint8_t UartRxBuffer[FIFO_RX_SIZE];
#define UART4_TX                                    PC_10
#define UART4_RX                                    PC_11
Gpio_t LevelShiftPin;
Uart_t Uart4;
#define GPRMC_LEN 128
bool Get_GPS_Data = false;
uint8_t Cmd[] = "@gcd\r\n"; //For GPS cold start.
#endif

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    const LoRaMacRegion_t region = ACTIVE_REGION;

    switch( port )
    {
    case 2:
        switch( region )
        {
            case LORAMAC_REGION_CN470:
            case LORAMAC_REGION_CN779:
            case LORAMAC_REGION_EU433:
            case LORAMAC_REGION_EU868:
            case LORAMAC_REGION_IN865:
            case LORAMAC_REGION_KR920:
            {
                AppDataSizeBackup = AppDataSize = 10;
                AppData[0] = (GPS_Latitude >> 24) & 0xFF;
                AppData[1] = (GPS_Latitude >> 16) & 0xFF;
                AppData[2] = (GPS_Latitude >> 8) & 0xFF;
                AppData[3] = GPS_Latitude & 0xFF;
                AppData[4] = (GPS_Longitude >> 24) & 0xFF;
                AppData[5] = (GPS_Longitude >> 16) & 0xFF;
                AppData[6] = (GPS_Longitude >> 8) & 0xFF;
                AppData[7] = GPS_Longitude & 0xFF;
                AppData[8] = GPS_LatPole;
                AppData[9] = GPS_LongPole;
                break;
            }
            case LORAMAC_REGION_AS923:
            case LORAMAC_REGION_AU915:
            case LORAMAC_REGION_US915:
            case LORAMAC_REGION_US915_HYBRID:
            {
                AppDataSizeBackup = AppDataSize = 10;
                AppData[0] = (GPS_Latitude >> 24) & 0xFF;
                AppData[1] = (GPS_Latitude >> 16) & 0xFF;
                AppData[2] = (GPS_Latitude >> 8) & 0xFF;
                AppData[3] = GPS_Latitude & 0xFF;
                AppData[4] = (GPS_Longitude >> 24) & 0xFF;
                AppData[5] = (GPS_Longitude >> 16) & 0xFF;
                AppData[6] = (GPS_Longitude >> 8) & 0xFF;
                AppData[7] = GPS_Longitude & 0xFF;
                AppData[8] = GPS_LatPole;
                AppData[9] = GPS_LongPole;
                break;
            }
            default:
                // Unsupported region.
                break;
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;
            mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
            }
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
//        GpioWrite( &Led1, 0 );
//        TimerStart( &Led1Timer );
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent( );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
//                GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSizeBackup = AppDataSize;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
//                    GpsStop( );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = AppDataSizeBackup;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
//                    GpsStart( );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < MIN( AppDataSize, LORAWAN_APP_DATA_MAX_SIZE ); i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = AppDataSizeBackup;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
//                        GpsStart( );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                        if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                        {
                            DeviceState = DEVICE_STATE_SLEEP;
                        }
                        else
                        {
                            DeviceState = DEVICE_STATE_CYCLE;
                        }
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
//    GpioWrite( &Led2, 0 );
//    TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                MlmeReq_t mlmeReq;
                mlmeReq.Type = MLME_JOIN;
                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            OnTxNextPacketTimerEvent( );
            break;
        }
        default:
            break;
    }
}

#if defined( GPS_CXD5603_ENABLE )
/* GPS relative functions */
void UartIrqNotify( UartNotifyId_t id,uint8_t *uartdata)
{
    tNmeaGpsData gpsData;
//	uint8_t msg[128];
    uint16_t len;

    /* Transmit the received data when RX interrupt occurs */
    if( id == UART_NOTIFY_RX )
    {
        char* wellhandled_string = NULL;
        if ((wellhandled_string = strstr((char *)uartdata, "$GPRMC")) != NULL)
        {
            for(int i=0; i < GPRMC_LEN ;i++)
            {
                if(wellhandled_string[i] == '\n')
                {
                    wellhandled_string[i] = '\0'; //replace ‘\n’ with null
                    len = i;
                }
            }
        }

        if(GpsCheckGPRMC((uint8_t*)wellhandled_string)==SUCCESS)
        {
            Get_GPS_Data = true;
            if(GpsParseGpsData((int8_t *)wellhandled_string, len, &gpsData) == SUCCESS &&
                        gpsData.NmeaDataStatus[0] == 'A')
            {

                GPS_Latitude = CXD5603ConverLatitudeFromDMMtoDD(gpsData.NmeaLatitude)*1000000;
                GPS_Longitude = CXD5603ConverLongitudeFromDMMtoDD(gpsData.NmeaLongitude)*1000000;
                if(CXD5603ConverLatitudePole(gpsData.NmeaLatitudePole) == true)
                    GPS_LatPole = 0; /*north*/
                else
                    GPS_LatPole = 1; /*south*/
    
                if(CXD5603ConverLongitudePole(gpsData.NmeaLongitudePole) == true)
                    GPS_LongPole = 0; /*east*/
                else
                    GPS_LongPole = 1; /*west*/
            }
        }
        else
            Get_GPS_Data = false;
            
     }
        
    if( id == UART_NOTIFY_TX )
    {
//       memset(msg,'\0',sizeof(msg));
//       sprintf((char*)msg, (char*)uartdata);
    }
}

void InitUart()
{
    /* Initial FIFOs for UART1 transmit/receive respectively */
    FifoInit( &Uart4.FifoTx, UartTxBuffer, FIFO_TX_SIZE );
    FifoInit( &Uart4.FifoRx, UartRxBuffer, FIFO_RX_SIZE );

    /* Set UART1 notification callback */
    Uart4.IrqNotify = UartIrqNotify;

    /* Initializes the UART object and MCU peripheral */
    UartInit( &Uart4, UART_4, UART4_TX, UART4_RX );

    /* Configures the UART object and MCU peripheral */
    UartConfig( &Uart4, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
}

void SendCmdToGPS(void)
{
    UartSendGPS(&Uart4, Cmd);
}

void OpenLevelShift(void)
{
    GpioInit( &LevelShiftPin, PC_6, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
}

void CloseLevelShift(void)
{
    GpioInit( &LevelShiftPin, PC_6, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

#endif

/**
 * Main application entry point.
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );
    BoardInitPeriph( );
#if defined( GPS_CXD5603_ENABLE )
    OpenLevelShift();
    InitUart();
#endif
    DeviceState = DEVICE_STATE_INIT;

    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
#if( OVER_THE_AIR_ACTIVATION != 0 )
				MlmeReq_t mlmeReq;

				// Initialize LoRaMac device unique ID
				//AcSiP(-), DevEui value is set by define currently
				//BoardGetUniqueId( DevEui );

				mlmeReq.Type = MLME_JOIN;

				mlmeReq.Req.Join.DevEui = DevEui;
				mlmeReq.Req.Join.AppEui = AppEui;
				mlmeReq.Req.Join.AppKey = AppKey;
				//mlmeReq.Req.Join.NbTrials = 3;

				if( NextTx == true )
				{
					LoRaMacMlmeRequest( &mlmeReq );
				}
				DeviceState = DEVICE_STATE_SLEEP;
#else
				// Choose a random device address if not already defined in Commissioning.h
				if( DevAddr == 0 )
				{
					// Random seed initialization
					srand1( BoardGetRandomSeed( ) );

					// Choose a random device address
					DevAddr = randr( 0, 0x01FFFFFF );
				}

				mibReq.Type = MIB_NET_ID;
				mibReq.Param.NetID = LORAWAN_NETWORK_ID;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_DEV_ADDR;
				mibReq.Param.DevAddr = DevAddr;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_NWK_SKEY;
				mibReq.Param.NwkSKey = NwkSKey;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_APP_SKEY;
				mibReq.Param.AppSKey = AppSKey;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_NETWORK_JOINED;
				mibReq.Param.IsNetworkJoined = true;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_A;
                LoRaMacMibSetRequestConfirm( &mibReq );

				DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
#if defined( GPS_CXD5603_ENABLE )
                if(!Get_GPS_Data)
                    SendCmdToGPS();
#endif

                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    //TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                    TxDutyCycleTime = 10000;
                }
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
#if LOW_POWER_MODE                                
                TimerLowPowerHandler( );
#endif                
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
