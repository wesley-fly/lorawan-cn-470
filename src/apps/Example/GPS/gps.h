/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic driver for the GPS receiver UP501

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __GPS_H__
#define __GPS_H__
#include <stdint.h>
#define GPS_TYPE_LEN						6
#define GPS_UTC_TIME_LEN				10
#define GPS_DATA_STATUS_LEN			2
#define GPS_LAT_LEN							11
#define GPS_LAT_POL_LEN					2
#define GPS_LONG_LEN						12
#define GPS_LONG_POL_LEN				2
#define GPS_FIX_QL_LEN					2
#define GPS_SAT_TRACKED_LEN			3
#define GPS_HDOP_LEN         		6
#define GPS_ALT_LEN             8
#define GPS_ALT_UNIT_LEN        2
#define GPS_HT_GEO_ID_LEN       8
#define GPS_HT_GEO_ID_UNIT_LEN  2
#define GPS_SPEED_LEN           8
#define GPS_DET_ANGLE_LEN       8
#define GPS_DATE_LEN            8

#define NMEA_MAXSAT         (12)
#define NMEA_SATINPACK      (4)
#define NMEA_NSATPACKS      (NMEA_MAXSAT / NMEA_SATINPACK)
#define NMEA_CONVSTR_BUF    (80)

/* Structure to handle the GPS parsed data in ASCII */
typedef struct
{
    char NmeaDataType[GPS_TYPE_LEN];
    char NmeaUtcTime[GPS_UTC_TIME_LEN];
    char NmeaDataStatus[GPS_DATA_STATUS_LEN];
    char NmeaLatitude[GPS_LAT_LEN];
    char NmeaLatitudePole[GPS_LAT_POL_LEN];
    char NmeaLongitude[GPS_LONG_LEN];
    char NmeaLongitudePole[GPS_LONG_POL_LEN];
    char NmeaFixQuality[GPS_FIX_QL_LEN];
    char NmeaSatelliteTracked[GPS_SAT_TRACKED_LEN];
    char NmeaHorizontalDilution[GPS_HDOP_LEN];
    char NmeaAltitude[GPS_ALT_LEN];
    char NmeaAltitudeUnit[GPS_ALT_UNIT_LEN];
    char NmeaHeightGeoid[GPS_HT_GEO_ID_LEN];
    char NmeaHeightGeoidUnit[GPS_HT_GEO_ID_UNIT_LEN];
    char NmeaSpeed[GPS_SPEED_LEN];
    char NmeaDetectionAngle[GPS_DET_ANGLE_LEN];
    char NmeaDate[GPS_DATE_LEN];
}tNmeaGpsData;

/**
 * Information about satellite
 * @see nmeaSATINFO
 * @see nmeaGPGSV
 */
typedef struct _nmeaSATELLITE
{
    int     id;         /**< Satellite PRN number */
    int     in_use;     /**< Used in position fix */
    int     elv;        /**< Elevation in degrees, 90 maximum */
    int     azimuth;    /**< Azimuth, degrees from true north, 000 to 359 */
    int     sig;        /**< Signal, 00-99 dB */

} nmeaSATELLITE;

/**
 * GSV packet information structure (Satellites in view)
 */
typedef struct _nmeaGPGSV
{
    int     pack_count; /**< Total number of messages of this type in this cycle */
    int     pack_index; /**< Message number */
    int     sat_count;  /**< Total number of satellites in view */
    nmeaSATELLITE sat_data[NMEA_SATINPACK];

} nmeaGPGSV;

/********/

/*!
 * \brief Parses the NMEA sentence.
 *
 * \remark Only parses GPGGA and GPRMC sentences
 *
 * \param [IN] rxBuffer Data buffer to be parsed
 * \param [IN] rxBufferSize Size of data buffer
 *
 * \retval status [OK, FAIL]
 */
uint8_t GpsParseGpsData( int8_t *rxBuffer, int32_t rxBufferSize, tNmeaGpsData * NmeaGpsData);
int nmea_parse_GPGSV(const char *buff, int buff_sz, nmeaGPGSV *pack);

uint8_t GpsCheckGPGGA(uint8_t *gpsData);
uint8_t GpsCheckGPRMC(uint8_t *gpsData);
uint8_t GpsCheckGPGSV(uint8_t *gpsData);
#endif  // __GPS_H__
