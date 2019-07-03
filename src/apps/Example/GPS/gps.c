/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic driver for any GPS receiver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "gps.h"
#include "utilities.h"
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>

#define NMEA_TYPE_PGTOP     0
#define NMEA_TYPE_GPGGA     1
#define NMEA_TYPE_GPGSA     2
#define NMEA_TYPE_GPRMC     3
#define NMEA_TYPE_GPVTG     4

#define NMEA_NAME_VAL       7

uint8_t NmeaPgtopString[NMEA_NAME_VAL] = "$PGTOP";
uint8_t NmeaGpggaString[NMEA_NAME_VAL] = "$GPGGA";
uint8_t NmeaGpgsaString[NMEA_NAME_VAL] = "$GPGSA";
uint8_t NmeaGprmcString[NMEA_NAME_VAL] = "$GPRMC";
uint8_t NmeaGpvtgString[NMEA_NAME_VAL] = "$GPVTG";
uint8_t NmeaGpgsvString[NMEA_NAME_VAL] = "$GPGSV";

/* Various type of NMEA data we can receive with the Gps */
const char NmeaDataTypeGPGGA[] = "GPGGA";   //This one is in use
const char NmeaDataTypeGPGSA[] = "GPGSA";
const char NmeaDataTypeGPGSV[] = "GPGSV";
const char NmeaDataTypeGPRMC[] = "GPRMC";   //And this one.

#define NMEA_TOKS_COMPARE   (1)
#define NMEA_TOKS_PERCENT   (2)
#define NMEA_TOKS_WIDTH     (3)
#define NMEA_TOKS_TYPE      (4)

uint8_t GpsCheckGPGGA(uint8_t *gpsData)
{
    uint8_t CheckNmeaName[NMEA_NAME_VAL];
    memcpy(CheckNmeaName, gpsData, (NMEA_NAME_VAL-1));
    CheckNmeaName[NMEA_NAME_VAL - 1] = '\0';
    if( strcmp( (char *)CheckNmeaName, (char *)NmeaGpggaString) == 0){
        return SUCCESS;
    }
    else{
        return FAIL;
    }
}

uint8_t GpsCheckGPRMC(uint8_t *gpsData)
{
    uint8_t CheckNmeaName[NMEA_NAME_VAL];
    memcpy(CheckNmeaName, gpsData, (NMEA_NAME_VAL-1));
    CheckNmeaName[NMEA_NAME_VAL - 1] = '\0';
    if( strcmp( (char *)CheckNmeaName, (char *)NmeaGprmcString) == 0){
        return SUCCESS;
    }
    else{
        return FAIL;
    }
}

uint8_t GpsCheckGPGSV(uint8_t *gpsData)
{
    uint8_t CheckNmeaName[NMEA_NAME_VAL];
    memcpy(CheckNmeaName, gpsData, (NMEA_NAME_VAL-1));
    CheckNmeaName[NMEA_NAME_VAL - 1] = '\0';
    if( strcmp( (char *)CheckNmeaName, (char *)NmeaGpgsvString) == 0){
        return SUCCESS;
    }
    else{
        return FAIL;
    }
}

/*!
 * Calculates the checksum for a NMEA sentence
 *
 * Skip the first '$' if necessary and calculate checksum until '*' character is
 * reached (or buffSize exceeded).
 *
 * \retval chkPosIdx Position of the checksum in the sentence
 */
int32_t GpsNmeaChecksum( int8_t *nmeaStr, int32_t nmeaStrSize, int8_t * checksum )
{
    int i = 0;
    uint8_t checkNum = 0;

    // Check input parameters
    if( ( nmeaStr == NULL ) || ( checksum == NULL ) || ( nmeaStrSize <= 1 ) )
    {
        return -1;
    }

    // Skip the first '$' if necessary
    if( nmeaStr[i] == '$' )
    {
        i += 1;
    }

    // XOR until '*' or max length is reached
    while( nmeaStr[i] != '*' )
    {
        checkNum ^= nmeaStr[i];
        i += 1;
        if( i >= nmeaStrSize )
        {
            return -1;
        }
    }

    // Convert checksum value to 2 hexadecimal characters
    checksum[0] = Nibble2HexChar( checkNum / 16 ); // upper nibble
    checksum[1] = Nibble2HexChar( checkNum % 16 ); // lower nibble

    return i + 1;
}

/*!
 * Calculate the checksum of a NMEA frame and compare it to the checksum that is
 * present at the end of it.
 * Return true if it matches
 */
static bool GpsNmeaValidateChecksum( int8_t *serialBuff, int32_t buffSize )
{
    int32_t checksumIndex;
    int8_t checksum[2]; // 2 characters to calculate NMEA checksum

    checksumIndex = GpsNmeaChecksum( serialBuff, buffSize, checksum );

    // could we calculate a verification checksum ?
    if( checksumIndex < 0 )
    {
        return false;
    }

    // check if there are enough char in the serial buffer to read checksum
//    if( checksumIndex >= ( buffSize - 2 ) )
//    {
//        return false;
//    }

    // check the checksum
    if( ( serialBuff[checksumIndex] == checksum[0] ) && ( serialBuff[checksumIndex + 1] == checksum[1] ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t GpsParseGpsData( int8_t *rxBuffer, int32_t rxBufferSize, tNmeaGpsData * NmeaGpsData)
{
    uint8_t i = 1;
    uint8_t j = 0;
    uint8_t fieldSize = 0;
    
    if( rxBuffer[0] != '$' )
    {
        return FAIL;
    }

    if( GpsNmeaValidateChecksum( rxBuffer, rxBufferSize ) == false )
    {
        return FAIL;
    }

    fieldSize = 0;
    while( rxBuffer[i + fieldSize++] != ',' )
    {
        if( fieldSize > 6 )
        {
            return FAIL;
        }
    }
    for( j = 0; j < fieldSize; j++, i++ )
    {
        NmeaGpsData->NmeaDataType[j] = rxBuffer[i];
    }
    
    // Parse the GPGGA data 
    if( strncmp( ( const char* )NmeaGpsData->NmeaDataType, ( const char* )NmeaDataTypeGPGGA, 5 ) == 0 )
    {  
        // NmeaUtcTime
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_UTC_TIME_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize-1); j++, i++ )
        {
            NmeaGpsData->NmeaUtcTime[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaUtcTime[j] = 0x00;
        i++;
        // NmeaLatitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LAT_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLatitude[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLatitude[j] = 0x00;
        i++;
        // NmeaLatitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LAT_POL_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLatitudePole[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLatitudePole[j] = 0x00;
        i++;
        // NmeaLongitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LONG_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLongitude[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLongitude[j] = 0x00;
        i++;
        // NmeaLongitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LONG_POL_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLongitudePole[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLongitudePole[j] = 0x00;
        i++;
        // NmeaFixQuality
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_FIX_QL_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaFixQuality[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaFixQuality[j] = 0x00;
        i++;
        // NmeaSatelliteTracked
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_SAT_TRACKED_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaSatelliteTracked[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaSatelliteTracked[j] = 0x00;
        i++;
        // NmeaHorizontalDilution
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_HDOP_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaHorizontalDilution[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaHorizontalDilution[j] = 0x00;
        i++;
        // NmeaAltitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_ALT_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaAltitude[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaAltitude[j] = 0x00;
        i++;
        // NmeaAltitudeUnit
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_ALT_UNIT_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaAltitudeUnit[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaAltitudeUnit[j] = 0x00;
        i++;
        // NmeaHeightGeoid
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_HT_GEO_ID_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaHeightGeoid[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaHeightGeoid[j] = 0x00;
        i++;
        // NmeaHeightGeoidUnit
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_HT_GEO_ID_UNIT_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaHeightGeoidUnit[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaHeightGeoidUnit[j] = 0x00;
        return SUCCESS;
    }
    
    // Parse the GPRMC data 
    else if ( strncmp( ( const char* )NmeaGpsData->NmeaDataType, ( const char* )NmeaDataTypeGPRMC, 5 ) == 0 )
    {    
        // NmeaUtcTime
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_UTC_TIME_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaUtcTime[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaUtcTime[j] = 0x00;
        i++;
        // NmeaDataStatus
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_DATA_STATUS_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaDataStatus[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaDataStatus[j] = 0x00;
        i++;
        // NmeaLatitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LAT_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLatitude[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLatitude[j] = 0x00;
        i++;
        // NmeaLatitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LAT_POL_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLatitudePole[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLatitudePole[j] = 0x00;
        i++;
        // NmeaLongitude
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LONG_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLongitude[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLongitude[j] = 0x00;
        i++;
        // NmeaLongitudePole
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_LONG_POL_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaLongitudePole[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaLongitudePole[j] = 0x00;
        i++;
        // NmeaSpeed
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_SPEED_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaSpeed[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaSpeed[j] = 0x00;
        i++;
        // NmeaDetectionAngle
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_DET_ANGLE_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaDetectionAngle[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaDetectionAngle[j] = 0x00;
        i++;
        // NmeaDate
        fieldSize = 0;
        while( rxBuffer[i + fieldSize++] != ',' )
        {
            if( fieldSize > GPS_DATE_LEN )
            {
                return FAIL;
            }
        }
        for( j = 0; j < (fieldSize - 1); j++, i++ )
        {
            NmeaGpsData->NmeaDate[j] = rxBuffer[i];
        }
        NmeaGpsData->NmeaDate[j] = 0x00;
        return SUCCESS;
    }
    else
    {
        return FAIL;
    }
}

/**
 * \brief Convert string to fraction number
 */
double nmea_atof(const char *str, int str_sz)
{
    char *tmp_ptr;
    char buff[NMEA_CONVSTR_BUF];
    double res = 0;

    if(str_sz < NMEA_CONVSTR_BUF)
    {
        memcpy(&buff[0], str, str_sz);
        buff[str_sz] = '\0';
        res = strtod(&buff[0], &tmp_ptr);
    }

    return res;
}

/**
 * \brief Convert string to number
 */
int nmea_atoi(const char *str, int str_sz, int radix)
{
    char *tmp_ptr;
    char buff[NMEA_CONVSTR_BUF];
    int res = 0;

    if(str_sz < NMEA_CONVSTR_BUF)
    {
        memcpy(&buff[0], str, str_sz);
        buff[str_sz] = '\0';
        res = strtol(&buff[0], &tmp_ptr, radix);
    }

    return res;
}

/**
 * \brief Analyse string (specificate for NMEA sentences)
 */
int nmea_scanf(const char *buff, int buff_sz, const char *format, ...)
{
    const char *beg_tok;
    const char *end_buf = buff + buff_sz;

    va_list arg_ptr;
    int tok_type = NMEA_TOKS_COMPARE;
    int width = 0;
    const char *beg_fmt = 0;
    int snum = 0, unum = 0;

    int tok_count = 0;
    void *parg_target;

    va_start(arg_ptr, format);
    
    for(; *format && buff < end_buf; ++format)
    {
        switch(tok_type)
        {
        case NMEA_TOKS_COMPARE:
            if('%' == *format)
                tok_type = NMEA_TOKS_PERCENT;
            else if(*buff++ != *format)
                goto fail;
            break;
        case NMEA_TOKS_PERCENT:
            width = 0;
            beg_fmt = format;
            tok_type = NMEA_TOKS_WIDTH;
        case NMEA_TOKS_WIDTH:
            if(isdigit(*format))
                break;
            {
                tok_type = NMEA_TOKS_TYPE;
                if(format > beg_fmt)
                    width = nmea_atoi(beg_fmt, (int)(format - beg_fmt), 10);
            }
        case NMEA_TOKS_TYPE:
            beg_tok = buff;

            if(!width && ('c' == *format || 'C' == *format) && *buff != format[1])
                width = 1;

            if(width)
            {
                if(buff + width <= end_buf)
                    buff += width;
                else
                    goto fail;
            }
            else
            {
                if(!format[1] || (0 == (buff = (char *)memchr(buff, format[1], end_buf - buff))))
                    buff = end_buf;
            }

            if(buff > end_buf)
                goto fail;

            tok_type = NMEA_TOKS_COMPARE;
            tok_count++;

            parg_target = 0; width = (int)(buff - beg_tok);

            switch(*format)
            {
            case 'c':
            case 'C':
                parg_target = (void *)va_arg(arg_ptr, char *);
                if(width && 0 != (parg_target))
                    *((char *)parg_target) = *beg_tok;
                break;
            case 's':
            case 'S':
                parg_target = (void *)va_arg(arg_ptr, char *);
                if(width && 0 != (parg_target))
                {
                    memcpy(parg_target, beg_tok, width);
                    ((char *)parg_target)[width] = '\0';
                }
                break;
            case 'f':
            case 'g':
            case 'G':
            case 'e':
            case 'E':
                parg_target = (void *)va_arg(arg_ptr, double *);
                if(width && 0 != (parg_target))
                    *((double *)parg_target) = nmea_atof(beg_tok, width);
                break;
            };

            if(parg_target)
                break;
            if(0 == (parg_target = (void *)va_arg(arg_ptr, int *)))
                break;
            if(!width)
                break;

            switch(*format)
            {
            case 'd':
            case 'i':
                snum = nmea_atoi(beg_tok, width, 10);
                memcpy(parg_target, &snum, sizeof(int));
                break;
            case 'u':
                unum = nmea_atoi(beg_tok, width, 10);
                memcpy(parg_target, &unum, sizeof(unsigned int));
                break;
            case 'x':
            case 'X':
                unum = nmea_atoi(beg_tok, width, 16);
                memcpy(parg_target, &unum, sizeof(unsigned int));
                break;
            case 'o':
                unum = nmea_atoi(beg_tok, width, 8);
                memcpy(parg_target, &unum, sizeof(unsigned int));
                break;
            default:
                goto fail;
            };

            break;
        };
    }

fail:

    va_end(arg_ptr);

    return tok_count;
}

/**
 * \brief Parse GSV packet from buffer.
 * @param buff a constant character pointer of packet buffer.
 * @param buff_sz buffer size.
 * @param pack a pointer of packet which will filled by function.
 * @return 1 (true) - if parsed successfully or 0 (false) - if fail.
 */
int nmea_parse_GPGSV(const char *buff, int buff_sz, nmeaGPGSV *pack)
{
    int nsen, nsat;

//    NMEA_ASSERT(buff && pack);

    memset(pack, 0, sizeof(nmeaGPGSV));

//    nmea_trace_buff(buff, buff_sz);

    nsen = nmea_scanf(buff, buff_sz,
        "$GPGSV,%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%d,%d,%d,%d*",
        &(pack->pack_count), &(pack->pack_index), &(pack->sat_count),
        &(pack->sat_data[0].id), &(pack->sat_data[0].elv), &(pack->sat_data[0].azimuth), &(pack->sat_data[0].sig),
        &(pack->sat_data[1].id), &(pack->sat_data[1].elv), &(pack->sat_data[1].azimuth), &(pack->sat_data[1].sig),
        &(pack->sat_data[2].id), &(pack->sat_data[2].elv), &(pack->sat_data[2].azimuth), &(pack->sat_data[2].sig),
        &(pack->sat_data[3].id), &(pack->sat_data[3].elv), &(pack->sat_data[3].azimuth), &(pack->sat_data[3].sig));

    nsat = (pack->pack_index - 1) * NMEA_SATINPACK;
    nsat = (nsat + NMEA_SATINPACK > pack->sat_count)?pack->sat_count - nsat:NMEA_SATINPACK;
    nsat = nsat * 4 + 3 /* first three sentence`s */;

    if(nsen < nsat || nsen > (NMEA_SATINPACK * 4 + 3))
    {
        //nmea_error("GPGSV parse error!");
        return 0;
    }

    return 1;
}

