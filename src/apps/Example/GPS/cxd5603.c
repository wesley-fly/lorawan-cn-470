#include <stddef.h>
#include "board.h"
#include "gpio-board.h"
#include "cxd5603.h"

#define CXD5603_LAT_LEN 10
#define CXD5603_LONG_LEN 11
static GpsIrqHandler *GpsPpsCallback = NULL;

void CXD56037SetPpsCallback(GpsIrqHandler *irqHandler)
{
    GpsPpsCallback = irqHandler;
}


void CXD5603OnPpsSignal( void )
{
    if(GpsPpsCallback != NULL)
        GpsPpsCallback();
}

double CXD5603ConverLatitudeFromDMMtoDD(char *NmeaLatitude)
{
    int i;
    double Latitude;
    double valueTmp1;
    double valueTmp2;
    double valueTmp3;

    // Convert the latitude from ASCII to uint8_t values
    for( i = 0 ; i < CXD5603_LAT_LEN ; i++ )
    {
        NmeaLatitude[i] = NmeaLatitude[i] & 0xF;  
    }
    // Convert latitude from Degree/Minutes.m (DMM) format into decimal
    valueTmp1 = ( double )NmeaLatitude[0] * 10.0 + ( double )NmeaLatitude[1];
		
    valueTmp2 = ( double )NmeaLatitude[2] * 10.0 + ( double )NmeaLatitude[3];
		
    valueTmp3 = ( double )NmeaLatitude[5] * 10000.0 + ( double )NmeaLatitude[6] * 1000.0 + 
                ( double )NmeaLatitude[7] * 100.0 	+ ( double )NmeaLatitude[8] * 10.0 	 +
								( double )NmeaLatitude[9]	;
    
    Latitude = valueTmp1 + ( ( valueTmp2 + ( valueTmp3 * 0.00001 ) ) / 60.0 ); 
	
    return Latitude;
}

double CXD5603ConverLongitudeFromDMMtoDD(char *NmeaLongitude)
{
    int i;
    
    double Longitude = 0;
    double valueTmp1;
    double valueTmp2;
    double valueTmp3;

    // Convert the longitude from ASCII to uint8_t values
    for( i = 0 ; i < CXD5603_LONG_LEN ; i++ )
    {
        NmeaLongitude[i] = NmeaLongitude[i] & 0xF;  
    }
    // Convert longitude from Degree/Minutes.m (DMM) format into decimal
		
    valueTmp1 = ( double )NmeaLongitude[0] * 100.0 + ( double )NmeaLongitude[1] * 10.0 + ( double )NmeaLongitude[2];
    
		valueTmp2 = ( double )NmeaLongitude[3] * 10.0 + ( double )NmeaLongitude[4];
    
		valueTmp3 = ( double )NmeaLongitude[6] * 10000.0 + ( double )NmeaLongitude[7] * 1000.0 +
								( double )NmeaLongitude[8] * 100.0 	 + ( double )NmeaLongitude[9] * 10.0 	 +
								( double )NmeaLongitude[10] ;
    
		Longitude = valueTmp1 + ( ( valueTmp2 + ( valueTmp3 * 0.00001 ) ) / 60.0 );
		
        return Longitude;
}

bool CXD5603ConverLatitudePole(char *NmeaLatitudePole)
{
    if(NmeaLatitudePole[0] == 'S' )
    {
				return false;		//hanson mod
    }
		else
		{
				return true;
		}
}

bool CXD5603ConverLongitudePole(char *NmeaLongitudePole)
{
		
    if( NmeaLongitudePole[0] == 'W' )
    {
				return false;
    }
		else
		{
				return true;
		}
}
