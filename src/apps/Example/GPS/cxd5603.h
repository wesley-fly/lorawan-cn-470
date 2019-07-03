#ifndef __CXD5603_H__
#define __CXD5603_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
    
typedef void( GpsIrqHandler )(void);

void CXD5603SetPpsCallback(GpsIrqHandler *irqHandler);
void CXD5603OnPpsSignal( void );
double CXD5603ConverLatitudeFromDMMtoDD(char *NmeaLatitude);
double CXD5603ConverLongitudeFromDMMtoDD(char *NmeaLongitude);
bool CXD5603ConverLatitudePole(char *NmeaLatitudePole);
bool CXD5603ConverLongitudePole(char *NmeaLongitudePole);

#ifdef __cplusplus
}
#endif
#endif // __CXD5603_H__
