/*
    (C)2017 Gemtek

Description: This example describes how to configure LoRaMAC node to class A or Class C

*/

#include "board.h"
#include "LoRaMac.h"

#define CLASS_MODE (DeviceClass_t) CLASS_A //now we support CLASS_A and CLASS_C
//#define CLASS_MODE (DeviceClass_t) CLASS_C //now we support CLASS_A and CLASS_C

/**
 * Main function
 */
int main( void )
{
    /* Declare LoRaMAC MIB-RequestConfirm structure */
    MibRequestConfirm_t mibReq;

    /* Setup the request type */
    mibReq.Type = MIB_DEVICE_CLASS;

    /* Set Class Mode */
    mibReq.Param.Class = CLASS_MODE;

    /* LoRaMAC MIB-Set */
    LoRaMacMibSetRequestConfirm( &mibReq );
}
