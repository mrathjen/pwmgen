//*****************************************************************************
//
// rfid_reader.c - This function communicates with the RFID Reader Module
//                    via UART0. 
//
//*****************************************************************************

#include "rfid_reader.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rit128x96x4.h"
#include "inc/lm3s8962.h"

extern char readId[16];
char adminId[] = {0x02,'5','0','0','0','8','F','6','D','4','E','F','C',0x0D,0x0A,0x03};

short is_rfid_valid(char *id){
    short good = 0;
    // Check the incoming buffer against the Admin ID
    for(short t = 0; t < 16; t++){
      if(id[t] != adminId[t])
        return good;
    }
	// If the RFID matches return 1
    good = 1;
    return good;
}

//*****************************************************************************
//
// function hashit: Hashes a RFID for storage in the array
//
//*****************************************************************************
/*
short hashit(char *toHash){
  short sum = 0;
  // Sum the RFID
  for(short x = 0; x < 16; x++){
   sum += *toHash;
   toHash++;
  }
  sum = sum * 17;
  return (sum % 53);
}
*/
// Clears the buffer that the RFID Data is read into from UART0
void clearRFIDBuff(void){
  for(int x = 0; x<16; x++){
    readId[x] = 0xFF;
  }
}