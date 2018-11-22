

/*******************************************************************************
 *
 *  To manually save a suitable value for variable "compensator" to 
 *  EEPROM, use separate sketch "eeprom_put_for_clinometer.ino", as shown below.
 *    
 *******************************************************************************/


 
/*


// An eeprom_put routine for storing a single float value, to be retreived with another sketch.
// ============================================================================================ 


#include <EEPROM.h>


#define NRML


#ifdef NRML
float_t cmpstr = 0.01435f;      // Simply the "normal" (true horisontal) value for my instrument
#endif

#ifdef KONA
float_t cmpstr = 0.00000f;      // Value for my winter and "gravel" bike
#endif

#ifdef TONY
float_t cmpstr = 0.00000f;      // Value for my touring bike
#endif



void setup() {

    Serial.begin(19200);

    float_t f = cmpstr;         // Calibration compensator value to store. This is an angle expressed
                                // in radians to subtract from single measurements (actual or averaged).
                                // Value is determined empirically for each bike and mounting position.
                            
    int8_t eeAddress = 4;       // Location in EEPROM we want the data to be stored. This could be 
                                // advanced occasionally to avoid unnecessary wear.

    EEPROM.put(eeAddress, f);   // Simple call, with the address first and the object second.


    Serial.println("Written float data type!");
}


void loop() {
                                // Empty loop
}

*/
