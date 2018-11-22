/**
 * "Clinometer" Arduino sketch for the MPU6050 integrated accelerometer/gyrometer chip.
 * -----------------------------------------------------------------------------------
 * The Digital Motion Processor (DMP) of this sensor is accessed via Jeff Rowberg's MotionApps v2.0  
 * I2C device class library (MIT license).
 * 
 * Display is a monochrome Nokia 5110 LCD, code for which is based on a library example sketch
 * written by Limor Fried/Ladyada (BSD license).
 *
 * Running Average function is based on an example by Rob Tillaart (MIT license),
 *
 * Any other relevant contributions are by Martin Bergman <bergman.martin at gmail dot com> 2016.
 * License: CC BY-SA v4.0 - http://creativecommons.org/licenses/by-sa/4.0/legalcode 
 *  
 * µC <--> MPU6050
 * ==============
 * A4 <--> SDA
 * A5 <--> SCL
 * D2 <--> INT
 * 3V3 <-> Vcc
 * GND <-> GND
 *
 * µC <--> 5110/PCD8544
 * ===================
 * D6 <--> Serial clock out (SCLK)
 * D7 <--> Serial data out (DIN)
 * D8 <--> Data/Command select (D/C)
 * D9 <--> LCD chip select (CS)
 * D10 <-> LCD reset (RST)
 * 3V3 <-> Vcc
 * GND <-> GND
 *
 * (Compiles with Arduino IDE 1.6.6 and 1.6.12)
 **************************************************************************************/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"            // Not necessary if using MotionApps include file
#include "Wire.h"
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <EEPROM.h>

#define CAL_PIN  11             // Calibration pushbutton pin.
#define RAW_PIN  A2             // Battery voltage monitor input via a 1 to 2 potential divider.
#define MPU_PIN   2             // Pin connected to interrupt signal from MPU6050.
#define LM_SIZE  70             // Number of samples for the running average function. (70 is circa 1 s)
#define AAY_SWAY 250            // Max accel absolute value (aaReal.y, averaged) that will be ignored.
#define NUDGE     8             // A tweak factor for the output, to be determined empirically.
//#define DEBUGGER                // Uncomment to get information on serial output.

#ifdef DEBUGGER
    #define DBG_PRINT Serial.print
    #define DBG_PRINTLN Serial.println
#else
    #define DBG_PRINT
    #define DBG_PRINTLN
#endif


MPU6050             mpu;        // The class' default I2C address is 0x68 (= ADO low)
Adafruit_PCD8544    display = Adafruit_PCD8544(6, 7, 8, 9, 10);

// MPU control/status vars  ******************************************
bool        dmpReady = false;   // Will be set true if DMP init is successful
uint8_t     mpuIntStatus;       // Holds actual interrupt status byte from MPU
uint8_t     devStatus;          // Return status after each device operation (0 = success, !0 = error)
uint16_t    packetSize;         // Expected DMP packet size (default is 42 bytes)
uint16_t    fifoCount;          // Count of all bytes currently in FIFO
uint8_t     fifoBuffer[64];     // FIFO storage buffer

// Orientation/motion vars  ******************************************
Quaternion  q;                  // [w, x, y, z]         quaternion container
VectorInt16 aa;                 // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;             // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;            // [x, y, z]            gravity vector
float       euler[3];           // [psi, theta, phi]    Euler angle container
float       ypr[3];             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Output processing vars ********************************************
uint8_t     interrupt_count     = 0;        // Counts interrupts in order to select samples to average.
uint8_t     samples_count       = 0;        // Counts the number of samples currently in the bag (0 - 10).
float       sample_accumulator  = 0.0;      // The bag. (It's stuffed with radians.)
int32_t     aay_ra              = 0;        // Smoothed aaReal.y variable
int8_t      delay_aay           = 1;        // Counter for freezing output disturbed by runtime jitter.

int8_t          calib_status    = 0;        // 0 == Not calibrated; 1 == Started; 2 == Finished
float           pitch_percent   = 0.0;
float           pitch_radians   = 0.0;
float           compensator     = 0.0f; // Calibration value (radians) to compensate for device misalignment.

uint16_t        prev_adc_value  = 0;
float           voltage_divisor = 107.5;    // For approximating battery voltage from adc values
float           battery_volts   = 0.00;

uint8_t         timer1_counter;
uint32_t        monitor_pinger  = millis(); // For refreshing the battery voltage monitor 
static uint32_t boot_waiter     = millis(); // Starts boot-up timing routine already.
bool            go              = false;    // go goes true only when device has settled after booting
volatile bool   mpuInterrupt    = false;    // Indicates whether MPU interrupt pin has gone high
volatile bool   showtime        = false;    // Indicates that it is high time for a display update
volatile bool   even_sec        = false;    // On/off variable for enabling a blinking display text


static uint8_t calib_comp_addr  = 4;        // EEPROM byte-address of saved compensator value.

size_t print(uint32_t, uint8_t  = 3);       // Redefiniton of default number of decimals in Print.h.


// Interrupt Service Routines ******************************************** 

void dmpDataReady() {           // The actual DMP ISR -- Nice and short:
    mpuInterrupt = true;        // New data is in the pipeline
}

ISR(TIMER1_OVF_vect) {          // Timer ISR 
    TCNT1 = timer1_counter;     // Reload timer
    showtime = true;            // Hoist the display updater flag every second
    even_sec = !even_sec;       // State switching between even/odd seconds
}


/// ################################################################# BEGIN SETUP ############

void setup()
{
    Wire.begin();               // Join I2C bus (since I2Cdev library doesn't do this automatically)
    TWBR = 24;                  // 400kHz I2C clock (or 200kHz if CPU is running at 8MHz)

    Serial.begin(19200);        // For 8MHz or slower host processor, use maximally 38400 baud.
    
// =============================================================================

    // Initialize timer1 
    noInterrupts();             // Disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    
//  timer1_counter = 34286;     // Preload timer 65536-16MHz/256/2Hz
    timer1_counter = 17143;     // Preload timer 65536-16MHz/256/1Hz
    
    TCNT1 = timer1_counter;     // Preload timer
    TCCR1B |= (1 << CS12);      // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);     // Enable timer overflow interrupt
    interrupts();               // Enable all interrupts
    
// =============================================================================


    Serial.println(F("\n -- Clinometer v24 --\n"));

    // Initialize accelerometer
    DBG_PRINTLN(F("Initializing I2C devices...\n"));
    mpu.initialize();
    pinMode(MPU_PIN, INPUT);

    // Verify connection
    DBG_PRINTLN(F("Testing device connections..."));
    DBG_PRINTLN(mpu.testConnection() ? F("MPU6050 connection successful\n") : F("MPU6050 connection failed"));

    // Load and configure the DMP
    DBG_PRINTLN(F("Initializing DMP...\n"));
    devStatus = mpu.dmpInitialize();
    
        // ********************************************************************
            // Supply your own gyro offsets here, scaled for min sensitivity
//            mpu.setXGyroOffset(220);
//            mpu.setYGyroOffset(76);
//            mpu.setZGyroOffset(-85);
//            mpu.setZAccelOffset(1788); // 1788 factory default for my test chip
            mpu.setXGyroOffset(16);
            mpu.setYGyroOffset(15);
            mpu.setZGyroOffset(90);
            mpu.setXAccelOffset(-623);
            mpu.setYAccelOffset(1235);
            mpu.setZAccelOffset(1641); // 1788 factory default for my test chip
        // ********************************************************************

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready
        DBG_PRINTLN(F("Enabling DMP...\n"));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt (INT0) detection
        DBG_PRINTLN(F("Enabling interrupt detection (Arduino INT0)...\n"));
        attachInterrupt(digitalPinToInterrupt(MPU_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        DBG_PRINTLN(F("DMP ready! Waiting for first interrupt...\n"));
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      /* ERROR CODES:
         1 = initial memory load failed   (if it's going to break, then usually the code will be 1)
         2 = DMP configuration updates failed  */
        DBG_PRINT(F("DMP Initialization failed (code "));
        DBG_PRINT(devStatus);
        DBG_PRINTLN(F(")"));
    }

// =============================================================================

    // Initialize the LCD display
    display.begin();
    display.setContrast(60);

    splash();

    // Configure other I/O pins
    pinMode(CAL_PIN, INPUT);
    pinMode(RAW_PIN, INPUT);

    // Retrieve persistent value from EEPROM
    EEPROM.get(calib_comp_addr, compensator);   // Latest calibration makes the new default value
    DBG_PRINT(F("Retreiving EEPROM..."));
    DBG_PRINT(F("\n    compensator val: ")); DBG_PRINTLN(compensator, 5);

#ifndef DEBUGGER
    Serial.println(F("\n    No debugging tonight ;~)\n    **********************"));
#endif

}


/// ################################################################# BEGIN LOOP #############

void loop()
{
    if (!dmpReady) return;          // In case programming in setup() failed, don't try to do anything.

    // Assuming everything is hunky-dory:
    while (!mpuInterrupt) {   // While waiting for an MPU interrupt, we can manage the display.
        
        if (mpuInterrupt && fifoCount < packetSize){
            fifoCount = mpu.getFIFOCount();     /// NYTT: Try to break out of the infinite loop 
        }

        if (!go){                               // First case: We've just booted and...
            uint32_t now = millis();            // ... should begin by wasting some time here.
            if ((now - boot_waiter) > 20000){   // Take a 20 secs hiatus till output stabilizes.
                go = true;                      // Now we can start in earnest.
            }
        }

        else {
            if (showtime) {                     // Second case: Both <go> AND <showtime> are true.
                uint32_t now2 = millis();
                if ((now2 - monitor_pinger) > 10000){   // Keep the beat for the voltage monitoring.
                    battery_volts = monitorRefresh();   // (So this updates every 10 secs)
                    monitor_pinger = now2;
                }
                if ((abs(aay_ra)) < AAY_SWAY){          // Unless the aayReal.y signal is not too perturbed.
                    delay_aay--;                        // Counts down: 3=wait, 2=wait, 1=wait, 0=Go!.
                    if (!delay_aay) {                   // Down-counter has reached zero (0) and we go on.
                        // We can now proceed to perform the heavy lifting:
                        pitch_percent = (tan((sample_accumulator/samples_count)-compensator)) * (100 + NUDGE); // Phew!
                        showIsOver();
                        nokinoki(pitch_percent, false); // Draw to the "Nokia" LCD screen once a second.
                        delay_aay = 1;                  // Down-counter now can become 0 in next loop
                        DBG_PRINT("--\t");
                    }
                    else {                              // Delay down-counter is still not at zero (0).
                        pitch_percent = pitch_percent;  // Propagate last known-good value again.
                        showIsOver();      
                        nokinoki(pitch_percent, true);  // Draw to the "Nokia" LCD screen, with an alert.
                        DBG_PRINT("-#\t");
                    }
                }
                else {                                  // Y-axis "real" accelerations are messing it up.
                    pitch_percent = pitch_percent;      // Propagate last known-good value again.
                    showIsOver();
                    nokinoki(pitch_percent, true);      // Draw to the "Nokia" LCD screen, with an alert.
                    delay_aay = 3;                      // Reset the down-counting variable.
                    DBG_PRINT("##\t");
                }
                DBG_PRINT(pitch_percent);
                DBG_PRINT("\t");
                DBG_PRINTLN(aay_ra);
            }

            else if (digitalRead(CAL_PIN)){     // Third case: <go> AND <"button push"> but NOT <showtime>
                if (calib_status == 1){         // The calibration process has already been started,
                    calib_end();                // meaning now is the time to wrap it up.
                }
                else {                          // calib_status is either 0 (inited) or 2 (finished)
                    calib_begin();              // and we will now (re)start the calibration process.
                }
            }
        }
    }


    // Okay, the mpuInterrupt flag is now officially up! -- Reset it and get INT0_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();


    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
///        interrupt_count ++;   // Prevents rare crashes due to sampling gettting out of sync here

    // Otherwise, check for DMP data ready interrupt. (This should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // Wait for correct available data length. (Should be a VERY short wait)
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get the yaw/pitch/roll angles in radians "In Three Easy Steps"
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  // "Pitch" for us is really the sensor's roll-angle!
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        aay_ra = runningAverage(aaReal.y);          // Apply an RA smoothing filter

        // As the sensor seems to trigger the data-ready interrupt at 69-70 Hz, we get a nicely spread
        // sampling of circa ten items per second by saving the data from every seventh interrupt to an
        // accumulating variable, then dividing the sum by the number of samples (which should normally
        // amount to ten).
        interrupt_count ++;             
        if ((interrupt_count >= 7) && (samples_count < 12)){ // It's time to bag another sample.(And if 
            sample_accumulator += ypr[2];       // the bag is full the accumulated value stays on hold)
            interrupt_count = 0;
            samples_count ++;                   // Expect the showtime routine (above) to read and
                                                // reset samples_count before it hits 12.
        }
        else if (samples_count >= 12) {interrupt_count = 0;} // Skip counting when bag is already full
    }
}

/// ################################################################# END LOOP ###############


// Functions *************************************************

int16_t runningAverage(int16_t mess) {
    static int16_t  lm[LM_SIZE];                // Latest Measurements array
    static int8_t   index = 0;
    static int32_t  sum   = 0;
    static int8_t   count = 0;
    
    // Keep sum updated to improve speed.
    sum -= lm[index];                           // Subtract old value from sum
    lm[index] = mess;                           // Save new value
    sum += lm[index];                           // Add new value to sum
    index++;                                    // Increment index
    index = index % LM_SIZE;                    // Restart indexing when LM_SIZE is reached
    if (count < LM_SIZE) count++;               // Increment if array is not yet fully populated
    
    return sum / count;
}

void showIsOver() {
    showtime = false;                           // Reset flag and counter ...
    samples_count = 0;
    sample_accumulator = 0.0;                   // ... and purge the purse.
}


void calib_begin(void) {
    compensator = 0;                            // Delete value left over from possible earlier attempts
    pitch_radians = ypr[2];                     // Get angle in radians when in first orientation
    while (digitalRead(CAL_PIN));               // Wait until the button is released
    calib_status = 1;                           // Ready to act on the next button push
    printPrompt(calib_status);                  // Draw the little arrow-head symbol.
}

void calib_end(void) {
    compensator = (pitch_radians + ypr[2]) / 2; // Save half of the sum of the measurements

    while (digitalRead(CAL_PIN));               // Finally wait until button is released a second time
    calib_status = 2;                           // All set for now
    printPrompt(calib_status);                  // Display the two (counterpoised) arrowheads
///    EEPROM.put(calib_comp_addr, compensator);   // Finally save compensator as the new default. CRASHES!
    
    DBG_PRINT(F("\n    Put to eeprom: ")); DBG_PRINTLN(compensator, 5); // A serial message for debugging,
    DBG_PRINTLN(F("+++++++++++++++++++++++++++++++++++++++++ ")); // (use separate eeprom-writing sketch.)
}
