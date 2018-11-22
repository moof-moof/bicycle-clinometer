

void splash(void) {
    display.clearDisplay();         // Clear the screen and buffer
    display.setTextColor(WHITE, BLACK);
    display.setTextSize(1);         // Text size to paint canvas all black
    display.println                 // Print 70 spaces to practically fill the canvas
    ("                                                                      "); 
    display.setTextSize(2);         // Size for splash text
    display.setCursor(7,10);
    display.println("Boot'n");
    display.display();
    display.clearDisplay();
}

void nokinoki(float p_p, bool aay_alert) {
                                    // Main routine for the Nokia 5110 screen: We define three screen
                                    // positions for the digits, where the last two positions are
                                    // always populated: #1 for "tens", #2 for "ones", #3 for 1/10ths.
                                    // The incoming inclination value (which is a float) is hacked
                                    // and formatted into displayable figures (characters).
    if (p_p < 0){
        p_p = (p_p * (-1));         // Make all values positives
    }
    int8_t procent = (int8_t) p_p;  // A typecast saves the integral part
    int16_t promill = 10 * p_p;     // Swapping to per-mille values temporarily
    int8_t tenths = promill - (10 * procent);  // Extracting the fractional part (mantissa)
    int8_t tens = 0;

    if (procent > 29){              // More than 29.9% incline is considered "insurmountable": 
        procent = 0;                // Just devaluate...
        silly();                    // ... showing an appropriate word of advice
        return;
    }
    else if (procent > 19){
        tens = 2;
        procent -= 20;
    }
    else if (procent > 9){
        tens = 1;
        procent -= 10;
    }


    display.clearDisplay();         // Clear the screen and buffer
    display.setTextSize(5);         // Size for first digit(s)
    display.setTextColor(BLACK);    // Normally set colours to normal!

    display.setCursor(0,0);         // First screen position anchors here
    if (tens != 0){                 // Don't use if less than two digits before the radix point
        display.println(tens);      // The first digit (in case value is more than 9.9)
    }
    display.setCursor(30,0);        // Second screen position anchor
    display.println(procent);       // The penultimate digit

    display.setTextSize(4);         // Decrease font size for the last digit (the mantissa)
    display.setCursor(60,0);        // Third screen postion anchor
    display.println(tenths);        // The last digit

    printVoltsEtc(battery_volts);
    printCompercent();
    printPrompt(calib_status);

    if (aay_alert){
        drawAlert();               // Draw a 10x10 px "grey" square in upper left corner.
    }.

    display.display();
}

void silly(void) {
    display.clearDisplay();         // Clear the screen and buffer
    display.setTextColor(BLACK);
    display.setTextSize(2);         // Size for cautionary text
    display.setCursor(10,5); display.println("Silly");
    display.setCursor(10,22);display.println("steep!");
    display.display();
    display.clearDisplay();
}

float monitorRefresh(void) {
    int16_t this_adc_value = analogRead(RAW_PIN);
    int16_t sum_pair = prev_adc_value + this_adc_value;
    prev_adc_value = this_adc_value;
    
    return float(sum_pair / 2 / voltage_divisor);  // Prints average of a pair of succesive values
}

void printVoltsEtc(float v){
    display.setTextSize(1);             // Font size used for voltage monitor value
    display.setCursor(0,40);
    display.print(v, 2);
    display.display(); 
}


void printCompercent(void){
    float comp_as_percent = compensator * 100;
    display.setCursor(30,40);
    display.print(comp_as_percent, 3);  /// Using the redefined print function to show more decimals
    display.display();                  // Nödvändig???
}

void drawAlert(void){
    display.drawLine(0, 8, 1, 9, BLACK);
    display.drawLine(0, 6, 3, 9, BLACK);
    display.drawLine(0, 4, 5, 9, BLACK);
    display.drawLine(0, 2, 7, 9, BLACK);
    display.drawLine(0, 0, 9, 9, BLACK);
    display.drawLine(2, 0, 9, 7, BLACK);
    display.drawLine(4, 0, 9, 5, BLACK);
    display.drawLine(6, 0, 9, 3, BLACK);
    display.drawLine(8, 0, 9, 1, BLACK);
}




void printPrompt(int8_t state){
    if(state){                          // That is to say: It is _not_ "0"
        if(state == 1){                 // Go to first calib screen
            if(even_sec){
                display.setTextSize(1);
                display.setCursor(60,32);
                display.print("Next");
                display.setCursor(78,40);
                display.write(31);      // dwn-arrow in the lower right corner
                display.display();
            }
            else{                       // odd seconds
                display.setTextSize(1);
                display.setCursor(60,32);
                display.print("    "); // Blinking by blanking!
                display.display();
            }
        }
        if(state == 2){                 // Go to second calib screen
            display.setTextSize(1);
            display.setCursor(66,40);
            display.print("   "); 
            display.setCursor(72,40);
            display.write(30);          // up-arrow
            display.write(31);          // dwn-arrow
            display.display();
        }
    }
}
