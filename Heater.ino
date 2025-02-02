// 4.Use a multimeter to measure the resistance of the hot-bed terminal. 220V is usually 96.8Ω; 110V is usually 24.2Ω.

#include <Relay.h>              // Relay duty cycle control library
#include <PID_v1.h>             // PID library
#include ".\thermistor_1.h"     // Thermistor temp / resistance table

// Pin assignments
#define THERMISTORPIN   A0
#define SSRPIN          8
#define TEMPDIAL        A1

#define RELAYPERIOD     1
#define NUMSAMPLES      5

// PID related variables
double setTemperature, currentTemperature, dutyCycle;
double Kp = .08; double Ki = .0001;  double Kd = .002;

// Initialize pid and SSR objects
PID pid(&currentTemperature, &dutyCycle, &setTemperature, Kp, Ki, Kd, DIRECT);
Relay SSR(SSRPIN, RELAYPERIOD);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("THESE ARE THE COEFFICIENTS:");
    Serial.print(Kp, 3);
    Serial.print(" ");
    Serial.print(Ki, 3);
    Serial.print(" ");
    Serial.println(Kd, 3);

    delay(2000);

    pid.SetOutputLimits(0, 1);
    pid.SetMode(AUTOMATIC);    
    SSR.setRelayMode(relayModeAutomatic);

    pinMode(TEMPDIAL, INPUT);
    // pinMode(SSRPIN, OUTPUT);
    // digitalWrite(SSRPIN, LOW);
    // analogReference(EXTERNAL);  // Use 3.3V for reference voltage
}


void loop() {
    setTemperature = map(analogRead(TEMPDIAL), 0, 1023, 0, 1000.0) / 10.0;

    currentTemperature = readTemperature();

    pid.Compute();

    // Trim off unecessary on/off cycle
    // if(dutyCycle <= .05) dutyCycle = 0;
    if(dutyCycle >= .95) dutyCycle = 1;

    Serial.println( (String)setTemperature + " " + (String)currentTemperature + " " + (String)dutyCycle);

    SSR.loop();
    SSR.setDutyCyclePercent(dutyCycle);    
}

/**
 * Takes an average reading of thermistor.
 * @return {double} y - Interpolated temperature reading.
 */ 
float readTemperature() {
    float average = 0;

    for(int i = 0; i < NUMSAMPLES; i++) {
        average += analogRead(THERMISTORPIN);
        delay(10);
    }
    average /= NUMSAMPLES;

    // Serial.print("Average analog reading: ");
    // Serial.println(average);

    // // Convert value to resistance
    // const int seriesResistor = 5000;
    // average = (1023 / average) - 1;
    // average = seriesResistor / average;   
    // Serial.print("Average thermistor resistance: ");
    // Serial.println(average);


    // 
    // Since our array contains entities of type 'short' (16 bits or 2 bytes each),
    // sizeof(temptable) represents the size of the entire array (256 bytes).
    // sizeof(temptable[0]) represents the size of a single row of our array (4 bytes),
    // sizeof(temptable[0][0]) represents the size of a single entity in the array (2 bytes),
    //
    
    int numRows = sizeof(temptable)/sizeof(temptable[0]);
    // int numCols = sizeof(temptable[0])/sizeof(temptable[0][0]);
    // Serial.print("Number of rows = ");Serial.println(numRows);
    // Serial.print("Number of cols = ");Serial.println(numCols);

    return linearInterpolate(average, temptable, numRows);
}

/**
 * Linear interpolate a value using a table for lookup.
 * @param {double} x - X value to interpolate to.
 * @param {short} table - Table containing lookup values.
 * @param {int} numRows - Number of rows in table.
 */
float linearInterpolate(double x, const short table[][2], int numRows) {
    // numRows must be submitted as an argument since table is a pointer
    // therefore sizeof(table) will always be 2, causing errors
    
    float x0, x1, y0, y1, y;

    // TODO: Add code that if a value doesn't match (1023 or 0), proper flags are raised

    for (int i = 0; i < numRows; i++) { // for each row in table
        if (x > table[i][0] && x < table[i + 1][0]) {
            x0 = table[i][0];       // lower bound x
            x1 = table[i + 1][0];   // upper bound x
            y0 = table[i][1];       // lower bound y
            y1 = table[i + 1][1];   // upper bound y
        }
    }

    // Serial.print("x0: ");Serial.print(x0);
    // Serial.print(" y0: ");Serial.println(y0);
    // Serial.print("x1: ");Serial.print(x1);
    // Serial.print(" y1: ");Serial.println(y1);

    return y0 + ((y1 - y0) / (x1 - x0)) * (x - x0);

}