// 4.Use a multimeter to measure the resistance of the hot-bed terminal. 220V is usually 96.8Ω; 110V is usually 24.2Ω.

#include ".\thermistor_1.h"

#define THERMISTORPIN   A0
#define SSRPIN          8
#define TEMPDIAL        A1

const int thermistorNominal     = 100000;
const int numSamples            = 5;
const int seriesResistor        = 5000;

int setTemperature              = 0;
float currentTemperature        = 0;

// PID related
float previousError             = 0;
float PIDError                  = 0;
int PIDValue                    = 0;
float elapsedTime, Time, timePrev;

int kp = 140; int ki = 2;  int kd = 500;
int PID_p = 0;  int PID_i = 0;  int PID_d = 0;

// #define DEFAULT_bedKp 83.48  
// #define DEFAULT_bedKi 8.15 
// #define DEFAULT_bedKd 213.72

unsigned long previousMillis    = 0;
unsigned long currentMillis     = 0;
int tempReadDelay               = 1500;

int heatingTime                 = 1000;

void setup() {
    Serial.begin(9600);

    pinMode(TEMPDIAL, INPUT);
    pinMode(SSRPIN, OUTPUT);
    digitalWrite(SSRPIN, LOW);
    // analogReference(EXTERNAL);  // Use 3.3V for reference voltage
    Time = millis();  
}


void loop() {
    
    
    currentMillis = millis();

    if(currentMillis - previousMillis >= tempReadDelay) {
        setTemperature = analogRead(TEMPDIAL);
        setTemperature = map(setTemperature, 0, 1023, 0, 100);
        Serial.print("setTemperature: ");Serial.println(setTemperature);
        previousMillis = currentMillis;

        currentTemperature = readTemperature();

        PIDError = setTemperature - currentTemperature;     // Calculate PID error

        PID_p = kp * PIDError;                              // Calculate P value
        PID_i = PID_i + (ki * PIDError);                    // Calculate I value

        timePrev = Time;
        Time = millis();
        elapsedTime = (Time - timePrev) / 1000;

        PID_d = kd * ((PIDError - previousError) / elapsedTime);    // Calculate D value
        PIDValue = PID_p + PID_i + PID_d;

        previousError = PIDError;

        if(currentTemperature > setTemperature) {
            PIDValue = 0;
            PID_i = 0;
        }

        if(PIDValue > heatingTime) {
            PIDValue = heatingTime;
            PID_i = 0;
        }
        if(PIDValue < 0) {
            PIDValue = 0;
            PID_i = 0;
        }

        // Serial.print("PIDValue: ");Serial.println(PIDValue);

        int offTime = heatingTime - PIDValue;

        Serial.print("On Time: ");Serial.print(PIDValue);
        Serial.print("     Off Time: "); Serial.println(offTime);

        if(offTime > 0) {
            digitalWrite(SSRPIN, LOW);
            delay(offTime);
        }
        if(PIDValue > 10) {
            digitalWrite(SSRPIN, HIGH);
            delay(PIDValue);
        }

        //delay(1000);
        Serial.println();
    }

    

    
}
/**
 * Takes an average reading of thermistor.
 * @return {double} y - Interpolated temperature reading.
 */ 
float readTemperature() {
    float average = 0;

    for(int i = 0; i < numSamples; i++) {
        average += analogRead(THERMISTORPIN);
        delay(10);
    }
    average /= numSamples;

    // Serial.print("Average analog reading: ");
    // Serial.println(average);

    // // Convert value to resistance
    // average = (1023 / average) - 1;
    // average = seriesResistor / average;   
    // Serial.print("Average thermistor resistance: ");
    // Serial.println(average);


    // 
    // Since our array contains entities of type 'short' (16 bits or 2 bytes each),
    // sizeof(temptable[0][0]) represents the size of a single entity in the array (2 bytes),
    // sizeof(temptable[0]) represents the size of a single row of our array (4 bytes),
    // sizeof(temptable) represents the size of the entire array (256 bytes).
    //
    
    int numRows = sizeof(temptable)/sizeof(temptable[0]);
    // int numCols = sizeof(temptable[0])/sizeof(temptable[0][0]);
    // Serial.print("Number of rows = ");Serial.println(numRows);
    // Serial.print("Number of cols = ");Serial.println(numCols);

    float y = linearInterpolate(average, temptable, numRows);
    Serial.print("y: ");Serial.println(y);

    return y;
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