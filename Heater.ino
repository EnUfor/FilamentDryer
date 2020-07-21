// 4.Use a multimeter to measure the resistance of the hot-bed terminal. 220V is usually 96.8Ω; 110V is usually 24.2Ω.

#include ".\thermistor_1.h"

#define THERMISTORPIN A0

const int thermistorNominal     = 100000;
const int numSamples            = 5;
const int seriesResistor        = 5000;

int samples[numSamples];

void setup() {
    Serial.begin(9600);
    // analogReference(EXTERNAL);  // Use 3.3V for reference voltage  
}

void loop() {
    uint8_t i;
    float average = 0;

    for(int i = 0; i < numSamples; i++) {
        average += analogRead(THERMISTORPIN);
        delay(10);
    }
    average /= numSamples;


    Serial.print("Average analog reading: ");
    Serial.println(average);

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

    double y = linearInterpolate(average, temptable, numRows);
    Serial.print("y: ");Serial.println(y);

    delay(1000);
    Serial.println();

    
}

double linearInterpolate(double x, const short table[][2], int numRows) {
    // numRows must be submitted as an argument since table is a pointer
    // therefore sizeof(table) will always be 2, causing errors
    
    double x0, x1, y0, y1, y;

    // TODO: Add code that if a value doesn't match (1023 or 0), proper flags are raised

    for (int i = 0; i < numRows; i++) { // for each row in table
        if (x > table[i][0] && x < table[i + 1][0]) {
            x0 = table[i][0];       // lower bound x
            x1 = table[i + 1][0];   // upper bound x
            y0 = table[i][1];       // lower bound y
            y1 = table[i + 1][1];   // upper bound y
        }
    }

    Serial.print("x0: ");Serial.print(x0);
    Serial.print(" y0: ");Serial.println(y0);
    Serial.print("x1: ");Serial.print(x1);
    Serial.print(" y1: ");Serial.println(y1);

    return y0 + ((y1 - y0) / (x1 - x0)) * (x - x0);

}