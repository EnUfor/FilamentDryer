// Description: This code utilizes a pressure transducer and a solid state relay to regulate the
//              pressure in the boiler of a La Pavoni espresso machine via a PID feedback controller. 
//              The required components and a project description can be found here:
//              https://barrettsprojects.wordpress.com/2018/04/22/la-pavoni-pid-pressure-temperature-mod/ 
//
// Author: Barrett J. Anderies
//
// Last edited: April 29nd, 2018
//
// IMPORTANT NOTEs:   1. This code supports using a Bluetooth transceiver (HC-05), but use of this module is optional.
//                    The code will run as is without a Bluetooth transceiver, but the pin assigned to enable/disable
//                    the BT module will still be assigned and therefore not available for other uses. If you plan on adding
//                    functionality to this code and want to use the pin assigned to the BT module, simply comment out
//                    any lines referring to Bluetooth setup/operation. 
//
//                    2. Since settings must be read and saved from EEPROM, I use a flag system to determine
//                    if settings have been previously saved (which would not be the case when the system is first
//                    booted). The code looks for a value (EEPROM_FLAG) in the last EEPROM address (EEPROM.length() - 1).
//                    I picked a random value to check for. If this value is not found, the system assumes that 
//                    there are no settings to load, uses the default values, and saves them to EEPROM. If this value 
//                    is found, the system attempts to load the settings. IF, RANDOMLY, THERE HAPPENS TO BE THE FLAG 
//                    VALUE IN THE EEPROM ADDRESS, BUT THERE ARE NO SAVED SETTINGS, THE CODE WILL LOAD WHATEVER IS THERE, 
//                    RESULTING IN TOTALLY RANDOM PID SETTINGS. Fortunately, this is pretty unlikely (1 in 256 chance), 
//                    and if it does occur won't damage anything - your controller will just behave strangely. 
//                    THEREFORE, WHEN YOU FIRST UPLOAD THIS CODE, IT WOULD BE WISE TO MAKE SURE THE SETTINGS LOOK CORRECT 
//                    IN THE SERIAL MONITOR.
//
//                    3. The Bluetooth transceiver only turns on when needed to save power. To activate the Bluetooth transeiver,
//                    turn the system on with the dial turned all the way up. The dial position is checked on bootup to determine
//                    transceiver power status, and therefore, the Bluetooth power status cannot be changed while the system is 
//                    running. The system must be turned off, the dial position set to select the desired power state, then turned on.
//
//                    4. The formula on line 209 converts the raw pressure transducer signal in to atmospheres (~bars).
//                    Since the conversion is linear, it has slope and y-intercept coefficients. These coefficients will be unique to
//                    your pressure sensor (they may be close enough, but you shouldn't count on it). Because of this YOU MUST CALIBRATE 
//                    THESE VALUES YOURSELF. I calibrated mine by recording the raw value at 0 atm (based on my machine gauge) and at 1 atm
//                    (again based on my machine gauge). This gives two points in the format (raw, atm), which can be converted into a linear
//                    function (y = mx + b). Update the coefficients in this code to be what you calculated. Note that this does not have 
//                    to be perfect. This code will still hold a setpoint, no matter how far off the actual value it is, but if you want the
//                    reported value to match what you see on your pressure gauge, you will need to calibrate.
//
// USAGE:             BEFORE BOOTING: 
//                    1. Set dial position to max (highest value) if you want the Bluetooth transceiver to be on. Else, leave the dial
//                    on a lower setting and the Bluetooth transceiver will not be activated. 2. Turn the La Pavoni on. 3. Turn the PID controller on.
//                    
//                    AFTER BOOTING:
//                    4. Wait for the Yellow light to come on to indicate that the controller is booted. If it does not, the battery might be dead.
//                    5. Choose desired setpoint pressure. 6. Wait for the green light to come on to indicate that the machine has reached the
//                    desired pressure.
//
//                    SETPOINT AND PARAMETER CONTROL (REGULAR VS OVERRIDE MODE):
//                    In normal operation (i.e. booting the device up and not sending any serial commands), all you can do is change the presssure
//                    setpoint by turning the knob (potentiometer). By connecting the the Arduino to a computer/phone either via a USB cable or Bluetooth, 
//                    you can send commands over the serial connection (in addition to read live data from the controller). 
//                    Commands can be sent from the serial monitor using the format "charfloat" or "char", depending on the command. 
//                    The PID settings are stored in EEPROM and will be automatically loaded when the system is rebooted. However, the system will alwasy boot up 
//                    in regular mode. 
//                    The following is a breakdown of the supported commands and their formats (THE NUMBER GIVEN DOESN'T HAVE TO BE A FLOAT, IT WILL BE PARSED TO ONE AUTOMATICALLY):
//
//                    "p#.##" - This sets the PID P coefficient to the number specified by #.##. Any positive number can be given. Example: "p15" or "p15.0" will result in the PID P value being set to 15.0
//                    
//                    "i#.##" - This sets the PID I coefficient to the number specified by #.##. Any positive number can be given. Example: "i0.005" will result in the PID I value being set to 0.005
//
//                    "d#.##" - This sets the PID I coefficient to the number specified by #.##. Any positive number can be given. Example: "d0.7" will result in the PID D value being set to 0.7
//
//                    "m"     - This command toggles between the regular and override modes. When in override mode, the knob (potentiometer) won't do anything, and the pressure setpoint is set by the following command.
//
//                    "s#.##" - If in override mode, this sets the setpoint to #.##. Any positive number can be given, although the machine pstat will not allow the pressure above a certain value (~ 1bar).


#include <PID_v1.h> // PID Library
#include <Relay.h> // Relay duty cycle control library
#include <EEPROM.h> // EEPROM library for saving PID settings in between power cycles

// pin assignments
#define SETPOINT_PIN A0 // this pin monitors the potentiometer used to control the desired pressure (can be reassigned to any analog pin)
#define PRESSURE_PIN A2 // this pin monitors the pressure transducer (can be reassigned to any analog pin)
#define RELAY_PIN 5     // this pin controls the relay (goes to the + terminal on the realy. The - terminal on the relay goes to ground) 
#define YELLOW_PIN 4    // this pin controls the yellow LED (connect to LED + lead)
#define GREEN_PIN 3     // this pin controls the green LED (connect to LED + lead)
#define BT_POWER_PIN 6  // this pin controls the power state of the Bluetooth transceiver

// default PID coefficients if EEPROM is empty
#define KP 15
#define KI 1023
#define KD 1

// define PID settings structure, which will be saved to EEPROM every time it is updated
struct pidSettings {
  double p;
  double i;
  double d;
};

// create instance of the pidSetting structure
struct pidSettings settings;

// pressure transducer rolling average array length (to smooth out noise on the pressure signal)
#define WINDOW_SAMPLE_WIDTH 10

#define UPPER_LIMIT 1.0       // upper PID output limit. Set to 1.0 (i.e. 100 percent of the duty cycle =  full on)
#define LOWER_LIMIT 0.0       // lower PID output limit. Set to 0.0 (i.e. 0 percent of the ducy cycle = full off)
//#define PID_SAMPLE_TIME 50  // can be used to make the PID sample time longer or shorter. I just used the default update time (automatically assigned), but you can change this as needed -  you would need to call pid.SetSampleTime(PID_SAMPLE_TIME); in the setup loop. 
#define RELAY_PERIOD 1        // relay period = 1 second. This is the shortest period the relay library allows, and is sufficient for our purpose. Faster switching may cause unnecessary heating/damage to the relay
#define GREEN_THRESHOLD 0.02  // if the measured pressure is within this value of the setpoint (in atms), the green light will be activiated

// EEPROM flag value to check on startup. If this value is NOT found at the last EEPROM address (dependent on EEPROM.length()), the program will assume the pidSettings have not been previously stored, and will write the default coefficients to EEPROM
#define EEPROM_FLAG 152 // this is a randomply chosen value. Unfortunately, if this value happens to be in the EEPROM at the last address, the code will act funny (i.e. it will think that there are setting when there aren't any, and will load random values), so if you are having problems the first time you use this code, set this to another value
#define PID_SETTINGS_WRITE_ADDRESS 0 // the index at which the EEPROM.put() fucntion starts for storing the pidSettings object. 

double atm, dutyCycle, setpoint;
int raw[WINDOW_SAMPLE_WIDTH], addPosition = 0;
bool setpointModeOverride = false; // start with the potentiometer controlling the setpoint. If setpointModeOverride = true, setpoing is controlled via the serial monitor

// create PID (with default PID coefficients) and Relay instances - note that the pid parameters are passed by reference, so they are updated by calling only the Compute() function in loop()
PID pid(&atm, &dutyCycle, &setpoint, KP, KI, KD, DIRECT);
Relay relay(RELAY_PIN, RELAY_PERIOD);

void setup()
{   
    // open serial connection for debugging
    Serial.begin(115200);
    
    // set LED, relay, and Bluetooth control pins to OUTPUT mode
    pinMode(YELLOW_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BT_POWER_PIN, OUTPUT);

    // determine Bluetooth status. If control dial turned all the way up -> turn Bluetooth transceiver on, else -> turn Bluetooth transceiver off to save power.
    if (analogRead(SETPOINT_PIN) > 1000) {
      Serial.println("Bluetooth transceiver on.");
      digitalWrite(BT_POWER_PIN, LOW);
    } else {
      Serial.println("Bluetooth transceiver off.");
      digitalWrite(BT_POWER_PIN, HIGH);
    }
    
    // set the relay library instance to automatic to allow it use of the setDutyCyclePercent(dutyCycle) function
    relay.setRelayMode(relayModeAutomatic);
    
    // initialize setpoint variable to 0
    setpoint = 0;
    
    // configure PID
    pid.SetOutputLimits(LOWER_LIMIT, UPPER_LIMIT);
    pid.SetMode(AUTOMATIC);

    // PID settings retrieval code 
    Serial.print("\nChecking EEPROM address ");
    Serial.print(EEPROM.length() - 1);
    Serial.print(", looking for ");
    Serial.println(EEPROM_FLAG);
    Serial.print("Found ");
    Serial.println(EEPROM.read(EEPROM.length() - 1));
    if (EEPROM.read(EEPROM.length() - 1) == EEPROM_FLAG) { // PID settings have been written to EEPROM previously, load and set tunings
      Serial.println("Flag found... Loading settings: ");
      EEPROM.get(PID_SETTINGS_WRITE_ADDRESS, settings);
      Serial.print("P: ");
      Serial.println(settings.p, 3);
      Serial.print("I: ");
      Serial.println(settings.i, 3);
      Serial.print("D: ");
      Serial.println(settings.d, 3);
      Serial.println("*Values displayed up to 3 decimal places");
      pid.SetTunings(settings.p, settings.i, settings.d);
    } else { // settings have NOT been written to EEPROM previously
      Serial.println("Flag not found... Assigning default coefficients:");
      Serial.print("P: ");
      Serial.println(KP, 3);
      Serial.print("I: ");
      Serial.println(KI, 3);
      Serial.print("D: ");
      Serial.println(KD, 3);
      Serial.println("*Values displayed up to 3 decimal places");
      Serial.print("Setting flag ");
      Serial.print(EEPROM_FLAG);
      Serial.print(" to EEPROM address ");
      Serial.println(EEPROM.length() - 1);
      settings.p = KP; // no seetings found in EEPROM, assigne default P value
      settings.i = KI; // no seetings found in EEPROM, assigne default I value
      settings.d = KD; // no seetings found in EEPROM, assigne default D value
      EEPROM.put(PID_SETTINGS_WRITE_ADDRESS, settings); // write the settings object with the default coefficients to EEPROM
      EEPROM.write(EEPROM.length() - 1, EEPROM_FLAG); // set the EEPROM flag at the end of the EEPROM memory so that the next time the program start it will know that there are settings saved
    }

    // Turn the yellow pin on to indicate that the system has booted/is on.
    Serial.println("Finished booting...");
    digitalWrite(YELLOW_PIN, HIGH);

} // end setup()

void loop()
{
  // read pressure transducer and insert into raw array at addPosition
  raw[addPosition] = analogRead(PRESSURE_PIN);
  
  // increment addPosition and reset of greater than window width
  addPosition++;
  if (addPosition >= WINDOW_SAMPLE_WIDTH) addPosition = 0;
  
  // calcualte average signal value over the number of samples sepcified by WINDOW_SAMPLE_WIDTH
  for (int i = 0; i < WINDOW_SAMPLE_WIDTH; i++) {
      atm += raw[i];
    }
  atm = atm / WINDOW_SAMPLE_WIDTH;
  
  // convert raw analog read value to atmospheres
  atm = 0.00635*atm - 0.64; // Using 3.3V input - I CALIBRATED MY SENSOR USING MY LA PAVONI'S PRESSURE GAUGE, YOU WILL HAVE TO DO THE SAME AND UPDATE THE COEFFICIENTS HERE

  // read the potentiometer to get the pressure setpoint if not in override mode. If in override mode, setpoint is set via Bluetooth in serialEvent()
  if (!setpointModeOverride) {
    setpoint = map(analogRead(SETPOINT_PIN), 0, 1023, 0, 98)/100.0;
  }

  // call the PID compute function (update rate determined by PID_SAMPLE_TIME)
  pid.Compute();

  // print debug info
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print("   ");
  Serial.print("Pressure: ");
  Serial.print(atm);
  Serial.print("   ");
  Serial.print("DutyCycle: ");
  Serial.print(dutyCycle);
  Serial.print("   ");
  Serial.print("P: ");
  Serial.print(settings.p, 3);
  Serial.print("   ");
  Serial.print("I: ");
  Serial.print(settings.i, 3);
  Serial.print("   ");
  Serial.print("D: ");
  Serial.println(settings.d, 3);
 
  // call relay loop function and update the duty cycle from the dutyCycle variable (which is continuously updated by the PID).
  relay.loop();
  relay.setDutyCyclePercent(dutyCycle);

  // activate the green "ready" LED if threshold condition is met
  if (abs(setpoint - atm) < GREEN_THRESHOLD) {
    digitalWrite(GREEN_PIN, HIGH);
  } else {
    digitalWrite(GREEN_PIN, LOW); 
  } 
} // end loop()

// function to handle serial communication events
void serialEvent() {
  float val;
  char coeff;
  // read in p, i or d value by parsing the first character, then the following value
  coeff = Serial.read(); // get character to determine command sent
  val = Serial.parseFloat(); // get command variable
  // check for validity, set PID coefficients, and write to EEPROM if possible
  if (val >= 0) { // for the currently supported functions, values less than 0 don't make sense
    switch (coeff) {
      case 'p': // PID P value
        settings.p = val;
        break;
      case 'i': // PID I value
        settings.i = val;
        break;
      case 'd': // PID D value
        settings.d = val;
        break;
      case 'm': // switch mode command - no associated value to parse, only the command character
        setpointModeOverride = !setpointModeOverride; // switch setpoint control 
        break;
      case 's': // PID setpoint value. This value is only applied if the system is in override mode (by sending 'm')
        if (setpointModeOverride) {
          setpoint = val;
        }
        break;
      default: // let user know that an unrecognized character was sent
        Serial.println("Unrecognized character given.");
        delay(3000);
    }
    // set new PID values and save to EEPROM
    pid.SetTunings(settings.p, settings.i, settings.d);
    EEPROM.put(PID_SETTINGS_WRITE_ADDRESS, settings); // this only executes if the settings have been changed to reduce EEPROM wear
  } else {
    Serial.println("Unable to parse command value because a negative value was given");
    delay(3000);
  }      
  
  // flush anything leftover from the serial buffer to prevent reading problems the next time serialEvent() is called 
  Serial.flush();
}

// Notes:
//  relay.setRelayPosition(relayPositionOpen);
//  relay.setRelayPosition(relayPositionClosed);
//  float atm = 0.0065*raw - 0.65044248; // Using 5V input
//  pid.SetSampleTime(PID_SAMPLE_TIME);

//  if (Serial.peek() == 'p') {
//    coeff = Serial.read();
//    val = Serial.parseFloat();
//  }
//  if (Serial.peek() == 'i') {
//    coeff = Serial.read();
//    val = Serial.parseFloat();
//  }
//  if (Serial.peek() == 'd') {
//    coeff = Serial.read();
//    val = Serial.parseFloat();
//  }
