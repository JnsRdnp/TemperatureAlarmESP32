#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <stdbool.h>
#include <LiquidCrystal_I2C.h>

float tempertatureFunction();
void buzzerFunction();
void rotaryFunction();
int8_t checkRotaryEncoder();

// Define to which pin of the Arduino the 1-Wire bus is connected:
#define ONE_WIRE_BUS 15

//buzzer pin
#define BUZZER_PIN 32

// potentiometer pins
#define CLK_PIN 19
#define DT_PIN 4
#define SW_PIN 18

// EEPROM SIZE
#define EEPROM_SIZE 2

// Create a new instance of the oneWire class to communicate with any OneWire device:
OneWire oneWire(ONE_WIRE_BUS);
// Pass the oneWire reference to DallasTemperature library:
DallasTemperature sensors(&oneWire);

// rotary encoder values
float rotationCounter = 0;
bool taster = LOW;
bool last_taster = LOW;
boolean counterSaved = false;

// lcd values
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); 

// temp values
float storedTemp;
float storedTempDiv;

void setup() {
  // Begin serial communication at a baud rate of 9600:
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  // lcd setup
  lcd.init();
  lcd.backlight();

  // Start up the library:
  sensors.begin();
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

}

void loop() {
  lcd.setCursor(0, 0);
  lcd.print("Set alarm temp:");

  Serial.println(rotationCounter);
  while(counterSaved==false){
    rotaryFunction();
  }

  Serial.println("Tempertature saved");
  lcd.setCursor(0, 0);
  lcd.print("Tempertature");
  lcd.setCursor(0, 1);
  lcd.print("saved");
  lcd.setCursor(6,1);
  lcd.print(rotationCounter);
  delay(2500);
  lcd.clear();
  //lcd.noBacklight();

  while(digitalRead(SW_PIN)==1){
    storedTemp=EEPROM.read(0);
    storedTempDiv = storedTemp/10;

    if (tempertatureFunction()>= storedTempDiv)
    {
      buzzerFunction();
    }
  }
  counterSaved = false;
  lcd.clear();

  lcd.backlight();
  Serial.println("Change tempertature");
  lcd.setCursor(0, 0);
  lcd.print("Change");
  lcd.setCursor(0, 1);
  lcd.print("tempertature");

  delay(2500);
  lcd.clear();
}

void rotaryFunction(){

  taster = !digitalRead(SW_PIN);
  // Has rotary encoder moved?
  // Get the movement (if valid)
  float rotationValue = checkRotaryEncoder();
  // If valid movement, do something
  if (rotationValue != 0){
      rotationCounter += rotationValue * 0.5;
      Serial.println(rotationCounter);

      lcd.setCursor(0, 1);
      lcd.print(rotationCounter);
  }

  if(taster != last_taster) {
      Serial.print(rotationCounter);
      Serial.print("|");
      Serial.println(taster);


      lcd.clear();

      delay(10);
      last_taster = 0;
      float rotationCounterTen = rotationCounter * 10;

      EEPROM.write(0, rotationCounterTen);

      // save position value to EEPROM and multiply by 10 to save decimals.

      //last_position = n;
      counterSaved = true;
      delay(500);
  }
}


float tempertatureFunction(){

  sensors.requestTemperatures();

  // Fetch the temperature in degrees Celsius for device index:
  float tempC = sensors.getTempCByIndex(0); // the index 0 refers to the first device

  // Print the temperature in Celsius in the Serial Monitor:
  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.print(" \xC2\xB0"); // shows degree symbol
  Serial.print("C");
  
  // print alert temp
  storedTemp=EEPROM.read(0);
  storedTempDiv = storedTemp/10;

  Serial.print(" | ");
  Serial.print("Alert temp: ");
  Serial.print(storedTempDiv);
  Serial.print(" \xC2\xB0"); // shows degree symbol
  Serial.println("C");

  lcd.setCursor(0, 0);
  lcd.print(tempC);
  lcd.print((char)223); // shows degree symbol
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print(storedTempDiv);
  lcd.print((char)223); // shows degree symbol
  lcd.print("C  ALERT");
  delay(100);

  // Wait 1 second:
  delay(1000);

  return tempC;
}

void buzzerFunction(){
  for(uint8_t i = 0; i < 50; i++) { // 500Hz frequency 
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1); // Delay 1ms
    digitalWrite(BUZZER_PIN, LOW);
    delay(1); // Delay 1ms
 }
  for(uint8_t i = 0; i < 50; i++) { // 167Hz frequency 
    digitalWrite(BUZZER_PIN, HIGH); 
    delay(3); // Delay 3ms
    digitalWrite(BUZZER_PIN, LOW); 
    delay(3); // Delay 3ms
 }
}


int8_t checkRotaryEncoder(){
    // Reset the flag that brought us here (from ISR)

    static uint8_t lrmem = 3;
    static int lrsum = 0;
    static int8_t TRANS[] = {0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0};

    // Read BOTH pin states to deterimine validity of rotation (ie not just switch bounce)
    int8_t l = digitalRead(CLK_PIN);
    int8_t r = digitalRead(DT_PIN);

    // Move previous value 2 bits to the left and add in our new values
    lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;

    // Convert the bit pattern to a movement indicator (14 = impossible, ie switch bounce)
    lrsum += TRANS[lrmem];

    /* encoder not in the neutral (detent) state */
    if (lrsum % 4 != 0)
    {
        return 0;
    }

    /* encoder in the neutral state - clockwise rotation*/
    if (lrsum == 4)
    {
        lrsum = 0;
        return -1;
    }

    /* encoder in the neutral state - anti-clockwise rotation*/
    if (lrsum == -4)
    {
        lrsum = 0;
        return 1;
    }

    // An impossible rotation has been detected - ignore the movement
    lrsum = 0;
    return 0;
}