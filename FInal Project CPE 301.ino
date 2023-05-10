//Sariah Warren
//CPE 301

#include <LiquidCrystal.h>
#include <Wire.h>
#include <DHT.h>
#include <Stepper.h>
#include <RTClib.h>

//lcd
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//clock
RTC_DS1307 rtc = RTC_DS1307();

//vent
const int stepsPerRevolution = 2038;
Stepper myStepper(stepsPerRevolution, 23, 25, 22, 24);

//vent button 1
int buttonState = 0;
int buttonState2 = 0; // current state of the button
int lastButtonState = 0; 
int lastButtonState2 = 0;// previous state of the button
bool isMovingClockwise = true; // flag to keep track of the motor direction

//led and on off button
const int buttonPin1 = 2;
const int buttonPin2 = 13;
const int greenLedPin = 29;
const int yellowLedPin = 28;
const int redLedPin = 27;
const int blueLedPin = 26;

//fan
#define ENABLE 5
#define DIRA 3
#define DIRB 4

//temp sensor
#define DHTPIN 6
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//adc registers for water sensor
volatile uint8_t* myADCSRA = (volatile uint8_t*) &ADCSRA;
volatile uint8_t* myADCSRB = (volatile uint8_t*) &ADCSRB;
volatile uint8_t* myADMUX  = (volatile uint8_t*) &ADMUX;
volatile uint16_t* myADCDATA = (volatile uint16_t*) &ADC;

//water sensor
int water_level_pin = A8;
int level_history = 0;

//dht11 read ready flag
bool read_dht11 = false;

//super states
bool on = false;
//substates
bool error_state = false;
bool idle_state = false;
bool running_state = false;

//buttons
bool button2 = false; //on/off button
bool button1 = false; //vent position button
volatile bool isStartButtonPressed = false;

void error_message(){
  clear_lcd();
  lcd.setCursor(0,0);
  lcd.print("WATER LEVEL LOW");
  lcd.setCursor(0,1);
  lcd.print("REFILL TANK");  
}

void clear_lcd(){
  lcd.setCursor(0,0);
  lcd.print("             ");
  lcd.setCursor(0,1);
  lcd.print("             ");
}

//clear lcd values displayed
void clear_values_lcd(){
  lcd.setCursor(6,0);
  lcd.print("     ");
  lcd.setCursor(9,1);
  lcd.print("     ");
}

void adc_setup(){
  //setup A register
  //set bit 7 to 1 to enable the ADC
  *myADCSRA |= 0b10000000;
  //set bit 5 to 0 to disable the ADC trigger mode
  *myADCSRA &= 0b11011111;
  //set bit 3 to 0 to disable the ADC interrupt
  *myADCSRA &= 0b11110111;

  //setup B register
  //set bit 2 to 0 to select single-ended input mode
  *myADCSRB &= 0b11111011;
  //set bit 3 to 0 to select 10-bit resolution
  *myADCSRB &= 0b11110111;

  //setup MUX register
  //set bit 7 to 0 to select AREF as voltage reference
  *myADMUX &= 0b01111111;
  //set bit 6 to 0 to disable left adjustment
  *myADMUX &= 0b10111111;
}

unsigned int adc_read(unsigned char adc_channel){
  //clear channel selection bits
  *myADMUX &= 0b11100000;
  //clear MUX5
  *myADCSRB &= 0b11110111;
  //set selection bit
  if(adc_channel > 7){
    adc_channel -= 8;
  //set MUX 5 to 1
    *myADCSRB |= 0b00001000;
  }
  //set the channel selection bits
  *myADMUX |= adc_channel;
  //start conversion
  *myADCSRA |= 0b01000000;
  //wait for the conversion to finish
  while((*myADCSRA & 0b01000000) != 0);
  return *myADCDATA; //return the result of the conversion
}

// Define possible states
enum State {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR
};

// Initialize variables
State state = DISABLED;
unsigned long lastUpdate = 0;
int currentVentPosition = 0;
int targetVentPosition = 0;
bool isFanOn = false;

void startButtonPressed() {
  isStartButtonPressed = true;
}

void setup() {
  // Initialize pins and devices using port manipulation
  DDRB &= ~_BV(buttonPin1);  // Set buttonPin1 to input (clear bit)
  DDRB &= ~_BV(buttonPin2);  // Set buttonPin2 to input (clear bit)
  DDRB |= _BV(greenLedPin);  // Set greenLedPin to output (set bit)
  DDRB |= _BV(yellowLedPin); // Set yellowLedPin to output (set bit)
  DDRB |= _BV(redLedPin);    // Set redLedPin to output (set bit)
  DDRB |= _BV(blueLedPin);   // Set blueLedPin to output (set bit)
  
  // Enable pull-up resistors for button pins
  PORTB |= _BV(buttonPin1);  // Enable pull-up for buttonPin1 (set bit)
  PORTB |= _BV(buttonPin2);  // Enable pull-up for buttonPin2 (set bit)

  //fan
  pinMode(ENABLE,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(button2), startButtonPressed, FALLING);
  myStepper.setSpeed(10);
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();
  adc_setup();
  rtc.begin();
}

void loop() {
  // Read input buttons
  button1 = !digitalRead(buttonPin1);
  button2 = !digitalRead(buttonPin2);

  buttonState = digitalRead(2);

  // Get current time
  DateTime now = rtc.now();

  if (buttonState != lastButtonState) {
    // If the button has just been pressed
    if (buttonState == HIGH) {
      // Toggle the motor direction
      isMovingClockwise = !isMovingClockwise;

      // Move the motor 90 degrees in the current direction
      if (isMovingClockwise) {
        myStepper.step(stepsPerRevolution/4);
      } else {
        myStepper.step(-stepsPerRevolution/4);
      }
    }
    lastButtonState = buttonState;
  }

  // Continuously monitor water level
  // Continuously monitor water level
  int waterLevel = adc_read(water_level_pin);
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();  // Continuously monitor water level

  buttonState2 = digitalRead(13);

  switch (state) {
    case DISABLED:
      Serial.print("DISABLED: ");
      Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
      Serial.print("\n");
      delay(100);

      // Yellow LED should be ON
      digitalWrite(yellowLedPin, HIGH);

      // No monitoring of temperature or water should be performed
      if (buttonState2 == HIGH) {
        digitalWrite(yellowLedPin, LOW);
        state = IDLE;
      }
      break;
    case IDLE:
      // Record transition time
      digitalWrite(greenLedPin, HIGH);
      Serial.print("IDLE: ");
      Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
      Serial.print("\n");
      delay(500);

      // Display temperature and humidity on LCD screen
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print(" C");

      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print(" %");


     if (waterLevel < 100) {
        digitalWrite(greenLedPin, LOW);
        state = ERROR;
      } else {
        digitalWrite(greenLedPin, LOW);
        state = RUNNING;
      }
      break;
    case RUNNING:
      digitalWrite(blueLedPin, HIGH);
      Serial.print("RUNNING: ");
      Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
      Serial.print("\n");
      delay(100);

      // Display temperature and humidity on LCD screen
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature);
      lcd.print(" C");

      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print(" %");

      // Turn on motor driver 
        digitalWrite(ENABLE, HIGH);
        digitalWrite(DIRA, HIGH);
        digitalWrite(DIRB, LOW);

      
      if (waterLevel < 100) {
        digitalWrite(blueLedPin, LOW);
        state = ERROR;
      }
      break;

      case ERROR:
        digitalWrite(redLedPin, HIGH);
        error_message();
        // turn on motor driver if temperature is above 20.0
        digitalWrite(ENABLE, LOW);
        digitalWrite(DIRA, LOW);
        digitalWrite(DIRB, LOW);

          // turn off motor driver if temperature is below 20.0
          //clear lcd values displayed
          clear_lcd();
          state = IDLE;
          digitalWrite(redLedPin, LOW);
        break;
        }
}

  





