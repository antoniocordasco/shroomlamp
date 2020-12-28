
#include "FastLED.h"
// How many leds in your strip?

#define MIN_SIDE_INTENSITY 10
#define MAX_SIDE_INTENSITY 30
#define MIN_TRUNK_INTENSITY -40
#define MAX_TRUNK_INTENSITY 30
#define INTENSITY_CHANGE 10

#define LEFT_LEDS 6
#define RIGHT_LEDS 6
#define TRUNK_LEDS 65

int total_leds = LEFT_LEDS + RIGHT_LEDS + TRUNK_LEDS;
#define DATA_PIN 12
#define LED_POWER_PIN1 2
#define LED_POWER_PIN2 3
#define LED_POWER_PIN3 4
#define LED_POWER_PIN4 5
#define LED_POWER_PIN5 6
#define LED_POWER_PIN6 7
#define LIGHT_SENSOR_PIN A6
#define VOLTAGE_PIN A7
CRGB leds[LEFT_LEDS + RIGHT_LEDS + TRUNK_LEDS];

int left_intensity[LEFT_LEDS];
int right_intensity[RIGHT_LEDS];
int trunk_intensity[TRUNK_LEDS];

// 0 = r, 1 = rg, 2 = g, 3 = gb, 4 = b, 5 = br
int dominant_colour;
int tmp_rnd;

void initializeIntensities() {
  for (int i=0; i<LEFT_LEDS; i++) {
    left_intensity[i] = random(MIN_SIDE_INTENSITY, MAX_SIDE_INTENSITY + 1);
  }
  for (int i=0; i<RIGHT_LEDS; i++) {
    right_intensity[i] = random(MIN_SIDE_INTENSITY, MAX_SIDE_INTENSITY + 1);
  }
  for (int i=0; i<TRUNK_LEDS; i++) {
    trunk_intensity[i] = random(MIN_TRUNK_INTENSITY, MAX_TRUNK_INTENSITY + 1);
  }
}


void changeDominantColour() {  
  Serial.println("changeDominantColour: ");
  // change dominant colour only when 0 or 1. Changing dominant colour happens rarely;
  int tmp_rnd = random(0, 200);
  if (tmp_rnd == 0) {
    dominant_colour--;
  } else if (tmp_rnd == 1) {
    dominant_colour++;
  }
  if (dominant_colour > 5) {
    dominant_colour = 0;
  } else if (dominant_colour < 0) {
    dominant_colour = 5;
  }
}

int getNewRandomIntensity(int old_intensity, String led_type) {
  int change = random(-INTENSITY_CHANGE, INTENSITY_CHANGE);
  int new_intensity = old_intensity + change;

/*
  Serial.print("old_intensity.: ");
  Serial.println(old_intensity);
  Serial.print("new_intensitys: ");
  Serial.println(new_intensity);
  Serial.print("change: ");
  Serial.println(change);
  */
  int min_intensity = MIN_TRUNK_INTENSITY;
  int max_intensity = MAX_TRUNK_INTENSITY;    
  if (led_type == "side") {
    min_intensity = MIN_SIDE_INTENSITY;
    max_intensity = MAX_SIDE_INTENSITY;     
  }
    
  if (new_intensity < min_intensity) {
    new_intensity = min_intensity;
  } else if (new_intensity > max_intensity) {
    new_intensity = max_intensity;
  }    
/*
  Serial.print("new_intensitys: ");
  Serial.println(new_intensity);
  */
  return new_intensity;  
}


void changeLed(String leds_group) {
  int random_led;
  int old_intensity;
  int led_id;
  String led_type = "side";
  int new_intensity = 0;
  
  if (leds_group == "left") {
    random_led = random(0, LEFT_LEDS);
    led_id = random_led;
    old_intensity = left_intensity[random_led];
    new_intensity = getNewRandomIntensity(old_intensity, "side");
    left_intensity[random_led] = new_intensity;
  } else if (leds_group == "right") {
    random_led = random(0, RIGHT_LEDS);
    led_id = random_led + LEFT_LEDS;
    old_intensity = right_intensity[random_led];
    new_intensity = getNewRandomIntensity(old_intensity, "side");
    right_intensity[random_led] = new_intensity;
  } else {
    random_led = random(0, TRUNK_LEDS);  
    led_id = random_led + LEFT_LEDS + RIGHT_LEDS;
    old_intensity = trunk_intensity[random_led];
    new_intensity = getNewRandomIntensity(old_intensity, "trunk");
    trunk_intensity[random_led] = new_intensity;
  }
/*
  Serial.print("dominant_colour: ");
  Serial.println(dominant_colour);
  Serial.print("led_id: ");
  Serial.println(led_id);
  Serial.print("old_intensity: ");
  Serial.println(old_intensity);
  Serial.print("new_intensity: ");
  Serial.println(new_intensity);
  */

  if (new_intensity < 0) {
    new_intensity = 0;
  }
  
  if (dominant_colour == 0) { 
    leds[led_id] = CRGB(new_intensity, 0, 0);      
  } else if (dominant_colour == 1) { 
    leds[led_id] = CRGB(new_intensity/2 , new_intensity/2, 0);      
  } else if (dominant_colour == 2) { 
    leds[led_id] = CRGB(0 , new_intensity, 0);      
  } else if (dominant_colour == 3) { 
    leds[led_id] = CRGB(0 , new_intensity/2, new_intensity/2);      
  } else if (dominant_colour == 4) { 
    leds[led_id] = CRGB(0 , 0, new_intensity);      
  } else if (dominant_colour == 5) { 
    leds[led_id] = CRGB(new_intensity/2 , 0, new_intensity/2);      
  }
  FastLED.show(); 
}

float getLight() {
  int  light = 0;
  delay(3);
  light += analogRead(LIGHT_SENSOR_PIN);
  delay(3);
  light += analogRead(LIGHT_SENSOR_PIN);
  return light / 2;
}

float getVoltage() {
  float voltageStep = 0.0278;
  float  voltage = 0;
  delay(3);
  voltage += analogRead(VOLTAGE_PIN) * voltageStep;
  delay(3);
  voltage += analogRead(VOLTAGE_PIN) * voltageStep;
  return voltage / 2;
}

void powerLedsOn() {
  digitalWrite(LED_POWER_PIN1, HIGH);
  digitalWrite(LED_POWER_PIN2, HIGH);
  digitalWrite(LED_POWER_PIN3, HIGH);
  digitalWrite(LED_POWER_PIN4, HIGH);
  digitalWrite(LED_POWER_PIN5, HIGH);
  digitalWrite(LED_POWER_PIN6, HIGH);
}

void powerLedsOff() {
  // setting all digital pins as LOW to save power
  for (int i = 0; i < 20; i++) {
    digitalWrite(i, LOW);
  }
}

int  light = 0;
float  voltage = 0;

void setup() {   
  Serial.begin(9600);  
  Serial.println("setup");
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, LEFT_LEDS + RIGHT_LEDS + TRUNK_LEDS);

  // setting all digital pins as LOW to save power
  for (int i = 0; i < 20; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  randomSeed(analogRead(A0));
  dominant_colour = random(0, 5);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  pinMode(LED_POWER_PIN1, OUTPUT);
  pinMode(LED_POWER_PIN2, OUTPUT);
  pinMode(LED_POWER_PIN3, OUTPUT);
  pinMode(LED_POWER_PIN4, OUTPUT);
  pinMode(LED_POWER_PIN5, OUTPUT);
  pinMode(LED_POWER_PIN6, OUTPUT);
  
  initializeIntensities();
}

void loop() { 
  // the sensors are using the LEDs 5v line, which is powered by digital pins.
  // so we need to set these pins to high to get a reading. 
  // in retrospect, this could have been done better and use the main 5v of the arduino for the sensors
  powerLedsOn();
  light = getLight();
  voltage = getVoltage();
  /*
  Serial.print("light: ");
  Serial.print(light);
  Serial.print(" voltage: ");
  Serial.println(voltage);
*/
  // if it's day time, or the batteries are running out, switch off everything
  if (light > 500) {
    powerLedsOff();
    FastLED.clear();
    FastLED.show(); 
    delay(8000);
    return;
    

    WDTCSR = (24);
    WDTCSR = (33);    
    WDTCSR |= (1<<6);

    ADCSRA &= ~(1 << 7);

    SMCR |= (1 << 2);
    SMCR |= 1;    

    MCUCR |= (3 << 5);
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6);

    __asm__ __volatile__("sleep");    
  } else {
    changeDominantColour();
    changeLed("left");
    changeLed("right");
    changeLed("trunk");
    delay(1000);
  }
}
