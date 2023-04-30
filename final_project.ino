#include <Servo.h>
#include <LiquidCrystal.h>
#include "dht.h"
dht DHT;
#define DHT11_PIN 11
LiquidCrystal lcd(53, 52, 51, 50, 49, 48);
Servo servo;
int angle = 10;



volatile unsigned char* pe_ddr = (unsigned char *) 0x2D;
volatile unsigned char* pin_e  = (unsigned char *) 0x2C;
volatile unsigned char* port_e = (unsigned char *) 0x2E;

volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

void setup() {
  Serial.begin(9600); // needs to be replaced.
  *ddr_b &= ~(0b01010000);
  *port_b |= 0x10;
  lcd.begin(16,2);
  lcd.clear();
}

void loop() {
  runLCD();
  runAdjustmentMotor();
  servo.attach(9);
  delay(100);
}

void runAdjustmentMotor() {
    if (*pin_b & 0b00010000){
      Serial.println("left");
      angle+=10;
      servo.write(angle);
    }
    if (*pin_b & 0b01000000) {
      Serial.println("right");
      angle-=10;
      servo.write(angle);
    }
}

void runLCD() {
  int chk = DHT.read11(DHT11_PIN);
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
  lcd.print("%");
 
}

void my_delay(double freq)
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV           
  *myTIFR1 |= 0x01;
}
