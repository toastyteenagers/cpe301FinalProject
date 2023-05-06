#include <DS3231.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "dht.h"
#include <Wire.h>

dht DHT;
#define DHT11_PIN 11
LiquidCrystal lcd(53, 52, 51, 50, 49, 48);
Servo servo;
int angle = 10;

DS3231 myRTC;

volatile unsigned char* pe_ddr = (unsigned char *) 0x2D;
volatile unsigned char* pin_e  = (unsigned char *) 0x2C;
volatile unsigned char* port_e = (unsigned char *) 0x2E;

volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

volatile unsigned char* port_d = (unsigned char*)0x2B;
volatile unsigned char* ddr_d = (unsigned char*)0x2A;
volatile unsigned char* pin_d = (unsigned char*)0x29;

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

#define RDA 0x80
#define TBE 0x20  
#define analogPin 0

#define WATER_THRESHOLD 40
#define TEMP_THRESHOLD 22

volatile unsigned char* pin_k  = (unsigned char *) 0x106;
volatile unsigned char* port_k = (unsigned char *) 0x108;
volatile unsigned char* pk_ddr = (unsigned char *) 0x107; 

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

int fanState = 1;
bool century = false;
bool h12;
bool pm;


#define IDLE 0
#define RUNNING 1
#define DISABLED 2
#define ERROR 3

#define startButtonPin 2
#define stopButtonPin 3

volatile bool systemRunning = true;
int programState = IDLE;

void setup() {
  U0Init(9600);
  *ddr_b &= ~(0b01010000);
  *ddr_d &= ~(0b00011000);
  *port_d |= 0x10;
  *port_b |= 0x10;

  set_as_output(pk_ddr,0);

  lcd.begin(16,2);
  servo.attach(9);
  lcd.clear();
  adc_init();

  Wire.begin(); // required for clock module.

  pinMode(41,OUTPUT);
  pinMode(43,OUTPUT);
  pinMode(45,OUTPUT);
  pinMode(40,OUTPUT);
  pinMode(39,OUTPUT);
  pinMode(38,OUTPUT);
  pinMode(23,INPUT);


  // Set the Interrupt Mask Register to Enable Interrupt 4 and Interrupt 5
  EIMSK |= (1 << INT4) | (1 << INT5);

  // Set the Interrupt Control Register to Trigger on Any Logical Change
  EICRB |= (1 << ISC40) | (1 << ISC50);
  sei();
}

ISR(INT4_vect) {
  // Interrupt Service Routine for Interrupt 4
  // This function will be called when a CHANGE is detected on Pin 2
  systemRunning=false;
}

ISR(INT5_vect) {
  systemRunning=true;
}

int getTemperature() {
  return DHT.temperature;
}

void set_as_output(unsigned char* port, unsigned char pin_num)
{
  *port |= 0x01 << pin_num;
}


void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

void write_port(unsigned char* port, unsigned char pin_num, unsigned char state)
{
  if(state == 0)
  {
    *port &= ~(0x01 << pin_num);
  }
  else
  {
    *port |= 0x01 << pin_num;
  }
}


void loop() {
  // Perform other tasks if the system is running
  // this is triggered from the interrupt! pin 2 = start, pin 3 = stop
  if (systemRunning) {
    static int waterTooLowPrinted = 0;
    if (programState == ERROR && digitalRead(4) == 0) { // error and reset is low
      if (waterTooLowPrinted == 0) {
          Serial.println("Water too low!");
          waterTooLowPrinted=1;
      }
      writeGreenLED(0);
      writeRedLED(1);
      writeYellowLED(0);
      writeBlueLED(0);
      writeFan(0);
    } else if (digitalRead(4) && (programState==ERROR)) { // reset pressed. Goto idle
        programState = IDLE;
        runLCD();
        runAdjustmentMotor();
        writeGreenLED(1);
        writeRedLED(0);
        writeYellowLED(0);
        writeBlueLED(0);
        writeFan(0);
        waterTooLowPrinted=0;
    } else if (getWaterLevel() < WATER_THRESHOLD) {
      programState = ERROR;
      writeFan(0);
      waterTooLowPrinted=0;
    } else if (getTemperature() <= TEMP_THRESHOLD && getWaterLevel() > WATER_THRESHOLD) {
      // idle state
      programState = IDLE;
      runLCD();
      runAdjustmentMotor();
      writeGreenLED(1);
      writeRedLED(0);
      writeYellowLED(0);
      writeBlueLED(0);
      writeFan(0);
      waterTooLowPrinted=0;
    } else if (getTemperature() > TEMP_THRESHOLD && getWaterLevel() > WATER_THRESHOLD) {
      // RUNNING STATE
      programState = RUNNING;
      runLCD();
      runAdjustmentMotor();
      writeGreenLED(0);
      writeRedLED(0);
      writeYellowLED(0);
      writeBlueLED(1);
      writeFan(1);
      waterTooLowPrinted=0;
    }
  } else {
    // disabled state...
    programState=DISABLED;
      writeGreenLED(0);
      writeRedLED(0);
      writeYellowLED(1);
      writeBlueLED(0);
      writeFan(0);

  }
  delay_milliseconds(100);
}


void writeFan(int state) {
     digitalWrite(43,state); // turn on fan?
     digitalWrite(45,LOW); // turn on fan?
}

int getWaterLevel() {
  return adc_read(0);
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void getTimeStamp() {
  Serial.print(myRTC.getMonth(century));
  Serial.print("/");
  Serial.print(myRTC.getDate());
  Serial.print("/");
  Serial.print(myRTC.getYear()); 
  Serial.print("  ");
  Serial.print(myRTC.getHour(h12,pm));
  Serial.print(":");
  Serial.print(myRTC.getMinute());
  Serial.print(":");
  Serial.print(myRTC.getSecond());
  Serial.println();

}
void writeRedLED(int state) {
  digitalWrite(41,state);
}

void writeYellowLED(int state) {
  digitalWrite(40,state);
}

void writeGreenLED(int state) {
  digitalWrite(39,state);
}

void writeBlueLED(int state) {
  digitalWrite(38,state);
}

void toggleFan() {
   if (fanState) {
     //turn on
     digitalWrite(43,HIGH); // turn on fan?
     digitalWrite(45,LOW); // turn on fan?
     fanState = 0;
     
   } else {
     //turn off
      digitalWrite(45,LOW); //enable
      digitalWrite(43,LOW); // turn on fan?
      fanState = 1;
   }
}

void runAdjustmentMotor() {
    if (*pin_b & 0b00010000){
      angle+=10;
      servo.write(angle);
    }
    if (*pin_b & 0b01000000) {
      if (angle > 0) {
        angle-=10;
      }
      servo.write(angle);
    }
}
void delay_milliseconds(unsigned long milliseconds) {
  TCCR1A = 0; // set timer 1 to normal mode
  TCCR1B = (1 << CS11) | (1 << CS10); // set prescaler to 64
  TCNT1 = 0; // reset the timer count
  while (TCNT1 < ((F_CPU / 64) * milliseconds) / 1000) {
    // do nothing, just wait
  }
  TCCR1B = 0; // stop the timer
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


