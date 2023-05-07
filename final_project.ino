#include <DS3231.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "dht.h"
#include <Wire.h> //needed for ds3231


// Registers
volatile unsigned char* pe_ddr = (unsigned char *) 0x2D;
volatile unsigned char* pin_e  = (unsigned char *) 0x2C;
volatile unsigned char* port_e = (unsigned char *) 0x2E;

volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

volatile unsigned char* port_d = (unsigned char*)0x2B;
volatile unsigned char* ddr_d = (unsigned char*)0x2A;
volatile unsigned char* pin_d = (unsigned char*)0x29;

volatile unsigned char* port_a = (unsigned char*)0x22;
volatile unsigned char* ddr_a = (unsigned char*)0x21;
volatile unsigned char* pin_a = (unsigned char*)0x20;

volatile unsigned char* port_g = (unsigned char*)0x34;
volatile unsigned char* ddr_g = (unsigned char*)0x33;
volatile unsigned char* pin_g = (unsigned char*)0x32;

volatile unsigned char* port_l = (unsigned char*)0x10B;
volatile unsigned char* ddr_l = (unsigned char*)0x10A;
volatile unsigned char* pin_l = (unsigned char*)0x109;

volatile unsigned char* pin_k  = (unsigned char *) 0x106;
volatile unsigned char* port_k = (unsigned char *) 0x108;
volatile unsigned char* pk_ddr = (unsigned char *) 0x107; 

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

#define RDA 0x80
#define TBE 0x20  
#define analogPin 0
#define DHT11_PIN 11

// thresholds, water threshold is a resitivity measure, 40 is low enough where if you hold it with your thumb it turns on.
// a cup of water is around 200, empty is 20 ish depending on ambient humidity
#define WATER_THRESHOLD 40

// this is about the average room temperature for my room, just so the fan runs.
#define TEMP_THRESHOLD 23


// program states
#define IDLE 0
#define RUNNING 1
#define DISABLED 2
#define ERROR 3

// initialize sensors and peripherals
dht DHT;

LiquidCrystal lcd(53, 52, 51, 50, 49, 48);

Servo servo;

DS3231 myRTC;

// angle for stepper motor is 10 by default;
int angle = 10;

// booleans for clock
bool century = false;
bool h12;
bool pm;

//System running controls if the interrupt has been triggered or not, enabling/ disabling the program.
volatile bool systemRunning = true;

// these two keep track of the state of the
int programState = IDLE;
int lastProgramState=IDLE;

void setup() {
  //init serial and peripherals
  U0Init(9600);
  lcd.begin(16,2);
  servo.attach(9);
  lcd.clear();
  adc_init();
  Wire.begin(); // required for clock module.

  // setup output/input pins.
  *ddr_b &= ~(0b01010000);
  *ddr_d &= ~(0b00011000);
  *ddr_g &= ~(0b00100000); 
  set_as_output(ddr_g,0); //41
  set_as_output(ddr_l,7); //42
  set_as_output(ddr_l,6); //43
  set_as_output(ddr_l,4); //45
  set_as_output(ddr_g,1); //40
  set_as_output(ddr_g,2); //39
  set_as_output(ddr_d,7); //38

  
  // Set the Interrupt Mask Register to Enable Interrupt 4 and Interrupt 5
  EIMSK |= (1 << INT4) | (1 << INT5);

  // Set the Interrupt Control Register to Trigger on Any Logical Change
  EICRB |= (1 << ISC40) | (1 << ISC50);
  sei();
}

ISR(INT4_vect) {
  // Interrupt Service Routine for Interrupt 4
  // This function will be called when a CHANGE is detected on Pin 2
  // This will toggle the OFF state.
  systemRunning=false;
}

ISR(INT5_vect) {
  // this interrupt toggles the ON state.
  systemRunning=true;
}

// return temp
int getTemperature() {
  return DHT.temperature;
}

// print the state of a the function, to see states, see above definitions.
void printState(int state) {
  if (state == IDLE) {
    print("IDLE");
  } else if (state == ERROR) {
    print("ERROR");
  } else if (state == RUNNING) {
    print("RUNNING");
  } else if (state == DISABLED) {
    print("DISABLED");
  }
}

// This will format the transitons very nicely in a readable message. 
void writeTransitionMessage() {
  if (programState != lastProgramState) {
    print("TRANSITION: ");
    printState(lastProgramState);
    print(" TO ");
    printState(programState);
    print(" AT ");
    getTimeStamp();
  }
}

// set pin as output, analgous to pinMode
void set_as_output(unsigned char* port, unsigned char pin_num)
{
  *port |= 0x01 << pin_num;
}

// init serial
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

// puts a single char to the serial monitor
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

// prints a string, with no newline.
void print(char *string) {
  int i = 0;
  while (string[i] != '\0') {
    U0putchar(string[i]);
    i++;
  }
}

// prints a string with a newline.
void println(char *string) {
  int i = 0;
  while (string[i] != '\0') {
    U0putchar(string[i]);
    i++;
  }
  U0putchar('\n');
}

// writes a state to a pin, analagous to digital write.
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
  // this is triggered from the interrupt! pin 2 = start, pin 3 = stop;
  if (systemRunning) {
    static int waterTooLowPrinted = 0; // prevent repeat messages
    if (programState == ERROR && (*pin_g & 0b00100000) == 0) { // error and reset is low 
      runLCD();
      runAdjustmentMotor();
      lastProgramState = programState;
      if (waterTooLowPrinted == 0) {
          println("Water too low!");
          waterTooLowPrinted=1;
      }
      writeGreenLED(0);
      writeRedLED(1);
      writeYellowLED(0);
      writeBlueLED(0);
      writeFan(0);
    } else if ((*pin_g & 0b00100000) && (programState==ERROR)) { // reset pressed. Goto idle
        lastProgramState = programState;
        programState = IDLE;
        runLCD();
        runAdjustmentMotor();
        writeGreenLED(1);
        writeRedLED(0);
        writeYellowLED(0);
        writeBlueLED(0);
        writeFan(0);
        waterTooLowPrinted=0;
    } else if (getWaterLevel() < WATER_THRESHOLD) { // water too low ERROR
      lastProgramState = programState;
      programState = ERROR;
      if (waterTooLowPrinted == 0) {
          println("Water too low!");
          waterTooLowPrinted=1;
      }
      writeGreenLED(0);
      writeRedLED(1);
      writeYellowLED(0);
      writeBlueLED(0);
      runLCD();
      runAdjustmentMotor();
      writeFan(0);
      waterTooLowPrinted=0;
    } else if (getTemperature() <= TEMP_THRESHOLD && getWaterLevel() > WATER_THRESHOLD) { 
      // idle state
      lastProgramState = programState;
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
      lastProgramState = programState;
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
    // disabled state, toggled via interrupt.
    lastProgramState = programState;
    programState=DISABLED;
    writeGreenLED(0);
    writeRedLED(0);
    writeYellowLED(1);
    writeBlueLED(0);
    writeFan(0);
  }
  writeTransitionMessage();
  delay_milliseconds(100);
}

// turns on/ off fan 
void writeFan(int state) {
    write_port(port_l, 6, state);// turn on fan (43)
    write_port(port_l, 4, 0); //pull other pin low (45)
}

//returns water level
int getWaterLevel() {
  return adc_read(0);
}

// initializes the adc.
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

// prints the time stamp from rtc.
void getTimeStamp() {
  U0putchar(myRTC.getMonth(century)/10+'0');
  U0putchar(myRTC.getMonth(century)%10+'0');
  U0putchar('/');
  U0putchar(myRTC.getDate()/10+'0');
  U0putchar(myRTC.getDate()%10+'0');
  U0putchar('/');
  U0putchar(myRTC.getYear()/10+'0');
  U0putchar(myRTC.getYear()%10+'0');
  U0putchar(' ');
  U0putchar(myRTC.getHour(h12,pm)/10+'0');
  U0putchar(myRTC.getHour(h12,pm)%10+'0');
  U0putchar(':');
  U0putchar(myRTC.getMinute()/10+'0');
  U0putchar(myRTC.getMinute()%10+'0');
  U0putchar(':');
  U0putchar(myRTC.getSecond()/10+'0');
  U0putchar(myRTC.getSecond()%10+'0');
  U0putchar('\n');
}

//these functions turn on/off their respective color led.
void writeRedLED(int state) {
  write_port(port_g, 0, state);
}

void writeYellowLED(int state) {
  write_port(port_g, 1, state);
}

void writeGreenLED(int state) {
  write_port(port_g, 2, state);
}

void writeBlueLED(int state) {
  write_port(port_l, 7, state);
}

// this function runs the stepper motor that adjusts the angle of the fan.
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

// custom delay function. 
void delay_milliseconds(unsigned long milliseconds) {
  TCCR1A = 0; // set timer 1 to normal mode
  TCCR1B = (1 << CS11) | (1 << CS10); // set prescaler to 64
  TCNT1 = 0; // reset the timer count
  while (TCNT1 < ((F_CPU / 64) * milliseconds) / 1000) {
    // do nothing, just wait
  }
  TCCR1B = 0; // stop the timer
}

// runs the LCD 
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

