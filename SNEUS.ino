#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <Wire.h>
#include <RFM22.h>
#include <TinyGPS.h>
#include "SparkFunCCS811.h"
#include "ClosedCube_HDC1080.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define RFM22_FREQUENCY 434.240

#define WEBSITE_PRINT_FREQ  20
#define WEBSITE_PRINT_ON    1
 
#define RTTY_ASCII    8       // RTTY_ASCII 7 or 8
#define RTTY_STOPBITS 2       // Either 1 or 2
#define RTTY_TXDELAY  0       // Delay between sentence TX's
#define RTTY_BAUD     300     // Baud rate for use with RFM22B Max = 600

#define RFM22B_SDN 8
#define RFM22B_REINIT_CNT 8

#define GEIGER_THRESHOLD      1000  // CPM threshold for fast avg mode
#define GEIGER_LONG_PERIOD    60    // # of samples to keep in memory in slow avg mode
#define GEIGER_SHORT_PERIOD   5     // # or samples for fast avg mode
#define GEIGER_SCALE_FACTOR   57    //  CPM to uSv/hr conversion factor (x10,000 to avoid float)

#define CCS811_ADDR 0x5A  //CS811 I2C Address

//Pins
#define RFM22B_CS_PIN   10
#define GEIGER_INT_PIN  3
#define LED             7
#define BAT_SENSE_PIN   A0
#define DUST_SENSE_PIN  A1

TinyGPS gps;
rfm22 radio1(RFM22B_CS_PIN);
CCS811 AirQ(CCS811_ADDR);
ClosedCube_HDC1080 hdc1080;
Adafruit_BMP280 bmp; 

char callsign[8] = "SNEUS-1"; //Callsign

byte gps_init_sucess = 0;

char          tracker_string[200];
char          sensor_string[80];
char          website_string[20] = "ukhas.org.uk";
char          checksum_str[6];
char          txstring[200] = "Waiting for GPS";
volatile int  txstatus=1;
volatile int  txstringlength=0;
volatile char txc;
volatile int  txi;
volatile int  txj;
unsigned int  count=0;
unsigned int  int_count=0;
long int      packet_cnt = 1;
long int      website_cnt = 1;
bool          newData = false;
bool          cBusy = true;

//Bat Measurement Global Variables
int   bat_sensorValue = 0;
float bat_actualValue = 0.00;
float bat_divider = 10.27;
char  bat_voltage[6];
int   bat_v1;
int   bat_v2;

// Geiger Global variables
volatile uint8_t    geiger_nobeep;    // flag used to mute beeper
volatile uint16_t   geiger_count;     // number of GM events that has occurred
volatile uint16_t   geiger_slowcpm;   // GM counts per minute in slow mode
volatile uint16_t   geiger_fastcpm;   // GM counts per minute in fast mode
volatile uint16_t   geiger_cps;       // GM counts per second, updated once a second
volatile uint8_t    geiger_overflow;  // overflow flag

volatile uint8_t    geiger_buffer[GEIGER_LONG_PERIOD];  // the sample buffer
volatile uint8_t    geiger_idx;       // sample buffer index
volatile uint8_t    geiger_eventflag; // flag for ISR to tell main loop if a GM event has occurred

//GPS Global Variables
float           flat, flon = 0;
unsigned long   age;
char            latbuf[12] = "0", lonbuf[12] = "0" ,altbuf[12] = "0";
int             hour = 0 , minute = 0 , second = 0, oldsecond = 0, sats = 0;
unsigned long   date, time;
long int        ialt = 0;

//RFM22 Global Variables
bool  rfm22_reinit = false;
bool  rfm22_reinit_done = false;
int   rfm22_reinitcntr = 0;

//Sensor Global Variables
int   CCS811_CO2 = 0;
int   CCS811_TVOC = 0;
int   HDC1080_temp = 0;
int   HDC1080_humidity = 0;
int   BMP280_temp = 0;
int   BMP280_pressure = 0;
int   BMP280_alt = 0;

 
void setup()
{ 
  pinMode(LED,OUTPUT);
  
  blink_led(1);
    
  init_timer_interrupt();
  
  Serial.begin(9600);
  
  //Setup GPS
  uint8_t setNav[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA,
  0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  
  while(!gps_init_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_init_sucess=getUBX_ACK(setNav);
    /*sendUBX(ecoMode, sizeof(ecoMode)/sizeof(uint8_t));
    gps_init_sucess&=getUBX_ACK(ecoMode);*/
  }
  gps_init_sucess=0;
  
  attachInterrupt(digitalPinToInterrupt(GEIGER_INT_PIN),geiger_pulse, RISING);

  strcat(website_string,"\n");
  
  delay(1000);
  
  //Setup RFM22B
  init_rfm22();

  // setup I2C sensors
  CCS811Core::status returnCode = AirQ.begin();
  hdc1080.begin(0x40);
  
  blink_led(4);
}
 
void loop()
{  
  if(parse_gps())
  {
    measure_battery();
    build_telem_string();
    
    //Read I2C Sensors
    if (AirQ.dataAvailable())
    {
      read_CCS811();
      read_HDC1080(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
      read_BMP280();
    }
  }
  
  //Reinitialisation of RFM22
  if(rfm22_reinit == true)
  {
    init_rfm22();
    rfm22_reinit = false;
  }
}

void blink_led(uint8_t count)
{
  for(uint8_t i = 0; i < count; i++)
  {
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH); 
    delay(500);
  }
}

void init_rfm22(void)
{
  pinMode(RFM22B_SDN, OUTPUT);    // RFM22B SDN is on ARDUINO A3
  digitalWrite(RFM22B_SDN, LOW);
  delay(1000);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  radio1.setFrequency(RFM22_FREQUENCY);
  radio1.write(0x6D, 0x04);// turn tx low power 11db
  radio1.write(0x07, 0x08);
  delay(500);
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
    //mySerial.print(MSG[i], HEX);
  }
  Serial.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  //mySerial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      //mySerial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      //mySerial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        //mySerial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}

uint8_t parse_gps(void)
{
  while(Serial.available())
  {
  char c = Serial.read();
  if (gps.encode(c)) // Did a new valid sentence come in?
  newData = true;
  }
  
  if (newData)
  {
    digitalWrite(LED,HIGH);

    //Pars GPS Data
    gps.f_get_position(&flat, &flon, &age);
    sats = gps.satellites();
    dtostrf(flat, 10, 6, latbuf);
    dtostrf(flon, 9, 6, lonbuf);
    
    if(lonbuf[0] == ' ')
    {
      lonbuf[0] = '+';
    }
    if(latbuf[0] == ' ')
    {
      latbuf[0] = '+';
    }
    ialt = (gps.altitude() / 100);
    
    itoa(ialt, altbuf, 10);
    
    gps.get_datetime(&date, &time, &age);
    hour = (time / 1000000);
    minute = ((time - (hour * 1000000)) / 10000);
    second = ((time - ((hour * 1000000) + (minute * 10000))));
    second = second / 100;
  }
  else
  {
    digitalWrite(LED,LOW);
  }

  return newData;
}

void measure_battery(void)
{
  //Read Battery Voltage        
    bat_sensorValue = analogRead(BAT_SENSE_PIN);                    
    bat_actualValue = (bat_sensorValue / 1023.00) * bat_divider;
    bat_v1 = bat_actualValue;
    bat_v2 = (bat_actualValue - bat_v1) * 100;
    snprintf(bat_voltage, 30, "%i.%02i", bat_v1, bat_v2);
}

void build_telem_string(void)
{
    //Build new data string
    cBusy = true;
    sprintf(tracker_string,"$$$$%s,%li,%02i:%02i:%02i,%s,%s,%s,%i,%s,",callsign,packet_cnt,hour,minute,second,latbuf,lonbuf,altbuf,sats,bat_voltage); 
    sprintf(sensor_string,"%i",geiger_slowcpm);
    strcat(tracker_string,sensor_string);
    unsigned int CHECKSUM = calc_crc16_checksum(tracker_string);  // Calculates the checksum for this tracker_string
    
    sprintf(checksum_str, "*%04X\n", CHECKSUM);
    strcat(tracker_string,checksum_str);
    cBusy = false;
}
 
uint16_t calc_crc16_checksum(char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first four $s
  for (i = 4; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}

void init_timer_interrupt(void)
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

//CCS811
void read_CCS811()
{
  AirQ.readAlgorithmResults();
  CCS811_CO2 = AirQ.getCO2();
  CCS811_TVOC = AirQ.getTVOC();
}

//HDC1080
void read_HDC1080(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature) 
{
  hdc1080.setResolution(humidity, temperature);
  HDC1080_Registers reg = hdc1080.readRegister();
  HDC1080_temp = hdc1080.readTemperature();
  HDC1080_humidity = hdc1080.readHumidity();
}

//BMP280
void read_BMP280()
{
  BMP280_temp = bmp.readTemperature();
  BMP280_pressure = bmp.readPressure();
  BMP280_alt = bmp.readAltitude(1013.25); // this should be adjusted to your local pressure
}

ISR(TIMER1_COMPA_vect)
{
  switch(txstatus) {
  case 0: // This is the optional delay between transmissions.
    txj++;
    if(txj>(RTTY_TXDELAY*RTTY_BAUD)) {
      txj=0;
      txstatus=1;
    }
    break;
  case 1: // Initialise transmission, take a copy of the string so it doesn't change mid transmission.
  if(rfm22_reinit == false)
  {
    if(cBusy == false)
    {
      if(rfm22_reinitcntr == RFM22B_REINIT_CNT)
      {
        rfm22_reinit = true;
        rfm22_reinitcntr = 0;
      }
      else
      {
        
        if((website_cnt == WEBSITE_PRINT_FREQ) && (WEBSITE_PRINT_ON == 1))
        {
          strcpy(txstring,website_string);
          website_cnt = 0;
        }
        else
        {
          strcpy(txstring,tracker_string);
          packet_cnt++;
          website_cnt++;
        }
        
        txstringlength=strlen(txstring);
        
        if(txstringlength != 0)
        txj=0;
  
        rfm22_reinitcntr++;
        txstatus=2;
      }
    }
  }
    break;
  case 2: // Grab a char and lets go transmit it.
    if ( txj < txstringlength)
    {
      txc = txstring[txj];
      txj++;
      txstatus=3;
      rtty_txbit (0); // Start Bit;
      txi=0;
    }
    else
    {
      txstatus=0; // Should be finished
      txj=0;
    }
    break;
  case 3:
    if(txi<RTTY_ASCII)
    {
      txi++;
      if (txc & 1) rtty_txbit(1);
      else rtty_txbit(0);  
      txc = txc >> 1;
      break;
    }
    else
    {
      rtty_txbit (1); // Stop Bit
      txstatus=4;
      txi=0;
      break;
    }
  case 4:
    if(RTTY_STOPBITS==2)
    {
      rtty_txbit (1); // Stop Bit
      txstatus=2;
      break;
    }
    else
    {
      txstatus=2;
      break;
    }
  }
  
  int_count++;
  if(int_count == (RTTY_BAUD - 1))
  {
    int_count=0;
    geiger_1s_calc();
  }
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    radio1.write(0x73,0x03); // High
  }
  else
  {
    radio1.write(0x73,0x00); // Low
  }
}

void geiger_pulse(void)
{
    if (geiger_count < UINT16_MAX)  // check for overflow, if we do overflow just cap the counts at max possible
    geiger_count++; // increase event counter
    
    geiger_eventflag = 1; // tell main program loop that a GM pulse has occurred
}

void geiger_1s_calc(void)
{
    uint8_t i, x;  // index for fast mode
    
    geiger_cps = geiger_count;
    geiger_slowcpm -= geiger_buffer[geiger_idx];   // subtract oldest sample in sample buffer
    
    if (geiger_count > UINT8_MAX) { // watch out for overflowing the sample buffer
      geiger_count = UINT8_MAX;
      geiger_overflow = 1;
    }
        
    geiger_slowcpm += geiger_count;   // add current sample
    geiger_buffer[geiger_idx] = geiger_count;  // save current sample to buffer (replacing old value)
    
    // Compute CPM based on the last SHORT_PERIOD samples
    geiger_fastcpm = 0;
    for(i=0; i<GEIGER_SHORT_PERIOD;i++) {
      x = geiger_idx - i;
      if (x < 0)
        x = GEIGER_LONG_PERIOD + x;
      geiger_fastcpm += geiger_buffer[x]; // sum up the last 5 CPS values
    }
    geiger_fastcpm = geiger_fastcpm * (GEIGER_LONG_PERIOD/GEIGER_SHORT_PERIOD); // convert to CPM
    
    // Move to the next entry in the sample buffer
    geiger_idx++;
    if (geiger_idx >= GEIGER_LONG_PERIOD)
      geiger_idx = 0;
    geiger_count = 0;  // reset counter
}

