/*  Project JimBob V0.1

My first Tracker code

Based on code including NTX2 Radio Test Part 2 and GPS Level Convertor Test Script.

    http://ukhas.org.uk
    http://ava.upuaut.net/store/

*/ 

// Required for GPS processing library, 
// note we are using a custom modification to V9 of TinyGPS which understaands the PUBX string
#include <TinyGPS.h>
TinyGPS gps;

// Required for Software Serial port used for debugging as we use the hardware Serial port for the GPS.
#include <SoftwareSerial.h>
SoftwareSerial Debugger(11, 12);  //RX, TX - Define pins used for debugger serial out 

// Macro to print out debug info
#define DEBUG_PRINT(str)    \
    Debugger.print(millis());     \
    Debugger.print(": ");    \
    Debugger.print(__PRETTY_FUNCTION__); \
    Debugger.print(' ');      \
    Debugger.print(__FILE__);     \
    Debugger.print(':');      \
    Debugger.print(__LINE__);     \
    Debugger.print(' ');      \
    Debugger.print(str);

#define RADIOPIN 13 // The Arduino pin that the NTX2 Tx pin is connected to.
 
#include <string.h>
#include <util/crc16.h>



int wait = 0;
unsigned long lastloopmillis = 0;  //What time did we last go around loop()
char s[100] = "0"; //Build the sentance to Tx in here


char datastring[80];
byte gps_set_sucess = 0;
 
void setup() {   
  // Initialise the software serial output to 9600 baud for debuging
  Debugger.begin(9600);
  DEBUG_PRINT("Project JimBob\n")
  
  DEBUG_PRINT("Set up the Radio\n")
  pinMode(RADIOPIN,OUTPUT);
  
  DEBUG_PRINT("Initialise the GPS Serial at 9600 Baud.  GPS is connected to hardware serial port\n")
  Serial.begin(9600);
  
  DEBUG_PRINT("Turn off all GPS NMEA strings apart from GPGGA on the uBlox modules\n")
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,ZDA,0,0,0,0*44");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  
  DEBUG_PRINT("Finally turn off GGA as we are going to poll for that\n")
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");

  DEBUG_PRINT("Set Flight Mode using UBX code, as we hope to be going over 12km high\n")
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  while(!gps_set_sucess)
  {
    DEBUG_PRINT("sendUBX Command\n")
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    delay(100);
    DEBUG_PRINT("getUBX Ack\n")
    gps_set_sucess=getUBX_ACK(setNav);
    DEBUG_PRINT("gps_set_sucess = ")
    Debugger.println(gps_set_sucess, DEC);
  }
  gps_set_sucess=0;

  
  DEBUG_PRINT("Send a startup message")
  rtty_txstringchk("JimBob Tracker");
  
// All of this lot is now in rtty_txstringchk so shouldent be needed, delet once sure it works.
//  sprintf(datastring,"RTTY TEST BEACON RTTY TEST BEACON"); // Puts the text in the datastring
//  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
//  char checksum_str[6];
//  sprintf(checksum_str, "*%04X\n", CHECKSUM);
//  strcat(datastring,checksum_str);
// 
//  rtty_txstring (datastring);

  delay(1000);
  DEBUG_PRINT("Finished Setup()\n\n\n\n")
}
  

void loop() {
  
  DEBUG_PRINT("Dont go around this loop more than once per second/n")
  if (millis() <= (lastloopmillis + 1000)) {
    DEBUG_PRINT("Waiting");
    delay(100);
  }
  lastloopmillis = millis();
  
  DEBUG_PRINT("Request Navigation Data from GPS module/n")
  Serial.println("$PUBX,00*33");
  
  // Ublox say the GPS sentance will be here within 1 second, but we dont want to proceed around loop() 
  // (which would ask for a sentance again) until this sentance is waiting for us
  while (!Serial.available()){
    DEBUG_PRINT("Delaying for Ublox to provide data/n")
    delay(100);
    wait++;
    if (wait > 20) {
      DEBUG_PRINT("Giving up waiting after 2 seconds, start again\n")
      return; // go back to the start of loop
    }
     
   DEBUG_PRINT("While there is data avaliable, pass it to gps.encode\n")
   DEBUG_PRINT("but if gps.encode says we have a full sentance, move on\n")
   DEBUG_PRINT("");
   while (Serial.available()) {
     Debugger.print("X");
     if (gps.encode(Serial.read())) {
       Debugger.print("\n");
       DEBUG_PRINT("gps.encode says we have a full sentance, move on\n")
       break;
     }
   }
   Debugger.print("\n");
    
 
 //$$CALLSIGN,sentence_id,time,latitude,longitude,altitude,optional speed,optional bearing,optional internal temperature,*CHECKSUM\n 
    
   // retrieves +/- lat/long in 100000ths of a degree
  gps.f_get_position(&g.flat, &g.flon);

  // time in hhmmsscc, date in ddmmyy
  gps.get_datetime(&g.date, &g.time, &g.fix_age);
  hour = (g.time / 1000000);
  minute = ((g.time - (hour * 1000000)) / 10000);
  second = ((g.time - ((hour * 1000000) + (minute * 10000))));
  second = second / 100;

  // returns speed in 100ths of a knot
  g.fkph = gps.f_speed_kmph();

  // course in 100ths of a degree
  g.fcourse = gps.f_course();

  g.falt = gps.f_altitude();

  // make stuff pretty
  floatToString(latbuf, g.flat, 4, 0);
  floatToString(lonbuf, g.flon, 4, 0);
  floatToString(coursebuf, g.fcourse, 4, 0);
  floatToString(kphbuf, g.fkph, 4, 0);
  ialt = long(g.falt);

  DEBUG_PRINT("Build the string to send\n")
  make_string();

  DEBUG_PRINT("Transmit the data\n")
  rtty_txstring(s);
  
  
  
}

void make_string()
{
  char checksum[10];
  
  snprintf(s, sizeof(s), "$$JIMBOB,%d,%d:%d:%d,%s,%s,%ld,%s,%s", count, hour, minute, second, latbuf, lonbuf, ialt, kphbuf, coursebuf);

  //snprintf(checksum, sizeof(checksum), "*%02X\n", xor_checksum(s));
  snprintf(checksum, sizeof(checksum), "*%04X\n", gps_CRC16_checksum(s));

  if (strlen(s) > sizeof(s) - 4 - 1)  {
	DEBUG_PRINT("Don't overflow the buffer. You should have made it bigger.")
        return;
  }

  // Also copy checksum's terminating \0 (hence the +1).
  memcpy(s + strlen(s), checksum, strlen(checksum) + 1);
}


void rtty_txstringchk (char * string)
{

  /* Take the string passed to us, calculate the checksum, concatenate the checksum
  ** and pass to rttty_txstring
  */
  unsigned int checksum_int; // The interger value of the checksum
  char checksum_str[6];  // The string equivilent of the checksum
 
  checksum_int = gps_CRC16_checksum(string);  // Calculates the checksum for this datastring
  sprintf(checksum_str, "*%04X\n", checksum_int);
  strcat(string,checksum_str);
 
  rtty_txstring (string); // pass the new string to rtty_txstring
  
}
 
void rtty_txstring (char * string)
{
 
  /* Simple function to sent a char at a time to 
   	** rtty_txbyte function. 
   	** NB Each char is one byte (8 Bits)
   	*/
 
  char c;
 
  c = *string++;
 
  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}
 
 
void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	**
   	*/
 
  int i;
 
  rtty_txbit (0); // Start bit
 
  // Send bits for for char LSB first	
 
  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1); 
 
    else rtty_txbit(0);	
 
    c = c >> 1;
 
  }
 
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}
 
void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    digitalWrite(RADIOPIN, HIGH);
  }
  else
  {
    // low
    digitalWrite(RADIOPIN, LOW);
 
  }
 
  //                  delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
 
}
 
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}    

// UBX Functions

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
  Serial.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }
 
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}

