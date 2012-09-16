/*  Project JimBob V0.1

My first Tracker code

Based on code including NTX2 Radio Test Part 2 and GPS Level Convertor Test Script.

    http://ukhas.org.uk
    http://ava.upuaut.net/store/

*/ 

// Tiny GPS Struct
typedef struct {
  float flat, flon, falt;
  unsigned long age, time, date;
} gpsd;
gpsd g; // instantiate above struct

// TX Sentance Struct
typedef struct {
  unsigned int id;
  int hour, minute, second;
  char latbuf[12], lonbuf[12];
  long int alt;  
  int sats;
} sent;
sent s; // instantiate above struct

// Build the sentance to Tx in here
char sentance[100] = "0"; 

// Required for Software Serial port used for debugging as we use the hardware Serial port for the GPS.
#include <SoftwareSerial.h>
SoftwareSerial Debugger(11, 12);  // RX, TX - Define pins used for debugger serial out 

// Required for GPS processing library, 
// note we are using a custom modification to V12 of TinyGPS which understaands the PUBX string
#include <TinyGPS.h>
TinyGPS gps;

#define RADIOPIN 8 // The Arduino pin that the NTX2 Tx pin is connected to.
#define LEDPIN 13 // LED pin
 
#include <string.h>
#include <util/crc16.h>



//*********************************************************************************
//
// setup()
//
//**********************************************************************************
 
void setup() {   
  
  // Remove once we know rtty_txstringchk works.
  // char datastring[80];
  boolean gps_set_sucess = 0; // Does the UBlox module aaccept our command?
  
  // Initialise the software serial output to 9600 baud for debuging
  Debugger.begin(9600);
  delay(100); // required, otherwise the next println is corrupted.
  Debugger.println(F("\r\n\r\n\r\nsetup() Project JimBob")); // F() stops the string being stored in Ram - I was running out and causing reboots.

  Debugger.println(F("setup() Set up the Radio"));
  pinMode(RADIOPIN,OUTPUT);
  
  Debugger.println(F("setup() Set up the LED and turn it off"));
  pinMode(LEDPIN,OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  Debugger.println(F("setup() Initialise the GPS Serial at 9600 Baud"));
  Debugger.println(F("setup() GPS is connected to hardware serial port"));
  Serial.begin(9600);
  delay(100);
    
  Debugger.println(F("setup() Turn off all GPS NMEA strings apart from GPGGA on the uBlox modules"));

  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,ZDA,0,0,0,0*44");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  
  Debugger.println(F("setup() Finally turn off GGA as we are going to poll for that"));
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
   
  Debugger.println(F("setup() Set Flight Mode using UBX code, as we hope to be going over 12km high"));
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  while(!gps_set_sucess)
  {
    Debugger.println(F("setup() sendUBX Command"));
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    //Serial.write(setNav, sizeof(setNav));
    //Serial.println();
    delay(100);
    Debugger.println(F("setup() getUBX Ack"));
    gps_set_sucess=getUBX_ACK(setNav);
    if(!gps_set_sucess){
      Debugger.println(F("setup() ERROR gps_set_sucess not set"));
    }
  }
  gps_set_sucess=0; //reset gps_set_sucess for next time

  //This seems to make loop() never start, so dont send from here
  //Debugger.println(F("setup() Send a startup message to the Radio"));
  //rtty_txstringchk("JimBob Tracker");


// All of this lot is now in rtty_txstringchk so shouldent be needed, delet once sure it works.
//  sprintf(datastring,"RTTY TEST BEACON RTTY TEST BEACON"); // Puts the text in the datastring
//  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
//  char checksum_str[6];
//  sprintf(checksum_str, "*%04X\r\n", CHECKSUM);
//  strcat(datastring,checksum_str);
// 
//  rtty_txstring (datastring);

  Debugger.println(F("setup() Finished Setup\r\n\r\n\r\n\r\n"));
  delay(2000);
    
}



//*********************************************************************************
//
// loop()
//
//**********************************************************************************

//Variables that we use inside loop()

unsigned long lastloopmillis = 0;  //What time did we last go around loop()


void loop() {
  
  int wait = 0; //Break out of various loops if we get stuck
  
  Debugger.println(F("loop()"));
  
  Debugger.println(F("loop() Dont go around this loop more than once every 3 seconds"));
  Debugger.print(F("loop() "));
  if (millis() <= (lastloopmillis + 3000)) {
    Debugger.print(F("Waiting, "));
    delay(100);
  }
  lastloopmillis = millis();
  Debugger.println("");
  
  Debugger.println(F("loop() Request Navigation Data from GPS module"));
  Serial.println("$PUBX,00*33");
  
  // Ublox say the GPS sentance will be here within 1 second, but we dont want to proceed around loop() 
  // (which would ask for a sentance again) until this sentance is waiting for us
  while (!Serial.available()){
    Debugger.println(F("loop() Delaying for Ublox to provide data"));
    delay(100);
    wait++;
    if (wait > 20) {
      Debugger.println(F("loop() ERROR: Giving up waiting after 2 seconds, start loop() again"));
      return; // go back to the start of loop
    }
  }
     
  Debugger.println(F("loop() While there is data avaliable, pass it to gps.encode"));
  Debugger.println(F("loop() but if gps.encode says we have a full sentance, move on"));
  Debugger.print(F("loop() Reading data from GPS: "));
  while (Serial.available()) {
    Debugger.print("X");
    if (gps.encode(Serial.read())) {
      Debugger.println("");
      Debugger.println(F("loop() gps.encode says we have a full sentance, move on"));
      break;
    }
  }
  Debugger.println("");
   
  Debugger.println(F("loop() Convert some of the data from Tiny GPS format"));
  Debugger.println(F("loop() to the UKHAS format for tx'ing"));
  // An example senatnce might be
  // $$CALLSIGN,sentence_id,time,latitude,longitude,altitude,optional speed,optional bearing,optional internal temperature,*CHECKSUM\r\n
  //
  // our sentance will be
  // %%JimBob, id, time, lat, lon, alt, sats, *CHECKSUM\r\n
  //
  // Before Fix:  $$JIMBOB,1,4294:96:72,1000.0000,1000.0000,1000000,255*8647
  // With fix: 
  //
  // http://ukhas.org.uk/communication:protocol
    
  Debugger.println(F("loop() Increment sentance id, a unique id for each sentance"));
  s.id = s.id + 1; 
    
  Debugger.println(F("loop() Convert time to hhmmss"));
  gps.get_datetime(&g.date, &g.time, &g.age);
  s.hour = (g.time / 1000000);
  s.minute = (g.time - (s.hour * 1000000)) / 10000;
  s.second = (g.time - ((s.hour * 1000000) + (s.minute * 10000)));
  s.second = s.second / 100;  
    
  Debugger.println(F("loop() Retrieve +/- lat/long in 100000ths of a degree"));
  gps.f_get_position(&g.flat, &g.flon);
  floatToString(s.latbuf, g.flat, 4, 0);
  floatToString(s.lonbuf, g.flon, 4, 0);

  Debugger.println(F("loop() Get Altitude"));
  g.falt = gps.f_altitude();
  s.alt = long(g.falt);
  
  s.sats = gps.satellites();
  
  Debugger.println(F("loop() Build the string to send"));
  make_string();

  Debugger.println(F("loop() Transmit the data"));
  rtty_txstring(sentance);

}




void make_string()
{
  char checksum[10];
  
  snprintf(sentance, sizeof(sentance), "$$JIMBOB,%d,%d:%d:%d,%s,%s,%ld,%d", s.id, s.hour, s.minute, s.second, s.latbuf, s.lonbuf, s.alt, s.sats);

  //snprintf(checksum, sizeof(checksum), "*%02X\r\n", xor_checksum(sentance));
  snprintf(checksum, sizeof(checksum), "*%04X\r\n", gps_CRC16_checksum(sentance));

  if (strlen(sentance) > sizeof(sentance) - 4 - 1)  {
	Debugger.println(F("make_string() ERROR Don't overflow the buffer!"));
        return;
  }

  // Also copy checksum's terminating \0 (hence the +1).
  memcpy(sentance + strlen(sentance), checksum, strlen(checksum) + 1);
}


void rtty_txstringchk (char * string)
{
  Debugger.println(F("rtty_txstringchk()"));
  
  /* Take the string passed to us, calculate the checksum, concatenate the checksum
  ** and pass to rttty_txstring
  */
  unsigned int checksum_int; // The interger value of the checksum
  char checksum_str[6];  // The string equivilent of the checksum
 
  Debugger.println(F("rtty_txstringchk() Calculate the checksum for this datastring"));
  checksum_int = gps_CRC16_checksum(string);
  
  Debugger.println(F("rtty_txstringchk() Convert the checksum to a string in the right format"));
  sprintf(checksum_str, "*%04X\r\n", checksum_int);
  
  Debugger.println(F("rtty_txstringchk() Concat the checksum string with the original string"));
  strcat(string,checksum_str);
 
  Debugger.println(F("rtty_txstringchk() pass the new string to rtty_txstring"));
  rtty_txstring (string);
  
}
 
void rtty_txstring (char * string)
{
  Debugger.println(F("rtty_txstring()"));
  
  Debugger.println(F("rtty_txstring() Simple function to sent a char at a time to rtty_txbyte")); 
  Debugger.println(F("rtty_txstring() NB Each char is one byte (8 Bits)"));
 
  char c;
 
  c = *string++;
 
  Debugger.print(F("rtty_txstring() Send: "));
  while ( c != '\0')
  {
    Debugger.print(c);
    rtty_txbyte (c);
    c = *string++;
  }
  Debugger.println("");
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
 
  //delayMicroseconds(3370); // 300 baud
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


char* floatToString(char * outstr, double val, byte precision, byte widthp){
  char temp[16];
  byte i;
  // compute the rounding factor and fractional multiplier
  double roundingFactor = 0.5;
  unsigned long mult = 1;
  for (i = 0; i < precision; i++)
  {
    roundingFactor /= 10.0;
    mult *= 10;
  }

  temp[0]='\0';
  outstr[0]='\0';

  if(val < 0.0){
    strcpy(outstr,"-\0");
    val = -val;
  }

  val += roundingFactor;

  strcat(outstr, itoa(int(val),temp,10));  //prints the int part
  if( precision > 0) {
    strcat(outstr, ".\0"); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
      mult *=10;

    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;

    while(frac1 /= 10)
      padding--;

    while(padding--)
      strcat(outstr,"0\0");

    strcat(outstr,itoa(frac,temp,10));
  }

  // generate space padding
  if ((widthp != 0)&&(widthp >= strlen(outstr))){
    byte J=0;
    J = widthp - strlen(outstr);

    for (i=0; i< J; i++) {
      temp[i] = ' ';
    }

    temp[i++] = '\0';
    strcat(temp,outstr);
    strcpy(outstr,temp);
  }

  return outstr;
}

// UBX Functions

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  Debugger.println(F("sendUBX()"));
  Debugger.print(F("sendUBX() Byte: "));
  for(int i=0; i<len; i++) {
    Debugger.print(i, DEC);
    Serial.write(MSG[i]);
  }
  Serial.println();
  Debugger.println("");
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  
  Debugger.println(F("getUBX_ACK() 1"));
    
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
 
  Debugger.println(F("getUBX_ACK() 2 Construct the expected ACK packet"));  
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
 
  Debugger.println(F("getUBX_ACK() 3 Calculate the checksums"));
  Debugger.print(F("getUBX_ACK() Checksums: "));
  for (uint8_t i=2; i<8; i++) {
    Debugger.print(i, DEC);
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  Debugger.print(ackPacket[8], HEX);
  Debugger.print(ackPacket[9], HEX);
  Debugger.println("");
 
  while (1) {
 
    Debugger.println(F("getUBX_ACK() 4 Test for success?"));
    if (ackByteID > 9) {
      Debugger.println(F("getUBX_ACK() 4 All packets in order!"));
      return true;
    }
 
    Debugger.println(F("getUBX_ACK() 6 Timeout if no valid response in 3 seconds"));
    if (millis() - startTime > 3000) { 
      Debugger.println(F("getUBX_ACK() 7 ERROR Timeout!"));
      return false;
    }
 
    Debugger.println(F("getUBX_ACK() 8 Make sure data is available to read"));
    if (Serial.available()) {
      b = Serial.read();
 
      Debugger.println(F("getUBX_ACK() 9 Check that bytes arrive in sequence as per expected ACK packet"));
      if (b == ackPacket[ackByteID]) {
        Debugger.println(F("getUBX_ACK() 10 This ackByteID Correct"));
        ackByteID++;
      } 
      else {
        Debugger.println(F("getUBX_ACK() 11 ERROR This ackByteID Incorrect, reset and start again"));
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}

