#include <SPI.h>
#include "decoders.h"
#include "rfm69_constants.h"
#include <RFM69.h>
//#include "local_sprintf.h"

// Modified to decode the AcuRite 5 in 1 weather station
// ** Acurite known message types
#define ACURITE_MSGTYPE_TOWER_SENSOR                    0x04
#define ACURITE_MSGTYPE_6045M                           0x2f
#define ACURITE_MSGTYPE_5N1_WINDSPEED_WINDDIR_RAINFALL  0x31
#define ACURITE_MSGTYPE_5N1_WINDSPEED_TEMP_HUMIDITY     0x38

#define RX_PIN 14    // 14 == A0. Must be one of the analog pins, b/c of the analog comparator being used.

#undef DEBUG

#define VERSION "20180811"

// From draythomp/Desert-home-rtl_433
// matches acu-link internet bridge values
// The mapping isn't circular, it jumps around.
char * acurite_5n1_winddirection_str[] =
    {"NW",  // 0  315
     "WSW", // 1  247.5
     "WNW", // 2  292.5
     "W",   // 3  270
     "NNW", // 4  337.5
     "SW",  // 5  225
     "N",   // 6  0
     "SSW", // 7  202.5
     "ENE", // 8  67.5
     "SE",  // 9  135
     "E",   // 10 90
     "ESE", // 11 112.5
     "NE",  // 12 45
     "SSE", // 13 157.5
     "NNE", // 14 22.5
     "S"};  // 15 180


const float acurite_5n1_winddirections[] =
    { 315.0, // 0 - NW
      247.5, // 1 - WSW
      292.5, // 2 - WNW
      270.0, // 3 - W
      337.5, // 4 - NNW
      225.0, // 5 - SW
      0.0,   // 6 - N
      202.5, // 7 - SSW
      67.5,  // 8 - ENE
      135.0, // 9 - SE
      90.0,  // a - E
      112.5, // b - ESE
      45.0,  // c - NE
      157.5, // d - SSE
      22.5,  // e - NNE
      180.0, // f - S
    };

    float tempc, tempf, wind_dird, rainfall = 0.0, wind_speed_kph, wind_speed_mph;
    uint8_t humidity, sensor_status, message_type, sequence_num;
    char channel, *wind_dirstr = "";
    char channel_str[2];
    uint16_t sensor_id;
    int raincounter, temp, battery_low;
// 5n1 keep state for how much rain has been seen so far
    int acurite_5n1raincounter = 0;  // for 5n1 decoder
    int acurite_5n1t_raincounter = 0;  // for combined 5n1/TXR decoder


// AcuRite5N1Decoder manages the decoding of the physical layer
AcuRite5N1Decoder adx;


byte packetBuffer[60], packetFill;

volatile word pulse_width;
word last_width;
word last_poll = 0;

SPISettings SPI_settings(2000000, MSBFIRST, SPI_MODE0);

RFM69 radio;

//#define intPin 9
#define selPin 10
// MOSI is 11; MISO is 12; SCK is 13.

ISR(ANALOG_COMP_vect) {
    word now = micros();
    pulse_width = now - last_width;
    last_width = now;
}


static void setupPinChangeInterrupt () {
    pinMode(RX_PIN, INPUT);
    digitalWrite(RX_PIN, HIGH); // enable pullup
    
    // enable analog comparator with fixed voltage reference.
    // This will trigger an interrupt on any change of value on RX_PIN.
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
    ADCSRA &= ~ _BV(ADEN);
    ADCSRB |= _BV(ACME);
    ADMUX = RX_PIN - 14; // Note that this is specific to the analog pins...
}

//
// Packet Buffer stuff
void addData(const byte *buf, byte len)
{
#ifdef DEBUG
  Serial.print("addData: ");
    for (byte i = 0; i < len; ++i) {
        Serial.print(' ');
        Serial.print((int) buf[i], HEX);
    }
    Serial.println();
#endif
  
    if (packetFill + len < sizeof(packetBuffer)) {
        memcpy(packetBuffer + packetFill, buf, len);
        packetFill += len;
    }
}

byte removeData(byte *buf, byte len_requested)
{
  // If there's no data in the buffer, then just return
  if (packetFill == 0)
    return 0;
  
  // If the buffer is the same size as (or smaller than) what's requested, then return what's in the buffer
  if (packetFill <= len_requested) {
    memcpy(buf, packetBuffer, packetFill);
    len_requested = packetFill;
    packetFill = 0;
    return len_requested;
  }
  
  // The buffer must have more data in it than the request.
  //  copy the data to return in to the return buffer
  memcpy(buf, packetBuffer, len_requested);
  //  move the remaining data down in packetBuffer
  for (int i=0; i<packetFill - len_requested; i++) {
    packetBuffer[i] = packetBuffer[i + len_requested];
  }
  packetFill -= len_requested;
  //  return how many bytes we copied (same as the requested length)
  return len_requested;
}

//
// Runs the RF pulse detector in AcuRite5N1Decoder
static void runPulseDecoders (volatile word& pulse) {
  // get next pulse with and reset it - need to protect against interrupts
  cli();
  word p = pulse;
  pulse = 0;
  sei();

  // if we had a pulse, go through each of the decoders
  if (p != 0) { 
    if (adx.nextPulse(p)) {
      byte size;
      const byte *data = adx.getData(size);
      addData(data, size);
      adx.resetDecoder();
    }
  }
}

//
// talk to the RFM69
uint16_t xfer16(uint16_t cmd)
{
  uint16_t reply;

  SPI.beginTransaction(SPI_settings);

  if (selPin != -1)
    digitalWrite(selPin, LOW);
  reply = SPI.transfer(cmd >> 8) << 8;
  reply |= SPI.transfer(cmd & 0xFF);
  if (selPin != -1)
    digitalWrite(selPin, HIGH);

  SPI.endTransaction();

  return reply;
}

uint8_t rfm69_read(uint8_t reg)
{
  uint8_t reply;
  
  SPI.beginTransaction(SPI_settings);
  if (selPin != -1)
    digitalWrite(selPin, LOW);
    
  SPI.transfer(reg);
  reply = SPI.transfer(0x00);
    
  if (selPin != -1)
    digitalWrite(selPin, HIGH);
    
  SPI.endTransaction();
  
  return reply;
}

void rfm69_write(uint8_t reg, uint8_t val)
{
  SPI.beginTransaction(SPI_settings);
  if (selPin != -1)
    digitalWrite(selPin, LOW);
    
  SPI.transfer(reg | 0x80); // write bit
  SPI.transfer(val);
    
  if (selPin != -1)
    digitalWrite(selPin, HIGH);
    
  SPI.endTransaction();
}

static void rf69_init_OOK () {
//  uint8_t dev_id = rfm69_read(RegVersion);
//  if (dev_id != 0x00 || dev_id != 0xff)
//    return;

  rfm69_write(RegOpMode, RegOpModeStandby);
  rfm69_write(RegDataModul, RegDataModulContinuous | RegDataModulOOK); // Set continuous OOK mode
  RegBitrateSet(8000); // 8.0kb/s
  RegFrfSet(433920000); // fundamental frequency = 433.92MHz (really 433.920044 MHz)
  
  rfm69_write(RegRxBw, RegRxBwDccFreq4 | RegRxBwOOK50k); // 4% DC cancellation; 50k bandwidth in OOK mode
  rfm69_write(RegLna, RegLnaZ200 | RegLnaGainSelect12db); // 200 ohm, -12db

  rfm69_write(RegOokPeak,
   RegOokThreshPeak | RegOokThreshPeakStep0d5 | RegOokThreshPeakDec1c );

  rfm69_write(RegOpMode, RegOpModeRX);
  rfm69_write(RegAfcFei, RegAfcFeiAfcClear);
}

void setup () {
    Serial.begin(115200);
    Serial.print("\n[AcuRite decoder version ");
    Serial.print(VERSION);
    Serial.println("]");
    
//    pinMode(intPin, INPUT);
//    digitalWrite(intPin, LOW); // turn off pull-up until it's running properly
    
    rf69_init_OOK();
    
    setupPinChangeInterrupt();
}

byte calcParity(byte b)
{
  byte result = 0;
  for (byte i=0; i<=6; i++) {
    result ^= (b & (1<<i)) ? 1 : 0;
  }
  return result ? 0x80 : 0x00;
}

void PrintSource(const byte *data)
{
  Serial.print("Source: ");
  unsigned long id = ((data[0] & 0x3f) << 7) | (data[1] & 0x7f);
  Serial.print(id);
  
  Serial.print("/");
  switch (data[0] & 0xC0) {
  case 0xc0:
    Serial.print("A");
    break;
  case 0x80:
    Serial.print("B");
    break;
  case 0x00:
    Serial.print("C");
    break;
  default:
    Serial.print("x");
    break;
  }
}
float celsius2fahrenheit(float celsius) {
  return celsius * 9 / 5 + 32;
}


float fahrenheit2celsius(float fahrenheit) {
  return (fahrenheit - 32) / 1.8;
}


float kmph2mph(float kmph) {
  return kmph / 1.609344;
}

float mph2kmph(float mph) {
  return mph * 1.609344;
}


float mm2inch(float mm) {
  return mm * 0.039370;
}

float inch2mm(float inch) {
  return inch / 0.039370;
}


float kpa2psi(float kpa) {
  return kpa / 6.89475729;
}

float psi2kpa(float psi) {
    return psi * 6.89475729;
}


float hpa2inhg(float hpa) {
    return hpa / 33.8639;
}

float inhg2hpa(float inhg) {
    return inhg * 33.8639;
}

// The sensor sends the same data three times, each of these have
// an indicator of which one of the three it is. This means the
// checksum and first byte will be different for each one.
// The bits 5,4 of byte 0 indicate which copy of the 65 bit data string
//  00 = first copy
//  01 = second copy
//  10 = third copy
//  1100 xxxx  = channel A 1st copy
//  1101 xxxx  = channel A 2nd copy
//  1110 xxxx  = channel A 3rd copy
static int acurite_5n1_getMessageCaught(uint8_t byte){
    return (byte & 0x30) >> 4;
}

//
// Helper functions copied from rtl_433 and reused :)
// Temperature encoding for 5-n-1 sensor and possibly others
static float acurite_getTemp (uint8_t highbyte, uint8_t lowbyte) {
    // range -40 to 158 F
    int highbits = (highbyte & 0x0F) << 7 ;
    int lowbits = lowbyte & 0x7F;
    int rawtemp = highbits | lowbits;
    float temp_F = (rawtemp - 400) / 10.0;
    return temp_F;
}

static float acurite_getWindSpeed_kph (uint8_t highbyte, uint8_t lowbyte) {
    // range: 0 to 159 kph
    // raw number is cup rotations per 4 seconds
    // http://www.wxforum.net/index.php?topic=27244.0 (found from weewx driver)
	int highbits = ( highbyte & 0x1F) << 3;
    int lowbits = ( lowbyte & 0x70 ) >> 4;
    int rawspeed = highbits | lowbits;
    float speed_kph = 0;
    if (rawspeed > 0) {
        speed_kph = rawspeed * 0.8278 + 1.0;
    }
    return speed_kph;
}

static int acurite_getHumidity (uint8_t byte) {
    // range: 1 to 99 %RH
    int humidity = byte & 0x7F;
    return humidity;
}

static int acurite_getRainfallCounter (uint8_t hibyte, uint8_t lobyte) {
    // range: 0 to 99.99 in, 0.01 in incr., rolling counter?
	int raincounter = ((hibyte & 0x7f) << 7) | (lobyte & 0x7F);
    return raincounter;
}

// The high 2 bits of byte zero are the channel (bits 7,6)
//  00 = C
//  10 = B
//  11 = A
static char chLetter[4] = {'C','E','B','A'}; // 'E' stands for error

static char acurite_getChannel(uint8_t byte){
    int channel = (byte & 0xC0) >> 6;
    return chLetter[channel];
}

// 5-n-1 sensor ID is the last 12 bits of byte 0 & 1
// byte 0     | byte 1
// CC RR IIII | IIII IIII
//
static uint16_t acurite_5n1_getSensorId(uint8_t hibyte, uint8_t lobyte){
    return ((hibyte & 0x0f) << 8) | lobyte;
}


void DecodePacket(const byte *data)
{
#ifdef DEBUG
  Serial.print("DecodePacket: ");
    for (byte i = 0; i < 8; i++) {
        Serial.print(' ');
        Serial.print((int) data[i], HEX);
    }
    Serial.println();
#endif
  
  
    // Check parity bits. (Byte 0 and byte 7 have no parity bits.)
    // ... Byte 1 might also not be parity-ing correctly. A new device
    // I bought in early 2018 has a parity error in byte 1, but
    // otherwise seems to work correctly.
    for (int i=2; i<=6; i++) {
      if (calcParity(data[i]) != (data[i] & 0x80)) {
#ifdef DEBUG
        Serial.print("Parity failure in byte ");
        Serial.println(i);
#endif
        return;
      }
    }

    // Check mod-256 checksum of bytes 0 - 6 against byte 7
    unsigned char cksum = 0;
    for (int i=0; i<=6; i++) {
      cksum += data[i];
    }
    if ((cksum & 0xFF) != data[7]) {
      PrintSource(data);
      Serial.print(" checksum failure - calcd 0x");
      Serial.print(cksum, HEX);
      Serial.print(", expected 0x");
      Serial.println(data[6], HEX);
      return;
    }
    
    channel = acurite_getChannel(data[0]);
    sprintf(channel_str, "%c", channel);
    message_type = data[2] & 0x3f;
    sensor_id = acurite_5n1_getSensorId(data[0],data[1]);
    sequence_num = acurite_5n1_getMessageCaught(data[0]);
    battery_low = (data[2] & 0x40) >> 6;

    if (message_type == ACURITE_MSGTYPE_5N1_WINDSPEED_WINDDIR_RAINFALL) {
      // Wind speed, wind direction, and rain fall
      wind_speed_kph = acurite_getWindSpeed_kph(data[3], data[4]);
      wind_speed_mph = kmph2mph(wind_speed_kph);
      wind_dird = acurite_5n1_winddirections[data[4] & 0x0f];
      wind_dirstr = acurite_5n1_winddirection_str[data[4] & 0x0f];
      raincounter = acurite_getRainfallCounter(data[5], data[6]);
      if (acurite_5n1t_raincounter > 0) {
          // track rainfall difference after first run
          // FIXME when converting to structured output, just output
          // the reading, let consumer track state/wrap around, etc.
          rainfall = ( raincounter - acurite_5n1t_raincounter ) * 0.01;
          if (raincounter < acurite_5n1t_raincounter) {
              fprintf(stderr, "%s Acurite 5n1 sensor 0x%04X Ch %c, rain counter reset or wrapped around (old %d, new %d)\n",
                  /*time_str,*/ sensor_id, channel, acurite_5n1t_raincounter, raincounter);
              acurite_5n1t_raincounter = raincounter;
          }
        } else {
            // capture starting counter
            acurite_5n1t_raincounter = raincounter;
            fprintf(stderr, "%s Acurite 5n1 sensor 0x%04X Ch %c, Total rain fall since last reset: %0.2f\n",
            /*time_str,*/ sensor_id, channel, raincounter * 0.01);
        }

    } else if (message_type == ACURITE_MSGTYPE_5N1_WINDSPEED_TEMP_HUMIDITY) {
      // Wind speed, temperature and humidity
      wind_speed_kph = acurite_getWindSpeed_kph(data[3], data[4]);
      wind_speed_mph = kmph2mph(wind_speed_kph);
      tempf = acurite_getTemp(data[4], data[5]);
      tempc = fahrenheit2celsius(tempf);
      humidity = acurite_getHumidity(data[6]);
    }
}

void loop () {
    runPulseDecoders(pulse_width);
    
    while (packetFill >= (ACURITE_5N1_BITLEN / 8)) {
      byte dbuf[7];
      removeData(dbuf, (ACURITE_5N1_BITLEN / 8));
      DecodePacket(dbuf);
    }
}
