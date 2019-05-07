#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include "RF24.h"
#define TX 4
#define RX 5
TinyGPS gps;
SoftwareSerial nss(TX,RX);
// GPS functions
static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_float(float val,float invalid,int len,int prec);
static void print_int(unsigned long val, unsigned long invalid,int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str,int len);

// GPS vars
struct package {
  float latitude;
  float longitude;
  float altude;
  float kph;
  float ttlDist;
  bool broadcast;
};
typedef struct package Package;
Package data;
float maxSpeed = 0,lastFlat = 0,lastFlon = 0,totalDistance = 0;
bool start = 1;
// Radio vars
bool radioNumber = 0;
RF24 radio(D3,D8);
byte addresses[][6] = {"0","1"}; // Recieve and transmit addresses
bool role = 0;  // Set to recieve or transmit

void setup() {
  Serial.begin(115200);
  nss.begin(9600);
  Serial.println(F("Starting: Buddy System..."));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Start the radio listening for data
  radio.startListening();
} // End Setup **************************************************************************

void loop() {
  bool newdata = false;
  unsigned long start = millis();
 /****************** Ping Out Role ***************************/  
  if (role == 1)  {
      
      radio.stopListening();                                    // First, stop listening so we can talk.
      
      //radio.printDetails();
      Serial.println(F("Now sending"));
      unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete
      //const char text[] = "Hello World";
      if (!radio.write( &data, sizeof(data) )){
          Serial.println(F("failed"));
      }
        
      while (millis() - start < 1000) {
          if (feedgps()) {
            newdata = true;
          }
      }
    gpsdump(gps,data);
  } else {
  
  }


  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T'){      
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
      data.broadcast = true;
   }else if ( c == 'R'){
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
       //radio.startListening();
       totalDistance = 0;
       data.broadcast = false;
   }
  }
} // End Loop ********************************************************************************

unsigned long calc_dist(float flat1,float flon1,float flat2,float flon2) {
  float dist_calc = 0, dist_calc2 = 0,diflat = 0,diflon = 0;

  diflat = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1); 
  dist_calc2 *= cos(flat2);
  dist_calc2 *= sin(diflon/2.0);
  dist_calc2 *= sin(diflon/2.0);
  dist_calc += dist_calc2;

  dist_calc = (2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  dist_calc *= 6371000.0;
  return dist_calc;
}

static bool feedgps () {
  while (nss.available()) {
    if (gps.encode(nss.read())) {
      return true;
    }
  }
  return false;
}

static void gpsdump(TinyGPS &gps,Package &data) {
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;

  gps.f_get_position(&flat,&flon,&age);
  if (age == TinyGPS::GPS_INVALID_AGE) {
    Serial.println("No fix detected");
  } else if (age > 5000) {
    Serial.println("Warning: possible stale data!");
  } else {
    Serial.println("Data is current");
  }
  print_date(gps);
  gps.stats(&chars,&sentences,&failed); 
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  Serial.println();
  Serial.print("Latitude: ");
  Serial.print((float)(flat),6);
  Serial.print(" Longitude ");
  Serial.print((float)(flon),6);
  Serial.print(" Age: ");
  Serial.print(age);
  Serial.println();
  Serial.print("Speed(KPH) ");
  Serial.print(gps.f_speed_mph());
  Serial.print(" Altitude: ");
  Serial.print(gps.f_altitude());
  Serial.print(" Broadcast: ");
  Serial.print(data.broadcast);
  Serial.println();
  data.latitude = flat;
  data.longitude = flon;
  data.altude = gps.f_altitude();
  data.kph = gps.f_speed_kmph();
  if (gps.f_speed_kmph() > 3.0) {
    if (start == 1) {
      start = 0;
      lastFlat = flat;
      lastFlon = flon;
    } else {
      totalDistance = totalDistance + calc_dist(flat,flon,lastFlat,lastFlon);
      data.ttlDist = totalDistance;
    }
  }
  Serial.print(" TotalDst: ");
  Serial.println(totalDistance);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  feedgps();
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  feedgps();
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  feedgps();
}
