#include <TimeLib.h>

// Define constants for UBX_NAV protocol
const uint16_t UBX_TS_MS = 1000; // Sample time in ms 
const uint8_t UBX_HEADER[2] = {0xB5, 0x62};
const uint8_t UBX_NAV_MSG_CLASS = 0x01;

// UBX_NAV_EOE Constants
const uint8_t UBX_NAV_EOE_ID = 0x61;
const uint8_t UBX_NAV_EOE_LEN[2] = {0x04, 0x00}; // Little endian
const uint8_t EOE_LEN = 8; // package length that will be cheksum

// UBX_NAV_DOP Constantts
const uint8_t UBX_NAV_DOP_ID = 0x04;
const uint8_t UBX_NAV_DOP_LEN[2] = {0x12, 0x00}; // Little endian
const uint8_t DOP_LEN = 22;

// Conversion factors 
const float DEG2RAD = 0.01745329251;
const float RAD2DEG = 57.295779513;

// Message buffer
uint8_t msg[100];

// Checksum buffer
uint8_t checksum[2];

uint32_t start_time_ms;
time_t posix;
tmElements_t tm;
int32_t sec_dec;

// Define ECEF coordinate structure
struct ECEF
{
  float x_m;
  float y_m;
  float z_m;
};

// Define NED coordinate structure - for moving baseline
struct NED
{
  float x_m;
  float y_m;
  float z_m;
};

// Define LLA coordinate structure
struct LLA
{
  float lat_deg;
  float lon_deg;
  float h_m;
};

// Define GPS time structure
struct GPS_time
{
  uint32_t gps_time;
  uint16_t gps_week;
  uint32_t gps_tow_ms; 
};

// Define structure for ellipsoid parameters
struct WGS84
{
  float a;
  float e;
};

// Declair WGS84 constant
WGS84 wgs84 = {6378137.0, 0.0818191};

// Convert uint32_t to four uint8_t bytes
union X4
{
  uint32_t l;
  uint8_t b[4];
};


// Declare states
X4 time_of_week; // Separate for serial communication

// Positioning states of beacon 1
LLA lla_1 = {33.21529812, -87.54356012, 0};
ECEF ecef_1 = {0, 0, 0};
GPS_time gps_time;

// Test string
String GPZDA = "$GPZDA,105146.14,31,07,2022,00,00";

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin (115200);

}

void loop() 
{
  proc_gpzda(&GPZDA, &tm, &sec_dec);
  posix = makeTime(tm);
  posix2gps(&posix, &gps_time, &sec_dec);
  start_time_ms = millis();
  time_of_week.l = gps_time.gps_tow_ms;
  lla2ecef(&lla_1, &ecef_1, &wgs84);
  //send_dop();
  send_eoe();
  delay (UBX_TS_MS - millis() + start_time_ms);
}

// Function to send UBX_NAV_EOE message
void send_eoe()
{
  Serial.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_EOE_ID;
  msg[2] = UBX_NAV_EOE_LEN[0];
  msg[3] = UBX_NAV_EOE_LEN[1];
  msg[4] = time_of_week.b[0];
  msg[5] = time_of_week.b[1];
  msg[6] = time_of_week.b[2];
  msg[7] = time_of_week.b[3];
  Serial.write(msg,EOE_LEN);
  calc_checksum(EOE_LEN);
  Serial.write(checksum,2);
}

// Function to send UBX_NAV_DOP message
void send_dop()
{
  // All DOP values are mock values 1.56 and are conservative uncertainty of beacon system
  Serial.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_DOP_ID;
  msg[2] = UBX_NAV_DOP_LEN[0];
  msg[3] = UBX_NAV_DOP_LEN[1];
  msg[4] = time_of_week.b[0];
  msg[5] = time_of_week.b[1];
  msg[6] = time_of_week.b[2];
  msg[7] = time_of_week.b[3];
  msg[8] = 0x9C;
  msg[9] = 0x00;
  msg[10] = 0x9C;
  msg[11] = 0x00;
  msg[12] = 0x9C;
  msg[13] = 0x00;
  msg[14] = 0x9C;
  msg[15] = 0x00;
  msg[16] = 0x9C;
  msg[17] = 0x00;
  msg[18] = 0x9C;
  msg[19] = 0x00;
  msg[20] = 0x9C;
  msg[21] = 0x00;
  Serial.write(msg,DOP_LEN);
  calc_checksum(DOP_LEN);
  Serial.write(checksum,2);
}

// Generate checksum bytes of UBX messages based on UBX description
void calc_checksum (uint8_t payload_size)
{
  checksum[0] = 0;
  checksum[1] = 0;
  for (uint8_t i=0; i<payload_size;i++)
  {
    checksum[0] += msg[i];
    checksum[1] += checksum[0];
  }

}

// Convert LLA to NED frame
void lla2ecef (struct LLA *lla, struct ECEF *ecef, struct WGS84 *wgs)
{
  float lat_rad = lla->lat_deg * DEG2RAD;
  float lon_rad = lla->lon_deg * DEG2RAD;
  float slat = sin(lat_rad);
  float clat = cos(lat_rad);
  float slon = sin(lon_rad);
  float clon = cos(lon_rad);
  float sq_e = sq(wgs->e);
  float N = wgs->a / sqrt(1 - sq_e * sq(slat));
  ecef->x_m = (N + lla->h_m) * clat * clon;
  ecef->y_m = (N + lla->h_m) * clat * slon;
  ecef->z_m = ((1 - sq_e) * N + lla->h_m) * slat;
}

// Process GPZDA string to get UTC time element object
void proc_gpzda(String *gpzda, tmElements_t *tm, long int *sec_dec)
{
  // Extract time data from GPZDA string and parse value in the tm time_elements structure
  String str_temp = gpzda->substring(7,9);
  tm->Hour = str_temp.toInt();
  str_temp = gpzda->substring(9, 11);
  tm->Minute = str_temp.toInt();
  str_temp = gpzda->substring(11,13);
  tm->Second = str_temp.toInt();
  str_temp = gpzda->substring(14,16);
  *sec_dec = str_temp.toInt();
  str_temp = gpzda->substring(17,19);
  tm->Day = str_temp.toInt();
  str_temp = gpzda->substring(20,22);
  tm->Month = str_temp.toInt();
  str_temp = gpzda->substring(23,27);
  tm->Year = str_temp.toInt() - 1970;
}

// Convert UNIX to GPS timestamps
void posix2gps(time_t *posix, GPS_time *gps_time, int32_t *sec_dec)
{
  gps_time->gps_time = *posix - 315964800 + 18;
  gps_time->gps_week = int(gps_time->gps_time / 604800);
  uint32_t tow_s = gps_time->gps_time - gps_time->gps_week * 604800;
  gps_time->gps_tow_ms = tow_s * 1000 + *sec_dec * 10;
}

// Parse GPGGA string and pass data to LLA struct
void proc_gpgga(String *gpgga, struct LLA *lla)
{
  // Extract time data from GPZDA string and parse value in the tm time_elements structure
  String str_temp = gpgga->substring(17,19);
  float coor_int = str_temp.toFloat();
  str_temp = gpgga->substring(19,28);
  lla->lat_deg = coor_int + str_temp.toFloat()/60;
  str_temp = gpgga->substring(29,30);
  if (str_temp == "S")
  {
    lla->lat_deg *= -1;
  }
  str_temp = gpgga->substring(31,34);
  coor_int = str_temp.toFloat();
  str_temp = gpgga->substring(34,43);
  lla->lon_deg = coor_int + str_temp.toFloat()/60;
  str_temp = gpgga->substring(44,45);
  if (str_temp == "W")
  {
    lla->lon_deg *= -1;
  }
  str_temp = gpgga->substring(55,58);
  lla->h_m = str_temp.toFloat();
}
