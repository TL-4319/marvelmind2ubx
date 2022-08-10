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


// UBX_NAV_PVT Constants
const uint8_t UBX_NAV_PVT_ID = 0x07;
const uint8_t UBX_NAV_PVT_LEN[2] = {0x5C, 0x00}; //Little endian (92 bytes)
const uint8_t PVT_LEN = 96;

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
  float vx_mps;
  float vy_mps;
  float vz_mps;
};

// Define NED coordinate structure - for moving baseline
struct NED
{
  float x_m;
  float y_m;
  float z_m;
  float vx_mps;
  float vy_mps;
  float vz_mps;
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
union U4
{
  uint32_t u32;
  uint8_t u8[4];
};

// Convert uint8_t to 2 uint8_t bytes
union U2
{
  uint8_t u16;
  uint8_t u8[2];
};

// Convert int32_t to 4 int8_t bytes
union I4
{
  int32_t i32;
  int8_t i8[4];
  uint8_t u8[4];
};

// Declare states
U4 time_of_week; // Separate for serial communication
U2 year_short;
I4 lat_deg1, lon_deg1, h_mm, gspeed, mot_head, vx_1, vy_1, vz_1;

// Positioning states of beacon 1
LLA lla_1 = {33.21529812, -87.54356012, 0};
LLA prev_lla_1;
ECEF ecef_1 = {0, 0, 0, 0, 0, 0};
GPS_time gps_time;

// Test string
String GPZDA = "$GPZDA,105146.14,31,07,2022,00,00";

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin (115200);
  Serial1.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

}

void loop() 
{
  proc_gpzda(&GPZDA, &tm, &sec_dec);
  posix = makeTime(tm);
  posix2gps(&posix, &gps_time, &sec_dec);
  start_time_ms = millis();
  time_of_week.u32 = gps_time.gps_tow_ms;
  year_short.u16 = tm.Year;
  lla2ecef(&lla_1, &ecef_1, &wgs84);
  lat_deg1.i32 = lla_1.lat_deg * 1000000;
  lon_deg1.i32 = lla_1.lon_deg * 1000000;
  h_mm.i32 = lla_1.h_m * 1000;
  //send_dop();
  //send_pvt();
  send_eoe();
  delay (UBX_TS_MS - millis() + start_time_ms);
}

// Function to send UBX_NAV_EOE message
void send_eoe()
{
  Serial.write(UBX_HEADER,2);
  Serial1.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_EOE_ID;
  msg[2] = UBX_NAV_EOE_LEN[0];
  msg[3] = UBX_NAV_EOE_LEN[1];
  msg[4] = time_of_week.u8[0];
  msg[5] = time_of_week.u8[1];
  msg[6] = time_of_week.u8[2];
  msg[7] = time_of_week.u8[3];
  Serial.write(msg,EOE_LEN);
  Serial1.write(msg,EOE_LEN);
  calc_checksum(EOE_LEN);
  Serial.write(checksum,2);
  Serial1.write(checksum,2);
}

// Function to send UBX_NAV_DOP message
void send_dop()
{
  // All DOP values are mock values 1.56 and are conservative uncertainty of beacon system
  Serial.write(UBX_HEADER,2);
  Serial1.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_DOP_ID;
  msg[2] = UBX_NAV_DOP_LEN[0];
  msg[3] = UBX_NAV_DOP_LEN[1];
  msg[4] = time_of_week.u8[0];
  msg[5] = time_of_week.u8[1];
  msg[6] = time_of_week.u8[2];
  msg[7] = time_of_week.u8[3];
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
  Serial1.write(msg,DOP_LEN);
  calc_checksum(DOP_LEN);
  Serial.write(checksum,2);
  Serial1.write(checksum,2);
}

// Function to send UBX_NAV_PVT message
void send_pvt()
{
  Serial.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_PVT_ID;
  msg[2] = UBX_NAV_PVT_LEN[0];
  msg[3] = UBX_NAV_PVT_LEN[1];
  msg[4] = time_of_week.u8[0];
  msg[5] = time_of_week.u8[1];
  msg[6] = time_of_week.u8[2];
  msg[7] = time_of_week.u8[3];
  msg[8] = year_short.u8[1];
  msg[8] = year_short.u8[0];
  msg[9] = tm.Month;
  msg[10] = tm.Day;
  msg[11] = tm.Hour;
  msg[12] = tm.Minute;
  msg[13] = tm.Second;
  // Emulate all data valid for bit field  11 (b00001111)
  msg[14] = 0x0F; 
  // Emulate time accuracy, locked at 1us for now
  msg[15] = 0xE8;
  msg[16] = 0x03;
  msg[17] = 0x00;
  msg[18] = 0x00;
  // Emulate nano second part of time, locked at 0ns for now
  msg[19] = 0x00;
  msg[20] = 0x00;
  msg[21] = 0x00;
  msg[22] = 0x00;
  // Fix type byte, assume 3d fix 
  msg[23] = 0x03;
  // Emulate flag for bit field 21 (00000001)
  msg[24] = 0x01;
  // Emulate flag for bit field 22 (11100000)
  msg[25] = 0xE5;
  // Number of SV - Marvelmind assumes 8
  msg[26] = 0x08;
  // Lat in deg times 10^6
  msg[27] = lat_deg1.u8[3];
  msg[28] = lat_deg1.u8[2];
  msg[29] = lat_deg1.u8[1];
  msg[30] = lat_deg1.u8[0];
  // Lon in deg times 10^6
  msg[31] = lon_deg1.u8[3];
  msg[32] = lon_deg1.u8[2];
  msg[33] = lon_deg1.u8[1];
  msg[34] = lon_deg1.u8[0];
  // Height above ellipsoid in mm
  msg[35] = h_mm.u8[3];
  msg[36] = h_mm.u8[2];
  msg[37] = h_mm.u8[1];
  msg[38] = h_mm.u8[0];
  // Height above MSL in mm the same as heigh above ellipsoid in this implementation
  msg[39] = h_mm.u8[3];
  msg[40] = h_mm.u8[2];
  msg[41] = h_mm.u8[1];
  msg[42] = h_mm.u8[0];
  // Horizontal accuracy, conservative estimate of 5cm
  msg[43] = 0x32;
  // Vertical accuracy, conservative estimate of 7cm
  msg[47] = 0x3C; 
  // NED north velocity - constant 0 for now NEED TO UPDATE WITH ACTUAL SPEED!!!
  msg[51] = vx_1.u8[3];
  msg[52] = vx_1.u8[2];
  msg[53] = vx_1.u8[1];
  msg[54] = vx_1.u8[0];
  // NED east velocity - constant 0 for now NEED TO UPDATE WITH ACTUAL SPEED!!!
  msg[55] = vy_1.u8[3];
  msg[56] = vy_1.u8[2];
  msg[57] = vy_1.u8[1];
  msg[58] = vy_1.u8[0];
    // NED down velocity - constant 0 for now NEED TO UPDATE WITH ACTUAL SPEED!!!
  msg[59] = vz_1.u8[3];
  msg[60] = vz_1.u8[2];
  msg[61] = vz_1.u8[1];
  msg[62] = vz_1.u8[0];
  // G speed - constant 0 for now
  msg[63] = gspeed.u8[3];
  msg[64] = gspeed.u8[2];
  msg[65] = gspeed.u8[1];
  msg[66] = gspeed.u8[0];
  // heading of motion - constant 0 for now
  msg[67] = mot_head.u8[3];
  msg[68] = mot_head.u8[2];
  msg[69] = mot_head.u8[1];
  msg[70] = mot_head.u8[0];
  // Speed accuracy - constant 9mm/s
  msg[71] = 0x09;
  msg[72] = 0x00;
  msg[73] = 0x00;
  msg[74] = 0x00;
  // DOP
  msg[75] = 0x9C;
  msg[76] = 0x00;
  // 
  msg[77] = 0x00;
  msg[78] = 0x02;
  // 
  msg[83] = mot_head.u8[3];
  msg[84] = mot_head.u8[2];
  msg[85] = mot_head.u8[1];
  msg[86] = mot_head.u8[0]; 
  Serial.write(msg,PVT_LEN);
  calc_checksum(PVT_LEN);
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
