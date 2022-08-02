#include <TimeLib.h>
// Conversion factors 
const float DEG2RAD = 0.01745329251;
const float RAD2DEG = 57.295779513;

uint32_t start_time_ms;
time_t posix;
tmElements_t tm;
int32_t sec_dec;

struct ECEF
{
  float x_m;
  float y_m;
  float z_m;
};

struct LLA
{
  float lat_deg;
  float lon_deg;
  float h_m;
};

struct WGS84
{
  float a;
  float e;
};

struct GPS_time
{
  uint32_t gps_time;
  uint16_t gps_week;
  uint32_t gps_tow_ms; 
};

// Declair WGS84 constant
WGS84 wgs84 = {6378137.0, 0.0818191};
LLA lla_1;
ECEF ecef_1;
GPS_time gps_time;

String GPGGA = "$GPGGA,105146.14,3312.917887,N,08732.613607,W,1,08,1.2,2.2,M,0.0,M,,,*12";
String GPZDA = "$GPZDA,105146.14,31,07,2022,00,00";
String input = GPZDA;

void setup()
{
  Serial.begin(115200);
  while(!Serial);
  String header = input.substring(1,6);
  if (header == "GPZDA")
  {
    proc_gpzda(&input, &tm, &sec_dec);
    posix = makeTime(tm);
    posix2gps(&posix, &gps_time, &sec_dec);
    Serial.println(gps_time.gps_time);
    Serial.println(gps_time.gps_week);
    Serial.println(gps_time.gps_tow_ms);
  }
  else if (header == "GPGGA")
  {
    proc_gpgga(&input, &lla_1);
    lla2ecef(&lla_1, &ecef_1, &wgs84);
    Serial.println (lla_1.lat_deg,6);
    Serial.println (lla_1.lon_deg,6);
    Serial.println (lla_1.h_m,6);
    Serial.println (ecef_1.x_m,6);
    Serial.println (ecef_1.y_m,6);
    Serial.println (ecef_1.z_m,6);
  }

  
}

void loop()
{
  
}

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

void posix2gps(time_t *posix, GPS_time *gps_time, int32_t *sec_dec)
{
  gps_time->gps_time = *posix - 315964800 + 18;
  gps_time->gps_week = int(gps_time->gps_time / 604800);
  uint32_t tow_s = gps_time->gps_time - gps_time->gps_week * 604800;
  gps_time->gps_tow_ms = tow_s * 1000 + *sec_dec * 10;
}
