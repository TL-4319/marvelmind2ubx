#include<TimeLib.h>

class Iir {
  public:
    void Init(float cutoff_hz, float samp_hz, float initial_val){
      float fc = cutoff_hz / samp_hz;
      float c_twoPI_fc = cos(2.0 * PI * fc);
      b_ = 2.0 - c_twoPI_fc - sqrt(pow(2.0 - c_twoPI_fc, 2.0) - 1.0);
      a_ = 1.0 - b_;
      prev_output_ = initial_val;
    }
    float Filter (float val){
      float ret;
      ret = a_ * val + b_ * prev_output_;
      prev_output_return_ = prev_output_;
      prev_output_ = ret;
      return ret;
    }
    float prev_val(){
      return prev_output_return_;
    }
  private:
    float a_, b_, prev_output_, prev_output_return_;
};

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

// UBX_NAV_TIMEGPS Constants
const uint8_t UBX_NAV_TIMEGPS_ID = 0x20;
const uint8_t UBX_NAV_TIMEGPS_LEN[2] = {0x10, 0x00};
const uint8_t TIMEGPS_LEN = 16;

// Message buffer
uint8_t msg[100];

// Checksum buffer
uint8_t checksum[2];

// UBX_NAV_PVT Constants
const uint8_t UBX_NAV_PVT_ID = 0x07;
const uint8_t UBX_NAV_PVT_LEN[2] = {0x5C, 0x00}; //Little endian (92 bytes)
const uint8_t PVT_LEN = 96;

// Marvelmind constants
const uint8_t HEDGEHOG_HEADER = 0xFF;
const uint8_t PACKET_TYPE = 0x47;
const uint8_t DATA_CODE_LSB = 0x00;
const uint8_t DATA_CODE_MSB = 0x81; //only process mm position with timestamps
const uint8_t HEDGEHOG_DATA_LEN = 0x1A;

// Define marvelmind buffer and populate the headers
uint8_t byte_buf[33];
uint8_t incoming_byte;

int64_t prev_timestamp_ms, start_time_ms, cur_time_ms;
int32_t prev_x_pos_mm, prev_y_pos_mm, prev_z_pos_mm;
Iir xpos_filter, ypos_filter, zpos_filter, xvel_filter, yvel_filter, zvel_filter;

uint8_t msg_pos, k; 
float _dt_s;
bool filter_init = false;
float filtered_x_pos, filtered_y_pos, filtered_z_pos;

// Declare unions
typedef union { uint8_t u1[2]; uint16_t u2; int16_t i2;} union_16;
typedef union { uint8_t u1[4]; uint32_t u4; int32_t i4;} union_32;
typedef union { uint8_t u1[8]; int64_t i8;} union_64;

union_16 year_short, gps_week;
union_32 x_pos_mm, y_pos_mm, z_pos_mm, vel_x_mmps, vel_y_mmps, vel_z_mmps, gps_tow_ms, lat_deg, lon_deg, gspeed, gcourse;
union_64 timestamp;

time_t posix;

void setup() {
  //Serial.begin(57600);
  Serial.begin(115200);
  msg_pos = 0;
  byte_buf[0] = HEDGEHOG_HEADER;
  byte_buf[1] = PACKET_TYPE;
  byte_buf[2] = DATA_CODE_MSB;
  byte_buf[3] = DATA_CODE_LSB;
  byte_buf[4] = HEDGEHOG_DATA_LEN;
  prev_timestamp_ms = 0;
  prev_x_pos_mm = 0;
  prev_y_pos_mm = 0;
  prev_z_pos_mm = 0;
  start_time_ms = 0;
}

void loop() {
  while(Serial.available() > 0)
  {
    incoming_byte = Serial.read();
    switch(msg_pos)
    {
      case 0:
      {
        if (incoming_byte == HEDGEHOG_HEADER)
        {
          msg_pos ++;
        }
        else
        {
          msg_pos = 0;
        }
        break;
      }
      
      case 1:
      {
        if (incoming_byte == PACKET_TYPE)
        {
          msg_pos++;
        }
        else
        {
          msg_pos = 0;
        }
        break;
      }
      
      case 2:
      {
        if (incoming_byte == DATA_CODE_MSB)
        {
          msg_pos++;
        }
        else
        {
          msg_pos = 0;
        }
        break;
      }
      
      case 3:
      {
        if (incoming_byte == DATA_CODE_LSB)
        {
          msg_pos++;
        }
        else
        {
          msg_pos = 0;
        }
        break;
      }
      
      case 4:
      {
        if (incoming_byte == HEDGEHOG_DATA_LEN)
        {
          msg_pos = 0;
          delay(1);
          // Get data into buffer
          for (uint8_t i = 5; i < 33; i++)
          {
            byte_buf[i] = Serial.read();
            delay(1);
          }

          // Parse data from buffer
          for (uint8_t i = 0; i < 4; i++)
          {
            timestamp.u1[i] = byte_buf [5 + i];
            timestamp.u1[4 + i] = byte_buf [9 + i];
            x_pos_mm.u1[i] = byte_buf [13 + i];
            y_pos_mm.u1[i] = byte_buf [17 + i];
            z_pos_mm.u1[i] = byte_buf [21 + i];
          }
          /* Begin processing data */
          if (!filter_init){
            // Cut off freq of 2 Hz and sampling freq of 4 Hz
            xpos_filter.Init(1.5, 4.0, x_pos_mm.i4);
            ypos_filter.Init(1.5, 4.0, y_pos_mm.i4);
            zpos_filter.Init(1.5, 4.0, z_pos_mm.i4);
            xvel_filter.Init(1.5, 4.0, 0.0);
            yvel_filter.Init(1.5, 4.0, 0.0);
            zvel_filter.Init(1.5, 4.0, 0.0);
            filter_init = true;
            continue;
          }
          else {
            filtered_x_pos = xpos_filter.Filter(x_pos_mm.i4);
            filtered_y_pos = ypos_filter.Filter(y_pos_mm.i4);
            filtered_z_pos = zpos_filter.Filter(z_pos_mm.i4);
          }
          timestamp.i8 += 18001200;
          posix = timestamp.i8 / 1000;
          year_short.u2 = year(posix);
          
          _dt_s = 1000.0 / float(timestamp.i8 - prev_timestamp_ms);
          vel_x_mmps.i4 = int(float(filtered_x_pos - xpos_filter.prev_val()) * _dt_s) ;
          vel_x_mmps.i4 = xvel_filter.Filter(vel_x_mmps.i4);
          vel_y_mmps.i4 = -int(float(filtered_y_pos - ypos_filter.prev_val()) * _dt_s) ;
          vel_y_mmps.i4 = yvel_filter.Filter(vel_y_mmps.i4);
          vel_z_mmps.i4 = -int(float(filtered_z_pos - zpos_filter.prev_val()) * _dt_s) ;
          vel_z_mmps.i4 = zvel_filter.Filter(vel_z_mmps.i4);
          prev_timestamp_ms = timestamp.i8;

          // Generate mock GPS data
          posix2gps(&timestamp.i8, &gps_week.u2, &gps_tow_ms.u4);
          lat_deg.i4 = 332154770 + int(float(filtered_x_pos) * 0.09);
          lon_deg.i4 = -875436600 - int(float(filtered_y_pos) * 0.107);
          gspeed.u4 = int(sqrt((sq(vel_x_mmps.i4) + sq(vel_y_mmps.i4))));
          gcourse.u4 = (int(atan2(vel_x_mmps.i4, vel_y_mmps.i4) * 57.29578049 * 100000.0) + 360) % 360;

          send_dop();
          send_pvt();
          send_eoe();
          
        }
        else
        {
          msg_pos = 0;
        }
        break;
      }
      
      default:
      {
        msg_pos = 0;
      }
    }

    
  }
  
}

// Convert UNIX to GPS timestamps
void posix2gps(int64_t *posix, uint16_t *gps_week, uint32_t *gps_tow_ms)
{
  uint64_t gps_time = *posix - 315964800000 + 18000;
  *gps_week = gps_time / 604800000;
  *gps_tow_ms = gps_time % 604800000;
}

// Function to send UBX_NAV_EOE message
void send_eoe()
{
  Serial.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_EOE_ID;
  msg[2] = UBX_NAV_EOE_LEN[0];
  msg[3] = UBX_NAV_EOE_LEN[1];
  msg[4] = gps_tow_ms.u1[0];
  msg[5] = gps_tow_ms.u1[1];
  msg[6] = gps_tow_ms.u1[2];
  msg[7] = gps_tow_ms.u1[3];
  Serial.write(msg,EOE_LEN);
  calc_checksum(EOE_LEN);
  Serial.write(checksum,2);
}

// Function to send UBX_NAV_TIMEGPS message
void send_timegps()
{
  Serial.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_TIMEGPS_ID;
  msg[2] = UBX_NAV_TIMEGPS_LEN[0];
  msg[3] = UBX_NAV_TIMEGPS_LEN[1];
  msg[4] = gps_tow_ms.u1[0];
  msg[5] = gps_tow_ms.u1[1];
  msg[6] = gps_tow_ms.u1[2];
  msg[7] = gps_tow_ms.u1[3];
  msg[8] = 0x00;
  msg[9] = 0x00;
  msg[10] = 0x00;
  msg[11] = 0x00;
  msg[12] = gps_week.u1[0];
  msg[13] = gps_week.u1[1];
  msg[14] = 0x12;
  msg[15] = 0x07;
  msg[16] = 0x00;
  msg[17] = 0x00;
  msg[18] = 0x00;
  msg[19] = 0x00;
  Serial.write(msg,TIMEGPS_LEN);
  calc_checksum(TIMEGPS_LEN);
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
  msg[4] = gps_tow_ms.u1[0];
  msg[5] = gps_tow_ms.u1[1];
  msg[6] = gps_tow_ms.u1[2];
  msg[7] = gps_tow_ms.u1[3];
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

// Function to send UBX_NAV_PVT message
void send_pvt()
{
  Serial.write(UBX_HEADER,2);
  msg[0] = UBX_NAV_MSG_CLASS;
  msg[1] = UBX_NAV_PVT_ID;
  msg[2] = UBX_NAV_PVT_LEN[0];
  msg[3] = UBX_NAV_PVT_LEN[1];
  msg[4] = gps_tow_ms.u1[0];
  msg[5] = gps_tow_ms.u1[1];
  msg[6] = gps_tow_ms.u1[2];
  msg[7] = gps_tow_ms.u1[3];
  msg[8] = year_short.u1[0];
  msg[9] = year_short.u1[1];
  msg[10] = month(posix);
  msg[11] = day(posix);
  msg[12] = hour(posix);
  msg[13] = minute(posix);
  msg[14] = second(posix);
  // Emulate all data valid for bit field  11 (b00001111)
  msg[15] = 0x0F; 
  // Emulate time accuracy, locked at 256ns for now
  msg[16] = 0x00;
  msg[17] = 0x01;
  msg[18] = 0x00;
  msg[19] = 0x00;
  // Emulate nano second part of time, locked at 0ns for now
  msg[20] = 0x00;
  msg[21] = 0x00;
  msg[22] = 0x00;
  msg[23] = 0x00;
  // Fix type byte, assume 3d fix 
  msg[24] = 0x03;
  // Emulate flag for bit field 21 (00000001)
  msg[25] = 0x01;
  // Emulate flag for bit field 22 (11100000)
  msg[26] = 0xE5;
  // Number of SV - Marvelmind assumes 8 after 10 seconds of 0 SV
  msg[27] = 8;
  // Lat in deg times 10^6
  msg[28] = lon_deg.u1[0];
  msg[29] = lon_deg.u1[1];
  msg[30] = lon_deg.u1[2];
  msg[31] = lon_deg.u1[3];
  // Lon in deg times 10^6
  msg[32] = lat_deg.u1[0];
  msg[33] = lat_deg.u1[1];
  msg[34] = lat_deg.u1[2];
  msg[35] = lat_deg.u1[3];
  // Height above ellipsoid in mm
  msg[36] = z_pos_mm.u1[0];
  msg[37] = z_pos_mm.u1[1];
  msg[38] = z_pos_mm.u1[2];
  msg[39] = z_pos_mm.u1[3];
  // Height above MSL in mm the same as heigh above ellipsoid in this implementation
  msg[40] = z_pos_mm.u1[0];
  msg[41] = z_pos_mm.u1[1];
  msg[42] = z_pos_mm.u1[2];
  msg[43] = z_pos_mm.u1[2];
  // Horizontal accuracy, conservative estimate of 5cm
  msg[44] = 0xF0;
  msg[45] = 0x00;
  msg[46] = 0x00;
  msg[47] = 0x00;
  // Vertical accuracy, conservative estimate of 7cm
  msg[48] = 0xF0; 
  msg[49] = 0x00;
  msg[50] = 0x00;
  msg[51] = 0x00;
  // NED north velocity
  msg[52] = vel_x_mmps.u1[0];
  msg[53] = vel_x_mmps.u1[1];
  msg[54] = vel_x_mmps.u1[2];
  msg[55] = vel_x_mmps.u1[3];
  // NED east velocity
  msg[56] = vel_y_mmps.u1[0];
  msg[57] = vel_y_mmps.u1[1];
  msg[58] = vel_y_mmps.u1[2];
  msg[59] = vel_y_mmps.u1[3];
    // NED down velocity
  msg[60] = vel_z_mmps.u1[0];
  msg[61] = vel_z_mmps.u1[1];
  msg[62] = vel_z_mmps.u1[2];
  msg[63] = vel_z_mmps.u1[3];
  // G speed
  msg[64] = gspeed.u1[0];
  msg[65] = gspeed.u1[1];
  msg[66] = gspeed.u1[2];
  msg[67] = gspeed.u1[3];
  // heading of motion
  msg[68] = gcourse.u1[0];
  msg[69] = gcourse.u1[1];
  msg[70] = gcourse.u1[2];
  msg[71] = gcourse.u1[3];
  // Speed accuracy - constant 9mm/s
  msg[72] = 0x09;
  msg[73] = 0x00;
  msg[74] = 0x00;
  msg[75] = 0x00;
  // Heading accuracy
  msg[76] = 0x00;
  msg[77] = 0x00;
  msg[78] = 0x01;
  msg[79] = 0x00;
  // DOP
  msg[80] = 0x9C;
  msg[81] = 0x00;
  // Additional flag at 78 
  msg[82] = 0x00;
  
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
