const uint8_t HEDGEHOG_HEADER = 0xFF;
const uint8_t PACKET_TYPE = 0x47;
const uint8_t DATA_CODE_LSB = 0x00;
const uint8_t DATA_CODE_MSB = 0x11;
const uint8_t HEDGEHOG_POS_LEN = 0x16

uint8_t byte_buf[40];
uint8_t incoming_byte;
bool good_byte;

uint8_t msg_pos;

typedef union { uint8_t u1[2]; uint16_t u2; int16_t i2;} union_16;
typedef union { uint8_t u1[4]; uint32_t u4; int32_t i4;} union_32;

void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);
  msg_pos = 0;

}

void loop() {
  union_16 un16;
  union_32 un32;
  
  while(Serial1.available() > 0)
  {
    incoming_byte = Serial1.read();
    switch(msg_pos)
    {
      case 0:
      {
        good_byte = (incoming_byte == HEDGEHOG_HEADER);
        break;
      }
      case 1:
      {
        good_byte = (incoming_byte == PACKET_TYPE);
        break;
      }
      case 2:
      {
        good_byte = (incoming_byte == DATA_CODE_MSB);
        break;
      }
      case 3:
      {
        good_byte = (incoming_byte == DATA_CODE_LSB);
        break;
      }
      case 4:
      {
        good_byte = (incoming_byte == HEDGEHOG_POS_LEN);
        break;
      }
      default:
      {
        good_byte = true;
        break;
      }
    }

    if (!good_byte)
    {
      msg_pos = 0;
      continue;
    }

    
  }

}
