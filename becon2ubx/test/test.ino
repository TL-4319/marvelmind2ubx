void setup()
{
  Serial.begin(115200);
  Serial1.begin(19200);
}

void loop()
{
  while (Serial1.available() > 0)
  {
    Serial.write(Serial1.read());  
  }
}
