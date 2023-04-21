void setup()
{
  Serial.begin(115200);
}

void loop()
{
  // delay(1000);
}

void serialEvent()
{
  if (Serial.available())
  {
    String s = Serial.readString();
    if (s.length() > 0)
    {
      Serial.println("Hello, I received:");
      Serial.println(s);
    }
  }
}
