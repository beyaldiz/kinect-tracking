#include <Servo.h>
char inp[6];
Servo servo1,servo2;

void setup() 
{
  Serial.begin(9600);
  servo1.attach(9);
  servo2.attach(11);
  servo1.write(50);
  servo2.write(20);
}


void loop()
{
  if(Serial.available())
  {
      Serial.readBytes(inp,6);
      int i, val1 = 0, val2 = 0;
      for(i = 0; i <= 2; i++)
      {
        val1 *= 10;
        val1 += inp[i] - '0';
      }
      for(i = 3; i <= 5; i++)
      {
        val2 *= 10;
        val2 += inp[i] - '0';
      } 
      if(val1 >= 1 && val1 <= 170)
        servo1.write(top1);
      if(val2 >= 1 && val2 <= 170)
        servo2.write(top2);
      Serial.flush();
  }
}