# My_git
image_processing
// Arduino and Visual Basic: Receiving Data From the Arduino
// A simple example of receiving data from an Arduino
//
// Sends the string "1234" out over serial every second
 
byte LEDpin = 13;
 
void setup() 
{
     pinMode(LEDpin, OUTPUT); 
     Serial.begin(9600);
}
 
void loop() 
{
     Serial.println("1234");
     digitalWrite(LEDpin,HIGH);
     delay(100);
     digitalWrite(LEDpin,LOW);
     delay(900);
}     
 
