
#include <math.h>
const int B = 4275000;  // B value of the thermistor
const int R0 = 100000;  // RO = 100k
const int pinTempSensor = A0;// B value of the thermistor

#if defined (ARDUINO_ARCH_AVR)
#define debug Serial
#elif defined (ARDUINO_ARCH_SAMD) || defined (ARDUINO_ARCH_SAM)
#define debug SerialUSB
#else
#define debug Serial
#endif


void setup()
{
Serial.begin(9600);
}
void loop()
{
int a = analogRead(pinTempSensor);
float R = 1023.0/a-1.0;
R = R0*R;
float temperature = 1.0/(log(R/R0)/B+1/298.15)-273.15; // convert to temperature via datasheet
Serial.print("temperature - ");
Serial.println(temperature);
delay(100);
}
