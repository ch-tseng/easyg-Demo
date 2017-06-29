#include <SoftwareSerial.h>
#include <AD_005.h>

SoftwareSerial mySerial(7, 8); // RX, TX

void setup() {
  AD_005_init();

  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Software serial ready.");
} 


void loop() {
  byte gest, dist, prev_dist;

  gest = AD_005_get();
  if (gest == 0)
    return;
  Serial.println(gest, HEX);
  mySerial.write(gest);
  prev_dist = 0;
  while (gest & 0x40)
  {
    dist = AD_005_distance();
    if (prev_dist != dist)
    {
      Serial.println(dist);
      mySerial.write(dist);
      prev_dist = dist;
    }
    gest = AD_005_get(1);
  }
}  // loop()
