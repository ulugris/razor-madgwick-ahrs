#define RAZOR 4     // Razor ID number (run first with WRITE=false to get the ID)
#define WRITE true // Actually write data to EEPROM

#include "Calibration.h"

#include <EEPROM.h>

byte ID;

void setup() {
  EEPROM.get(0, ID); // ID number

  Serial.begin(57600);

  Serial.print("Razor #");
  Serial.println(ID);

  Serial.print("Factory OSCCAL: 0x");
  Serial.println(OSCCAL, HEX);
  Serial.println();

  if (WRITE) {
    ID = RAZOR;
    int pos = 0;
    EEPROM.put(pos, ID);
    Serial.print("Writing ID: ");
    Serial.println(ID);
    Serial.println();    
    pos += sizeof(ID);

    EEPROM.put(pos, Ka);
    Serial.print("Writing Ka at 0x0");
    Serial.println(pos, HEX);
    pos += sizeof(Ka);

    EEPROM.put(pos, a0);
    Serial.print("Writing a0 at 0x");
    Serial.println(pos, HEX);
    Serial.println();
    pos += sizeof(a0);

    EEPROM.put(pos, Km);
    Serial.print("Writing Km at 0x");
    Serial.println(pos, HEX);
    pos += sizeof(Km);

    EEPROM.put(pos, m0);
    Serial.print("Writing m0 at 0x");
    Serial.println(pos, HEX);
    Serial.println();
    pos += sizeof(m0);

    EEPROM.put(pos, Kg);
    Serial.print("Writing Kg at 0x");
    Serial.println(pos, HEX);
    pos += sizeof(Kg);

    EEPROM.put(pos, g0);
    Serial.print("Writing g0 at 0x");
    Serial.println(pos, HEX);
  }
}

void loop() {}
