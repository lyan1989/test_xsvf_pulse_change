#include <Arduino.h>

/*
  The JTAG Whisperer: An Arduino library for JTAG.

  By Mike Tsao <http://github.com/sowbug>.

  Copyright © 2012 Mike Tsao. Use, modification, and distribution are
  subject to the BSD-style license as described in the accompanying
  LICENSE file.

  See README for complete attributions.


  Serial.println(pgm_read_word_far(pgm_get_far_address(data2)+k),HEX);
      Serial.println(pgm_read_word_far(pgm_get_far_address(data3)+k),HEX);*/



#include <Arduino.h>
#include "xsvf_code.h"
#include "parse.h"
#include "JTAGPortArduino.h"


const int BLINK_PIN = 6;  //teensy++2.0 use the LED in PIN6
static bool is_pin_on;
bool runtest = false;
uint8_t *bp_;
uint32_t instruction_count=0;

uint8_t i=0;
uint8_t result1,result2,result3;
bool m_program = false;
bool analog_right = false;

const PROGMEM uint8_t  data[100]={0x07,0x00,0x13,0x00,0x14,0x00,0x12,0x00,0x12,0x01,0x02,0x08,0x01,0x08,0x00,0x00,
  0x00,0x20,0x01,0x0F,0xFF,0x8F,0xFF};



void blink() {
  digitalWrite(BLINK_PIN, is_pin_on);
  is_pin_on = !is_pin_on;
}

void setup() {
  pinMode(BLINK_PIN, OUTPUT);
  Serial.begin(115200);
  delayMicroseconds(100);
  Serial.flush();
  parseInit();
}

bool program()
{
  // Wait for user input before proceeding
  // while (!Serial.available());
  // // a key was pressed1
  // while (Serial.available()) Serial.read();
  // fist data block
  result1 = parse((data1), 16525);
  //second data block
  result2 = parse((data2), 16279);

  // 3rd data block
  result3 = parse((data3), 10836);


  if(result1|result2|result3)
  {
    digitalWrite(6,HIGH);
    // Serial.println("program failed!");
    return false;
  }
  else
  {
    digitalWrite(6,LOW);
    // Serial.println("program successfully!");
    return true;
  }
}

bool test_id()
{
  // Wait for user input before proceeding
  while (!Serial.available());
  // a key was pressed1
  while (Serial.available()) Serial.read();
  // fist data block
    for(i=0;i<10;i++)
      Serial.print("3-");
      Serial.println();
    return  parse((data1), 2000);
}

void loop(){

    if( digitalRead(24) )
    {
      if(!m_program)
      {
        parseInit();
        program();

      }
      m_program = true;
    }
    else
      m_program = false;

    delayMicroseconds(100);

}
