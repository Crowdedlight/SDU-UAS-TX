#include <ArduinoJson.h>

/***************************************************************************
# SDU UAS Center TX firmware 
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# This library is based on the source example Generate_PPM_signal_V0.2 obtained
# from https://code.google.com/archive/p/generate-ppm-signal/downloads
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
This firmware is developed for the v03-2018 version of the SDU UAS Transmitter

Revision
2018-05-01 KJ First released test version
****************************************************************************/
/* parameters */



/****************************************************************************/
/* input defines */
#define PIN_LEFT_X A5
#define PIN_LEFT_Y A4
#define PIN_RIGHT_X A3
#define PIN_RIGHT_Y A2
#define PIN_3_POS_SW_LEFT A7
#define PIN_3_POS_SW_RIGHT A6
#define PIN_LEFT_BUTTON 3
#define PIN_RIGHT_BUTTON 4
#define PIN_2_POS_SW_LEFT 7
#define PIN_2_POS_SW_RIGHT 8
#define PIN_POT A1

#define PIN_TX 6
#define PIN_AUDIO 11
#define PIN_BATT_VOLT A0
#define PIN_BUZZER 5
#define PIN_LED_RED 9
#define PIN_LED_GREEN 10

/* ppm defines */
#define ppm_number 8  //set the number of ppm chanels
#define analog_number 8  //set the number of ppm chanels
#define default_servo_value 1500  //set the default servo value
#define default_thrust_value 1150
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative

/****************************************************************************/
/* variables */
/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
long ppm[ppm_number];
long analog[analog_number];
short count;
boolean led_state;
boolean buzzer_state;



/****************************************************************************/
void setup()
{  
  // setup digital output pins
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(PIN_LED_RED, OUTPUT); 
  pinMode(PIN_LED_GREEN, OUTPUT); 
  pinMode(PIN_BUZZER, OUTPUT); 

  // enable pull-up resistor on digital input pins 
  digitalWrite (PIN_LEFT_BUTTON, HIGH);
  digitalWrite (PIN_RIGHT_BUTTON, HIGH);
  digitalWrite (PIN_2_POS_SW_LEFT, HIGH);
  digitalWrite (PIN_2_POS_SW_RIGHT, HIGH);

  Serial.begin(115200);

  //initiallize default ppm values
  for(int i=1; i<ppm_number; i++)
  {
    ppm[i]= default_servo_value;
  }
  ppm[0]=default_thrust_value;

  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_AUDIO, OUTPUT);
  digitalWrite(PIN_TX, !onState);  //set the PPM signal pin to the default state (off)
  digitalWrite(PIN_AUDIO, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}
/****************************************************************************/
ISR(TIMER1_COMPA_vect)
{
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state)
  {  //start pulse
    digitalWrite(PIN_TX, onState);
    digitalWrite(PIN_AUDIO, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else
  {  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(PIN_TX, !onState);
    digitalWrite(PIN_AUDIO, !onState);
    state = true;

    if(cur_chan_numb >= ppm_number)
    {
      digitalWrite(PIN_TX, !onState);
      digitalWrite(PIN_AUDIO, !onState);
       cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else
    {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
/****************************************************************************/
void loop()
{
  //put main code here
  static int val = 1;
  count ++;

  // read analog input
  analog[0] = analogRead(PIN_LEFT_Y);
  analog[1] = analogRead(PIN_LEFT_X);
  analog[2] = analogRead(PIN_RIGHT_Y);
  analog[3] = analogRead(PIN_RIGHT_X);
  analog[4] = analogRead(PIN_3_POS_SW_LEFT);
  analog[5] = analogRead(PIN_3_POS_SW_RIGHT);
  analog[6] = analogRead(PIN_POT);
  analog[7] = analogRead(PIN_BATT_VOLT);

  int left = digitalRead(PIN_2_POS_SW_LEFT);
  int right = digitalRead(PIN_2_POS_SW_RIGHT);

  if(analog[7] < 360)
  {
   tone(PIN_BUZZER,1000);
  }
  else if(Serial.available() && right)
  {
    // setup JSON
    StaticJsonBuffer<100> jsonBuffer;
    //String json = Serial.readStringUntil("\n");
  
    // Root of the object tree.
    //Serial.println(json);
    //
    // It's a reference to the JsonObject, the actual bytes are inside the
    // JsonBuffer with all the other nodes of the object tree.
    // Memory is freed when jsonBuffer goes out of scope.
    JsonObject& root = jsonBuffer.parseObject(Serial);
    // Test if parsing succeeds.
    if (!root.success()) {
      Serial.println("parseObject() failed");
      return;
    }

    
    // Fetch values.
    //
    // Most of the time, you can rely on the implicit casts.
    // In other case, you can do root["time"].as<long>();
    double thrust = root["t"];
    double roll = root["r"];
    double pitch = root["p"];
    double yaw = root["y"];
  
    ppm[0] = thrust;
    ppm[1] = roll;
    ppm[2] = pitch;
    ppm[3] = yaw;

    Serial.print (ppm[0]);
    Serial.print (" ");
    Serial.print (ppm[1]);
    Serial.print (" ");
    Serial.print (ppm[2]);
    Serial.print (" ");
    Serial.print (ppm[3]);
    Serial.print (" ");
    Serial.print (ppm[4]);
    Serial.print (" ");
    Serial.print (ppm[5]);
    Serial.print (" ");
    Serial.print (ppm[6]);
    Serial.print (" ");                       
    Serial.println (ppm[7]);
  }
  else if(!right)
  {
    // Use the potentiometer value to restrict the range of the output for pitch and roll
    int outputRange = 700*analog[6]/1023;
    int offset = 1500-outputRange/2;
    ppm[0] = (analog[0]-65)*700/910 + 1150; // throttle
    ppm[1] = (950-(analog[3]-60))*outputRange/950 + offset; // roll (aileron)
    ppm[2] = (analog[2]-65)*outputRange/910 + offset; // pitch (elevator)
    ppm[3] = (955-(analog[1]-60))*700/955 + 1150; // yaw (rudder)
    /*
    Serial.print (analog[0]);
    Serial.print (" ");
    Serial.print (analog[1]);
    Serial.print (" ");
    Serial.print (analog[2]);
    Serial.print (" ");
    Serial.print (analog[3]);
    Serial.print (" ");
    Serial.print (analog[4]);
    Serial.print (" ");
    Serial.print (analog[5]);
    Serial.print (" ");
    Serial.print (analog[6]);
    Serial.print (" ");
    Serial.print (analog[7]);
    Serial.print (" ");
    Serial.print (digitalRead(PIN_LEFT_BUTTON));
    Serial.print (" ");
    Serial.print (digitalRead(PIN_RIGHT_BUTTON));
    Serial.print (" ");
    Serial.print (digitalRead(PIN_2_POS_SW_LEFT));
    Serial.print (" ");
    Serial.println (digitalRead(PIN_2_POS_SW_RIGHT)); 
    */
  }

    Serial.print (ppm[0]);
    Serial.print (" ");
    Serial.print (ppm[1]);
    Serial.print (" ");
    Serial.print (ppm[2]);
    Serial.print (" ");
    Serial.print (ppm[3]);
    Serial.print (" ");
    Serial.print (ppm[4]);
    Serial.print (" ");
    Serial.print (ppm[5]);
    Serial.print (" ");
    Serial.print (ppm[6]);
    Serial.print (" ");                       
    Serial.println (ppm[7]);

  // update LED
  if (count % 200 == 0)
    digitalWrite(PIN_LED_GREEN, HIGH);
  else
    digitalWrite(PIN_LED_GREEN, LOW);



  // handle special case of arming AutoQuad
  if (analog[0] < 450 && analog[1] > 1000)
  {
    ppm[0] = 1150;
    ppm[3] = 1850;  
  }

  // handle left 3-way switch
  if (analog[4] < 300)
    ppm[4] = 1150;
  else if (analog[4] < 700)
    ppm[4] = 1500;
  else
    ppm[4] = 1850;
  
  // handle right 3-way switch
  if (analog[5] < 300)
    ppm[5] = 1150;
  else if (analog[5] < 700)
    ppm[5] =  1500;
  else
    ppm[5] = 1850; 

  // unused for now 
  ppm[6] = default_servo_value;
  ppm[7] = default_servo_value;
}
/****************************************************************************/

