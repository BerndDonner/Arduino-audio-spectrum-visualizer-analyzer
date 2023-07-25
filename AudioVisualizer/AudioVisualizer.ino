/*
Copyright (c) 2019 Shajeeb TM, 2023 Bernd Donner

https://github.com/shajeebtm/Arduino-audio-spectrum-visualizer-analyzer/
https://create.arduino.cc/projecthub/Shajeeb/32-band-audio-spectrum-visualizer-analyzer-902f51

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <MD_MAX72xx.h>
#include <SPI.h>
#include <assert.h>
#include "fix_fft.h"


#define FFT_N          128                    // set to 64 point fft
#define LOG2_FFT_N     7                      // log2(N_WAVE)
#define twoPi          6.28318531
#define HARDWARE_TYPE  MD_MAX72XX::FC16_HW    // Set display type  so that  MD_MAX72xx library treets it properly
#define MAX_DEVICES    4                      // Total number display modules
#define CLK_PIN        13                     // Clock pin to communicate with display
#define DATA_PIN       11                     // Data pin to communicate with display
#define CS_PIN         10                     // Control pin to communicate with display
#define xres           32                     // Total number of  columns in the display
#define yres           8                      // Total number of  rows in the display
#undef DEBUG


//int MY_MODE_1[] = {1, 3, 7, 15, 31, 63, 127, 255, 255}; // standard pattern
int MY_MODE_1[] = {0, 128, 192, 224, 240, 248, 252, 254, 255}; // standard pattern
int MY_MODE_2[] = {0, 128, 64,  32,  16,  8,   4,   2,   1};   // only peak pattern
int MY_MODE_3[] = {0, 128, 192, 160, 144, 136, 132, 130, 129}; // only peak +  bottom point
int MY_MODE_4[] = {0, 128, 192, 160, 208, 232, 244, 250, 253}; // one gap in the top , 3rd light onwards
int MY_MODE_5[] = {0b00000000, 0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111, 0b01111111, 0b11111111}; // standard pattern, mirrored vertically

int* MY_ARRAY = MY_MODE_1;
 

int yvalue;
int displaycolumn , displayvalue;
const int buttonPin = 5;    // the number of the pushbutton pin
int state = HIGH;           // the current reading from the input pin
int previousState = LOW;    // the previous reading from the input pin
int displaymode = 1;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);   // display object


void setup()
{
  ADCSRA = 0b11100100;      // set ADC to free running mode and set pre-scalar to 32 (0xe5)
  ADMUX =  0b00000000;       // use pin A0 and external voltage reference
  pinMode(buttonPin, INPUT);
  mx.begin();           // initialize display
  delay(50);            // wait to get reference voltage stabilized

  #ifdef DEBUG
    Serial.begin(9600);
  #endif
}


void loop()
{
  uint8_t data_avgs[xres];
  static int peaks[xres];
  int16_t fr[FFT_N], fi[FFT_N];

  #ifdef DEBUG
    Serial.println("Sampling Started!");
  #endif
   
  // ++ Sampling
  for(uint16_t i=0; i < FFT_N; i++)
  {
    uint16_t value = 0;
    for (uint8_t k = 0; k < 16; ++k)    //oversampling by 2
    {
      while(!(ADCSRA & 0x10));        // wait for ADC to complete current conversion ie ADIF bit set
      ADCSRA = 0b11110100 ;           // clear ADIF bit so that ADC can do next operation (0xf5)
      value += ADC;
    }
    value /= 16*8;                    // average for oversampling and remove lowest 3 bits 
    fr[i] = (int16_t)(value) - 64;    // remove offset and copy to bins after compressing
    fi[i] = 0;                         
  }
  // -- Sampling

  #ifdef DEBUG
    Serial.println("Hamming Window multiplication started!");
  #endif

  // BEGIN ----- multiply with hamming window
  for (uint8_t i = 0; i < (FFT_N >> 1); i++)
  {
    double ratio = ((double) i / (double)(FFT_N - 1));
    double weighingFactor = (0.54 - (0.46 * cos(twoPi * ratio)))*16;

    fr[i] = ((double)fr[i]) * weighingFactor;
    fr[FFT_N - (i + 1)] = ((double)fr[FFT_N - (i + 1)]) * weighingFactor;
  }
  // END ----- multiply with hamming window

  #ifdef DEBUG
    Serial.println("FFT Started!");
  #endif

  // ++ FFT
  fix_fft(fr, fi, LOG2_FFT_N, 0);
  // -- FFT

  #ifdef DEBUG
    Serial.println("FFT Finisched!");
  #endif

    
  // ++ re-arrange FFT result to match with no. of columns on display ( xres )
  uint8_t step = (FFT_N >> 1)/xres; 
  for(uint8_t i = 0; i < xres; i++)  
  {
    data_avgs[i] = 0;
    for (uint8_t k = 0; k < step; k++)
    {
      float mag2 = (int32_t)fr[step*i + k] * fr[step*i + k] + (int32_t)fi[step*i + k] * fi[step*i + k];  // fr, fi should be between +-64
      data_avgs[i] += sqrt(mag2);
    }
  }
  // -- re-arrange FFT result to match with no. of columns on display ( xres )

    
  // ++ send to display according measured value 
  for(uint8_t i = 0; i < xres; i++)
  {
    data_avgs[i] = constrain(data_avgs[i], 0, step*64);            // set max & min values for buckets
    data_avgs[i] = map(data_avgs[i], 0, step*64, 0, yres);         // remap averaged values to yres
    yvalue=data_avgs[i];

    peaks[i] = peaks[i]-1;    // decay by one light
    if (yvalue > peaks[i])    //peaks[i] = max(peaks[i], yvalue);
      peaks[i] = yvalue ;
    yvalue = peaks[i];        //yvalue   = max(peaks[i], yvalue);
    displayvalue = MY_ARRAY[yvalue];
    displaycolumn = 31-i;
    mx.setColumn(displaycolumn, displayvalue);              // for left to right
  }
  // -- send to display according measured value 
     
  displayModeChange ();         // check if button pressed to change display mode
} 

void displayModeChange() {
  int reading = digitalRead(buttonPin);
  if (reading == HIGH && previousState == LOW && millis() - lastDebounceTime > debounceDelay) // works only when pressed
  {
    switch (displaymode)
    {
      case 1:    //       move from mode 1 to 2
        displaymode = 2;
        MY_ARRAY=MY_MODE_2;
        break;
      case 2:    //       move from mode 2 to 3
        displaymode = 3;
        MY_ARRAY=MY_MODE_3;
        break;
      case 3:    //     move from mode 3 to 4
        displaymode = 4;
        MY_ARRAY=MY_MODE_4;
        break;
      case 4:    //     move from mode 4 to 5
        displaymode = 5;
        MY_ARRAY=MY_MODE_5;
        break;
      case 5:    //      move from mode 5 to 1
        displaymode = 1;      
        MY_ARRAY=MY_MODE_1;
        break;
    }
    lastDebounceTime = millis();
  }
  previousState = reading;
}
