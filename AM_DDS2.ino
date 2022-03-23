/*=====================| Used libraries: |========================
 * 
 * MD_AD9833      https://github.com/MajicDesigns/MD_AD9833
 * u8g2 (display) https://github.com/olikraus/u8g2
 * Arduino core   https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Arduino.h
 * SPI.h          https://github.com/codebendercc/arduino-library-files/blob/master/libraries/SPI/SPI.h
 * EEPROM.h       https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/EEPROM/src/EEPROM.h
 * 
 */


/*=====================| AM Generator by SP6GK, v2|========================
 * 
 *  This code is for AM generator based on AD9833 (DDS) as an oscillator,
 * with 4 button interface and 0.96" 128x64 display
 * 
 * This is an open source and hardware project, feel free to redistribute 
 * and/or modifiy this code under the terms of the GNU Lesser General Public 
 * License. This code is provided with no warranty of any kind.
 * 
 * Main goal is to create simple and frequency stable aparature to tune
 * and repair vintage radios. This generator can also be used for education,
 *  experimentation and playing music over AM radios.
 * On board switch allows to output just a sine wave.
 * Internal timer is used to obtain 1KHz modulation or external 3.5mm jack
 * can be used.
 * 
 * 
 * Instructions:
 * 1. Setting the frequency: use up and down
 * 
 * 2. Setting the step (how much the frequency is incremented/ decremented)
 * ,pressing the step button changes step from the list always increasing, 
 * list rollovers when maximum step is reached
 * 
 * 3. To use preprogrammed frequency: press memory button, use up and down
 * to select channel, double press the step to select the channel, if you
 * want to return withouth the change, press memory once more and then step
 * 
 * On each change of frequency the value is written to EEPROM so that
 * on power cycle the device will return to previous frequency.
 * This wears out the EEPROM (about 100k cycles).
 * Some alghorithm may be implemented in the future. 
 * (ex idea, on each power up change addres and store number of power cycles
 * in order to retrive the last used address)
 * 
 * 
 * SP6GK, 02.03.2022
 *=========================================================================
*/


#include <MD_AD9833.h>
#include <Arduino.h>
#include <U8x8lib.h>
#include <SPI.h>
#include <EEPROM.h>

#define DATA 11
#define CLK 13
#define CS 10

U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(/* cs=*/ 5, /* dc=*/ 9, /* reset=*/ 8);
MD_AD9833 AD(CS);

const uint8_t button_up = 18;
const uint8_t button_dwn = 16;
const uint8_t button_stp = 17;
const uint8_t button_mem = 19;

const uint8_t G_LED = 15;  //output  LED_3 pin 24
const uint8_t R_LED = 14;  //command LED_2 pin 23
const uint8_t B_LED = 2;   //step    LED_1 pin 22

uint8_t button_up_state = 0;
uint8_t button_dwn_state = 0;
uint8_t button_stp_state = 0;
uint8_t button_mem_state = 0;

long frequency=990000;
long temp_frequency;
unsigned long f_step_arr[4] = {1000, 10000, 100000, 1000000};
unsigned long f_step;
uint8_t step_n = 0;
int memo_n = 0;
bool mem_mode = false;

uint8_t addr = 20;
uint8_t mem_addr[4] = {0, 4, 8, 12};
uint8_t digits[4];

bool start = false;

void setup() {
  EEPROM.put(mem_addr[0], 455000);
  EEPROM.put(mem_addr[1], 550000);
  EEPROM.put(mem_addr[2], 1550000);
  EEPROM.put(mem_addr[3], 7000000);

  AD.begin();
  u8x8.begin();
  u8x8.clear();
  u8x8.setFlipMode(1);
  u8x8.setInverseFont(0);

  u8x8.setFont(u8x8_font_chroma48medium8_r); 
  u8x8.setCursor(0,16);
  u8x8.print("by SP6GK, v2.0");
  delay(2000);
  u8x8.clear();

    step_n = 1;
    f_step = f_step_arr[step_n];
    //pre(frequency);
    start = true;
    mem_mode = false;
    EEPROM.get(addr, frequency);  //if it shuts down we loose last addres, so write to 0 always
    AD.setFrequency(MD_AD9833::CHAN_0, frequency);
  
  
  pre(frequency, f_step, memo_n);
  delay(100);

  DDRD |= (1<<PD3);
  TCCR2A |= (1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20);
  TCCR2B = (1<<CS22);
  OCR2B = 128;    //PIN 1 OC2
  

  pinMode(button_up, INPUT);
  pinMode(button_dwn, INPUT);
  pinMode(button_stp, INPUT);
  pinMode(button_mem, INPUT);
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  delay(100);
  Serial.begin(9600);
}

void pre(unsigned long freq, unsigned long f_step, uint8_t memo_n)
{
  u8x8.clear();
  unsigned long freq_KHz = freq/1000;
  unsigned long freq_MHz = freq_KHz/1000;

  u8x8.setFont(u8x8_font_chroma48medium8_r); 
  u8x8.setCursor(0,16);
  u8x8.print("AM GENERATOR ~");

  digits[4] = freq_KHz % 10;        //1KHz
  digits[3] = (freq_KHz/10) % 10;   //10KHz
  digits[2] = (freq_KHz/100) %10;   //100KHz
  digits[1] = (freq_KHz/1000)%10;   //1MHz
  digits[0] = (freq_KHz/10000)%10;  //10MHz
  Serial.println("Frequency: " + String(freq_KHz) + "[KHz]");

  if((digits[1] <1) and (digits[0] == 0)){     //print format below 1MHz
    u8x8.setFont(u8x8_font_px437wyse700b_2x2_r); 
    u8x8.setCursor(0,1);
    u8x8.print(String(digits[2])+String(digits[3])+String(digits[4])+".00");
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_f); 
    u8x8.setCursor(13,2);
    u8x8.print("KHz");
  }
  else if((digits[1] >= 1) or (digits[0] != 0)){ //Print format above 1MHz
    u8x8.setFont(u8x8_font_px437wyse700b_2x2_r); 
    u8x8.setCursor(0,1); 
    u8x8.print(String(digits[0])+String(digits[1])+String(".")+String(digits[2])+String(digits[3])+String(digits[4])); 
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_f); 
    u8x8.setCursor(13,2);
    u8x8.print("MHz");
  }


    u8x8.setFont(u8x8_font_chroma48medium8_r); 
    u8x8.setCursor(0,4);
    u8x8.print("Step: "+String(f_step/1000)+" KHz");
    
    if(mem_mode == true){
    u8x8.setCursor(0,6);  
    u8x8.print("CHN: "+String(memo_n));
    }
    
}

int get_addr(){
  addr = addr + sizeof(float);
  if(addr == EEPROM.length()){
    addr = 20;
  }
 return addr;
}

void loop() {
  if(start == false){
    Serial.println("Booted up!  Step ld: "+String(f_step)+" Frequency ld: "+String(frequency)+" Ver.2.0 by SP6GK");
    //read frequency and step from EEPROM
    
  }
  
  button_up_state = digitalRead(button_up);
  button_dwn_state = digitalRead(button_dwn);
  button_stp_state = digitalRead(button_stp);
  button_mem_state = digitalRead(button_mem);
  delay(100);


  if(button_up_state == LOW){
    frequency = frequency + f_step;
    if(frequency >= 12000000){
      frequency = 12000000;
    }
    pre(frequency, f_step, memo_n);
    AD.setFrequency(MD_AD9833::CHAN_0, frequency);
    digitalWrite(R_LED, HIGH);
    delay(300);
    addr = get_addr();
    addr = 20;
    EEPROM.put(addr, frequency);
    digitalWrite(R_LED,LOW);
  }
  
  else  if(button_dwn_state == LOW){
    frequency = frequency - f_step;
    if(frequency <= 10){
      frequency = 10;
    }
    pre(frequency, f_step, memo_n);
    AD.setFrequency(MD_AD9833::CHAN_0, frequency);
    digitalWrite(R_LED, HIGH);
    delay(300);
    addr = get_addr();
    addr = 20;
    EEPROM.put(addr, frequency);
    digitalWrite(R_LED,LOW);
  }
  
  else if(button_stp_state == LOW){
    if(step_n > 3){
      step_n = 0;
      pre(frequency, f_step, memo_n);
    }
    f_step = f_step_arr[step_n];
    step_n ++;
    pre(frequency, f_step, memo_n);
    //display step on screen
    
    Serial.println("Step [KHz]: "+String(step_n)+" : "+String(f_step));
    digitalWrite(B_LED, HIGH);
    delay(300);
    digitalWrite(B_LED,LOW);
  }

  else if(button_mem_state == LOW){
    //memmory mode
    mem_mode = true;
    pre(frequency, f_step, memo_n);

    while(mem_mode == true){

       button_up_state = digitalRead(button_up);
       button_dwn_state = digitalRead(button_dwn);
       button_stp_state = digitalRead(button_stp);
       button_mem_state = digitalRead(button_mem);
      
      if(button_up_state == LOW){ //Go up a channel
        delay(300);
        memo_n ++;
        if(memo_n > 2){
          memo_n = 3;
        }
        EEPROM.get(mem_addr[memo_n], temp_frequency);
        pre(temp_frequency, f_step, memo_n); //display frequency stored in channel
      }
      
      if(button_dwn_state == LOW){  //Go down a channel
        delay(300);
        memo_n --;
        if(memo_n < 0){
          memo_n = 0;
        }
        EEPROM.get(mem_addr[memo_n], temp_frequency);
        pre(temp_frequency, f_step, memo_n); //display frequency stored in channel
      }

       if(button_stp_state == LOW){ //Select the channel
          frequency = temp_frequency;
          pre(frequency, f_step, memo_n);
          AD.setFrequency(MD_AD9833::CHAN_0, frequency);
          digitalWrite(R_LED, HIGH);
          delay(300);
          digitalWrite(R_LED, LOW);
          mem_mode = false;
        }

      if(button_stp_state == LOW){ //Leave the memory mode
        mem_mode = false;
        delay(200);
      }
      
    }
    
    
  }

}
