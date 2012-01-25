//**************************************************************************
//*   Simple AVR wavetable synthesizer V1.0                                *
//*                                                                        *
//*   Implements 4 voices using selectable waveform and envelope tables    *
//*   Uses 8-bit PWM @ 62.5 kHz for audio output                           *
//*                                                                        *
//*   (C) DZL 2008                                                         *
//**************************************************************************

#define SET(x,y) (x |=(1<<y))		        //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))       	// |
#define CHK(x,y) (x & (1<<y))           	// |
#define TOG(x,y) (x^=(1<<y))            	//-+

#include "avr/interrupt.h"
#include "avr/pgmspace.h"
#include "sin256.h"
#include "ramp256.h"
#include "saw256.h"
#include "square256.h"
#include "noise256.h"
#include "tria256.h"
#include "env0.h"
#include "env1.h"
#include "env2.h"
#include "env3.h"



  float q; //process noise covariance
  float r; //measurement noise covariance
  float x; //value
  float p; //estimation error covariance
  float k; //kalman gain



void kalmanupdate(float measurement)
{
  //prediction update
  //omit x = x
  p = p + q;
   
  //measurement update
  k = p / (p + r);
  x = x + k * (measurement - x);
  p = (1.0f - k) * p;
}

void kalmaninit(float q, float r, float p, float intial_value)
{
 
  q = q;
  r = r;
  p = p;
  x = intial_value;
 
 
}


//*********************************************************************
//	Audio interrupt
//*********************************************************************

volatile unsigned int PCW[4]={
  0,0,0,0};				//-Wave phase accumolators
volatile unsigned int FTW[4]={
  1000,200,300,400};              	//-Wave frequency tuning words
volatile unsigned char AMP[4]={
  255,255,255,255};                //-Wave amplitudes [0-255]
volatile unsigned int PITCH[4]={
  500,500,500,500};               //-Voice pitch
volatile int MOD[4]={
  20,0,64,127};                             	//-Voice envelope modulation [0-1023 512=no mod. <512 pitch down >512 pitch up]
volatile unsigned int wavs[4];                                  //-Wave table selector [address of wave in memory]
volatile unsigned int envs[4];                                  //-Envelopte selector [address of envelope in memory]
volatile unsigned int EPCW[4]={
  0x8000,0x8000,0x8000,0x8000};    //-Envelope phase accumolator
volatile unsigned int EFTW[4]={
  10,10,10,10};                    //-Envelope speed tuning word
volatile unsigned int tim=0;                                    //-Sample counter eg. for sequencer
volatile unsigned char divider=4;                               //-Sample rate decimator for envelope

#define FS 16000.0                                              //-Sample rate

float cap_fast,cap_dif;
SIGNAL(TIMER1_COMPA_vect)
{
  OCR1A+=250;					                //-16kHz.

  //SET(DDRB,0); // test togler leg 8
  //TOG(PORTB,0);

  if(divider)
  {
    divider--;
  }
  else
  {

    //-------------------------------
    // Volume envelope generator
    //-------------------------------

    divider=4;

  }
  //-------------------------------
  //  Synthesizer/audio mixer
  //-------------------------------

  OCR0A=127+
    ((
  (((signed char)pgm_read_byte_near(wavs[0]+((PCW[0]+=FTW[0])>>8))*AMP[0])>>8)+
    (((signed char)pgm_read_byte_near(wavs[1]+((PCW[1]+=FTW[1])>>8))*AMP[1])>>8)+
    (((signed char)pgm_read_byte_near(wavs[2]+((PCW[2]+=FTW[2])>>8))*AMP[2])>>8)+
    (((signed char)pgm_read_byte_near(wavs[3]+((PCW[3]+=FTW[3])>>8))*AMP[3])>>8)
    )>>2);

  tim++;
}

//*********************************************************************
//  Setup all voice parameters
//*********************************************************************

void setup_voice(unsigned char voice,unsigned int waveform, float pitch, unsigned int envelope, float length, unsigned int mod)
{
  wavs[voice]=waveform;//[address in program memory]
  envs[voice]=envelope;//[address in program memory]
  EFTW[voice]=(1.0/length)/(FS/(32767.5*10.0));//[s];
  PITCH[voice]=pitch/(FS/65535.0); //[Hz]
  MOD[voice]=mod;//0-1023 512=no mod
}

//*********************************************************************
//  Midi trigger
//*********************************************************************

void mtrigger(unsigned char voice,unsigned char note)
{
  PITCH[voice]=(440. * exp(.057762265 * (note - 69.)))/(FS/65535.0); //[MIDI note]
  EPCW[voice]=0;
}

//*********************************************************************
//  Simple trigger
//*********************************************************************

void trigger(unsigned char voice)
{
  EPCW[voice]=0;
}

//*********************************************************************
//  Make Arduino happy
//*********************************************************************
void setup(void)

{

}
//*********************************************************************
//  Main
//*********************************************************************
boolean midiMode = false;

#define controlNum 10
int controlValues[controlNum];
void loop(void)
{
  setupPole();
  /*if(digitalRead(4)!=LOW)
  {
    midiMode = true;
    midiSetup();
  }*/

  if(!midiMode)
  {
    TCCR1B=0x02;                                    //-Start audio interrupt
    SET(TIMSK1,OCIE1A);                             // |
    sei();                                          //-+
    SET(DDRD,6);				          //-PWM pin

    TCCR0A=0x83;                                    //-8 bit audio PWM
    TCCR0B=0x01;                                    // |
    OCR0A=127;                                      //-+

    setup_voice(0,(unsigned int)SinTable,200.0,(unsigned int)Env1,1.0,1500);
    setup_voice(1,(unsigned int)SinTable,100.0,(unsigned int)Env1,1.0,1500);
    setup_voice(2,(unsigned int)TriangleTable,100.0,(unsigned int)Env2 ,.5,1500);
    setup_voice(3,(unsigned int)SinTable,1200.0,(unsigned int)Env3,.02,1500);
    Serial.begin(9600);
  }
  unsigned int counter=0;
  unsigned char bar;
  unsigned int  demo;


  float color;
  float colorSpeed = 1;

  float lastPoleValue = 0;
  float change = 0;
  while(1)
  {

   
    float poleValue =getPoleValue();
    float tmpChange =  abs(poleValue-lastPoleValue) ;
    if( change < tmpChange && poleValue > 0.60)
    {
      change =change *0.5+ tmpChange * 0.5;
    }
    else
    {
      change = change * 0.99 +tmpChange * 0.01; 
    } 

    lastPoleValue = poleValue;
    float poleValue_sensitive = sqrt(poleValue);

    color = color+colorSpeed;

    if(poleValue> 0.50)
    {
      colorSpeed = colorSpeed * 0.9999f + poleValue*300.0f  * 0.0001f;
    }
    else
    {
      colorSpeed = colorSpeed * 0.995f;
    }
    colorSpeed = max(0.3,min(colorSpeed,100));




    if(!midiMode)
    {
      PITCH[0]= PITCH[0] * 0.99 + 0.01 *colorSpeed*40.0f; // slow-fast;
      PITCH[1]= PITCH[1] * 0.9 + 0.1 * (1500.0f*change+ colorSpeed*40.0f) ;
      PITCH[2]= 200;  //*poleValue*350.0f * (sin( poleValue*10)+1.0f)/2 ;
      PITCH[3]= PITCH[3] * 0.95 + 0.05 *(poleValue_sensitive*1000.0+change*60000.0f); 
      ;// PITCH[3] * 0.99f + poleValue* 0.01f ;
      AMP[0]= colorSpeed * 255.0f;  // rising it is all good
      AMP[1]= min(200,change*555.0f); //hummm
      AMP[2]=min(200,poleValue_sensitive*change * 2000); //du dom
      AMP[3] =155;//  (sin( millis()/10000.0f)+1)*88;
    }




    /*   if(tim>1000*(100-colorSpeed)/50.0)
     {
     
     bar=counter&0x1f;
     demo=counter>>5;
     
     tim=0;
     
     } */

    //************************************************
    //  Modulation engine
    //************************************************

    FTW[0]=PITCH[0]+(PITCH[0]*(EPCW[0]/(32767.5*128.0  ))*((int)MOD[0]-512));
    FTW[1]=PITCH[1]+(PITCH[1]*(EPCW[1]/(32767.5*128.0  ))*((int)MOD[1]-512));
    FTW[2]=PITCH[2]+(PITCH[2]*(EPCW[2]/(32767.5*128.0  ))*((int)MOD[2]-512));
    FTW[3]=PITCH[3]+(PITCH[3]*(EPCW[3]/(32767.5*128.0  ))*((int)MOD[3]-512));
    
    if(midiMode)
    {
     sendControlValues();
    setControlValue(1,change*127.0f);
    setControlValue(2,poleValue*127.0f);
    setControlValue(3,colorSpeed);
    setControlValue(4,poleValue_sensitive*127.0f);
    }
  }
}

//*****************************************************************************************************************
//  
//*****************************************************************************************************************

// CapSense   cs_4_2 = CapSense(A5,2); 

// POLE SYSTEM
 
void setupPole()
{

 // Serial.begin(9600);
  pinMode(A3, OUTPUT);
  pinMode(A5, INPUT);
  digitalWrite(4, HIGH); //pustbutton
  pinMode(4,INPUT);  // pushbutton
  pinMode(11,OUTPUT);
  kalmaninit(0.4, 20, 20, 0);

}
float high=0, low=0;
float value = 0;

float baseLine = 0;
float topLine = 0;
float lastValue =0;
float tmpValue;

float tmpValueH;
float getPoleValue()
{
  if(digitalRead(4)==LOW)
  {
    baseLine = value*0.95f;
  }
  topLine = analogRead(0)/2 +baseLine ;

  digitalWrite(A3,HIGH);
  high = analogRead(5);

  digitalWrite(A3,LOW);
  low = analogRead(5);
  
kalmanupdate((high-low)/1024.0f);
value = x;
  tmpValue = map(value,baseLine,topLine,0.0f,650.0f)/650.0f;
  tmpValue = constrain(tmpValue,0.0f,1.0f);
  tmpValue = sqrt(tmpValue);
  // higher level touch
  tmpValueH = map(value,topLine,980,0.0f,650.0f)/650.0f;
  tmpValueH = constrain(tmpValueH,0.0f,1.0f);
  tmpValueH = sqrt(tmpValueH);


  analogWrite(11,abs(tmpValue -lastValue)*2000*tmpValue);
  lastValue = tmpValue;

  analogWrite(3,tmpValue*100.0f +tmpValueH*100.0f);
  if(!midiMode)
  {
    Serial.print(baseLine);
    Serial.print("  ");
    Serial.print(topLine);
    Serial.print("  ");
    Serial.print(round(baseLine-value));
    Serial.print("  ");
    Serial.print(round(tmpValue*100));
    Serial.print("  ");
    Serial.print(round(tmpValueH*100));
    Serial.print("  ");
    Serial.print((high-low));
    Serial.println();
  }
  else
  {
    setControlValue(5,round(tmpValue*127.0f));
    setControlValue(6,round(tmpValueH*127.0f));
  }

  return (tmpValue+tmpValueH)/2.0f; 

}

#define debugMidi false
void midiSetup()
{
  Serial.begin(31250);
  
}

void setControlValue(int num, int value)
{
    value = constrain(value,0,127);
    controlValues[num] = value;
}

void sendControlValues()
{
   if(debugMidi)
   {
     sendControl(analogRead(0)/1024.f * controlNum, 126);
     
   }
   else
   {
   for(int i = 0 ; i < controlNum ;i++)
   {
     
     sendControl(i, controlValues[i]);
   }
   }
   
  
}

void noteOn(int cmd, int pitch, int velocity) {
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}
void sendControl(int controller, int value) {
  Serial.write(176);  // control change on channel 1
  Serial.write(20 + controller);
  Serial.write(value);
}



