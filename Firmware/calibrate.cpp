#include "settings.h"
#include "variables.h"
#include "daisysp.h"
#include "daisy_seed.h"
using namespace daisysp;
using namespace daisy;

void calibrate()
{
  // bool next = 0;
  // float enc1 = 0;
  // float enc2 = 0;
  // return;
  // while(next == 0)
  // {
  //   enc1 = (encoder1_read()*100);
  //   if(enc1 != 0)
  //   {
  //     lowerpm1 = lowerpm1 + enc1;
  //     //Serial.println(String("LowerPM1:") + lowerpm1);
  //   }
  //   for(int i = 0; i<= 15;i++)
  //   {
  //     if(i == 0)
  //     {
  //       setpwm(i,(upperpm1 - lowerpm1) - pm1.getValue());
  //     }
  //     else
  //     {
  //       setpwm(i,0);
  //     }
  //   }
  //   switch2.update();
  //   if(switch2.fell())
  //   {
  //     next = 1;
  //   }
  // } //calibrate lower pm1 
  // next = 0;
  // while(next == 0)
  // {
  //   enc2 = (encoder2_read()*100);
  //   if(enc2 != 0)
  //   {
  //     upperpm1 = upperpm1 + enc2; 
  //     //Serial.println(String("UpperPM1:") + upperpm1);
  //   }
  //   for(int i = 0; i<= 15;i++)
  //   {
  //     if(i == 1)
  //     {
  //       setpwm(i,(upperpm1 - lowerpm1) - pm1.getValue());
  //     }
  //     else
  //     {
  //       setpwm(i,0);
  //     }
  //   }
  //   switch2.update();
  //   if(switch2.fell())
  //   {
  //     next = 1;
  //   }
  // } //calibrate upper pm1 
  // next = 0;
  // return;
}