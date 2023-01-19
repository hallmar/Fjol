#include "settings.h"
#include "variables.h"
#include "daisysp.h"
#include "daisy_seed.h"
using namespace daisysp;
using namespace daisy;

float encoder_read(int8_t step)
{
  float value = 0;
  if(step == 1)
  {
  
      if(System::GetNow()-lastmillis1 < velocity_mid)
      {
        value = 0.04;
      }
      else if(System::GetNow()-lastmillis1 < velocity_low)
      {
        value = 0.02;
      }
      else 
      {
        value = 0.01; 
      }
   // value = 0.01;
    lastmillis1 = System::GetNow();
    return value;
   
  }
  else if(step == -1)
  {

      if(System::GetNow()-lastmillis1 < velocity_mid)
      {
        value = -0.04;
      }
      else if(System::GetNow()-lastmillis1 < velocity_low)
      {
        value = -0.02;
      }
      else 
      {
        value = -0.01; 
      }
    lastmillis1 = System::GetNow();
    return value;
  }
  return 0;
} //enc read endar

void led_update(uint8_t index)
{
    //only update one paramater led
    for(int i = 0; i<17; i++)
    {
      if(i/2 == index)
      {
        setpwm(i,led_fade(param[i],green));
        setpwm(i+1,led_fade(param[i],red));
      }
    }
    //update all leds
    if(index == 8)
      for(int i = 0; i<17;i++)
      {
        if(i%2 == 0)
        {
          setpwm(i,led_fade(param[i],green));
          setpwm(i+1,led_fade(param[i],red));
        }
      } 
}

void led_menu_step(uint8_t number,uint8_t color)
{
  uint8_t params_index[8] = {1,3,5,7,9,11,13,15};

  if(color == green)
  {
    for(int i = 0; i<= 15;i++)
    {
      if(i == params_index[number] )
      {
        setpwm(i,brightness*pwm_resolution);
      }
      else
      {
        setpwm(i,0);
      }
    }
  }
  else if(color == red)
  {
    for(int i = 0; i<= 15;i++)
    {
      if(i == (params_index[number]-1))
      {
        setpwm(i,brightness*pwm_resolution);
      }
      else
      {
        setpwm(i,0);
      }
    }
  }
  else{return;}
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int led_fade (float value, uint8_t color) // 0 = grænn 1 = rauður
{
  
  int gr;
  int rd;
  switch(color)
  {
    case green:
  
       if(value <= 0.5)
      {
        gr = map(value,0,0.5,0,pwm_resolution);
        if(gr > pwm_resolution) gr = pwm_resolution;
        if(gr <= 0) gr = 0;
        return gr;
      }
      else if(value > 0.5 && value <= 1)
      {
        gr = map(value,1,0.5,0,pwm_resolution);
        if(gr > pwm_resolution) gr = pwm_resolution;
        if(gr < 0) gr = 0;
        return gr;
      }  
    break;
    case red:
      rd = map(value, 0.3, 1, 0, pwm_resolution);
      if(rd > pwm_resolution) rd = pwm_resolution;
      if(rd < 0) rd = 0;
      return rd;
    break;
  } //switch endar
  return 0;
}

