#include "settings.h"
#include "variables.h"
#include "daisysp.h"
#include "daisy_seed.h"
using namespace daisysp;
using namespace daisy;

// Typical Switch case for Message Type.
void HandleMidiMessage(MidiEvent m)
{
  if(learn_active)
  {
    midichannel = m.channel;
    learn_active = 0;
    led_menu_step(midichannel,green);
    getorsetsettings(save);
  }

  if(m.channel == midichannel)
  {
    switch(m.type)
    {
      case NoteOn:
      {
        NoteOnEvent p = m.AsNoteOn();
        if(m.data[1] != 0)
        {
            p = m.AsNoteOn();
            //osc.SetFreq(mtof(p.note));
            voiceON(p.note);   
        }
      }
      break;
      case NoteOff:
      {
        NoteOffEvent p = m.AsNoteOff();
        if(m.data[1] != 0)
        {
            p = m.AsNoteOff();
            
            voiceOFF(p.note);        
        }
      }
      break;
      case ControlChange:
      {
        ControlChangeEvent p = m.AsControlChange();
        switch(p.control_number)
        {
            case 1:
            
            default: break;
        }
        break;
      } //case control change ends
      case PitchBend:
      {
        PitchBendEvent p = m.AsPitchBend();
        float bended;
        if (p.value >= 0)
        {
          bended = (float)p.value / 8191;
          pbend = 1 + bended;
        }
        if (p.value < 0)
        {
          bended = (float)p.value / 8192;
          pbend = 1 + 0.5 * bended;
        }
      } //case pitchbend ends
      default: break;
    } //switch ends
  }//if midi channel ends
}


void pitchbend(uint8_t channel, int bend)
{
  float bended;
  

  if (channel == (midichannel))
  {
    
  }
  //midi_read = 0;
} //pitchbend ends

void controlchange(uint8_t channel, uint8_t number, uint8_t value)
{

  float fvalue = value;
  if (channel == midichannel)
  {
    switch (number)
    {
    case 5:
      glidetime = 2 * (fvalue / 127);
      //if(debug_midi) Serial.println(String("glidetime:") + glidetime);
      
      break;
    // case 20:
    //   osc2lvl = (fvalue/127)*100;
    //   AudioNoInterrupts();
    //   osc2v1.amplitude((osc2lvl/100)*waveampmult);
    //   osc2v2.amplitude((osc2lvl/100)*waveampmult);
    //   osc2v3.amplitude((osc2lvl/100)*waveampmult);
    //   osc2v4.amplitude((osc2lvl/100)*waveampmult);
    //   AudioInterrupts();
    // break;
    }
  }
  //midi_read = 0;
}
