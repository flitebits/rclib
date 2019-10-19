#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

#include "Adc.h"
#include "Pwm.h"
#include "Spi.h"
#include "Serial.h"
#include "SBus.h"
#include "SportSensor.h"

#include "leds/Pixel.h"

#define VOLT_APIN (8)
#define AMP_APIN (9)

#define LED_CNT (70)
#define BODY_CNT (44)
#define TIP_CNT (8)
#define TAIL_CNT (10)
#define BODY_IDX (0)
#define LTIP_IDX (BODY_CNT)
#define RTIP_IDX (BODY_CNT + TIP_CNT)
#define TAIL_IDX (BODY_CNT + 2 * TIP_CNT)

#define TAIL_BASE_BRT (16)  // base brightness of tail in pulse mode

led::RGB leds[LED_CNT];
u8_t work[LED_CNT];

#define LWING_PWM (2)
#define RWING_PWM (3)

#define LIGHT_SET_CH (4)  // Turns lights off/Low/Hi
#define LIGHT_MODE_CH (5)  // Sets Lights Mode
#define LIGHT_BRIGHT_CH (6)  // Adjust leading edge light brightness.
#define LIGHT_THROTTLE_CH (7)
led::RGB heat [5] = {
  led::RGB(0x00, 0x00, 0x00),
  led::RGB(0x80, 0x00, 0x00),
  led::RGB(0xFF, 0x80, 0x00),
  led::RGB(0xFF, 0xFF, 0x00),
  led::RGB(0xFF, 0xFF, 0xFF),
};
void PulseMode(int thr) {
  const u32_t PULSE_SCALE = 32;  // 4 bit value
  const u32_t SPEED_SCALE = 48;  // 4 bit value
  const i16_t PULSE_MIN = 128; 
  
  // "Cool" Body leds
  for (int i=0; i < BODY_CNT + 4; i++) {
    work[i] = (work[i] * int(0xF0)) >> 8;
  }

  // pulse_val is logically 12 bits
  static int pulse_val = 0;  // 12 bit (0 -> 1)
  static int pulse_loc = 0;  // Has 8 bits (0 -> BODY_CNT)
  pulse_val += PULSE_MIN + ((thr * PULSE_SCALE) >> 8);
  if (pulse_val < 0xFFF) {
    int val = work[2] + (pulse_val >> 4);
    work[2] = (val > 0xFF) ? 0xFF : val;  // center of pulse
    val = work[1] + (pulse_val >> 6);
    work[1] = (val > 0xFF) ? 0xFF : val;
    val = work[3] + (pulse_val >> 6);
    work[3] = (val > 0xFF) ? 0xFF : val;
    val = work[0] + (pulse_val >> 8);
    work[0] = (val > 0xFF) ? 0xFF : val;
    val = work[4] + (pulse_val >> 8);
    work[4] = (val > 0xFF) ? 0xFF : val;
  } else {
    pulse_val = 0xFFF;
    int prev_pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
    pulse_loc += PULSE_MIN + ((thr * SPEED_SCALE + (1 << 7)) >> 8);
    int pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
    for (int i = prev_pulse_idx; i <= pulse_idx + 2; ++i) {
      work[i] = 0xFF;
    }
    int val = work[pulse_idx + 3] + (0xFFF >> 6);
    work[pulse_idx + 3] = (val > 0xFF) ? 0xFF : val;
    val = work[pulse_idx + 4] + (0xFFF >> 8);
    work[pulse_idx + 4] = (val > 0xFF) ? 0xFF : val;
    if (pulse_idx > BODY_CNT - 8) {
      val = work[TAIL_IDX] + (0xFFF >> 6);
      work[TAIL_IDX] = (val > 0xFF) ? 0xFF : val;
    }    
    if (pulse_idx > BODY_CNT - 1) {
      work[TAIL_IDX] = 0xFF;
      pulse_loc = 0;
      pulse_val = 0;
      work[TAIL_IDX] = 0xFF;
    }
  }
  
  for (int i=0; i<BODY_CNT; i++) {
    led::RGB clr = led::Lookup5(heat, work[i]);
    leds[i] = clr;
  }
  
  const u8_t base = TAIL_BASE_BRT;
  work[TAIL_IDX] = base + (((work[TAIL_IDX] - base) * 0xD8) >> 8);
  led::RGB clr = led::Lookup5(heat, work[TAIL_IDX]);

  led::Fill(leds + TAIL_IDX, TAIL_CNT, clr);
}

void Trippy(int thr) {
  const u32_t SPEED_SCALE = 32;  // 4 bit value
  const i16_t SPEED_MIN   = 96; 
  const u32_t HUE_SCALE   = 12;  // 6 bit value
  const u32_t HUE_MIN     =  4;  // 6 bit value
  
  // "Cool" Body leds
  for (int i=0; i < BODY_CNT; i++) {
    work[i] = (work[i] * int(0xF8)) >> 8;
  }

  static int pulse_hue = 0;
  pulse_hue += HUE_MIN + ((thr * HUE_SCALE + (1 << 7)) >> 8);
  led::RGB pix = led::HsvToRgb(led::HSV(pulse_hue>>6, 0xFF, 0xFF));

  static int pulse_loc = 0;  // Has 8 bits (0 -> BODY_CNT)
  u8_t prev_pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
  pulse_loc += SPEED_MIN + ((thr * SPEED_SCALE + (1 << 7)) >> 8);
  u8_t pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
  if (pulse_idx >= BODY_CNT) {
     led::Fill(leds + prev_pulse_idx, BODY_CNT - prev_pulse_idx, pix);
     prev_pulse_idx = 0;
     pulse_loc -= (BODY_CNT << 8);
     pulse_idx = ((pulse_loc + (1 << 7)) >> 8);
  }
  led::Fill(leds + prev_pulse_idx, pulse_idx - prev_pulse_idx + 1, pix);
  led::Fill(leds + TAIL_IDX, TAIL_CNT, pix);
}

class Lights {
public:
  Lights(Pwm* pwm) :
    curr_level_(255),
    curr_scale_(0),
    curr_mode_(255),
    pwm_(pwm) {
    pwm_->Set(LWING_PWM, 0);
    pwm_->Set(RWING_PWM, 0);
  }
  void SetLeftNav(led::RGB color) {
    led::Fill(leds + 52, 8, color);
  }
  void SetRightNav(led::RGB color) {
    led::Fill(leds + 44, 8, color);
  }
  void SetTail(led::RGB color) {
    led::Fill(leds + 60, 10, color);
  }
  void SetBody(led::RGB color) {
    led::Fill(leds, 44, color);
  } 

  void UpdateMode(u8_t new_level, u8_t new_mode, int brt, int thr) {
    if (new_level == 0){
      if (curr_level_ == 0) return;
      curr_level_ = new_level;
      curr_mode_ = 255;
      pwm_->Set(LWING_PWM, 0);
      pwm_->Set(RWING_PWM, 0);
      curr_scale_ = 0;
      led::Fill(leds, LED_CNT, led::clr::black);
      Push();
      return;
    }

    if (brt < 32) brt = 0;
    if (new_level == 1) brt = brt >> 6;
    else brt = (brt >> 3);

    pwm_->Set(LWING_PWM, brt);
    pwm_->Set(RWING_PWM, brt);

    if (curr_mode_ == new_mode && curr_level_ == new_level) return;
    curr_scale_ = (new_level == 1) ? 0x40 : 0xFF;
    switch (new_mode) {
    case 0:
      SetLeftNav(led::clr::red);
      SetRightNav(led::clr::green);
      SetTail(led::clr::white);
      SetBody(led::clr::white);
      break;
    case 1:
      SetLeftNav(led::clr::red);
      SetRightNav(led::clr::green);
      if (curr_mode_ != new_mode) {
        for (int i=0; i > LED_CNT; ++i) {
	  work[i] = 0;
        }
        work[TAIL_IDX] = TAIL_BASE_BRT;
        SetBody(led::clr::black);
      }      
      PulseMode(thr);
      break;
    case 2:
      SetLeftNav(led::clr::red);
      SetRightNav(led::clr::green);
      if (curr_mode_ != new_mode) {
        SetBody(led::clr::black);
      }      
      Trippy(thr);
      break;
    }
    curr_level_ = new_level;
    curr_mode_ = new_mode;
    Push();
  }

  void Update(int thr) {
    switch (curr_mode_) {
    case 0: return;
    case 1: PulseMode(thr); break;
    case 2: Trippy(thr); break;
    }
    Push();
  }
  
  void Push() {
    Spi::spi.UpdateLeds(leds, LED_CNT, curr_scale_);    
  }

protected:
  u8_t curr_level_;  // Brightness level 0 -> off, 1 -> 12%, 2 -> 100%
  u8_t curr_scale_;  // Scale value derived from curr_level_
  u8_t curr_mode_;   // Modes, 0 -> static, 1 -> pulse, 2 -> ???
  Pwm* pwm_;
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/4, /*use_internal_32Kclk=*/false);
  SetupRtcClock(/*use_internal_32K=*/false);
  Adc adc;
  Spi::spi.SetupAPA102(Spi::PINS_PA47, 2000000);
  Pwm pwm(PORT_D, 400);
  pwm.Enable(LWING_PWM);
  pwm.Enable(RWING_PWM);

  PORTF.OUT  = 0;  // Set all Low
  PORTF.DIR  |= ((1 << 6) | (1 << 2));  // Pins PF2 & PF5 set as output pints.

  led::Fill(leds, LED_CNT, led::clr::black);
  Spi::spi.UpdateLeds(leds, LED_CNT, 0x10);
  Lights lights(&pwm);

  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(SBUS);
  DBG_LEVEL_HI(SPORT);
  DBG_LEVEL_HI(APP);
  DBG_MD(APP, ("Hello World\n"));

  SportSensor sport(&Serial::usart1, /*invert=*/true, /*use_alt_pins=*/false);
  i8_t sport_current_idx, sport_volt_idx;
  sport.AddFcs40Sensors(&sport_current_idx, &sport_volt_idx);

  SBus sbus(&Serial::usart3, false);

  Adc::ConfigurePin(VOLT_APIN);
  Adc::ConfigurePin(AMP_APIN);
  // Enable interrupts program execution really begins.
  sei();
  
  bool read_volt = true;
  adc.StartRead(VOLT_APIN);

  u8_t update_4 = 0;  // ~1/64 sec
  u8_t update_8 = 0;  // ~1/4 sec
  u8_t update_10 = 0;  // ~1 sec

  int volt_avg = 0;
  int amp_avg = 0;
  while (true) {
    const unsigned long now = FastTimeMs();
    
    const u8_t now_8 = now >> 8; // 1/4 sec (1024 / 256)
    if (now_8 != update_8) {
      update_8 = now_8;
      PORTF.OUTTGL = (1 << 6);  // Pin PF6 toggle, simple heartbeat.
    }

    // Update telemetry if we were polled (call very frequently since there
    // is a limited time window to respond when polled).  This uses the last
    // value set for the polled device.
    sport.Run();
    if (sbus.Run()) {
	// Run returns true of a new frame of channel data was recieved
      lights.UpdateMode(SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_SET_CH)),
			SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH)),
			sbus.GetChannel(LIGHT_BRIGHT_CH),
			sbus.GetChannel(LIGHT_THROTTLE_CH));
    }

    const u8_t now_4 = now >> 4;  // roughly 60fps.
    if (update_4 != now_4) {
      update_4 = now_4;
      lights.Update(sbus.GetChannel(LIGHT_THROTTLE_CH));

      // OK this is overly clever but it avoids waiting on A2D
      // conversion basically it alternates reading amps/volts and
      // starts the read of one immediately after finishing the read
      // of the other.  The frequency here is probly serious overkill.
      // since it's only actually sent to controller every couple
      // 100ms.
      if (read_volt) {
	// 10bits (0 -> 0V, 1024 -> 5V)
	// 4:1 voltage divider -> 1024 = 20V
	int volts = adc.FinishRead();
	adc.StartRead(AMP_APIN);
	volts = ((20L * 100 * volts) + (1<<9)) >> 10;
	volt_avg = (volt_avg == 0) ? volts :
	  ((volt_avg * 7 + volts) + (1<<2)) >> 3;
	sport.SetSensor(sport_volt_idx, volt_avg);
      } else {
	// 0.5V = 0 Amps, 4.5V = 30 Amps
	// 1024 = 5V -> 102.4 = .5V
	// 4V = 4 * (1024 / 5) = 819 -> 30 A
	// 300 / 819 * 1024 = 375 (10bit FP scale)
	const int ampsV = adc.FinishRead();
	adc.StartRead(VOLT_APIN);
	int amps =  ((375L * (ampsV - 102)) + (1<< 9)) >> 10;
	amp_avg = (amp_avg == 0) ? amps :
	  ((amp_avg * 7 + amps) + (1<<2)) >> 3;
      	sport.SetSensor(sport_current_idx, amp_avg);
      }
      read_volt = !read_volt;
    }
    
    u8_t now_10 = now >> 10;  // ~1/sec, print debug info..
    if (now_10 != update_10) {
      update_10 = now_10;
      //sbus.Dump();
    }      
  }
}
