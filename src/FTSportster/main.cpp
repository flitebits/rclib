#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Basic bootstraping/framework code
#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

// Subsystem includes
#include "Adc.h"
#include "Pwm.h"
#include "Rand.h"
#include "Serial.h"
#include "SBus.h"
#include "SportSensor.h"
#include "Util.h"
#include "Pins.h"
#include "WS2812.h"

// LED helper functions.
#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "leds/Clr.h"


const u8_t RWING_PWM = 0;
const u8_t LWING_PWM = 1;
const u8_t VOLT_APIN = 14;
const u8_t AMP_APIN = 15;  // Not used
const u8_t LIGHT_LEVEL_CH = 4;  // Turns lights off/Low/Hi
const u8_t LIGHT_MODE_CH = 5;  // Sets Lights Mode
const u8_t LIGHT_BRIGHT_CH = 6;  // Adjust leading edge light brightness.
const u8_t LIGHT_THROTTLE_CH = 7;

const u8_t log_map[9] = {0, 1, 2, 5, 11, 24, 52, 115, 255};

namespace {
const led::RGB heat [5] = {
			   led::RGB(0x00, 0x00, 0x00),
			   led::RGB(0x80, 0x00, 0x00),
			   led::RGB(0xFF, 0x80, 0x00),
			   led::RGB(0xFF, 0xFF, 0x00),
			   led::RGB(0xFF, 0xFF, 0xFF),
};

u8_t logify(u8_t val) {
  u8_t lo = val & 0x1F;
  u8_t hi = val >> 5;
  int ret = ((u16_t(log_map[hi]) << 5) +
	     (i16_t(log_map[hi + 1]) - log_map[hi]) * lo);
  return (ret + (1<<4)) >> 5;
}

}  // anonymous namespace

// Note that this mixes RGBW and RGB SK6812 style leds in on string.
// Fortunately we just need to push the bits out right and it will all
// work, so we sort of lie and say there are 100 RGBW leds
// since that is 400 bytes.
struct LedGroups {
  LedGroups(Pwm* pwm)
    : trippy_hue_(0),
      trippy_loc_(0),
      pulse_val_(0),
      pulse_loc_(0),
      left_tip_clr_(led::clr::red),
      right_tip_clr_(led::clr::green),
      curr_level_(255),
      curr_scale_(0),
      curr_mode_(255),
      pwm_(pwm) {
    OffMode();
    pwm_->Set(LWING_PWM, 0);
    pwm_->Set(RWING_PWM, 0);
  }

  enum {
	kWingNLed = 22,
	kTipNLed = 6,
	kWingMidNLed = 3,
	kTailNLed = 4,
	// #LED factoring in 3 & 4 sub pixels, pretending as if all
	// #were 4 sub pixel
	kTotalLed = ((kWingNLed    * 4 * 4 +
		      kWingMidNLed * 1 * 4 +
		      kTipNLed     * 2 * 3 +
		      kTailNLed    * 3 * 3) + 3) >> 2,
  };

  led::RGBW rightF[kWingNLed];
  led::RGB  rightT[kTipNLed];
  led::RGBW rightB[kWingNLed];
  led::RGBW midB[kWingMidNLed];
  led::RGBW leftB[kWingNLed];
  led::RGB  leftT[kTipNLed];
  led::RGBW leftF[kWingNLed];
  led::RGB  rightE[kTailNLed];
  led::RGB  tail[kTailNLed];
  led::RGB  leftE[kTailNLed];
  
  void FillWing(led::RGBW clr) {
    FillLWing(clr);
    FillRWing(clr);
    led::Fill(midB, kWingMidNLed, clr);
  }
  void FillLWing(led::RGBW clr) {
    led::Fill(leftF, kWingNLed, clr);
    led::Fill(leftB, kWingNLed, clr);
  }
  void FillRWing(led::RGBW clr) {
    led::Fill(rightF, kWingNLed, clr);
    led::Fill(rightB, kWingNLed, clr);
  }
  void SetLWingPix(u8_t idx, led::RGBW clr) {
    leftF[kWingNLed - 1 - idx] = clr;
    leftB[idx] = clr;
  }
  void SetRWingPix(u8_t idx, led::RGBW clr) {
    rightF[idx] = clr;
    rightB[kWingNLed - 1 - idx] = clr;
  }
  
  void PushLeds() {
	SendWS2812(PinId(PIN_A4), rightF, kTotalLed * 4, curr_scale_);
  }
  
  void UpdateMode(u8_t new_level, u8_t new_mode, int brt, int thr) {
    if (new_level == 0){
      if (curr_level_ == 0) return;
      curr_level_ = new_level;
      curr_mode_ = 255;
      pwm_->Set(LWING_PWM, 0);
      pwm_->Set(RWING_PWM, 0);
      curr_scale_ = 0;
      OffMode();
      PushLeds();
      return;
    }

	brt = logify(brt >> 3);
    if (new_level == 1) brt = brt >> 2;

    pwm_->Set(LWING_PWM, brt);
    pwm_->Set(RWING_PWM, brt);

    if (curr_mode_ == new_mode && curr_level_ == new_level) return;
    curr_scale_ = (new_level == 1) ? 0x20 : 0xFF;

    switch (new_mode) {
    case 0:
      SolidMode();
      break;
    case 1:
      SetNav();
      if (curr_mode_ != new_mode) PulseInit();
      PulseMode(thr);
      break;
    case 2:
      SetNav();
      if (curr_mode_ != new_mode) TrippyInit();
      TrippyMode(thr);
      break;
    }
    curr_level_ = new_level;
    curr_mode_ = new_mode;
    PushLeds();
  }
  void Update(int thr) {
	if (curr_level_ == 0) return;
    if (curr_mode_ == 0) return;
    if (curr_mode_ == 1) PulseMode(thr);
    else TrippyMode(thr);
    PushLeds();
  }

  void OffMode() {
    led::Fill(rightF, kTotalLed,    led::wclr::black);
  }

  void SetNav() {
    led::Fill(leftT,  kTipNLed,     led::clr::red);
    led::Fill(rightT, kTipNLed,     led::clr::green);
    led::Fill(leftE,  kTailNLed,    led::clr::red);
    led::Fill(tail,   kTailNLed,    led::clr::white);
    led::Fill(rightE, kTailNLed,    led::clr::green);
  }

  void SolidMode() {
    led::Fill(midB,   kWingMidNLed, led::wclr::white);
    led::Fill(leftF,  kWingNLed,    led::wclr::white);
    led::Fill(leftB,  kWingNLed,    led::wclr::red);
    led::Fill(rightF, kWingNLed,    led::wclr::white);
    led::Fill(rightB, kWingNLed,    led::wclr::green);
    SetNav();
  }

  void TrippyInit() {
    trippy_hue_ = 0;
    trippy_loc_ = 0;
    FillWing(led::clr::white);
  }

  void TrippyMode(int thr) {
    const u32_t SPEED_SCALE = 32;  // 4 bit value
    const i16_t SPEED_MIN   = 96; 
    const u32_t HUE_SCALE   = 12;  // 6 bit value
    const u32_t HUE_MIN     =  4;  // 6 bit value

    trippy_hue_ += HUE_MIN + ((thr * HUE_SCALE + (1 << 7)) >> 8);
    while (trippy_hue_ > (0xFF << 6)) {
      trippy_hue_ -= (0xFF << 6); 
    }
    led::RGB pix = led::HsvToRgb(led::HSV(trippy_hue_ >> 6, 0xFF, 0xFF));

    trippy_loc_ += SPEED_MIN + ((thr * SPEED_SCALE + (1 << 7)) >> 8);
    u8_t steps = (trippy_loc_ >> 8);
    trippy_loc_ = trippy_loc_ & 0xFF;  // Just keep fractional part.
    for (int i = 0; i < kWingNLed - steps; ++i) {
      leftB[kWingNLed - 1 - i] = leftB[(kWingNLed - 1) - i - steps];
      rightF[kWingNLed - 1 - i] = rightF[(kWingNLed - 1) - i - steps];
      leftF[i] = leftF[i + steps];
      rightB[i] = rightB[i + steps];
    }
    Fill(midB, kWingMidNLed, pix);
    for (int i =0; i < steps; ++i) {
      SetLWingPix(i, pix);
      SetRWingPix(i, pix);
    }
  }

  void PulseInit() {
    pulse_val_ = 0;
    pulse_loc_ = 0;
    memset(work_, sizeof(work_), 0);
    memset(frontWhite_, sizeof(frontWhite_), 0xFF);
    FillWing(led::wclr::black);
  }

  void PulseMode(int thr) {
    const u32_t PULSE_SCALE = 25;  // 4 bit value
    const u32_t SPEED_SCALE = 24;  // 4 bit value
    const i16_t PULSE_MIN = 50; 

    // "Cool" wing leds
    for (int i=0; i < kWingNLed + 3; ++i) {
      work_[i] = (work_[i] * int(0xF0)) >> 8;
      frontWhite_[i] = (frontWhite_[i] * int(0xF0)) >> 8;
    }

    pulse_val_ += PULSE_MIN + ((thr * PULSE_SCALE) >> 8);
    if (pulse_val_ < 0xFFF) {
      int val = work_[0] + (pulse_val_ >> 4);
      work_[0] = (val > 0xFF) ? 0xFF : val;  // center of pulse
      val = work_[1] + (pulse_val_ >> 6);
      work_[2] = (val > 0xFF) ? 0xFF : val;
      val = work_[3] + (pulse_val_ >> 8);
      work_[3] = (val > 0xFF) ? 0xFF : val;
    } else {
      pulse_val_ = 0xFFF;
      int prev_pulse_idx = ((pulse_loc_ + (1 << 7)) >> 8);
      pulse_loc_ += PULSE_MIN + ((thr * SPEED_SCALE + (1 << 7)) >> 8);
      int pulse_idx = ((pulse_loc_ + (1 << 7)) >> 8);
      for (int i = prev_pulse_idx; i <= pulse_idx + 2; ++i) {
	work_[i] = 0xFF;
      }
      int val = work_[pulse_idx + 3] + (0xFFF >> 6);
      work_[pulse_idx + 3] = (val > 0xFF) ? 0xFF : val;
      val = work_[pulse_idx + 4] + (0xFFF >> 8);
      work_[pulse_idx + 4] = (val > 0xFF) ? 0xFF : val;
      if (pulse_idx >= kWingNLed) {
	pulse_loc_ = 0;
	pulse_val_ = 0;
	memset(frontWhite_, sizeof(frontWhite_), 0xff);
	left_tip_clr_  = led::clr::white;
	right_tip_clr_ = led::clr::white;
      }
    }
  
    for (int i=0; i < kWingNLed; i++) {
      led::RGBW clr(led::Lookup5(heat, work_[i]), frontWhite_[i]);
      SetLWingPix(i, clr);
      SetRWingPix(i, clr);
    }
    led::Blend(&left_tip_clr_, led::clr::red, 16);
    led::Blend(&right_tip_clr_, led::clr::green, 16);
    led::Fill(leftT, kTipNLed, left_tip_clr_);
    led::Fill(rightT, kTipNLed, right_tip_clr_);
  }

  int trippy_hue_;  // Has 6 bits frac
  int trippy_loc_;  // Has 8 bits frac (0 -> kWingNLed)

  // pulse_val is logically 12 bits
  int pulse_val_;  // 12 bit (0 -> 1)
  int pulse_loc_;  // Has 8 bits (0 -> kWingNLed + 3)

  led::RGB left_tip_clr_;
  led::RGB right_tip_clr_;
  u8_t work_[kWingNLed + 3];
  u8_t frontWhite_[kWingNLed + 3];

  u8_t curr_level_;  // Brightness level 0 -> off, 1 -> 12%, 2 -> 100%
  u8_t curr_scale_;  // Scale value derived from curr_level_
  u8_t curr_mode_;   // Modes, 0 -> static, 1 -> pulse, 2 -> ???
  Pwm* pwm_;
};

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/2, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_MD(APP);
  DBG_LEVEL_LO(SBUS);

  Adc adc(Adc::VREF_43);
  adc.ConfigurePin(6);    // Read a "mostly random" seed from Analog pin 6 (PD6)
  randomSeed(adc.Read(6));
  Adc::ConfigurePin(VOLT_APIN);
  Adc::ConfigurePin(AMP_APIN);
  
  PORTA.OUTCLR = (1 << 4);  // Set PA4 low
  PORTA.PIN4CTRL = 0;  // Not inverted, not pullup enabled, no interrupts.
  PORTA.DIRSET = 1 << 4;  // Pin PA4 as output pin.
  
  SBus sbus(&Serial::usart1, /*invert=*/false);

  SportSensor sport(&Serial::usart2, /*invert=*/true, /*use_alt_pins=*/false);
  i8_t sport_current_idx, sport_volt_idx;
  sport.AddFcs40Sensors(&sport_current_idx, &sport_volt_idx);

  // Pwm module use the 3 TCA timers as split 8bit PWM control providing
  // up to 6 channels of PWM.
  Pwm pwm(PORT_D, 1000);  // Select what port to use and freq of PWM
  pwm.Set(LWING_PWM, 0);
  pwm.Enable(LWING_PWM);
  pwm.Set(RWING_PWM, 0);
  pwm.Enable(RWING_PWM);

  LedGroups leds(&pwm);
  
  // Configure blinky LED on pin PF2
  PORTF.OUT = 1 << 2;  // Set PF2 high, all others low
  PORTF.PIN2CTRL = 0;  // Not inverted, not pullup enabled, no interrupts.
  PORTF.DIR = 1 << 2;  // Pin PF2 as output pin.

  // Enable interrupts (end of init, real program begin).
  sei();

  leds.PushLeds();

  adc.StartRead(VOLT_APIN);

  const bool heartbeat_enable = true;  // True to get heartbeat on board led.
  u8_t update_4 = 255;  // ~1/64 sec  (~16ms each)
  u8_t update_8 = 255;  // ~1/4 sec  (~250ms each )
  u8_t update_10 = 255;  // ~1 sec
  while (1) {
    // Update telemetry if we were polled (call very frequently since there
    // is a limited time window to respond when polled).  This uses the last
    // value set for the polled device.
    sport.Run();
    if (sbus.Run()) {
      // Run returns true of a new frame of channel data was received
      leds.UpdateMode(SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_LEVEL_CH)),
                      SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH)),
                      sbus.GetChannel(LIGHT_BRIGHT_CH),
                      sbus.GetChannel(LIGHT_THROTTLE_CH));
    }

    u32_t now = FastTimeMs();
    u8_t now_4 = now >> 4;
    if (now_4 != update_4) {  // ~30fps
      leds.Update(sbus.GetChannel(LIGHT_THROTTLE_CH));
      int volts = adc.FinishRead();
      adc.StartRead(VOLT_APIN);
      // 1024 -> 4.3v * 4 -> 17.4v, scale is 100x, so scale is 1740 / 1024
      volts =  (1740L * volts + (1<<9)) >> 10;
      sport.SetSensor(sport_volt_idx, volts);
      sport.SetSensor(sport_current_idx, 0);  // not tracking amps.
      DBG_HI(APP, ("Volts: %d\n", volts));
    }

    u8_t now_10 = now >> 10;
    if (now_10 != update_10) {  // ~60fps
      update_10 = now_10;
    }
    if (heartbeat_enable) {
      u8_t now_8 = now >> 8;
      if (now_8 != update_8) {  // ~4fps
        update_8 = now_8;
        PORTF.OUTTGL = 1 << 2;  // Toggle PF2
      }
    }
  }
}

