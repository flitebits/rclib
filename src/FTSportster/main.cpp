#include <avr/io.h>
#include <avr/interrupt.h>

// Basic bootstraping/framework code
#include "Boot.h"
#include "Dbg.h"
#include "IntTypes.h"
#include "RtcTime.h"

// Subsystem includes
#include "Adc.h"
#include "Pwm.h"
#include "Rand.h"
#include "Spi.h"
#include "Serial.h"
#include "SBus.h"
#include "SportSensor.h"

// LED helper functions.
#include "leds/Rgb.h"
#include "leds/Pixel.h"
#include "leds/Clr.h"


const u8_t RWING_PWM = 0;
const u8_t LWING_PWM = 1;
const u8_t VOLT_APIN = 14;
const u8_t AMP_APIN = 15;
const u8_t LIGHT_LEVEL_CH = 4;  // Turns lights off/Low/Hi
const u8_t LIGHT_MODE_CH = 5;  // Sets Lights Mode
const u8_t LIGHT_BRIGHT_CH = 6;  // Adjust leading edge light brightness.
const u8_t LIGHT_THROTTLE_CH = 7;

const u8_t log_map[9] = {1, 2, 4, 8, 16, 32, 64, 128, 255};

const int kWingNLed = 22;
const int kTipNLed = 6;
const int kWingMidNLed = 3;
const int kTailNLed = 4;

// #LED factoring in 3 & 4 sub pixels, pretending as if all were 4 sub pixel
const int kTotalLed = ((kWingNLed    * 4 * 4 +
			    kWingMidNLed * 1 * 4 +
			    kTipNLed     * 2 * 3 +
			    kTailNLed    * 3 * 3) + 3) >> 2;

// Note that this mixes RGBW and RGB SK6812 style leds in on string.
// Fortunately we just need to push the bits out right and it will all
// work, so we sort of lie to SPI and tell it there are 100 RGBW leds
// since that is 400 bytes.
struct LedGroups {
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
  
  void FillWing(bool left, led::RGBW clr) {
    if (left) {
      led::Fill(leftF, kWingNLed, clr);
      led::Fill(leftB, kWingNLed, clr);
    } else {
      led::Fill(rightF, kWingNLed, clr);
      led::Fill(rightB, kWingNLed, clr);
    }
  }
  void SetWingPix(bool left, u8_t idx, led::RGBW clr) {
    if (left) {
      leftF[kWingNLed - 1 - idx] = clr;
      leftB[idx] = clr;
    } else {
      rightF[idx] = clr;
      rightB[kWingNLed - 1 - idx] = clr;
    }
  }
  void UpdateMode(u8_t level, u8_t mode, u16_t bright, u16_t throttle) { }
} ledg;

void OffMode() {
  led::Fill(ledg.rightF, kTotalLed,    led::wclr::black);
}

void SolidMode() {
  led::Fill(ledg.midB,   kWingMidNLed, led::wclr::white);
  led::Fill(ledg.leftF,  kWingNLed,    led::wclr::white);
  led::Fill(ledg.leftB,  kWingNLed,    led::wclr::red);
  led::Fill(ledg.rightF, kWingNLed,    led::wclr::white);
  led::Fill(ledg.rightB, kWingNLed,    led::wclr::green);
  led::Fill(ledg.leftT,  kTipNLed,     led::clr::red);
  led::Fill(ledg.rightT, kTipNLed,     led::clr::green);
  led::Fill(ledg.leftE,  kTailNLed,    led::clr::red);
  led::Fill(ledg.tail,   kTailNLed,    led::clr::white);
  led::Fill(ledg.rightE, kTailNLed,    led::clr::green);
}

u8_t work[kWingNLed + 3];
const led::RGB heat [5] = {
  led::RGB(0x00, 0x00, 0x00),
  led::RGB(0x80, 0x00, 0x00),
  led::RGB(0xFF, 0x80, 0x00),
  led::RGB(0xFF, 0xFF, 0x00),
  led::RGB(0xFF, 0xFF, 0xFF),
};
led::RGB left_tip_clr = led::clr::red;
led::RGB right_tip_clr = led::clr::green;
void PulseMode(int thr) {
  const u32_t PULSE_SCALE = 32;  // 4 bit value
  const u32_t SPEED_SCALE = 48;  // 4 bit value
  const i16_t PULSE_MIN = 128; 

  // "Cool" Body leds
  for (int i=0; i < kWingNLed + 3; ++i) {
    work[i] = (work[i] * int(0xF0)) >> 8;
  }

  // pulse_val is logically 12 bits
  static int pulse_val = 0;  // 12 bit (0 -> 1)
  static int pulse_loc = 0;  // Has 8 bits (0 -> kWingNLed + 3)
  pulse_val += PULSE_MIN + ((thr * PULSE_SCALE) >> 8);
  if (pulse_val < 0xFFF) {
    int val = work[0] + (pulse_val >> 4);
    work[0] = (val > 0xFF) ? 0xFF : val;  // center of pulse
    val = work[1] + (pulse_val >> 6);
    work[2] = (val > 0xFF) ? 0xFF : val;
    val = work[3] + (pulse_val >> 8);
    work[3] = (val > 0xFF) ? 0xFF : val;
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
    if (pulse_idx >= kWingNLed) {
      pulse_loc = 0;
      pulse_val = 0;
	  left_tip_clr = led::clr::white;
	  right_tip_clr = led::clr::white;
    }
  }
  
  for (int i=0; i < kWingNLed; i++) {
    led::RGB clr = led::Lookup5(heat, work[i]);
    ledg.SetWingPix(true, i, led::RGBW(clr));
    ledg.SetWingPix(false, i, led::RGBW(clr));
  }
  led::Blend(&left_tip_clr, led::clr::red, 16);
  led::Blend(&right_tip_clr, led::clr::green, 16);
  led::Fill(ledg.leftT, kTipNLed, left_tip_clr);
  led::Fill(ledg.rightT, kTipNLed, right_tip_clr);
 }

u8_t logify(u8_t val) {
  u8_t lo = val & 0x1F;
  u8_t hi = val >> 5;
  int ret = ((u16_t(log_map[hi]) << 5) + (i16_t(log_map[hi + 1]) - log_map[hi]) * lo);
  return (ret + (1<<4)) >> 5;
}

int main(void)
{
  // Do very basic chip config, in particular setup base clocks.
  Boot(/*target_pdiv=*/2, /*use_internal_32Kclk=*/true);
  SetupRtcClock(/*use_internal_32K=*/true);
  DBG_INIT(Serial::usart0, 115200);
  DBG_LEVEL_HI(APP);

  Adc adc(Adc::VREF_43);
  adc.ConfigurePin(6);    // Read a "mostly random" seed from Analog pin 6 (PD6)
  randomSeed(adc.Read(6));
  Adc::ConfigurePin(VOLT_APIN);
  Adc::ConfigurePin(AMP_APIN);
  
  Spi::spi.SetupSK6812(Spi::PINS_PA47);
  
  SportSensor sport(&Serial::usart2, /*invert=*/true, /*use_alt_pins=*/false);
  i8_t sport_current_idx, sport_volt_idx;
  sport.AddFcs40Sensors(&sport_current_idx, &sport_volt_idx);

  SBus sbus(&Serial::usart1, false);

  // Pwm module use the 3 TCA timers as split 8bit PWM control providing
  // up to 6 channels of PWM.
  Pwm pwm(PORT_D, 1000);  // Select what port to use and freq of PWM
  pwm.Set(LWING_PWM, 0);
  pwm.Enable(LWING_PWM);
  pwm.Set(RWING_PWM, 0);
  pwm.Enable(RWING_PWM);
  
  PORTF.OUT = 1 << 2;  // Set PF2 high, all others low
  PORTF.PIN2CTRL = 0;  // Not inverted, not pullup enabled, no interrupts.
  PORTF.DIR = 1 << 2;  // Pin PF2 as output pin.

  // Enable interrupts (end of init, real program begin).
  sei();

  OffMode();
  Spi::spi.UpdateLeds(&ledg.rightF[0], kTotalLed, 0);

  // Configure blinky LED on pin PF2
  SolidMode();
  pwm.Set(LWING_PWM, 4);
  pwm.Set(RWING_PWM, 4);


  adc.StartRead(VOLT_APIN);

  const bool heartbeat_enable = true;  // True to get heartbeat on board led.
  u8_t update_4 = 255;  // ~1/64 sec  (~16ms each)
  u8_t update_8 = 255;  // ~1/4 sec  (~250ms each )
  while (1) {
    // Update telemetry if we were polled (call very frequently since there
    // is a limited time window to respond when polled).  This uses the last
    // value set for the polled device.
    sport.Run();
    if (sbus.Run()) {
      // Run returns true of a new frame of channel data was received
      ledg.UpdateMode(SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_LEVEL_CH)),
                      SBus::ThreePosSwitch(sbus.GetChannel(LIGHT_MODE_CH)),
                      sbus.GetChannel(LIGHT_BRIGHT_CH),
                      sbus.GetChannel(LIGHT_THROTTLE_CH));
    }

    u32_t now = FastTimeMs();
    u8_t now_4 = now >> 4;
    if (now_4 != update_4) {  // ~60fps
      update_4 = now_4;
	  PulseMode(128);
      Spi::spi.UpdateLeds(&ledg.rightF[0], kTotalLed, 8);
      int volts = adc.FinishRead();
      adc.StartRead(VOLT_APIN);
      // 1024 -> 4.3v * 4 -> 17.4v, scale is 100x
      volts =  (1740L * volts + (1<<9)) >> 10;
      sport.SetSensor(sport_volt_idx, volts);
      sport.SetSensor(sport_current_idx, 0);
      DBG_MD(APP, ("Volts: %d\n", volts));
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

