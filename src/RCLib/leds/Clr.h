#ifndef _LED_CLR_
#define _LED_CLR_

#include "Rgb.h"

namespace led {

namespace clr {
const RGB white(0xFF);
const RGB black(0x00);
const RGB red(0xFF, 0x00, 0x00);
const RGB yellow(0xFF, 0xFF, 0x00);
const RGB green(0x00, 0xFF, 0x00);
const RGB cyan(0x00, 0xFF, 0xFF);
const RGB blue(0x00, 0x00, 0xFF);
const RGB purple(0xFF, 0x00, 0xFF);
}  // namespace clr

namespace wclr {
const RGBW white(0xFF);
const RGBW black(0x00);
const RGBW whiteFull(0xFF, 0xFF);  // white led + RGB
const RGBW red(0xFF, 0x00, 0x00);
const RGBW yellow(0xFF, 0xFF, 0x00);
const RGBW green(0x00, 0xFF, 0x00);
const RGBW cyan(0x00, 0xFF, 0xFF);
const RGBW blue(0x00, 0x00, 0xFF);
const RGBW purple(0xFF, 0x00, 0xFF);
}  // namespace wclr

}  // namespace led

#endif  // _LED_CLR_
