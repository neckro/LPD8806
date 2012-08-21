/*
Arduino library to control LPD8806-based RGB LED Strips
Copyright (C) Adafruit Industries
MIT license

Clearing up some misconceptions about how the LPD8806 drivers work:

The LPD8806 is not a FIFO shift register.  The first data out controls the
LED *closest* to the processor (unlike a typical shift register, where the
first data out winds up at the *furthest* LED).  Each LED driver 'fills up'
with data and then passes through all subsequent bytes until a latch
condition takes place.  This is actually pretty common among LED drivers.

All color data bytes have the high bit (128) set, with the remaining
seven bits containing a brightness value (0-127).  A byte with the high
bit clear has special meaning (explained later).

The rest gets bizarre...

The LPD8806 does not perform an in-unison latch (which would display the
newly-transmitted data all at once).  Rather, each individual byte (even
the separate G, R, B components of each LED) is latched AS IT ARRIVES...
or more accurately, as the first bit of the subsequent byte arrives and
is passed through.  So the strip actually refreshes at the speed the data
is issued, not instantaneously (this can be observed by greatly reducing
the data rate).  This has implications for POV displays and light painting
applications.  The 'subsequent' rule also means that at least one extra
byte must follow the last pixel, in order for the final blue LED to latch.

To reset the pass-through behavior and begin sending new data to the start
of the strip, a number of zero bytes must be issued (remember, all color
data bytes have the high bit set, thus are in the range 128 to 255, so the
zero is 'special').  This should be done before each full payload of color
values to the strip.  Curiously, zero bytes can only travel one meter (32
LEDs) down the line before needing backup; the next meter requires an
extra zero byte, and so forth.  Longer strips will require progressively
more zeros.  *(see note below)

In the interest of efficiency, it's possible to combine the former EOD
extra latch byte and the latter zero reset...the same data can do double
duty, latching the last blue LED while also resetting the strip for the
next payload.

So: reset byte(s) of suitable length are issued once at startup to 'prime'
the strip to a known ready state.  After each subsequent LED color payload,
these reset byte(s) are then issued at the END of each payload, both to
latch the last LED and to prep the strip for the start of the next payload
(even if that data does not arrive immediately).  This avoids a tiny bit
of latency as the new color payload can begin issuing immediately on some
signal, such as a timer or GPIO trigger.

Technically these zero byte(s) are not a latch, as the color data (save
for the last byte) is already latched.  It's a start-of-data marker, or
an indicator to clear the thing-that's-not-a-shift-register.  But for
conversational consistency with other LED drivers, we'll refer to it as
a 'latch' anyway.

* This has been validated independently with multiple customers'
  hardware.  Please do not report as a bug or issue pull requests for
  this.  Fewer zeros sometimes gives the *illusion* of working, the first
  payload will correctly load and latch, but subsequent frames will drop
  data at the end.  The data shortfall won't always be visually apparent
  depending on the color data loaded on the prior and subsequent frames.
  Tested.  Confirmed.  Fact.
*/

#include "SPI.h"
#include "LPD8806.h"

/*****************************************************************************/

// gamma lookup table, stolen from LEDbeltKit_alt example
PROGMEM prog_uchar gammaTable[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,
    4,  4,  4,  4,  5,  5,  5,  5,  5,  6,  6,  6,  6,  6,  7,  7,
    7,  7,  7,  8,  8,  8,  8,  9,  9,  9,  9, 10, 10, 10, 10, 11,
   11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 15, 15, 16, 16,
   16, 17, 17, 17, 18, 18, 18, 19, 19, 20, 20, 21, 21, 21, 22, 22,
   23, 23, 24, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30,
   30, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 37, 37, 38, 38, 39,
   40, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48, 49, 50,
   50, 51, 52, 52, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60, 61, 62,
   62, 63, 64, 65, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 74, 75,
   76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91,
   92, 93, 94, 95, 96, 97, 98, 99,100,101,102,104,105,106,107,108,
  109,110,111,113,114,115,116,117,118,120,121,122,123,125,126,127
};

// Constructor for use with hardware SPI (specific clock/data pins):
LPD8806::LPD8806(uint16_t n) {
  pixels = NULL;
  begun  = false;
  updateLength(n);
  updatePins();
}

// Constructor for use with arbitrary clock/data pins:
LPD8806::LPD8806(uint16_t n, uint8_t dpin, uint8_t cpin) {
  pixels = NULL;
  begun  = false;
  updateLength(n);
  updatePins(dpin, cpin);
}

// via Michael Vogt/neophob: empty constructor is used when strip length
// isn't known at compile-time; situations where program config might be
// read from internal flash memory or an SD card, or arrive via serial
// command.  If using this constructor, MUST follow up with updateLength()
// and updatePins() to establish the strip length and output pins!
LPD8806::LPD8806(void) {
  numLEDs = numBytes = 0;
  pixels  = NULL;
  begun   = false;
  updatePins(); // Must assume hardware SPI until pins are set
}

// Activate hard/soft SPI as appropriate:
void LPD8806::begin(void) {
  if (hardwareSPI) startSPI();
    else startBitbang();
  begun = true;
}

// Change pin assignments post-constructor, switching to hardware SPI:
void LPD8806::updatePins(void) {
  hardwareSPI = true;
  datapin     = clkpin = 0;
  // If begin() was previously invoked, init the SPI hardware now:
  if(begun == true) startSPI();
  // Otherwise, SPI is NOT initted until begin() is explicitly called.

  // Note: any prior clock/data pin directions are left as-is and are
  // NOT restored as inputs!
}

// Change pin assignments post-constructor, using arbitrary pins:
void LPD8806::updatePins(uint8_t dpin, uint8_t cpin) {
  datapin     = dpin;
  clkpin      = cpin;
  clkport = dataport = 0;
  clkpinmask = datapinmask = 0;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__) || (__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  clkport     = portOutputRegister(digitalPinToPort(cpin));
  clkpinmask  = digitalPinToBitMask(cpin);
  dataport    = portOutputRegister(digitalPinToPort(dpin));
  datapinmask = digitalPinToBitMask(dpin);
#endif

  if (begun) { // If begin() was previously invoked...
    // If previously using hardware SPI, turn that off:
    if (hardwareSPI) SPI.end();
    startBitbang(); // Regardless, now enable 'soft' SPI outputs
  } // Otherwise, pins are not set to outputs until begin() is called.
  // Note: any prior clock/data pin directions are left as-is and are
  // NOT restored as inputs!

  hardwareSPI = false;
}

#ifndef SPI_CLOCK_DIV8
  #define SPI_CLOCK_DIV8 4
#endif

// Enable SPI hardware and set up protocol details:
void LPD8806::startSPI(void) {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2 MHz
  // SPI bus is run at 2MHz.  Although the LPD8806 should, in theory,
  // work up to 20MHz, the unshielded wiring from the Arduino is more
  // susceptible to interference.  Experiment and see what you get.

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__) || (__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

  // Issue initial latch/reset to strip:
  SPDR = 0; // Issue initial byte
  for(uint16_t i=((numLEDs+31)/32)-1; i>0; i--) {
    while(!(SPSR & (1<<SPIF))); // Wait for prior byte out
    SPDR = 0;                   // Issue next byte
  }
#else
  SPI.transfer(0);
  for(uint16_t i=((numLEDs+31)/32)-1; i>0; i--) {
    SPI.transfer(0);
  }
#endif
}

// Enable software SPI pins and issue initial latch:
void LPD8806::startBitbang() {
  pinMode(datapin, OUTPUT);
  pinMode(clkpin , OUTPUT);
  if (dataport != 0) {
    // use low level bitbanging when we can
    *dataport &= ~datapinmask; // Data is held low throughout (latch = 0)
    for(uint16_t i=((numLEDs+31)/32)*8; i>0; i--) {
      *clkport |=  clkpinmask;
      *clkport &= ~clkpinmask;
    }
  } else {
    // can't do low level bitbanging, revert to digitalWrite
    digitalWrite(datapin, LOW);
    // Send initial latch/reset
    for (uint16_t i=((numLEDs+31)/32)*8; i>0; i--) {
      digitalWrite(clkpin, HIGH);
      digitalWrite(clkpin, LOW);
    }
  }
}

// Change strip length (see notes with empty constructor, above):
void LPD8806::updateLength(uint16_t n) {
  numLEDs  = n;
  numBytes = numLEDs * 3; // 3 bytes per pixel
  latchBytes = (n + 31) / 32;
  // Free existing data (if any)
  free(pixels);

  // Allocate and zero new data
  if (NULL == (pixels = (uint8_t *)calloc(numBytes, 1))) {
    // Allocation failed!
    numLEDs = numBytes = latchBytes = 0; 
  }
  // 'begun' state does not change -- pins retain prior modes
}

uint16_t LPD8806::numPixels(void) {
  return numLEDs;
}

// This is how data is pushed to the strip.  Unfortunately, the company
// that makes the chip didnt release the protocol document or you need
// to sign an NDA or something stupid like that, but we reverse engineered
// this from a strip controller and it seems to work very nicely!
void LPD8806::show(void) {
  uint8_t *ptr = pixels;
  uint16_t i = numBytes + latchBytes;
  uint8_t bit, val;

  while (i) {
    if (i-- > latchBytes) {
      // color byte with high bit set
      val = 0x00 | (*ptr++); //testing
      //val = 0x80 | pgm_read_byte(&gammaTable[*ptr++]);
    } else {
      // latch byte
      val = 0;
    }
    if (hardwareSPI) {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__) || (__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
      while (!(SPSR & (1 << SPIF))); // Wait for prior byte out
      SPDR = val; // Issue new byte
#else
      SPI.transfer(val);
#endif
    } else {
      for (bit=0x80; bit; bit >>= 1) {
        if (dataport == 0) {
          // bitbang
          if (val & bit) {
            digitalWrite(datapin, HIGH);
          } else {
            digitalWrite(datapin, LOW);
          }
          digitalWrite(clkpin, HIGH);
          digitalWrite(clkpin, LOW);
        } else {
          // SPI
          if (val & bit) {
            *dataport |= datapinmask;
          } else {
            *dataport &= ~datapinmask;
          }
          *clkport |=  clkpinmask;
          *clkport &= ~clkpinmask;
        }
      }
    }
  }
}

// Clear all pixels
void LPD8806::clear(void) {
  memset(pixels, 0x00, numLEDs * 3);
}

// Convert separate R, G, B into combined RGB
uint32_t LPD8806::Color(uint8_t r, uint8_t g, uint8_t b) {
  return (uint32_t)r << 16 |
         (uint32_t)g <<  8 |
         (uint32_t)b;
}

// Query color from previously-set pixel (returns 32-bit RGB value)
uint32_t LPD8806::getPixelColor(uint16_t n) {
  if (n < numLEDs) {
    return ((uint32_t)pixels[(n*3)+0] <<  8) |
           ((uint32_t)pixels[(n*3)+1] << 16) |
            (uint32_t)pixels[(n*3)+2];
  }
  return 0; // Pixel # is out of bounds
}

// Set pixel color from 32-bit RGB value -- high byte is ignored
void LPD8806::setPixelColor(uint16_t pos, uint32_t color) {
  if (pos < numLEDs) {
    uint8_t *p = &pixels[pos * 3];
    *p++ = color >> 8;  // g
    *p++ = color >> 16; // r
    *p++ = color;       // b
  }
}

// Set pixel color from separate R, G, B components
void LPD8806::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if (n < numLEDs) {
    uint8_t *p = &pixels[n * 3];
    *p++ = g;
    *p++ = r;
    *p++ = b;
  }
}

// Blend new color (32-bit RGB) with existing color at specified position
void LPD8806::blendPixel(uint16_t pos, uint32_t color, uint8_t alpha) {
  if (alpha > 0 && pos < numLEDs) {
    uint8_t* sp = (uint8_t*) &color;
    uint8_t* dp = &pixels[pos * 3];
    *(dp+1) = (255 - (255 - *(dp+1)) * (255 - (*sp++ * alpha / 255)) / 255); // r
    *(dp+0) = (255 - (255 - *(dp+0)) * (255 - (*sp++ * alpha / 255)) / 255); // g
    *(dp+2) = (255 - (255 - *(dp+2)) * (255 - (*sp++ * alpha / 255)) / 255); // b
  }
}

// Render a "pixel" between two pixels, 256 subpixels per pixel
// Position ranges from 0 to (numLEDs >> 8).
// Note that the last 256 subpixels in this range "wrap around" to pixel #0.
// This is so that subpixel 0 = pixel 0.
// If you don't want wrapping, don't use the last 256 subpixels!
void LPD8806::blendSubpixel(uint32_t pos, uint32_t color, uint8_t alpha) {
  uint8_t a = (alpha * (pos % 256)) / 255;
  blendPixel( (pos >> 8)      % numLEDs, color, 255 - a);
  /* We get a slight boundary problem between full pixels which results in a
  pixel being shown with a=255 twice in a row (e.g. at pos=255 and pos=256).
  Since this would make any animations hitch briefly, offset the second pixel
  by -1 so the hitch happens (invisibly) at a=0 instead. */
  if (a > 1) a--;
  blendPixel(((pos >> 8) + 1) % numLEDs, color, a);
}
