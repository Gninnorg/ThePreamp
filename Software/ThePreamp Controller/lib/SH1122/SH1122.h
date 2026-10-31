// Minimal driver for 256x64 SH1122 OLED displays on hardware SPI (ESP32 / Arduino).
//
// Replaces U8g2 for this project. The API is a small, call-compatible subset of U8g2's
// full-buffer mode (drawStr, drawBox, drawXBMP, sendBuffer, ...) so existing code keeps working,
// but the frame buffer is 4 bits per pixel, so the controller's 16 grey levels can be used.
//
// Draw colors:  0 = pixel off (grey 0), 1 = current grey level (setGrey, default 15), 2 = XOR (inverts grey).
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "SH1122_fonts.h"

class SH1122
{
public:
  static constexpr uint16_t WIDTH = 256;
  static constexpr uint16_t HEIGHT = 64;
  static constexpr uint16_t BUFFER_SIZE = WIDTH * HEIGHT / 2;

  SH1122(uint8_t csPin, uint8_t dcPin, uint8_t resetPin);

  // Lifecycle / bus
  void setBusClock(uint32_t hz) { _busClock = hz; }
  bool begin();                       // returns false if the frame buffer could not be allocated
  void setPowerSave(bool off);
  void setContrast(uint8_t contrast);

  // Buffer
  void clearBuffer();                 // clears the RAM buffer only
  void clearDisplay();                // clears buffer and display
  void sendBuffer();

  // State
  void setDrawColor(uint8_t color);   // 0, 1 or 2 (XOR)
  void setGrey(uint8_t level);        // grey level (0..15) used when draw color is 1
  void setFontMode(uint8_t mode)  { _fontTransparent = mode != 0; }    // 1 = transparent background
  void setBitmapMode(uint8_t mode) { _bitmapTransparent = mode != 0; } // 1 = transparent background
  void setFont(const uint8_t *font);          // U8g2 1-bit font
  void setFont(const SH1122GreyFont *font);   // anti-aliased font, alpha-blended toward the draw color

  // Primitives
  void drawPixel(int16_t x, int16_t y);
  void drawHLine(int16_t x, int16_t y, int16_t w);
  void drawVLine(int16_t x, int16_t y, int16_t h);
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
  void drawBox(int16_t x, int16_t y, int16_t w, int16_t h);
  void drawFrame(int16_t x, int16_t y, int16_t w, int16_t h);
  void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2);

  // Bitmaps
  void drawXBMP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *bitmap); // 1 bpp, XBM bit order
  void drawGreyBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *bitmap); // 4 bpp, high nibble first, rows padded to whole bytes

  // Grey scale helpers
  void drawGradientBox(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t leftGrey, uint8_t rightGrey);

  // Text (y is the baseline). Strings are treated as single-byte (Latin-1) characters.
  int16_t drawStr(int16_t x, int16_t y, const char *s);
  int16_t getStrWidth(const char *s);

private:
  struct FontInfo
  {
    const uint8_t *data = nullptr;
    uint8_t bitsPer0 = 0, bitsPer1 = 0, bitsWidth = 0, bitsHeight = 0, bitsX = 0, bitsY = 0, bitsDeltaX = 0;
    uint16_t startUpperA = 0, startLowerA = 0;
  };

  struct BitReader
  {
    const uint8_t *ptr;
    uint8_t bitPos;
  };

  void writeCommand(uint8_t cmd);
  void writeCommand(uint8_t cmd, uint8_t arg);
  void beginTransfer();
  void endTransfer();
  void setLevel(int16_t x, int16_t y, uint8_t level);
  void plotColor(int16_t x, int16_t y, uint8_t color);
  void fillSpan(int16_t x, int16_t y, int16_t w, uint8_t color);
  uint8_t colorLevel(uint8_t color) const;
  const uint8_t *findGlyph(uint8_t encoding) const;
  uint8_t readBits(BitReader &r, uint8_t count) const;
  int8_t readSignedBits(BitReader &r, uint8_t count) const;
  int16_t drawGlyph(int16_t x, int16_t y, uint8_t encoding);
  int16_t drawGreyGlyph(int16_t x, int16_t y, uint8_t encoding);
  void blendLevel(int16_t x, int16_t y, uint8_t coverage);

  uint8_t _cs, _dc, _rst;
  uint32_t _busClock = 4000000;
  uint8_t *_buf = nullptr;
  uint8_t _drawColor = 1;
  uint8_t _grey = 15;
  bool _fontTransparent = false;
  bool _bitmapTransparent = false;
  FontInfo _font;
  const SH1122GreyFont *_greyFont = nullptr;
};
