#pragma once

#include <stdint.h>

// Fonts in U8g2 compressed bitmap format (copied from U8g2, names kept for compatibility).
extern const uint8_t u8g2_font_luBS18_tf[];
extern const uint8_t u8g2_font_profont10_mf[];

// Anti-aliased monospace fonts: 4 bits per pixel (coverage), see tools/make_grey_fonts.py.
struct SH1122GreyGlyph
{
  uint16_t offset;  // into the bitmap, rows padded to whole bytes, high nibble first
  uint8_t w, h;     // 0 for empty glyphs such as space
  int8_t x;         // left edge relative to the pen position
  int8_t top;       // rows above the baseline
};

struct SH1122GreyFont
{
  uint8_t first, last;  // character range covered by 'glyphs'
  uint8_t advance;      // monospace cell width
  const SH1122GreyGlyph *glyphs;
  const uint8_t *bitmap;
};

extern const SH1122GreyFont SH1122_FONT_INB24;  // balance, trigger countdown
extern const SH1122GreyFont SH1122_FONT_INB42;  // input name
extern const SH1122GreyFont SH1122_FONT_INB63;  // volume
