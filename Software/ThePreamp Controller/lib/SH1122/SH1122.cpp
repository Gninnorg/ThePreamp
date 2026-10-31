#include "SH1122.h"

SH1122::SH1122(uint8_t csPin, uint8_t dcPin, uint8_t resetPin) : _cs(csPin), _dc(dcPin), _rst(resetPin) {}

// ---------------------------------------------------------------------------------------------
// Bus / lifecycle
// ---------------------------------------------------------------------------------------------

void SH1122::beginTransfer()
{
  SPI.beginTransaction(SPISettings(_busClock, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
}

void SH1122::endTransfer()
{
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

void SH1122::writeCommand(uint8_t cmd)
{
  digitalWrite(_dc, LOW);
  SPI.transfer(cmd);
}

void SH1122::writeCommand(uint8_t cmd, uint8_t arg)
{
  digitalWrite(_dc, LOW);
  SPI.transfer(cmd);
  SPI.transfer(arg);
}

bool SH1122::begin()
{
  if (_buf == nullptr)
  {
    _buf = static_cast<uint8_t *>(malloc(BUFFER_SIZE));
    if (_buf == nullptr)
      return false;
  }

  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);
  pinMode(_dc, OUTPUT);
  pinMode(_rst, OUTPUT);

  digitalWrite(_rst, HIGH);
  delay(1);
  digitalWrite(_rst, LOW);
  delay(10);
  digitalWrite(_rst, HIGH);
  delay(20);

  beginTransfer();
  delay(1);
  writeCommand(0xAE);          // display off
  writeCommand(0x40);          // display start line
  writeCommand(0xA0);          // remap
  writeCommand(0xC0);          // remap
  writeCommand(0x81, 0x80);    // contrast
  writeCommand(0xA8, 0x3F);    // multiplex ratio 1/64
  writeCommand(0xAD, 0x81);    // built-in DC-DC, 0.6 * 500 kHz
  writeCommand(0xD5, 0x50);    // clock divide ratio / oscillator frequency
  writeCommand(0xD3, 0x00);    // display offset
  writeCommand(0xD9, 0x22);    // pre-charge / discharge period
  writeCommand(0xDB, 0x35);    // VCOM deselect level
  writeCommand(0xDC, 0x35);    // pre-charge output voltage
  writeCommand(0x30);          // discharge level
  delay(1);
  endTransfer();

  clearDisplay();
  setPowerSave(false);
  return true;
}

void SH1122::setPowerSave(bool off)
{
  beginTransfer();
  writeCommand(off ? 0xAE : 0xAF);
  endTransfer();
}

void SH1122::setContrast(uint8_t contrast)
{
  beginTransfer();
  writeCommand(0x81, contrast);
  endTransfer();
}

void SH1122::clearBuffer()
{
  if (_buf != nullptr)
    memset(_buf, 0, BUFFER_SIZE);
}

void SH1122::clearDisplay()
{
  clearBuffer();
  sendBuffer();
}

void SH1122::sendBuffer()
{
  if (_buf == nullptr)
    return;

  constexpr uint16_t rowBytes = WIDTH / 2;
  beginTransfer();
  for (uint8_t row = 0; row < HEIGHT; row++)
  {
    digitalWrite(_dc, LOW);
    SPI.transfer(0xB0);   // row address
    SPI.transfer(row);
    SPI.transfer(0x00);   // column address low
    SPI.transfer(0x10);   // column address high
    digitalWrite(_dc, HIGH);
    SPI.writeBytes(_buf + row * rowBytes, rowBytes);
  }
  endTransfer();
}

// ---------------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------------

void SH1122::setDrawColor(uint8_t color)
{
  _drawColor = color > 2 ? 1 : color;
}

void SH1122::setGrey(uint8_t level)
{
  _grey = level > 15 ? 15 : level;
}

uint8_t SH1122::colorLevel(uint8_t color) const
{
  return color == 0 ? 0 : _grey;
}

// ---------------------------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------------------------

void SH1122::setLevel(int16_t x, int16_t y, uint8_t level)
{
  if (_buf == nullptr || x < 0 || y < 0 || x >= WIDTH || y >= HEIGHT)
    return;
  uint8_t &b = _buf[y * (WIDTH / 2) + (x >> 1)];
  if (x & 1)
    b = (b & 0xF0) | (level & 0x0F);
  else
    b = (b & 0x0F) | (level << 4);
}

void SH1122::plotColor(int16_t x, int16_t y, uint8_t color)
{
  if (_buf == nullptr || x < 0 || y < 0 || x >= WIDTH || y >= HEIGHT)
    return;
  uint8_t &b = _buf[y * (WIDTH / 2) + (x >> 1)];
  const bool low = x & 1;
  const uint8_t level = (color == 2) ? ((low ? b : (b >> 4)) & 0x0F) ^ 0x0F : colorLevel(color);
  if (low)
    b = (b & 0xF0) | level;
  else
    b = (b & 0x0F) | (level << 4);
}

void SH1122::fillSpan(int16_t x, int16_t y, int16_t w, uint8_t color)
{
  if (w <= 0 || y < 0 || y >= HEIGHT)
    return;
  int16_t x1 = x + w;
  if (x < 0)
    x = 0;
  if (x1 > WIDTH)
    x1 = WIDTH;
  for (; x < x1; x++)
    plotColor(x, y, color);
}

void SH1122::drawPixel(int16_t x, int16_t y)
{
  plotColor(x, y, _drawColor);
}

void SH1122::drawHLine(int16_t x, int16_t y, int16_t w)
{
  fillSpan(x, y, w, _drawColor);
}

void SH1122::drawVLine(int16_t x, int16_t y, int16_t h)
{
  for (int16_t i = 0; i < h; i++)
    plotColor(x, y + i, _drawColor);
}

void SH1122::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
  const int16_t dx = abs(x1 - x0);
  const int16_t dy = -abs(y1 - y0);
  const int16_t sx = x0 < x1 ? 1 : -1;
  const int16_t sy = y0 < y1 ? 1 : -1;
  int16_t err = dx + dy;
  for (;;)
  {
    plotColor(x0, y0, _drawColor);
    if (x0 == x1 && y0 == y1)
      break;
    const int16_t e2 = 2 * err;
    if (e2 >= dy)
    {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx)
    {
      err += dx;
      y0 += sy;
    }
  }
}

void SH1122::drawBox(int16_t x, int16_t y, int16_t w, int16_t h)
{
  for (int16_t i = 0; i < h; i++)
    fillSpan(x, y + i, w, _drawColor);
}

void SH1122::drawFrame(int16_t x, int16_t y, int16_t w, int16_t h)
{
  if (w <= 0 || h <= 0)
    return;
  fillSpan(x, y, w, _drawColor);
  if (h > 1)
    fillSpan(x, y + h - 1, w, _drawColor);
  for (int16_t i = 1; i < h - 1; i++)
  {
    plotColor(x, y + i, _drawColor);
    if (w > 1)
      plotColor(x + w - 1, y + i, _drawColor);
  }
}

void SH1122::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
  // Sort vertices by y, then fill scanline by scanline between the long edge and the two short edges.
  if (y0 > y1) { std::swap(y0, y1); std::swap(x0, x1); }
  if (y1 > y2) { std::swap(y1, y2); std::swap(x1, x2); }
  if (y0 > y1) { std::swap(y0, y1); std::swap(x0, x1); }

  auto edgeX = [](int16_t xa, int16_t ya, int16_t xb, int16_t yb, int16_t y) -> int16_t {
    if (yb == ya)
      return xa;
    const int32_t num = static_cast<int32_t>(xb - xa) * (y - ya);
    const int32_t den = yb - ya;
    return xa + static_cast<int16_t>((num * 2 + (num >= 0 ? den : -den)) / (2 * den));
  };

  for (int16_t y = y0; y <= y2; y++)
  {
    const int16_t xa = edgeX(x0, y0, x2, y2, y);
    const int16_t xb = (y < y1 || y1 == y2) ? edgeX(x0, y0, x1, y1, y) : edgeX(x1, y1, x2, y2, y);
    const int16_t left = min(xa, xb);
    const int16_t right = max(xa, xb);
    fillSpan(left, y, right - left + 1, _drawColor);
  }
}

// ---------------------------------------------------------------------------------------------
// Bitmaps and grey scale
// ---------------------------------------------------------------------------------------------

void SH1122::drawXBMP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *bitmap)
{
  const int16_t stride = (w + 7) / 8;
  const uint8_t bg = _drawColor == 0 ? 1 : 0;
  for (int16_t row = 0; row < h; row++)
  {
    for (int16_t col = 0; col < w; col++)
    {
      const bool on = bitmap[row * stride + (col >> 3)] & (1 << (col & 7));
      if (on)
        plotColor(x + col, y + row, _drawColor);
      else if (!_bitmapTransparent)
        plotColor(x + col, y + row, bg);
    }
  }
}

void SH1122::drawGreyBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *bitmap)
{
  const int16_t stride = (w + 1) / 2;
  for (int16_t row = 0; row < h; row++)
  {
    for (int16_t col = 0; col < w; col++)
    {
      const uint8_t b = bitmap[row * stride + (col >> 1)];
      setLevel(x + col, y + row, (col & 1) ? (b & 0x0F) : (b >> 4));
    }
  }
}

void SH1122::drawGradientBox(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t leftGrey, uint8_t rightGrey)
{
  if (w <= 0)
    return;
  for (int16_t col = 0; col < w; col++)
  {
    const int16_t level = w > 1 ? leftGrey + ((static_cast<int32_t>(rightGrey) - leftGrey) * col + (w - 1) / 2) / (w - 1) : leftGrey;
    for (int16_t row = 0; row < h; row++)
      setLevel(x + col, y + row, static_cast<uint8_t>(level));
  }
}

// ---------------------------------------------------------------------------------------------
// Fonts (U8g2 compressed bitmap font format)
// ---------------------------------------------------------------------------------------------

void SH1122::setFont(const SH1122GreyFont *font)
{
  _greyFont = font;
}

void SH1122::setFont(const uint8_t *font)
{
  _greyFont = nullptr;
  _font.data = font;
  if (font == nullptr)
    return;
  _font.bitsPer0 = font[2];
  _font.bitsPer1 = font[3];
  _font.bitsWidth = font[4];
  _font.bitsHeight = font[5];
  _font.bitsX = font[6];
  _font.bitsY = font[7];
  _font.bitsDeltaX = font[8];
  _font.startUpperA = (font[17] << 8) | font[18];
  _font.startLowerA = (font[19] << 8) | font[20];
}

uint8_t SH1122::readBits(BitReader &r, uint8_t count) const
{
  uint8_t value = *r.ptr >> r.bitPos;
  uint8_t end = r.bitPos + count;
  if (end >= 8)
  {
    r.ptr++;
    value |= *r.ptr << (8 - r.bitPos);
    end -= 8;
  }
  r.bitPos = end;
  return value & ((1U << count) - 1);
}

int8_t SH1122::readSignedBits(BitReader &r, uint8_t count) const
{
  return static_cast<int8_t>(readBits(r, count)) - static_cast<int8_t>(1 << (count - 1));
}

const uint8_t *SH1122::findGlyph(uint8_t encoding) const
{
  const uint8_t *p = _font.data + 23;
  if (encoding >= 'a')
    p += _font.startLowerA;
  else if (encoding >= 'A')
    p += _font.startUpperA;

  while (p[1] != 0)
  {
    if (p[0] == encoding)
      return p + 2;
    p += p[1];
  }
  return nullptr;
}

int16_t SH1122::drawGlyph(int16_t x, int16_t y, uint8_t encoding)
{
  const uint8_t *data = _font.data != nullptr ? findGlyph(encoding) : nullptr;
  if (data == nullptr)
    return 0;

  BitReader r{data, 0};
  const uint8_t w = readBits(r, _font.bitsWidth);
  const uint8_t h = readBits(r, _font.bitsHeight);
  const int8_t gx = readSignedBits(r, _font.bitsX);
  const int8_t gy = readSignedBits(r, _font.bitsY);
  const int8_t advance = readSignedBits(r, _font.bitsDeltaX);
  if (w == 0)
    return advance;

  const int16_t left = x + gx;
  const int16_t top = y - (h + gy);
  const uint8_t fg = _drawColor;
  const uint8_t bg = fg == 0 ? 1 : 0;
  uint8_t lx = 0, ly = 0;

  auto run = [&](uint8_t len, bool foreground) {
    while (len > 0)
    {
      const uint8_t chunk = min<uint8_t>(len, w - lx);
      if (foreground)
        fillSpan(left + lx, top + ly, chunk, fg);
      else if (!_fontTransparent)
        fillSpan(left + lx, top + ly, chunk, bg);
      if (len < w - lx)
      {
        lx += len;
        return;
      }
      len -= w - lx;
      lx = 0;
      ly++;
    }
  };

  for (;;)
  {
    const uint8_t zeros = readBits(r, _font.bitsPer0);
    const uint8_t ones = readBits(r, _font.bitsPer1);
    do
    {
      run(zeros, false);
      run(ones, true);
    } while (readBits(r, 1) != 0);
    if (ly >= h)
      break;
  }
  return advance;
}

void SH1122::blendLevel(int16_t x, int16_t y, uint8_t coverage)
{
  if (_buf == nullptr || x < 0 || y < 0 || x >= WIDTH || y >= HEIGHT)
    return;
  uint8_t &b = _buf[y * (WIDTH / 2) + (x >> 1)];
  const bool low = x & 1;
  const int16_t current = low ? (b & 0x0F) : (b >> 4);
  const int16_t target = _drawColor == 0 ? 0 : (_drawColor == 2 ? 15 - current : _grey);
  const int16_t diff = (target - current) * coverage;
  const int16_t level = current + (diff >= 0 ? (diff + 7) / 15 : -((-diff + 7) / 15));
  if (low)
    b = (b & 0xF0) | level;
  else
    b = (b & 0x0F) | (level << 4);
}

int16_t SH1122::drawGreyGlyph(int16_t x, int16_t y, uint8_t encoding)
{
  const SH1122GreyFont &f = *_greyFont;
  if (encoding < f.first || encoding > f.last)
    return f.advance;

  const SH1122GreyGlyph &g = f.glyphs[encoding - f.first];
  const uint8_t stride = (g.w + 1) / 2;
  for (uint8_t row = 0; row < g.h; row++)
  {
    const uint8_t *src = f.bitmap + g.offset + row * stride;
    for (uint8_t col = 0; col < g.w; col++)
    {
      const uint8_t coverage = (col & 1) ? (src[col >> 1] & 0x0F) : (src[col >> 1] >> 4);
      if (coverage != 0)
        blendLevel(x + g.x + col, y - g.top + row, coverage);
    }
  }
  return f.advance;
}

int16_t SH1122::drawStr(int16_t x, int16_t y, const char *s)
{
  const int16_t start = x;
  for (; *s; s++)
    x += _greyFont != nullptr ? drawGreyGlyph(x, y, static_cast<uint8_t>(*s)) : drawGlyph(x, y, static_cast<uint8_t>(*s));
  return x - start;
}

int16_t SH1122::getStrWidth(const char *s)
{
  if (_greyFont != nullptr)
    return strlen(s) * _greyFont->advance;
  if (_font.data == nullptr)
    return 0;

  int16_t width = 0;
  int16_t lastAdvance = 0, lastWidth = 0, lastOffset = 0;
  for (; *s; s++)
  {
    const uint8_t *data = findGlyph(static_cast<uint8_t>(*s));
    if (data == nullptr)
      continue;
    BitReader r{data, 0};
    lastWidth = readBits(r, _font.bitsWidth);
    readBits(r, _font.bitsHeight);
    lastOffset = readSignedBits(r, _font.bitsX);
    readSignedBits(r, _font.bitsY);
    lastAdvance = readSignedBits(r, _font.bitsDeltaX);
    width += lastAdvance;
  }

  // Like U8g2, the last glyph counts with its real pixel extent instead of its advance.
  if (lastWidth != 0)
    width += lastWidth + lastOffset - lastAdvance;
  return width;
}
