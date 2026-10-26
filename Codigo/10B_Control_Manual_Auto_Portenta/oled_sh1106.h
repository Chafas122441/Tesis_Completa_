/**
 * @file oled_sh1106.h
 * @brief Control de la pantalla SH1106 y presentacion del menu de operacion.
 */
#ifndef OLED_SH1106_H
#define OLED_SH1106_H

#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>
#include <string.h>

/* Driver local y deliberadamente pequeno para SH1106 128x64. Evita que la
   etapa dependa de una biblioteca grafica externa. La pantalla es opcional:
   cualquier error I2C deja al robot y al diagnostico Serial funcionando. */
class OledSh1106 {
 public:
  bool begin(TwoWire &wire = Wire) {
    wire_ = &wire;
    wire_->begin();
    wire_->setClock(400000u);
    const uint8_t candidates[2] = {0x3Cu, 0x3Du};
    for (uint8_t candidate : candidates) {
      wire_->beginTransmission(candidate);
      if (wire_->endTransmission() == 0u) {
        address_ = candidate;
        break;
      }
    }
    if (address_ == 0u) return false;
    const uint8_t init[] = {
      0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
      0xAD, 0x8B, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F,
      0xD9, 0x22, 0xDB, 0x35, 0xA4, 0xA6, 0xAF
    };
    for (uint8_t command : init) {
      if (!sendCommand(command)) {
        address_ = 0u;
        return false;
      }
    }
    clear();
    return flush();
  }

  bool available() const { return address_ != 0u; }
  uint8_t address() const { return address_; }

  void clear() { memset(buffer_, 0, sizeof(buffer_)); }

  void line(uint8_t row, const char *text, bool inverse = false) {
    if (row >= 8u || text == nullptr) return;
    memset(&buffer_[row * 128u], inverse ? 0xFF : 0x00, 128u);
    uint8_t column = 1u;
    while (*text != '\0' && column + 5u < 128u) {
      uint8_t glyph[5];
      getGlyph(*text++, glyph);
      for (uint8_t i = 0u; i < 5u; ++i)
        buffer_[row * 128u + column++] = inverse
          ? static_cast<uint8_t>(~glyph[i]) : glyph[i];
      buffer_[row * 128u + column++] = inverse ? 0xFFu : 0x00u;
    }
  }

  bool flush() {
    if (!available()) return false;
    for (uint8_t page = 0u; page < 8u; ++page) {
      if (!flushPage(page)) return false;
    }
    return true;
  }

  /* Permite repartir una pantalla completa entre varios ciclos de control.
     Enviar los 1024 bytes de una vez bloqueaba Wire durante decenas de ms y
     perturbaba la cadencia del movimiento manual. */
  bool flushPage(uint8_t page) {
    if (!available() || page >= 8u) return false;
    if (!sendCommand(static_cast<uint8_t>(0xB0u + page)) ||
        !sendCommand(0x02u) || !sendCommand(0x10u)) return fail();
    for (uint8_t offset = 0u; offset < 128u; offset += 16u) {
      wire_->beginTransmission(address_);
      wire_->write(static_cast<uint8_t>(0x40u));
      wire_->write(&buffer_[page * 128u + offset], 16u);
      if (wire_->endTransmission() != 0u) return fail();
    }
    return true;
  }

 private:
  TwoWire *wire_ = nullptr;
  uint8_t address_ = 0u;
  uint8_t buffer_[1024]{};

  bool fail() { address_ = 0u; return false; }

  bool sendCommand(uint8_t command) {
    if (wire_ == nullptr || address_ == 0u) return false;
    wire_->beginTransmission(address_);
    wire_->write(static_cast<uint8_t>(0x00u));
    wire_->write(command);
    return wire_->endTransmission() == 0u;
  }

  static void copyGlyph(uint8_t out[5], uint8_t a, uint8_t b, uint8_t c,
                        uint8_t d, uint8_t e) {
    out[0] = a; out[1] = b; out[2] = c; out[3] = d; out[4] = e;
  }

  static void getGlyph(char input, uint8_t out[5]) {
    const char c = static_cast<char>(toupper(static_cast<unsigned char>(input)));
    switch (c) {
      case 'A': copyGlyph(out,0x7E,0x11,0x11,0x11,0x7E); break;
      case 'B': copyGlyph(out,0x7F,0x49,0x49,0x49,0x36); break;
      case 'C': copyGlyph(out,0x3E,0x41,0x41,0x41,0x22); break;
      case 'D': copyGlyph(out,0x7F,0x41,0x41,0x22,0x1C); break;
      case 'E': copyGlyph(out,0x7F,0x49,0x49,0x49,0x41); break;
      case 'F': copyGlyph(out,0x7F,0x09,0x09,0x09,0x01); break;
      case 'G': copyGlyph(out,0x3E,0x41,0x49,0x49,0x7A); break;
      case 'H': copyGlyph(out,0x7F,0x08,0x08,0x08,0x7F); break;
      case 'I': copyGlyph(out,0x00,0x41,0x7F,0x41,0x00); break;
      case 'J': copyGlyph(out,0x20,0x40,0x41,0x3F,0x01); break;
      case 'K': copyGlyph(out,0x7F,0x08,0x14,0x22,0x41); break;
      case 'L': copyGlyph(out,0x7F,0x40,0x40,0x40,0x40); break;
      case 'M': copyGlyph(out,0x7F,0x02,0x0C,0x02,0x7F); break;
      case 'N': copyGlyph(out,0x7F,0x04,0x08,0x10,0x7F); break;
      case 'O': copyGlyph(out,0x3E,0x41,0x41,0x41,0x3E); break;
      case 'P': copyGlyph(out,0x7F,0x09,0x09,0x09,0x06); break;
      case 'Q': copyGlyph(out,0x3E,0x41,0x51,0x21,0x5E); break;
      case 'R': copyGlyph(out,0x7F,0x09,0x19,0x29,0x46); break;
      case 'S': copyGlyph(out,0x46,0x49,0x49,0x49,0x31); break;
      case 'T': copyGlyph(out,0x01,0x01,0x7F,0x01,0x01); break;
      case 'U': copyGlyph(out,0x3F,0x40,0x40,0x40,0x3F); break;
      case 'V': copyGlyph(out,0x1F,0x20,0x40,0x20,0x1F); break;
      case 'W': copyGlyph(out,0x3F,0x40,0x38,0x40,0x3F); break;
      case 'X': copyGlyph(out,0x63,0x14,0x08,0x14,0x63); break;
      case 'Y': copyGlyph(out,0x07,0x08,0x70,0x08,0x07); break;
      case 'Z': copyGlyph(out,0x61,0x51,0x49,0x45,0x43); break;
      case '0': copyGlyph(out,0x3E,0x51,0x49,0x45,0x3E); break;
      case '1': copyGlyph(out,0x00,0x42,0x7F,0x40,0x00); break;
      case '2': copyGlyph(out,0x42,0x61,0x51,0x49,0x46); break;
      case '3': copyGlyph(out,0x21,0x41,0x45,0x4B,0x31); break;
      case '4': copyGlyph(out,0x18,0x14,0x12,0x7F,0x10); break;
      case '5': copyGlyph(out,0x27,0x45,0x45,0x45,0x39); break;
      case '6': copyGlyph(out,0x3C,0x4A,0x49,0x49,0x30); break;
      case '7': copyGlyph(out,0x01,0x71,0x09,0x05,0x03); break;
      case '8': copyGlyph(out,0x36,0x49,0x49,0x49,0x36); break;
      case '9': copyGlyph(out,0x06,0x49,0x49,0x29,0x1E); break;
      case '-': copyGlyph(out,0x08,0x08,0x08,0x08,0x08); break;
      case '+': copyGlyph(out,0x08,0x08,0x3E,0x08,0x08); break;
      case '.': copyGlyph(out,0x00,0x60,0x60,0x00,0x00); break;
      case ',': copyGlyph(out,0x00,0x40,0x20,0x00,0x00); break;
      case ':': copyGlyph(out,0x00,0x36,0x36,0x00,0x00); break;
      case '/': copyGlyph(out,0x20,0x10,0x08,0x04,0x02); break;
      case '[': copyGlyph(out,0x00,0x7F,0x41,0x41,0x00); break;
      case ']': copyGlyph(out,0x00,0x41,0x41,0x7F,0x00); break;
      case '>': copyGlyph(out,0x00,0x41,0x22,0x14,0x08); break;
      case '<': copyGlyph(out,0x08,0x14,0x22,0x41,0x00); break;
      case '=': copyGlyph(out,0x14,0x14,0x14,0x14,0x14); break;
      case '!': copyGlyph(out,0x00,0x00,0x5F,0x00,0x00); break;
      case '?': copyGlyph(out,0x02,0x01,0x51,0x09,0x06); break;
      default: copyGlyph(out,0x00,0x00,0x00,0x00,0x00); break;
    }
  }
};

#endif
