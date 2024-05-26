/* Copyright 2023 splitkb.com <support@splitkb.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "progmem.h"

// NOTE: This file is a copy of `crkbd/soundmonster/glcdfont.c`

// Corne 8x6 font with QMK Firmware Logo
// Online editor: https://helixfonteditor.netlify.com/
// See also: https://github.com/soundmonster/glcdfont_converter

const unsigned char font[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x04, 0xF8, 0x00, 0x00, 0xF8, 0x04,
  0x20, 0x1F, 0x00, 0x00, 0x1F, 0x20,
  0xFC, 0xF8, 0x00, 0x00, 0xF8, 0x04,
  0x3F, 0x1F, 0x00, 0x00, 0x1F, 0x20,
  0x04, 0xF8, 0x00, 0x00, 0xF8, 0xFC,
  0x20, 0x1F, 0x00, 0x00, 0x1F, 0x3F,
  0xFC, 0xF8, 0x00, 0x00, 0xF8, 0xFC,
  0x3F, 0x1F, 0x00, 0x00, 0x1F, 0x3F,
  0x00, 0x18, 0x24, 0x18, 0x00, 0x00,
  0xFF, 0xE7, 0xDB, 0xE7, 0xFF, 0x00,
  0x30, 0x48, 0x3A, 0x06, 0x0E, 0x00,
  0x26, 0x29, 0x79, 0x29, 0x26, 0x00,
  0x40, 0x7F, 0x05, 0x05, 0x07, 0x00,
  0x40, 0x7F, 0x05, 0x25, 0x3F, 0x00,
  0x5A, 0x3C, 0xE7, 0x3C, 0x5A, 0x00,
  0x7F, 0x3E, 0x1C, 0x1C, 0x08, 0x00,
  0x08, 0x1C, 0x1C, 0x3E, 0x7F, 0x00,
  0x14, 0x22, 0x7F, 0x22, 0x14, 0x00,
  0x5F, 0x5F, 0x00, 0x5F, 0x5F, 0x00,
  0x06, 0x09, 0x7F, 0x01, 0x7F, 0x00,
  0x00, 0x66, 0x89, 0x95, 0x6A, 0x00,
  0x60, 0x60, 0x60, 0x60, 0x60, 0x00,
  0x94, 0xA2, 0xFF, 0xA2, 0x94, 0x00,
  0x08, 0x04, 0x7E, 0x04, 0x08, 0x00,
  0x10, 0x20, 0x7E, 0x20, 0x10, 0x00,
  0x08, 0x08, 0x2A, 0x1C, 0x08, 0x00,
  0x08, 0x1C, 0x2A, 0x08, 0x08, 0x00,
  0x1E, 0x10, 0x10, 0x10, 0x10, 0x00,
  0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 0x00,
  0x30, 0x38, 0x3E, 0x38, 0x30, 0x00,
  0x06, 0x0E, 0x3E, 0x0E, 0x06, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x5F, 0x00, 0x00, 0x00,
  0x00, 0x07, 0x00, 0x07, 0x00, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00,
  0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00,
  0x23, 0x13, 0x08, 0x64, 0x62, 0x00,
  0x36, 0x49, 0x56, 0x20, 0x50, 0x00,
  0x00, 0x08, 0x07, 0x03, 0x00, 0x00,
  0x00, 0x1C, 0x22, 0x41, 0x00, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x00,
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x00,
  0x08, 0x08, 0x3E, 0x08, 0x08, 0x00,
  0x00, 0x80, 0x70, 0x30, 0x00, 0x00,
  0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
  0x00, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x00,
  0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00,
  0x00, 0x42, 0x7F, 0x40, 0x00, 0x00,
  0x72, 0x49, 0x49, 0x49, 0x46, 0x00,
  0x21, 0x41, 0x49, 0x4D, 0x33, 0x00,
  0x18, 0x14, 0x12, 0x7F, 0x10, 0x00,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x00,
  0x3C, 0x4A, 0x49, 0x49, 0x31, 0x00,
  0x41, 0x21, 0x11, 0x09, 0x07, 0x00,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x00,
  0x46, 0x49, 0x49, 0x29, 0x1E, 0x00,
  0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
  0x00, 0x40, 0x34, 0x00, 0x00, 0x00,
  0x00, 0x08, 0x14, 0x22, 0x41, 0x00,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x00,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x00,
  0x02, 0x01, 0x59, 0x09, 0x06, 0x00,
  0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x00,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00,
  0x7F, 0x49, 0x49, 0x49, 0x36, 0x00,
  0x3E, 0x41, 0x41, 0x41, 0x22, 0x00,
  0x7F, 0x41, 0x41, 0x41, 0x3E, 0x00,
  0x7F, 0x49, 0x49, 0x49, 0x41, 0x00,
  0x7F, 0x09, 0x09, 0x09, 0x01, 0x00,
  0x3E, 0x41, 0x41, 0x51, 0x73, 0x00,
  0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00,
  0x00, 0x41, 0x7F, 0x41, 0x00, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x00,
  0x7F, 0x08, 0x14, 0x22, 0x41, 0x00,
  0x7F, 0x40, 0x40, 0x40, 0x40, 0x00,
  0x7F, 0x02, 0x1C, 0x02, 0x7F, 0x00,
  0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00,
  0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x00,
  0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00,
  0x7F, 0x09, 0x19, 0x29, 0x46, 0x00,
  0x26, 0x49, 0x49, 0x49, 0x32, 0x00,
  0x03, 0x01, 0x7F, 0x01, 0x03, 0x00,
  0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00,
  0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00,
  0x63, 0x14, 0x08, 0x14, 0x63, 0x00,
  0x03, 0x04, 0x78, 0x04, 0x03, 0x00,
  0x61, 0x59, 0x49, 0x4D, 0x43, 0x00,
  0x00, 0x7F, 0x41, 0x41, 0x41, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00,
  0x00, 0x41, 0x41, 0x41, 0x7F, 0x00,
  0x04, 0x02, 0x01, 0x02, 0x04, 0x00,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x00,
  0x00, 0x03, 0x07, 0x08, 0x00, 0x00,
  0x20, 0x54, 0x54, 0x78, 0x40, 0x00,
  0x7F, 0x28, 0x44, 0x44, 0x38, 0x00,
  0x38, 0x44, 0x44, 0x44, 0x28, 0x00,
  0x38, 0x44, 0x44, 0x28, 0x7F, 0x00,
  0x38, 0x54, 0x54, 0x54, 0x18, 0x00,
  0x00, 0x08, 0x7E, 0x09, 0x02, 0x00,
  0x18, 0x24, 0x24, 0x1C, 0x78, 0x00,
  0x7F, 0x08, 0x04, 0x04, 0x78, 0x00,
  0x00, 0x44, 0x7D, 0x40, 0x00, 0x00,
  0x20, 0x40, 0x40, 0x3D, 0x00, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00, 0x00,
  0x00, 0x41, 0x7F, 0x40, 0x00, 0x00,
  0x7C, 0x04, 0x78, 0x04, 0x78, 0x00,
  0x7C, 0x08, 0x04, 0x04, 0x78, 0x00,
  0x38, 0x44, 0x44, 0x44, 0x38, 0x00,
  0x7C, 0x18, 0x24, 0x24, 0x18, 0x00,
  0x18, 0x24, 0x24, 0x18, 0x7C, 0x00,
  0x7C, 0x08, 0x04, 0x04, 0x08, 0x00,
  0x48, 0x54, 0x54, 0x54, 0x24, 0x00,
  0x04, 0x04, 0x3F, 0x44, 0x24, 0x00,
  0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00,
  0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00,
  0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00,
  0x44, 0x28, 0x10, 0x28, 0x44, 0x00,
  0x4C, 0x90, 0x90, 0x90, 0x7C, 0x00,
  0x44, 0x64, 0x54, 0x4C, 0x44, 0x00,
  0x00, 0x08, 0x36, 0x41, 0x00, 0x00,
  0x00, 0x00, 0x77, 0x00, 0x00, 0x00,
  0x00, 0x41, 0x36, 0x08, 0x00, 0x00,
  0x02, 0x01, 0x02, 0x04, 0x02, 0x00,
  0x3C, 0x26, 0x23, 0x26, 0x3C, 0x00,
  0x00, 0xF8, 0x04, 0x02, 0x82, 0x42,
  0x22, 0x12, 0x22, 0x42, 0x82, 0x02,
  0x02, 0x02, 0x82, 0x42, 0x22, 0x42,
  0x82, 0x02, 0x02, 0x04, 0xF8, 0x00,
  0x00, 0x1F, 0x20, 0x41, 0x41, 0x4F,
  0x48, 0x48, 0x48, 0x4F, 0x41, 0x41,
  0x40, 0x41, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x41, 0x40, 0x20, 0x1F, 0x00,
  0x00, 0xF8, 0xFC, 0xFE, 0x7E, 0xBE,
  0xDE, 0xEE, 0xDE, 0xBE, 0x7E, 0xFE,
  0xFE, 0xFE, 0x7E, 0xBE, 0xDE, 0xBE,
  0x7E, 0xFE, 0xFE, 0xFC, 0xF8, 0x00,
  0x00, 0x1F, 0x3F, 0x7E, 0x7E, 0x70,
  0x77, 0x77, 0x77, 0x70, 0x7E, 0x7E,
  0x7F, 0x7E, 0x7F, 0x7F, 0x7F, 0x7F,
  0x7F, 0x7E, 0x7F, 0x3F, 0x1F, 0x00,
  0x00, 0xF8, 0x04, 0x22, 0x22, 0x42,
  0x82, 0x02, 0x02, 0x22, 0x22, 0x02,
  0x02, 0x82, 0x42, 0x22, 0x12, 0x22,
  0x42, 0x82, 0x02, 0x04, 0xF8, 0x00,
  0x00, 0x1F, 0x20, 0x40, 0x40, 0x40,
  0x40, 0x41, 0x42, 0x44, 0x44, 0x40,
  0x40, 0x41, 0x42, 0x44, 0x48, 0x44,
  0x42, 0x41, 0x40, 0x20, 0x1F, 0x00,
  0x00, 0xF8, 0xFC, 0xDE, 0xDE, 0xBE,
  0x7E, 0xFE, 0xFE, 0xDE, 0xDE, 0xFE,
  0xFE, 0x7E, 0xBE, 0xDE, 0xEE, 0xDE,
  0xBE, 0x7E, 0xFE, 0xFC, 0xF8, 0x00,
  0x00, 0x1F, 0x3F, 0x7F, 0x7F, 0x7F,
  0x7F, 0x7E, 0x7D, 0x7B, 0x7B, 0x7F,
  0x7F, 0x7E, 0x7D, 0x7B, 0x77, 0x7B,
  0x7D, 0x7E, 0x7F, 0x3F, 0x1F, 0x00,
  0xC0, 0xC1, 0xC1, 0xC2, 0x04, 0x08,
  0xC0, 0x38, 0x04, 0x03, 0x00, 0x00,
  0xA0, 0x22, 0x24, 0x14, 0x12, 0x12,
  0xA0, 0x21, 0x22, 0x12, 0x11, 0x11,
  0x83, 0x7C, 0x41, 0x41, 0x40, 0x40,
  0x81, 0x82, 0x82, 0x84, 0x08, 0x10,
  0x00, 0x00, 0x00, 0x80, 0x80, 0x00,
  0x80, 0x70, 0x19, 0x06, 0x00, 0x00,
  0x80, 0x70, 0x0C, 0x03, 0x00, 0x00,
  0x00, 0x80, 0x00, 0x30, 0x30, 0x00,
  0x00, 0x80, 0x00, 0x30, 0x30, 0x00,
  0x49, 0x88, 0x08, 0x08, 0x08, 0x00,
  0x07, 0x10, 0x24, 0x44, 0x84, 0x04,
  0x20, 0x40, 0x80, 0x00, 0x00, 0x00,
  0x80, 0x40, 0x40, 0x40, 0x40, 0x20,
  0x00, 0x3C, 0xC2, 0x01, 0x01, 0x02,
  0x31, 0x35, 0x01, 0x8A, 0x7C, 0x00,
  0x10, 0x20, 0x40, 0x80, 0x00, 0x00,
  0x20, 0x21, 0x22, 0x12, 0x11, 0x11,
  0x40, 0x20, 0x20, 0x20, 0x20, 0x10,
  0x00, 0x1E, 0xE1, 0x00, 0x00, 0x01,
  0x00, 0x1C, 0xE2, 0x01, 0x01, 0x02,
  0x00, 0x18, 0x64, 0x82, 0x02, 0x02,
  0x02, 0x02, 0x04, 0x18, 0x60, 0x80,
  0x18, 0x18, 0x1B, 0x03, 0x00, 0x40,
  0x18, 0x06, 0x05, 0x98, 0x99, 0x84,
  0x12, 0x0B, 0x08, 0x08, 0x08, 0x08,
  0x11, 0x09, 0x08, 0x08, 0x08, 0x08,
  0x10, 0x10, 0xD0, 0x11, 0x0F, 0x21,
  0x10, 0x10, 0x10, 0x11, 0x0F, 0x01,
  0x10, 0x10, 0x10, 0x10, 0x08, 0x08,
  0x20, 0x20, 0x20, 0x10, 0x08, 0x04,
  0x01, 0x01, 0x02, 0x0C, 0x30, 0x40,
  0x0C, 0x0C, 0x0D, 0x01, 0x00, 0x40,
  0x04, 0x08, 0x08, 0x08, 0xE8, 0x08,
  0x04, 0x04, 0x04, 0x08, 0x30, 0x40,
  0x04, 0x08, 0x08, 0x08, 0x08, 0x08,
  0x10, 0x10, 0x10, 0x08, 0x08, 0x04,
  0x10, 0x10, 0x10, 0x08, 0x04, 0x02,
  0x05, 0x05, 0x09, 0x09, 0x10, 0x10,
  0x02, 0x02, 0x02, 0x04, 0x38, 0x40,
  0x04, 0x04, 0x08, 0x08, 0x10, 0x10,
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
  0x02, 0x04, 0x04, 0x02, 0x01, 0x00,
  0x02, 0x02, 0x02, 0x81, 0x80, 0x80,
  0x00, 0x00, 0x00, 0x02, 0x02, 0x04,
  0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
  0x02, 0x02, 0x02, 0x01, 0x00, 0x00,
  0x00, 0x01, 0xE1, 0x1A, 0x06, 0x09,
  0x00, 0x01, 0x01, 0x02, 0x04, 0x08,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x80, 0x40,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
  0x00, 0x00, 0x00, 0x60, 0x60, 0x00,
  0x00, 0x00, 0x01, 0x01, 0x00, 0x40,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
  0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
