#include QMK_KEYBOARD_H

#include "layers.h"
#include "oled.h"

void render_line(void) {
  static const char PROGMEM line[OLED_DISPLAY_HEIGHT] = {
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08
  };
  oled_write_raw_P(line, OLED_DISPLAY_HEIGHT);
  oled_advance_page(false);
}

static void render_default_layer(void) {
  switch (get_highest_layer(default_layer_state)) {
    case L_QWRTY:
      oled_write_P(PSTR("QWRTY"), false);
      break;

    case L_COLMK:
      oled_write_P(PSTR("COLMK"), false);
      break;

    default:
      oled_write_P(PSTR("?????"), false);
      break;
  }
}

void render_layer(void) {
  oled_write_P(PSTR("LAYER"), false);
  render_space();
  switch (get_highest_layer(layer_state)) {
    case L_QWRTY:
    case L_COLMK:
      render_default_layer();
      break;

    case L_GER:
      oled_write_P(PSTR(" GER "), false);
      break;

    case L_NAV:
      oled_write_P(PSTR(" NAV "), false);
      break;

    case L_NUM:
      oled_write_P(PSTR(" NUM "), false);
      break;

    case L_FUN:
      oled_write_P(PSTR(" FUN "), false);
      break;

    default:
      oled_write_P(PSTR("?????"), false);
      break;
  }
}

void render_mod_status_shift_ctrl(uint8_t modifiers) {
  static const char PROGMEM shift_off_1[] = {0x80, 0x81, 0};
  static const char PROGMEM ctrl_off_1[]  = {0x82, 0x83, 0};
  static const char PROGMEM shift_off_2[] = {0x84, 0x85, 0};
  static const char PROGMEM ctrl_off_2[]  = {0x86, 0x87, 0};

  static const char PROGMEM shift_on_1[] = {0x88, 0x89, 0};
  static const char PROGMEM ctrl_on_1[]  = {0x8a, 0x8b, 0};
  static const char PROGMEM shift_on_2[] = {0x8c, 0x8d, 0};
  static const char PROGMEM ctrl_on_2[]  = {0x8e, 0x8f, 0};

  static const char PROGMEM off_off_1[] = {0xa0, 0};
  static const char PROGMEM off_off_2[] = {0xa1, 0};
  static const char PROGMEM on_off_1[]  = {0xa2, 0};
  static const char PROGMEM on_off_2[]  = {0xa3, 0};
  static const char PROGMEM off_on_1[]  = {0xa4, 0};
  static const char PROGMEM off_on_2[]  = {0xa5, 0};
  static const char PROGMEM on_on_1[]   = {0xa6, 0};
  static const char PROGMEM on_on_2[]   = {0xa7, 0};

  if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(shift_on_1, false);
  } else {
    oled_write_P(shift_off_1, false);
  }

  if ((modifiers & MOD_MASK_SHIFT) && (modifiers & MOD_MASK_CTRL)) {
    oled_write_P(on_on_1, false);
  } else if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(on_off_1, false);
  } else if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(off_on_1, false);
  } else {
    oled_write_P(off_off_1, false);
  }

  if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(ctrl_on_1, false);
  } else {
    oled_write_P(ctrl_off_1, false);
  }

  if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(shift_on_2, false);
  } else {
    oled_write_P(shift_off_2, false);
  }

  if ((modifiers & MOD_MASK_SHIFT) && (modifiers & MOD_MASK_CTRL)) {
    oled_write_P(on_on_2, false);
  } else if(modifiers & MOD_MASK_SHIFT) {
    oled_write_P(on_off_2, false);
  } else if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(off_on_2, false);
  } else {
    oled_write_P(off_off_2, false);
  }

  if(modifiers & MOD_MASK_CTRL) {
    oled_write_P(ctrl_on_2, false);
  } else {
    oled_write_P(ctrl_off_2, false);
  }
}

void render_mod_status_alt_gui(uint8_t modifiers) {
  static const char PROGMEM alt_off_1[] = {0x90, 0x91, 0};
  static const char PROGMEM gui_off_1[] = {0x92, 0x93, 0};
  static const char PROGMEM alt_off_2[] = {0x94, 0x95, 0};
  static const char PROGMEM gui_off_2[] = {0x96, 0x97, 0};

  static const char PROGMEM alt_on_1[] = {0x98, 0x99, 0};
  static const char PROGMEM gui_on_1[] = {0x9a, 0x9b, 0};
  static const char PROGMEM alt_on_2[] = {0x9c, 0x9d, 0};
  static const char PROGMEM gui_on_2[] = {0x9e, 0x9f, 0};

  static const char PROGMEM off_off_1[] = {0xa0, 0};
  static const char PROGMEM off_off_2[] = {0xa1, 0};
  static const char PROGMEM on_off_1[]  = {0xa2, 0};
  static const char PROGMEM on_off_2[]  = {0xa3, 0};
  static const char PROGMEM off_on_1[]  = {0xa4, 0};
  static const char PROGMEM off_on_2[]  = {0xa5, 0};
  static const char PROGMEM on_on_1[]   = {0xa6, 0};
  static const char PROGMEM on_on_2[]   = {0xa7, 0};

  if(modifiers & MOD_MASK_ALT) {
    oled_write_P(alt_on_1, false);
  } else {
    oled_write_P(alt_off_1, false);
  }

  if ((modifiers & MOD_MASK_ALT) && (modifiers & MOD_MASK_GUI)) {
    oled_write_P(on_on_1, false);
  } else if(modifiers & MOD_MASK_ALT) {
    oled_write_P(on_off_1, false);
  } else if(modifiers & MOD_MASK_GUI) {
    oled_write_P(off_on_1, false);
  } else {
    oled_write_P(off_off_1, false);
  }

  if(modifiers & MOD_MASK_GUI) {
    oled_write_P(gui_on_1, false);
  } else {
    oled_write_P(gui_off_1, false);
  }

  if(modifiers & MOD_MASK_ALT) {
    oled_write_P(alt_on_2, false);
  } else {
    oled_write_P(alt_off_2, false);
  }

  if ((modifiers & MOD_MASK_ALT) && (modifiers & MOD_MASK_GUI)) {
    oled_write_P(on_on_2, false);
  } else if(modifiers & MOD_MASK_ALT) {
    oled_write_P(on_off_2, false);
  } else if(modifiers & MOD_MASK_GUI) {
    oled_write_P(off_on_2, false);
  } else {
    oled_write_P(off_off_2, false);
  }

  if(modifiers & MOD_MASK_GUI) {
    oled_write_P(gui_on_2, false);
  } else {
    oled_write_P(gui_off_2, false);
  }
}

void render_wpm(void) {
  uint8_t wpm = get_current_wpm();

  char wpm_str[6] = {};
  sprintf(wpm_str, " %03d ", wpm);

  oled_write_P(PSTR(" WPM "), false);
  render_space();
  oled_write_P(PSTR(wpm_str), false);
}

struct rgb_data_t {
  bool enable;
  uint8_t mode;
  HSV hsv;
  uint8_t speed;
};

static struct rgb_data_t read_rgb_data(void) {
  struct rgb_data_t rgb_data = {
    rgb_matrix_is_enabled(),
    rgb_matrix_get_mode(),
    rgb_matrix_get_hsv(),
    rgb_matrix_get_speed()
  };
  return rgb_data;
}

void render_rgb_data(void) {
  struct rgb_data_t rgb_data = read_rgb_data();

  render_space();
  oled_write_P(PSTR(" RGB "), rgb_data.enable);
  render_space();

  char mode_str[4];
  sprintf(mode_str, "%3d", rgb_data.mode);

  oled_write_P(PSTR("M:"), false);
  oled_write_P(PSTR(mode_str), false);

  render_line();

  char hue_str[4];
  char sat_str[4];
  char val_str[4];

  sprintf(hue_str, "%3d", rgb_data.hsv.h);
  sprintf(sat_str, "%3d", rgb_data.hsv.s);
  sprintf(val_str, "%3d", rgb_data.hsv.v);

  oled_write_P(PSTR("H:"), false);
  oled_write_P(PSTR(hue_str), false);
  oled_write_P(PSTR("S:"), false);
  oled_write_P(PSTR(sat_str), false);
  oled_write_P(PSTR("V:"), false);
  oled_write_P(PSTR(val_str), false);

  render_line();

  char speed_str[4];
  sprintf(speed_str, "%3d", rgb_data.speed);

  oled_write_P(PSTR("S:"), false);
  oled_write_P(PSTR(speed_str), false);
}

