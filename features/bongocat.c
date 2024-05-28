#include QMK_KEYBOARD_H

#define FRAME_DURATION 100

#define IDLE_TIMEOUT 1000
#define SLEEP_TIMEOUT 60000

#define NUM_IDLE_FRAMES 5
#define NUM_TAP_FRAMES 2

#define BONGOCAT_HEIGHT 4
#define BONGOCAT_WIDTH 9

#define BONGOCAT_X 9
#define BONGOCAT_Y 0

extern matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t prev_matrix[MATRIX_ROWS] = {};

enum bongocat_states { sleep, idle, prep, tap };

void render_wpm(void) {
  static uint8_t prev_wpm = 0xff;

  uint8_t cur_wpm = get_current_wpm();
  if (cur_wpm == prev_wpm) {
    return;
  }

  char wpm_str[10] = {};
  sprintf(wpm_str, "WPM: %-3d", get_current_wpm());

  oled_set_cursor(0, 0);
  oled_write(PSTR(wpm_str), false);

  prev_wpm = cur_wpm;
}

bool any_key_pressed(void) {
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    if (matrix[row] != 0) {
      return true;
    }
  }
  return false;
}

bool new_key_pressed(void) {
  bool key_pressed(matrix_row_t matrix[MATRIX_ROWS], uint8_t row, uint8_t col) {
    return (matrix[row] & ((matrix_row_t)1 << col));
  }

  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      if (key_pressed(matrix, row, col) && !key_pressed(prev_matrix, row, col)) {
        return true;
      }
    }
  }
  return false;
}

void save_matrix(void) {
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    prev_matrix[row] = matrix[row];
  }
}

uint8_t get_bongocat_state(void) {
  static uint8_t bongocat_state = idle;
  static uint32_t idle_timer = 0;

  switch (bongocat_state) {
    case sleep:
      if(any_key_pressed()) {
        bongocat_state = tap;
      }
      break;

    case idle:
      if(any_key_pressed()) {
        bongocat_state = tap;
      } else if (timer_elapsed32(idle_timer) > SLEEP_TIMEOUT) {
        bongocat_state = sleep;
      }
      break;

    case prep:
      if(any_key_pressed()) {
        bongocat_state = tap;
      } else if (timer_elapsed32(idle_timer) > IDLE_TIMEOUT) {
        bongocat_state = idle;
      }
      break;

    case tap:
      if (!any_key_pressed()) {
        idle_timer = timer_read32();
        bongocat_state = prep;
      }
      break;

    default:
      break;
  }

  return bongocat_state;
}

void render_bongocat_table(void) {
  static bool table_already_rendered = false;

  if (table_already_rendered) {
    return;
  }

  uint8_t x = BONGOCAT_X * OLED_FONT_WIDTH - 1;
  uint8_t y = BONGOCAT_Y * OLED_FONT_HEIGHT + 26;

  uint8_t n = 0;
  for (; x > 0; x--) {
    oled_write_pixel(x, y, true);
    if (n == 4) { y++; n = 0; }
    else { n++; }
  }

  x = (BONGOCAT_X + BONGOCAT_WIDTH) * OLED_FONT_WIDTH - 11;
  y = BONGOCAT_Y * OLED_FONT_HEIGHT + 15;

  n = 0;
  for (; x < OLED_DISPLAY_WIDTH; x++) {
    oled_write_pixel(x, y, true);
    if (n == 3) { y--; n = 0; }
    else { n++; }
  }

  table_already_rendered = true;
}

void render_bongocat_frame(const char PROGMEM frame[BONGOCAT_HEIGHT][BONGOCAT_WIDTH]) {
  static const char (*prev_frame)[BONGOCAT_WIDTH] = 0;

  if (frame == prev_frame)
    return;

  for (uint8_t i = 0; i < BONGOCAT_HEIGHT; i++) {
    oled_set_cursor(BONGOCAT_X, BONGOCAT_Y + i);
    oled_write_P(frame[i], false);
  }

  prev_frame = frame;
}

void render_bongocat_sleep(void) {
  static const char PROGMEM sleep_frame[BONGOCAT_HEIGHT][BONGOCAT_WIDTH] = {
    {0x20, 0xd4, 0xb3, 0xbe, 0xc3, 0xdc, 0},
    {0xb6, 0xcf, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
    {0xa1, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0x20, 0},
    {0xdf, 0xcd, 0xC4, 0xdd, 0x20, 0x20, 0x20, 0x20, 0}
  };

  render_bongocat_frame(sleep_frame);
}

void render_bongocat_idle(void) {
  static uint8_t idle_frame = 0;
  static uint32_t frame_timer = 0;

  static const char PROGMEM idle_frames[NUM_IDLE_FRAMES][BONGOCAT_HEIGHT][BONGOCAT_WIDTH] = {
    {
      {0x20, 0xd4, 0xb3, 0xc5, 0xc8, 0xdc, 0},
      {0xb5, 0xcf, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa8, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0xd2, 0},
      {0xdf, 0xcd, 0xc4, 0xdd, 0x20, 0x20, 0x20, 0x20, 0}
    },

    {
      {0xa6, 0xd4, 0xb3, 0xc6, 0xc0, 0xdc, 0},
      {0xb4, 0xcf, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa8, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0xd2, 0},
      {0xdf, 0xcd, 0xc4, 0xdd, 0x20, 0x20, 0x20, 0x20, 0}
    },

    {
      {0x20, 0xd5, 0xae, 0xbf, 0xb7, 0x20, 0},
      {0xaf, 0xcb, 0x20, 0x20, 0xd6, 0xa5, 0xad, 0},
      {0xa7, 0xd8, 0xb8, 0xa2, 0xba, 0xc7, 0xbd, 0xd2, 0},
      {0xdf, 0xcd, 0xc4, 0xdd, 0x20, 0x20, 0x20, 0x20, 0}
    },

    {
      {0x20, 0xd4, 0xb3, 0xbe, 0xc3, 0xdc, 0},
      {0xb6, 0xcF, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa1, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0xd2, 0},
      {0xdf, 0xcd, 0xc4, 0xDD, 0x20, 0x20, 0x20, 0x20, 0}
    },

    {
      {0x20, 0xd4, 0xb3, 0xbe, 0xc3, 0xdc, 0},
      {0xb6, 0xcf, 0xd3, 0x20, 0xa9, 0xa0, 0xb1, 0},
      {0xa1, 0xd9, 0xc1, 0xa3, 0xbb, 0xc9, 0xbd, 0xd2, 0},
      {0xdf, 0xcd, 0xC4, 0xdd, 0x20, 0x20, 0x20, 0x20, 0}
    }
  };

  if (timer_elapsed32(frame_timer) > FRAME_DURATION) {
    render_bongocat_frame(idle_frames[idle_frame]);
    idle_frame = (idle_frame + 1) % NUM_IDLE_FRAMES;

    if (timer_elapsed32(frame_timer) > 2 * FRAME_DURATION) {
      frame_timer = timer_read32();
    } else {
      frame_timer += FRAME_DURATION;
    }
  }
}

void render_bongocat_prep(void) {
  static const char PROGMEM prep_frame[BONGOCAT_HEIGHT][BONGOCAT_WIDTH] = {
    {0xa6, 0xd4, 0xb3, 0xc6, 0xc0, 0xdc, 0},
    {0xb4, 0xcc, 0xd3, 0x20, 0xa9, 0xd0, 0xb0, 0},
    {0xa8, 0xb9, 0xa4, 0xb2, 0xbb, 0xca, 0xce, 0xd2, 0},
    {0xdf, 0xaa, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0}
  };

  render_bongocat_frame(prep_frame);
}

void render_bongocat_tap(void) {
  static uint8_t tap_frame = 0;

  static const char PROGMEM tap_frames[NUM_TAP_FRAMES][BONGOCAT_HEIGHT][BONGOCAT_WIDTH] = {
    {
      {0xa6, 0xd4, 0xb3, 0xc6, 0xc0, 0xdc, 0},
      {0xb4, 0xcc, 0xd3, 0x20, 0xa9, 0xd1, 0xb1, 0},
      {0xa8, 0xb9, 0xa4, 0xb2, 0xbb, 0xc9, 0xbc, 0xab, 0},
      {0xdf, 0xaa, 0x20, 0x20, 0x20, 0x20, 0xda, 0xdb, 0}
    },

    {
      {0xa6, 0xd4, 0xb3, 0xc6, 0xc0, 0xdc, 0},
      {0xb4, 0xcf, 0xd3, 0x20, 0xa9, 0xd0, 0xb0, 0},
      {0xa8, 0x20, 0xd7, 0xa3, 0xbb, 0xca, 0xce, 0xd2, 0},
      {0xdf, 0xcd, 0xc2, 0xac, 0xde, 0x20, 0x20, 0x20, 0}
    }
  };

  if (new_key_pressed()) {
    render_bongocat_frame(tap_frames[tap_frame]);
    tap_frame = (tap_frame + 1) % NUM_TAP_FRAMES;
  }
}

void render_bongocat(void) {
  render_wpm();

  render_bongocat_table();
  switch (get_bongocat_state()) {
    case sleep:
      render_bongocat_sleep();
      break;

    case idle:
      render_bongocat_idle();
      break;

    case prep:
      render_bongocat_prep();
      break;

    case tap:
      render_bongocat_tap();
      break;

    default:
      break;
  }

  save_matrix();
}
