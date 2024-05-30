#include QMK_KEYBOARD_H

#define FRAME_DURATION 100

#define IDLE_TIMEOUT 1000
#define SLEEP_TIMEOUT 60000

#define NUM_IDLE_FRAMES 5
#define NUM_TAP_FRAMES 2

#define BONGOCAT_ROWS 4
#define BONGOCAT_COLS 47

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

  x = BONGOCAT_X * OLED_FONT_WIDTH + BONGOCAT_COLS - 11;
  y = BONGOCAT_Y * OLED_FONT_HEIGHT + 15;

  n = 0;
  for (; x < OLED_DISPLAY_WIDTH; x++) {
    oled_write_pixel(x, y, true);
    if (n == 3) { y--; n = 0; }
    else { n++; }
  }

  table_already_rendered = true;
}

void render_bongocat_frame(const char PROGMEM frame[BONGOCAT_ROWS][BONGOCAT_COLS]) {
  static const char (*prev_frame)[BONGOCAT_COLS] = 0;

  if (frame == prev_frame)
    return;

  for (uint8_t i = 0; i < BONGOCAT_ROWS; i++) {
    oled_set_cursor(BONGOCAT_X, BONGOCAT_Y + i);
    oled_write_raw_P(frame[i], BONGOCAT_COLS);
  }

  prev_frame = frame;
}

void render_bongocat_sleep(void) {
  static const char PROGMEM sleep_frame[BONGOCAT_ROWS][BONGOCAT_COLS] = {
    {0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, 0x02, 0x01, 0x01, 0x02, 0x0c, 0x30, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1e, 0xe1, 0x00, 0x00, 0x01, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0xc0, 0xc1, 0xc1, 0xc2, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00},
    {0x80, 0x70, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x0d, 0x01, 0x00, 0x40, 0xa0, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x0f, 0x01, 0x01},
    {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  };

  render_bongocat_frame(sleep_frame);
}

void render_bongocat_idle(void) {
  static uint8_t idle_frame = 0;
  static uint32_t frame_timer = 0;

  static const char PROGMEM idle_frames[NUM_IDLE_FRAMES][BONGOCAT_ROWS][BONGOCAT_COLS] = {
    {
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x08, 0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x38, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x1c, 0xe2, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0xc0, 0xc1, 0xc1, 0xc2, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x80, 0x70, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x0d, 0x01, 0x00, 0x40, 0xa0, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x0f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00},
      {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    {
      {0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, 0x02, 0x01, 0x01, 0x02, 0x0c, 0x30, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x1e, 0xe1, 0x00, 0x00, 0x01, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0xc0, 0xc1, 0xc1, 0xc2, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x80, 0x70, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x0d, 0x01, 0x00, 0x40, 0xa0, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x0f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00},
      {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    {
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x40, 0x40, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x08, 0x04, 0x02, 0x02, 0x04, 0x18, 0x60, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x3c, 0xc2, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x81, 0x82, 0x82, 0x84, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x80, 0x70, 0x19, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x1b, 0x03, 0x00, 0x40, 0xa0, 0x22, 0x24, 0x14, 0x12, 0x12, 0x12, 0x0b, 0x08, 0x08, 0x08, 0x08, 0x05, 0x05, 0x09, 0x09, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x0f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00},
      {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    {
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, 0x08, 0x04, 0x04, 0x04, 0x08, 0x30, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x18, 0x64, 0x82, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0xc0, 0xc1, 0xc1, 0xc2, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0xc0, 0x38, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x0d, 0x01, 0x00, 0x40, 0xa0, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x0f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00},
      {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    {
      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, 0x08, 0x04, 0x04, 0x04, 0x08, 0x30, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x18, 0x64, 0x82, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0xc0, 0xc1, 0xc1, 0xc2, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0xc0, 0x38, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x0d, 0x01, 0x00, 0x40, 0xa0, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x11, 0x0f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00},
      {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
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
  static const char PROGMEM prep_frame[BONGOCAT_ROWS][BONGOCAT_COLS] = {
    {0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, 0x02, 0x01, 0x01, 0x02, 0x0c, 0x30, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1e, 0xe1, 0x00, 0x00, 0x01, 0x02, 0x02, 0x02, 0x81, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0x00, 0x01, 0xe1, 0x1a, 0x06, 0x09, 0x31, 0x35, 0x01, 0x8a, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x80, 0x70, 0x0c, 0x03, 0x00, 0x00, 0x18, 0x06, 0x05, 0x98, 0x99, 0x84, 0x83, 0x7c, 0x41, 0x41, 0x40, 0x40, 0x20, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00},
    {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  };

  render_bongocat_frame(prep_frame);
}

void render_bongocat_tap(void) {
  static uint8_t tap_frame = 0;

  static const char PROGMEM tap_frames[NUM_TAP_FRAMES][BONGOCAT_ROWS][BONGOCAT_COLS] = {
    {
      {0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, 0x02, 0x01, 0x01, 0x02, 0x0c, 0x30, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x1e, 0xe1, 0x00, 0x00, 0x01, 0x02, 0x02, 0x02, 0x81, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x80, 0x70, 0x0c, 0x03, 0x00, 0x00, 0x18, 0x06, 0x05, 0x98, 0x99, 0x84, 0x83, 0x7c, 0x41, 0x41, 0x40, 0x40, 0x20, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0xd0, 0x11, 0x0f, 0x21, 0x49, 0x88, 0x08, 0x08, 0x08},
      {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00},
    },

    {
      {0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x40, 0x20, 0x20, 0x20, 0x20, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, 0x02, 0x01, 0x01, 0x02, 0x0c, 0x30, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x00, 0x1e, 0xe1, 0x00, 0x00, 0x01, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x30, 0x30, 0x00, 0x00, 0x01, 0xe1, 0x1a, 0x06, 0x09, 0x31, 0x35, 0x01, 0x8a, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x80, 0x70, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x40, 0xa0, 0x21, 0x22, 0x12, 0x11, 0x11, 0x11, 0x09, 0x08, 0x08, 0x08, 0x08, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00},
      {0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x08, 0xe8, 0x08, 0x07, 0x10, 0x24, 0x44, 0x84, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
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
