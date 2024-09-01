#pragma once

// qmk settings
#define COMBO_TERM 25
#define COMBO_SHOULD_TRIGGER

#define PERMISSIVE_HOLD_PER_KEY

// custom settings
#define ACHORDION_TIMEOUT 500
#define REQUIRE_PRIOR_IDLE_MS 150

#ifndef DRGSCRL
  #define DRGSCRL KC_TRNS
#endif

#ifndef SNIPING
  #define SNIPING KC_TRNS
#endif
