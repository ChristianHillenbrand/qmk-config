#pragma once

// number row
#ifndef X_NR
  #define X_NR
#endif

// left of left half
#ifndef X_LT // top row
  #define X_LT
#endif

#ifndef X_LM // middle row
  #define X_LM
#endif

#ifndef X_LB // bottom row
  #define X_LB
#endif

#ifndef X_LH // thumb row
  #define X_LH
#endif

// center
#ifndef X_CT // top row
  #define X_CT
#endif

#ifndef X_CM // middle row
  #define X_CM
#endif

#ifndef X_CB // bottom row
  #define X_CB
#endif

#ifndef X_CH // thumb row
  #define X_CH
#endif

// right of right half
#ifndef X_RT // top row
  #define X_RT
#endif

#ifndef X_RM // middle row
  #define X_RM
#endif

#ifndef X_RB // bottom row
  #define X_RB
#endif

#ifndef X_RH // thumb row
  #define X_RH
#endif

// special keys
#ifndef DRGSCRL
  #define DRGSCRL KC_TRNS
#endif

#ifndef SNIPING
  #define SNIPING KC_TRNS
#endif

// rgb keys
#ifdef RGB_MATRIX_ENABLE
#define RGB_TOGG RM_TOGG
#define RGB_NEXT RM_NEXT
#define RGB_PREV RM_PREV
#define RGB_HUEU RM_HUEU
#define RGB_HUED RM_HUED
#define RGB_SATU RM_SATU
#define RGB_SATD RM_SATD
#define RGB_VALU RM_VALU
#define RGB_VALD RM_VALD
#define RGB_SPDU RM_SPDU
#define RGB_SPDD RM_SPDD
#elif defined(RGBLIGHT_ENABLE)
#define RGB_TOGG UG_TOGG
#define RGB_NEXT UG_NEXT
#define RGB_PREV UG_PREV
#define RGB_HUEU UG_HUEU
#define RGB_HUED UG_HUED
#define RGB_SATU UG_SATU
#define RGB_SATD UG_SATD
#define RGB_VALU UG_VALU
#define RGB_VALD UG_VALD
#define RGB_SPDU UG_SPDU
#define RGB_SPDD UG_SPDD
#else
#define RGB_TOGG XXXXXXX
#define RGB_NEXT XXXXXXX
#define RGB_PREV XXXXXXX
#define RGB_HUEU XXXXXXX
#define RGB_HUED XXXXXXX
#define RGB_SATU XXXXXXX
#define RGB_SATD XXXXXXX
#define RGB_VALU XXXXXXX
#define RGB_VALD XXXXXXX
#define RGB_SPDU XXXXXXX
#define RGB_SPDD XXXXXXX
#endif
