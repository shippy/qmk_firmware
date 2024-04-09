/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/
#define ORYX_CONFIGURATOR
#undef DEBOUNCE
#define DEBOUNCE 30

#define USB_SUSPEND_WAKEUP_DELAY 0
#define FIRMWARE_VERSION u8"ZQoB7/6gmzq"
#define RAW_USAGE_PAGE 0xFF60
#define RAW_USAGE_ID 0x61
#define LAYER_STATE_8BIT

#define RGB_MATRIX_STARTUP_SPD 60

// Settings to do with home row mods
#define TAPPING_TERM 175
#define TAPPING_TERM_PER_KEY
#define PERMISSIVE_HOLD
// Must be less than or equal to TAPPING_TERM
#define QUICK_TAP_TERM 150
// Non-QMK value, used in `filterpaper` code snippet
#define REQUIRE_PRIOR_IDLE 175

// #define QUICK_TAP_TERM_PER_KEY
// needs uint16_t get_quick_tap_term(uint16_t keycode, keyrecord_t *record) {
