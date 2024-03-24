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

// Tap-hold configuration for home row mods.
#define ACHORDION_STREAK
#define TAPPING_TERM 175
#define PERMISSIVE_HOLD
#define QUICK_TAP_TERM_PER_KEY
