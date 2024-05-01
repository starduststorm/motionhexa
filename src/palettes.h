
#include <FastLED.h>

// Color palettes courtesy of cpt-city and its contributors:
//   http://soliton.vm.bytemark.co.uk/pub/cpt-city/
//
// Color palettes converted for FastLED using "PaletteKnife" v1:
//   http://fastled.io/tools/paletteknife/
//

// Gradient Color Palette definitions for 33 different cpt-city color palettes.
//    956 bytes of PROGMEM for all of the palettes together,
//   +618 bytes of PROGMEM for gradient palette code (AVR).
//  1,494 bytes total for all 34 color palettes and associated code.

// Gradient palette "ib_jul01_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/ing/xmas/tn/ib_jul01.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 16 bytes of program space.

DEFINE_GRADIENT_PALETTE( ib_jul01_gp ) {
  0, 194,  1,  1,
  94,   1, 29, 18,
  132,  57, 131, 28,
  255, 113,  1,  1
};

// Gradient palette "es_vintage_57_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/vintage/tn/es_vintage_57.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 20 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_vintage_57_gp ) {
  0,   2,  1,  1,
  53,  18,  1,  0,
  104,  69, 29,  1,
  153, 167, 135, 10,
  255,  46, 56,  4
};

// Gradient palette "es_vintage_01_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/vintage/tn/es_vintage_01.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 32 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_vintage_01_gp ) {
  0,   4,  1,  1,
  51,  16,  0,  1,
  76,  97, 104,  3,
  101, 255, 131, 19,
  127,  67,  9,  4,
  153,  16,  0,  1,
  229,   4,  1,  1,
  255,   4,  1,  1
};

// Gradient palette "es_rivendell_15_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/rivendell/tn/es_rivendell_15.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 20 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_rivendell_15_gp ) {
  0,   1, 14,  5,
  101,  16, 36, 14,
  165,  56, 68, 30,
  242, 150, 156, 99,
  255, 150, 156, 99
};

// Gradient palette "rgi_15_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/ds/rgi/tn/rgi_15.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 36 bytes of program space.

DEFINE_GRADIENT_PALETTE( rgi_15_gp ) {
  0,   4,  1, 31,
  31,  55,  1, 16,
  63, 197,  3,  7,
  95,  59,  2, 17,
  127,   6,  2, 34,
  159,  39,  6, 33,
  191, 112, 13, 32,
  223,  56,  9, 35,
  255,  22,  6, 38
};

// Gradient palette "retro2_16_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/ma/retro2/tn/retro2_16.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 8 bytes of program space.

DEFINE_GRADIENT_PALETTE( retro2_16_gp ) {
  0, 188, 135,  1,
  255,  46,  7,  1
};

// Gradient palette "Analogous_1_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/red/tn/Analogous_1.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 20 bytes of program space.

DEFINE_GRADIENT_PALETTE( Analogous_1_gp ) {
  0,   3,  0, 255,
  63,  23,  0, 255,
  127,  67,  0, 255,
  191, 142,  0, 45,
  255, 255,  0,  0
};

// Gradient palette "es_pinksplash_08_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/pink_splash/tn/es_pinksplash_08.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 20 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_pinksplash_08_gp ) {
  0, 126, 11, 255,
  127, 197,  1, 22,
  175, 210, 157, 172,
  221, 157,  3, 112,
  255, 157,  3, 112
};

// Gradient palette "es_pinksplash_07_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/pink_splash/tn/es_pinksplash_07.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_pinksplash_07_gp ) {
  0, 229,  1,  1,
  61, 242,  4, 63,
  101, 255, 12, 255,
  127, 249, 81, 252,
  153, 255, 11, 235,
  193, 244,  5, 68,
  255, 232,  1,  5
};

// Gradient palette "Coral_reef_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/other/tn/Coral_reef.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 24 bytes of program space.

DEFINE_GRADIENT_PALETTE( Coral_reef_gp ) {
  0,  40, 199, 197,
  50,  10, 152, 155,
  96,   1, 111, 120,
  96,  43, 127, 162,
  139,  10, 73, 111,
  255,   1, 34, 71
};

// Gradient palette "es_ocean_breeze_068_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/ocean_breeze/tn/es_ocean_breeze_068.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 24 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_ocean_breeze_068_gp ) {
  0, 100, 156, 153,
  51,   1, 99, 137,
  101,   1, 68, 84,
  104,  35, 142, 168,
  178,   0, 63, 117,
  255,   1, 10, 10
};

// Gradient palette "es_ocean_breeze_036_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/ocean_breeze/tn/es_ocean_breeze_036.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 16 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_ocean_breeze_036_gp ) {
  0,   1,  6,  7,
  89,   1, 99, 111,
  153, 144, 209, 255,
  255,   0, 73, 82
};

// Gradient palette "departure_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/mjf/tn/departure.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 88 bytes of program space.

DEFINE_GRADIENT_PALETTE( departure_gp ) {
  0,   8,  3,  0,
  42,  23,  7,  0,
  63,  75, 38,  6,
  84, 169, 99, 38,
  106, 213, 169, 119,
  116, 255, 255, 255,
  138, 135, 255, 138,
  148,  22, 255, 24,
  170,   0, 255,  0,
  191,   0, 136,  0,
  212,   0, 55,  0,
  255,   0, 55,  0
};

// Gradient palette "es_landscape_64_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/landscape/tn/es_landscape_64.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 36 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_landscape_64_gp ) {
  0,   0,  0,  0,
  37,   2, 25,  1,
  76,  15, 115,  5,
  127,  79, 213,  1,
  128, 126, 211, 47,
  130, 188, 209, 247,
  153, 144, 182, 205,
  204,  59, 117, 250,
  255,   1, 37, 192
};

// Gradient palette "es_landscape_33_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/landscape/tn/es_landscape_33.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 24 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_landscape_33_gp ) {
  0,   1,  5,  0,
  19,  32, 23,  1,
  38, 161, 55,  1,
  63, 229, 144,  1,
  66,  39, 142, 74,
  255,   1,  4,  1
};

// Gradient palette "rainbowsherbet_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/ma/icecream/tn/rainbowsherbet.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( rainbowsherbet_gp ) {
  0, 255, 33,  4,
  43, 255, 68, 25,
  86, 255,  7, 25,
  127, 255, 82, 103,
  170, 255, 255, 242,
  209,  42, 255, 22,
  255,  87, 255, 65
};

// Gradient palette "gr65_hult_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/hult/tn/gr65_hult.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 24 bytes of program space.

DEFINE_GRADIENT_PALETTE( gr65_hult_gp ) {
  0, 247, 176, 247,
  48, 255, 136, 255,
  89, 220, 29, 226,
  160,   7, 82, 178,
  216,   1, 124, 109,
  255,   1, 124, 109
};

// Gradient palette "gr64_hult_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/hult/tn/gr64_hult.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 32 bytes of program space.

DEFINE_GRADIENT_PALETTE( gr64_hult_gp ) {
  0,   1, 124, 109,
  66,   1, 93, 79,
  104,  52, 65,  1,
  130, 115, 127,  1,
  150,  52, 65,  1,
  201,   1, 86, 72,
  239,   0, 55, 45,
  255,   0, 55, 45
};

// Gradient palette "GMT_drywet_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/gmt/tn/GMT_drywet.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( GMT_drywet_gp ) {
  0,  47, 30,  2,
  42, 213, 147, 24,
  84, 103, 219, 52,
  127,   3, 219, 207,
  170,   1, 48, 214,
  212,   1,  1, 111,
  255,   1,  7, 33
};

// Gradient palette "ib15_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/ing/general/tn/ib15.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 24 bytes of program space.

DEFINE_GRADIENT_PALETTE( ib15_gp ) {
  0, 113, 91, 147,
  72, 157, 88, 78,
  89, 208, 85, 33,
  107, 255, 29, 11,
  141, 137, 31, 39,
  255,  59, 33, 89
};

// Gradient palette "Fuschia_7_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/ds/fuschia/tn/Fuschia-7.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 20 bytes of program space.

DEFINE_GRADIENT_PALETTE( Fuschia_7_gp ) {
  0,  43,  3, 153,
  63, 100,  4, 103,
  127, 188,  5, 66,
  191, 161, 11, 115,
  255, 135, 20, 182
};

// Gradient palette "es_emerald_dragon_08_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/emerald_dragon/tn/es_emerald_dragon_08.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 16 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_emerald_dragon_08_gp ) {
  0,  97, 255,  1,
  101,  47, 133,  1,
  178,  13, 43,  1,
  255,   2, 10,  1
};

// Gradient palette "lava_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/neota/elem/tn/lava.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 52 bytes of program space.

DEFINE_GRADIENT_PALETTE( lava_gp ) {
  0,   0,  0,  0,
  46,  18,  0,  0,
  96, 113,  0,  0,
  108, 142,  3,  1,
  119, 175, 17,  1,
  146, 213, 44,  2,
  174, 255, 82,  4,
  188, 255, 115,  4,
  202, 255, 156,  4,
  218, 255, 203,  4,
  234, 255, 255,  4,
  244, 255, 255, 71,
  255, 255, 255, 255
};

// Gradient palette "fire_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/neota/elem/tn/fire.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( fire_gp ) {
  0,   1,  1,  0,
  76,  32,  5,  0,
  146, 192, 24,  0,
  197, 220, 105,  5,
  240, 252, 255, 31,
  250, 252, 255, 111,
  255, 255, 255, 255
};

// Gradient palette "Colorfull_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/atmospheric/tn/Colorfull.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 44 bytes of program space.

DEFINE_GRADIENT_PALETTE( Colorfull_gp ) {
  0,  10, 85,  5,
  25,  29, 109, 18,
  60,  59, 138, 42,
  93,  83, 99, 52,
  106, 110, 66, 64,
  109, 123, 49, 65,
  113, 139, 35, 66,
  116, 192, 117, 98,
  124, 255, 255, 137,
  168, 100, 180, 155,
  255,  22, 121, 174
};

// Gradient palette "Magenta_Evening_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/atmospheric/tn/Magenta_Evening.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( Magenta_Evening_gp ) {
  0,  71, 27, 39,
  31, 130, 11, 51,
  63, 213,  2, 64,
  70, 232,  1, 66,
  76, 252,  1, 69,
  108, 123,  2, 51,
  255,  46,  9, 35
};

// Gradient palette "Pink_Purple_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/atmospheric/tn/Pink_Purple.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 44 bytes of program space.

DEFINE_GRADIENT_PALETTE( Pink_Purple_gp ) {
  0,  19,  2, 39,
  25,  26,  4, 45,
  51,  33,  6, 52,
  76,  68, 62, 125,
  102, 118, 187, 240,
  109, 163, 215, 247,
  114, 217, 244, 255,
  122, 159, 149, 221,
  149, 113, 78, 188,
  183, 128, 57, 155,
  255, 146, 40, 123
};

// Gradient palette "Sunset_Real_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/atmospheric/tn/Sunset_Real.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( Sunset_Real_gp ) {
  0, 120,  0,  0,
  22, 179, 22,  0,
  51, 255, 104,  0,
  85, 167, 22, 18,
  135, 100,  0, 103,
  198,  16,  0, 130,
  255,   0,  0, 160
};

// Gradient palette "es_autumn_19_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/es/autumn/tn/es_autumn_19.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 52 bytes of program space.

DEFINE_GRADIENT_PALETTE( es_autumn_19_gp ) {
  0,  26,  1,  1,
  51,  67,  4,  1,
  84, 118, 14,  1,
  104, 137, 152, 52,
  112, 113, 65,  1,
  122, 133, 149, 59,
  124, 137, 152, 52,
  135, 113, 65,  1,
  142, 139, 154, 46,
  163, 113, 13,  1,
  204,  55,  3,  1,
  249,  17,  1,  1,
  255,  17,  1,  1
};

// Gradient palette "BlacK_Blue_Magenta_White_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/basic/tn/BlacK_Blue_Magenta_White.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( BlacK_Blue_Magenta_White_gp ) {
  0,   0,  0,  0,
  42,   0,  0, 45,
  84,   0,  0, 255,
  127,  42,  0, 255,
  170, 255,  0, 255,
  212, 255, 55, 255,
  255, 255, 255, 255
};

// Gradient palette "BlacK_Magenta_Red_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/basic/tn/BlacK_Magenta_Red.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 20 bytes of program space.

DEFINE_GRADIENT_PALETTE( BlacK_Magenta_Red_gp ) {
  0,   0,  0,  0,
  63,  42,  0, 45,
  127, 255,  0, 255,
  191, 255,  0, 45,
  255, 255,  0,  0
};

// Gradient palette "BlacK_Red_Magenta_Yellow_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/basic/tn/BlacK_Red_Magenta_Yellow.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 28 bytes of program space.

DEFINE_GRADIENT_PALETTE( BlacK_Red_Magenta_Yellow_gp ) {
  0,   0,  0,  0,
  42,  42,  0,  0,
  84, 255,  0,  0,
  127, 255,  0, 45,
  170, 255,  0, 255,
  212, 255, 55, 45,
  255, 255, 255,  0
};

// Gradient palette "Blue_Cyan_Yellow_gp", originally from
// http://soliton.vm.bytemark.co.uk/pub/cpt-city/nd/basic/tn/Blue_Cyan_Yellow.png.index.html
// converted for FastLED with gammas (2.6, 2.2, 2.5)
// Size: 20 bytes of program space.

DEFINE_GRADIENT_PALETTE( Blue_Cyan_Yellow_gp ) {
  0,   0,  0, 255,
  63,   0, 55, 255,
  127,   0, 255, 255,
  191,  42, 255, 45,
  255, 255, 255,  0
};

// Following palettes and color matching by Opal Holley
// Colors are pulled from publically available flag values, then refined to render better on APA102-2020 LEDs manually

DEFINE_GRADIENT_PALETTE( Trans_Flag_gp ) {
  0,   0x2A, 0x9F, 0xFA,
  50,  0x2A, 0x9F, 0xFA,
  51,  0xF1, 0x55, 0x70,
  101, 0xF1, 0x55, 0x70,
  102, 0xFF, 0xFF, 0xFF,
  152, 0xFF, 0xFF, 0xFF,
  153, 0xF1, 0x55, 0x70,
  203, 0xF1, 0x55, 0x70,
  204, 0x2A, 0x9F, 0xFA,
  255, 0x2A, 0x9F, 0xFA,
};

DEFINE_GRADIENT_PALETTE( Bi_Flag_gp ) {
  0,   0xB6, 0x02, 0x40,
  101, 0xB6, 0x02, 0x40,
  102, 0x6E, 0x07, 0xD7,
  152, 0x6E, 0x07, 0xD7,
  153, 0x00, 0x38, 0xD8,
  255, 0x00, 0x38, 0xD8,
};

DEFINE_GRADIENT_PALETTE( Lesbian_Flag_gp ) {
  0,   0xD6, 0x29, 0x00,
  50,  0xD6, 0x29, 0x00,
  51,  0xCF, 0x5F, 0x20,
  101, 0xCF, 0x5F, 0x20,
  102, 0xFF, 0xFF, 0xFF,
  152, 0xFF, 0xFF, 0xFF,
  153, 0xD1, 0x50, 0x60,
  204, 0xD1, 0x55, 0x70,
  205, 0x90, 0x00, 0x52,
  255, 0x90, 0x00, 0x52,
};

DEFINE_GRADIENT_PALETTE( Pride_Flag_gp ) {
  0,   0xF4, 0x03, 0x03,
  42,  0xF4, 0x03, 0x03,
  43,  0xCF, 0x35, 0x00,
  85,  0xCF, 0x35, 0x00,
  86,  0xFF, 0xED, 0x00,
  127, 0xFF, 0xED, 0x00,
  128, 0x00, 0xC0, 0x26,
  170, 0x00, 0xC0, 0x26,
  171, 0x00, 0x2D, 0xFF,
  212, 0x00, 0x2D, 0xFF,
  213, 0x75, 0x07, 0xB7,
  255, 0x75, 0x07, 0xB7,
};

// if anyone can find a way to make the ace flag look good on leds, lemee know
// DEFINE_GRADIENT_PALETTE( Ace_Flag_gp ) {
//   0,   0x00, 0x00, 0x00,
//   63,  0x00, 0x00, 0x00,
//   64,  0x40, 0x40, 0x40,
//   127, 0xA4, 0xA4, 0xA4,
//   128, 0xFF, 0xFF, 0xFF,
//   195, 0xFF, 0xFF, 0xFF,
//   196, 0x81, 0x00, 0x81,
//   255, 0x81, 0x00, 0x81,
// };

DEFINE_GRADIENT_PALETTE( Enby_Flag_gp ) {
  // 0,   0xFF, 0xF4, 0x30,
  // 63,  0xFF, 0xF4, 0x30,
  // 64,  0xFF, 0xFF, 0xFF,
  // 127, 0xFF, 0xFF, 0xFF,
  // 128, 0x90, 0x30, 0xD1,
  // 195, 0x90, 0x30, 0xD1,
  // 196, 0x00, 0x00, 0x00,
  // 255, 0x00, 0x00, 0x00,

// cutting out the black because we can't do anything with it
  0,   0xFF, 0xF4, 0x30,
  85,  0xFF, 0xF4, 0x30,
  86,  0xFF, 0xFF, 0xFF,
  170, 0xFF, 0xFF, 0xFF,
  171, 0x6E, 0x07, 0xD7,
  255, 0x6E, 0x07, 0xD7,
};

DEFINE_GRADIENT_PALETTE( Genderqueer_Flag_gp ) {
  0,   0x8E, 0x20, 0xD7,
  85,  0x8E, 0x20, 0xD7,
  86,  0xFF, 0xFF, 0xFF,
  170, 0xFF, 0xFF, 0xFF,
  171, 0x28, 0x82, 0x10,
  255, 0x28, 0x82, 0x10,
};

DEFINE_GRADIENT_PALETTE( Intersex_Flag_gp ) {
  0,   0x6E, 0x07, 0xD7,
  63,  0x6E, 0x07, 0xD7,
  64,  0xFF, 0xFF, 0x00,
  127, 0xFF, 0xFF, 0x00,
  128, 0x6E, 0x07, 0xD7,
  191, 0x6E, 0x07, 0xD7,
  192, 0xFF, 0xFF, 0x00,
  255, 0xFF, 0xFF, 0x00,
};

// 2-band variant
// DEFINE_GRADIENT_PALETTE( Intersex_Flag_gp ) {
//   0,   0x6E, 0x07, 0xD7,
//   127,  0x6E, 0x07, 0xD7,
//   128, 0xFF, 0xFF, 0x00,
//   255, 0xFF, 0xFF, 0x00,
// };

DEFINE_GRADIENT_PALETTE( Pan_Flag_gp ) {
  0,   0xFF, 0x1B, 0x8D,
  85,  0xFF, 0x1B, 0x8D,
  86,  0xFF, 0xDA, 0x00,
  170, 0xFF, 0xDA, 0x00,
  171, 0x1B, 0xB3, 0xFF,
  255, 0x1B, 0xB3, 0xFF,
};

const TProgmemRGBGradientPalettePtr gPrideFlagPalettes[] = {
  Trans_Flag_gp,     // note: index is hard coded
  Enby_Flag_gp,
  Genderqueer_Flag_gp,
  Intersex_Flag_gp,  // note: index is hard coded
  Pride_Flag_gp,     // note: index is hard coded
  Bi_Flag_gp,
  Lesbian_Flag_gp,
  // Ace_Flag_gp,
  Pan_Flag_gp,
};

const uint8_t gPridePaletteCount =
  sizeof( gPrideFlagPalettes) / sizeof( TProgmemRGBGradientPalettePtr );

uint8_t paletteBandCount(TProgmemRGBGradientPalettePtr progpal) {
  // cribbed from FastLED for counting entries in DEFINE_GRADIENT_PALETTE 
  TRGBGradientPaletteEntryUnion* progent = (TRGBGradientPaletteEntryUnion*)(progpal);
  TRGBGradientPaletteEntryUnion u;
  uint16_t count = 0;
  do {
      u.dword = FL_PGM_READ_DWORD_NEAR(progent + count);
      ++count;
  } while ( u.index != 255);
        
  return count >> 1; // flag colors = 1/2 entries
}

//

const TProgmemRGBGradientPalettePtr gGradientPalettes[] = {
  Sunset_Real_gp,
  es_rivendell_15_gp,
  es_ocean_breeze_036_gp,
  rgi_15_gp,
  retro2_16_gp,
  Analogous_1_gp,
  es_pinksplash_08_gp,
  Coral_reef_gp,
  es_ocean_breeze_068_gp,
  es_pinksplash_07_gp,
  es_vintage_01_gp, //10
  departure_gp,
  es_landscape_64_gp,
  es_landscape_33_gp,
  rainbowsherbet_gp,
  gr65_hult_gp,
  gr64_hult_gp,
  GMT_drywet_gp,
  ib_jul01_gp,
  es_vintage_57_gp,
  ib15_gp,  // 20
  Fuschia_7_gp,
  es_emerald_dragon_08_gp,
  lava_gp,
  fire_gp,
  Colorfull_gp,
  Magenta_Evening_gp,
  Pink_Purple_gp,
  es_autumn_19_gp,
  BlacK_Blue_Magenta_White_gp,
  BlacK_Magenta_Red_gp, // 30
  BlacK_Red_Magenta_Yellow_gp,
  Blue_Cyan_Yellow_gp,
  Trans_Flag_gp,
  // Enby_Flag_gp,
  // Genderqueer_Flag_gp,
  // Pride_Flag_gp,
  // Bi_Flag_gp,
  // Lesbian_Flag_gp,
  // Pan_Flag_gp,
};


// Count of how many cpt-city gradients are defined:
const uint8_t gGradientPaletteCount =
  sizeof( gGradientPalettes) / sizeof( TProgmemRGBGradientPalettePtr );

/* --- */

static int linearBrightness(CRGB color) {
  // I'm looking at you fire_gp
  return (color.r + color.g + color.b);
}

template <class T>
class PaletteManager {
private:
  bool paletteHasColorBelowThreshold(T &palette, uint8_t minBrightness) {
    if (minBrightness == 0) {
      return false;
    }
    for (uint16_t i = 0; i < sizeof(T)/3; ++i) {
      if (linearBrightness(palette.entries[i]) < minBrightness) {
        return true;
      }
    }
    return false;
  }

  uint8_t paletteColorJump(T& palette, bool wrapped=false) {
    uint8_t maxJump = 0;
    CRGB lastColor = palette.entries[(wrapped ? sizeof(T)/3 - 1 : 1)];
    for (uint16_t i = (wrapped ? 0 : 1); i < sizeof(T)/3; ++i) {
      CRGB color = palette.entries[i];
      uint8_t distance = (abs((int)color.r - (int)lastColor.r) + abs((int)color.g - (int)lastColor.g) + abs((int)color.b - (int)lastColor.b)) / 3;
      if (distance > maxJump) {
        maxJump = distance;
      }
      lastColor = color;
    }
    return maxJump;
  }
public:  
  PaletteManager() {
  }

  T getPalette(int choice) {
    return gGradientPalettes[choice];
  }
  
  void getRandomPalette(T* palettePtr, uint8_t minBrightness=0, uint8_t maxColorJump=0xFF) {
    unsigned firstChoice = random16(gGradientPaletteCount);
    unsigned choice = firstChoice;
    *palettePtr = gGradientPalettes[choice];    
    bool belowMinBrightness = paletteHasColorBelowThreshold(*palettePtr, minBrightness);
    uint8_t colorJump = paletteColorJump(*palettePtr);
    int tries = 0;
    while (belowMinBrightness || colorJump > maxColorJump) {
      choice = addmod8(choice, 1, gGradientPaletteCount);
      assert(choice != firstChoice, "No palettes of acceptable brightness & color continuity");
      *palettePtr = gGradientPalettes[choice];
      belowMinBrightness = paletteHasColorBelowThreshold(*palettePtr, minBrightness);
      colorJump = paletteColorJump(*palettePtr);
      tries++;
    }
    logf("  Picked Palette %u, %i tries", choice, tries);
  }
};

/* -------------------------------------------------------------------- */

template<typename PaletteType>
void nblendPaletteTowardPalette(PaletteType& current, PaletteType& target, uint16_t maxChanges)
{
  uint8_t* p1;
  uint8_t* p2;
  uint16_t  changes = 0;

  p1 = (uint8_t*)current.entries;
  p2 = (uint8_t*)target.entries;

  const uint16_t totalChannels = sizeof(PaletteType);
  for( uint16_t i = 0; i < totalChannels; i++) {
    // if the values are equal, no changes are needed
    if( p1[i] == p2[i] ) { continue; }

    // if the current value is less than the target, increase it by one
    if( p1[i] < p2[i] ) { p1[i]++; changes++; }

    // if the current value is greater than the target,
    // increase it by one (or two if it's still greater).
    if( p1[i] > p2[i] ) {
        p1[i]--; changes++;
        if( p1[i] > p2[i] ) { p1[i]--; }
    }

    // if we've hit the maximum number of changes, exit
    if( changes >= maxChanges) { break; }
  }
}

template <typename PaletteType>
class PaletteRotation {
private:
  PaletteManager<PaletteType> manager;
  PaletteType currentPalette;
  PaletteType targetPalette;
protected:
  uint8_t *colorIndexes = NULL;
private:
  uint8_t colorIndexCount = 0;

  void assignPalette(PaletteType* palettePr) {
    manager.getRandomPalette(palettePr, minBrightness, maxColorJump);
  }

public:
  int secondsPerPalette = 10;
  uint8_t minBrightness = 0;
  uint8_t maxColorJump = 0xFF;
  bool pauseRotation = false;
  
  PaletteRotation(int minBrightness=0) {
    this->minBrightness = minBrightness;
    assignPalette(&currentPalette);
    assignPalette(&targetPalette);
  }

  virtual ~PaletteRotation() {
    delete [] colorIndexes;
  }
  
  void paletteRotationTick() {
    if (!pauseRotation) {
      EVERY_N_MILLISECONDS(40) {
        nblendPaletteTowardPalette<PaletteType>(currentPalette, targetPalette, sizeof(PaletteType) / 3);
      }
      EVERY_N_SECONDS(secondsPerPalette) {
        assignPalette(&targetPalette);
      }
    }
  }

  PaletteType& getPalette() {
    paletteRotationTick();
    return currentPalette;
  }

  // unblended override
  virtual void setPalette(PaletteType palette) {
    currentPalette = palette;
  }

  void randomizePalette() {
    assignPalette(&currentPalette);
  }

  CRGB getPaletteColor(uint8_t n, uint8_t brightness = 0xFF) {
    return ColorFromPalette(getPalette(), n, brightness);
  }

  CRGB getMirroredPaletteColor(uint16_t n, uint8_t brightness = 0xFF) {
    n = n % 0x200;
    if (n < 0xFF) {
      return ColorFromPalette(getPalette(), n, brightness);
    } else {
      return ColorFromPalette(getPalette(), 0x100-(n-0x100), brightness);
    }
  }

  CRGB getTrackedColor(uint8_t n, uint8_t *colorIndex=NULL) {
    assert(n < colorIndexCount, "getTrackedColor: index (%u) must be less than tracked color count (%u)", n, colorIndexCount);
    if (n >= colorIndexCount) {
      return CRGB::Black;
    }
    PaletteType& palette = getPalette();
    CRGB color = ColorFromPalette(palette, colorIndexes[n]);
    while (linearBrightness(color) < minBrightness) {
      colorIndexes[n] = addmod8(colorIndexes[n], 1, 0xFF);
      color = ColorFromPalette(palette, colorIndexes[n]);
    }
    if (colorIndex) {
      *colorIndex = colorIndexes[n];
    }
    return color;
  }

  void shiftTrackedColors(uint8_t addend) {
    for (unsigned i = 0; i < colorIndexCount; ++i) {
      colorIndexes[i] = addmod8(colorIndexes[i], addend, 0xFF);
    }
  }

  void prepareTrackedColors(uint8_t count, int paletteCyles=1) {
    if (colorIndexes) {
      delete [] colorIndexes;
    }
    colorIndexCount = count;
    colorIndexes = new uint8_t[colorIndexCount];
    for (unsigned i = 0; i < colorIndexCount; ++i) {
      colorIndexes[i] = paletteCyles * 0xFF * i / colorIndexCount;
    }
  }

  void releaseTrackedColors() {
    if (colorIndexes != NULL) {
      delete [] colorIndexes;
      colorIndexes = NULL;
      colorIndexCount = 0;
    }
  }

  uint8_t trackedColorsCount() {
    return colorIndexCount;
  }
};
