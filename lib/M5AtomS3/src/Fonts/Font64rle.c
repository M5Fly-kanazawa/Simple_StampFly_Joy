// Font 6 is intended to display numbers and time
//
// This font has been 8 bit Run Length Encoded to save FLASH space
//
// This font only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : - . a p m
// The Pipe character | is a narrow space to aid formatting
// All other characters print as a space
#include <pgmspace.h>
PROGMEM const unsigned char widtbl_f64[96] =  // character width table
    {
        12, 12, 12, 12, 12, 12, 12, 12,  // char 32 - 39
        12, 12, 12, 12, 12, 17, 15, 12,  // char 40 - 47
        27, 27, 27, 27, 27, 27, 27, 27,  // char 48 - 55
        27, 27, 15, 12, 12, 12, 12, 12,  // char 56 - 63
        12, 12, 12, 12, 12, 12, 12, 12,  // char 64 - 71
        12, 12, 12, 12, 12, 12, 12, 12,  // char 72 - 79
        12, 12, 12, 12, 12, 12, 12, 12,  // char 80 - 87
        12, 12, 12, 12, 12, 12, 12, 12,  // char 88 - 95
        12, 27, 12, 12, 12, 12, 12, 12,  // char 96 - 103
        12, 12, 12, 12, 12, 42, 12, 12,  // char 104 - 111
        29, 12, 12, 12, 12, 12, 12, 12,  // char 112 - 119
        12, 12, 12, 12, 7,  12, 12, 12   // char 120 - 127
};

PROGMEM const unsigned char chr_f64_20[] = {0x7F, 0x7F, 0x7F, 0x7F, 0x3F};

PROGMEM const unsigned char chr_f64_2D[] = {0x7F, 0x7F, 0x45, 0x8A, 0x05,
                                            0x8A, 0x05, 0x8A, 0x05, 0x8A,
                                            0x7F, 0x7F, 0x7F, 0x2B};

PROGMEM const unsigned char chr_f64_2E[] = {0x7F, 0x7F, 0x7F, 0x55, 0x84,
                                            0x09, 0x84, 0x09, 0x84, 0x09,
                                            0x84, 0x09, 0x84, 0x7F, 0x38};

PROGMEM const unsigned char chr_f64_30[] = {
    0x23, 0x87, 0x0F, 0x8D, 0x0B, 0x8F, 0x09, 0x91, 0x07, 0x86, 0x05, 0x86,
    0x06, 0x84, 0x09, 0x84, 0x06, 0x83, 0x0B, 0x83, 0x05, 0x84, 0x0B, 0x84,
    0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83,
    0x03, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x03, 0x83, 0x0D, 0x83,
    0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83, 0x04, 0x84, 0x0B, 0x84,
    0x05, 0x83, 0x0B, 0x83, 0x06, 0x84, 0x09, 0x84, 0x06, 0x86, 0x05, 0x86,
    0x07, 0x91, 0x09, 0x8F, 0x0B, 0x8D, 0x0F, 0x87, 0x7F, 0x7F, 0x32};

PROGMEM const unsigned char chr_f64_31[] = {
    0x29, 0x82, 0x17, 0x82, 0x16, 0x83, 0x16, 0x83, 0x15, 0x84, 0x14,
    0x85, 0x12, 0x87, 0x0D, 0x8C, 0x0D, 0x8C, 0x0D, 0x87, 0x00, 0x83,
    0x0D, 0x84, 0x03, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16,
    0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83,
    0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16,
    0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83,
    0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x7F, 0x7F, 0x4C};

PROGMEM const unsigned char chr_f64_32[] = {
    0x24, 0x87, 0x10, 0x8C, 0x0B, 0x8F, 0x09, 0x91, 0x08, 0x85, 0x05, 0x86,
    0x06, 0x84, 0x09, 0x84, 0x06, 0x84, 0x0A, 0x83, 0x06, 0x83, 0x0B, 0x84,
    0x04, 0x84, 0x0C, 0x83, 0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83,
    0x04, 0x83, 0x0D, 0x83, 0x16, 0x83, 0x15, 0x84, 0x15, 0x84, 0x14, 0x84,
    0x14, 0x85, 0x13, 0x85, 0x12, 0x86, 0x11, 0x87, 0x10, 0x88, 0x10, 0x88,
    0x0F, 0x88, 0x10, 0x87, 0x11, 0x86, 0x12, 0x85, 0x13, 0x85, 0x14, 0x84,
    0x14, 0x84, 0x15, 0x83, 0x16, 0x83, 0x15, 0x96, 0x03, 0x96, 0x03, 0x96,
    0x03, 0x96, 0x7F, 0x7F, 0x45};

PROGMEM const unsigned char chr_f64_33[] = {
    0x23, 0x87, 0x0F, 0x8D, 0x0B, 0x90, 0x08, 0x91, 0x07, 0x86, 0x05, 0x86,
    0x06, 0x84, 0x09, 0x84, 0x05, 0x84, 0x0B, 0x84, 0x04, 0x84, 0x0B, 0x84,
    0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83,
    0x15, 0x84, 0x15, 0x84, 0x14, 0x84, 0x14, 0x85, 0x0D, 0x8B, 0x0E, 0x89,
    0x10, 0x8B, 0x0E, 0x8C, 0x14, 0x86, 0x15, 0x84, 0x16, 0x84, 0x15, 0x84,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x84, 0x0D, 0x84, 0x02, 0x84, 0x0C, 0x85, 0x03, 0x84, 0x0B, 0x84,
    0x04, 0x85, 0x09, 0x85, 0x05, 0x86, 0x05, 0x86, 0x07, 0x91, 0x09, 0x8F,
    0x0B, 0x8D, 0x0F, 0x87, 0x7F, 0x7F, 0x4D};

PROGMEM const unsigned char chr_f64_34[] = {
    0x45, 0x83, 0x15, 0x84, 0x14, 0x85, 0x13, 0x86, 0x13, 0x86, 0x12, 0x87,
    0x11, 0x83, 0x00, 0x83, 0x11, 0x83, 0x00, 0x83, 0x10, 0x83, 0x01, 0x83,
    0x0F, 0x83, 0x02, 0x83, 0x0E, 0x84, 0x02, 0x83, 0x0E, 0x83, 0x03, 0x83,
    0x0D, 0x83, 0x04, 0x83, 0x0C, 0x83, 0x05, 0x83, 0x0B, 0x84, 0x05, 0x83,
    0x0B, 0x83, 0x06, 0x83, 0x0A, 0x83, 0x07, 0x83, 0x09, 0x84, 0x07, 0x83,
    0x09, 0x83, 0x08, 0x83, 0x08, 0x83, 0x09, 0x83, 0x07, 0x83, 0x0A, 0x83,
    0x07, 0x97, 0x02, 0x97, 0x02, 0x97, 0x02, 0x97, 0x11, 0x83, 0x16, 0x83,
    0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83,
    0x16, 0x83, 0x7F, 0x7F, 0x4A};

PROGMEM const unsigned char chr_f64_35[] = {
    0x3A, 0x91, 0x08, 0x91, 0x08, 0x91, 0x08, 0x91, 0x08, 0x82, 0x16, 0x83,
    0x16, 0x83, 0x16, 0x83, 0x16, 0x82, 0x17, 0x82, 0x17, 0x82, 0x16, 0x83,
    0x02, 0x87, 0x0B, 0x83, 0x00, 0x8B, 0x09, 0x91, 0x08, 0x92, 0x07, 0x86,
    0x05, 0x86, 0x06, 0x84, 0x09, 0x85, 0x05, 0x83, 0x0B, 0x84, 0x16, 0x83,
    0x16, 0x84, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83,
    0x03, 0x83, 0x0E, 0x83, 0x03, 0x83, 0x0D, 0x84, 0x03, 0x83, 0x0D, 0x83,
    0x04, 0x84, 0x0B, 0x84, 0x05, 0x84, 0x09, 0x85, 0x05, 0x86, 0x05, 0x86,
    0x07, 0x91, 0x09, 0x8F, 0x0B, 0x8D, 0x0F, 0x87, 0x7F, 0x7F, 0x32};

PROGMEM const unsigned char chr_f64_36[] = {
    0x24, 0x86, 0x11, 0x8B, 0x0C, 0x8E, 0x0A, 0x90, 0x08, 0x86, 0x05, 0x85,
    0x06, 0x85, 0x09, 0x84, 0x05, 0x84, 0x0B, 0x83, 0x05, 0x83, 0x0C, 0x84,
    0x03, 0x84, 0x0D, 0x83, 0x03, 0x83, 0x0E, 0x83, 0x03, 0x83, 0x16, 0x83,
    0x15, 0x83, 0x16, 0x83, 0x04, 0x86, 0x0A, 0x83, 0x02, 0x8B, 0x07, 0x83,
    0x00, 0x8E, 0x06, 0x94, 0x05, 0x88, 0x05, 0x86, 0x04, 0x86, 0x09, 0x84,
    0x04, 0x85, 0x0B, 0x84, 0x03, 0x84, 0x0D, 0x83, 0x03, 0x84, 0x0D, 0x84,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x03, 0x83, 0x0D, 0x84,
    0x03, 0x83, 0x0D, 0x83, 0x04, 0x84, 0x0B, 0x84, 0x05, 0x84, 0x09, 0x85,
    0x05, 0x86, 0x05, 0x86, 0x07, 0x91, 0x09, 0x8F, 0x0C, 0x8C, 0x0F, 0x88,
    0x7F, 0x7F, 0x31};

PROGMEM const unsigned char chr_f64_37[] = {
    0x37, 0x96, 0x03, 0x96, 0x03, 0x96, 0x03, 0x96, 0x16, 0x83, 0x15, 0x83,
    0x15, 0x83, 0x15, 0x83, 0x15, 0x84, 0x14, 0x84, 0x14, 0x84, 0x15, 0x83,
    0x15, 0x84, 0x14, 0x84, 0x15, 0x84, 0x14, 0x84, 0x15, 0x83, 0x15, 0x84,
    0x15, 0x83, 0x15, 0x84, 0x15, 0x84, 0x15, 0x83, 0x15, 0x84, 0x15, 0x83,
    0x16, 0x83, 0x15, 0x84, 0x15, 0x83, 0x16, 0x83, 0x16, 0x83, 0x16, 0x83,
    0x15, 0x84, 0x15, 0x83, 0x16, 0x83, 0x16, 0x83, 0x7F, 0x7F, 0x53};

PROGMEM const unsigned char chr_f64_38[] = {
    0x24, 0x85, 0x11, 0x8B, 0x0C, 0x8F, 0x09, 0x91, 0x07, 0x86, 0x05, 0x86,
    0x06, 0x84, 0x09, 0x84, 0x05, 0x84, 0x0B, 0x84, 0x04, 0x84, 0x0B, 0x84,
    0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83, 0x04, 0x83, 0x0D, 0x83,
    0x04, 0x84, 0x0B, 0x84, 0x05, 0x83, 0x0B, 0x83, 0x06, 0x84, 0x09, 0x84,
    0x07, 0x85, 0x05, 0x85, 0x09, 0x8F, 0x0B, 0x8D, 0x0B, 0x8F, 0x09, 0x91,
    0x07, 0x85, 0x07, 0x85, 0x05, 0x84, 0x0B, 0x84, 0x04, 0x83, 0x0D, 0x83,
    0x03, 0x84, 0x0D, 0x84, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x84, 0x0D, 0x84, 0x02, 0x84, 0x0D, 0x84, 0x03, 0x84, 0x0B, 0x84,
    0x04, 0x86, 0x07, 0x86, 0x05, 0x93, 0x07, 0x91, 0x09, 0x8F, 0x0D, 0x89,
    0x7F, 0x7F, 0x31};

PROGMEM const unsigned char chr_f64_39[] = {
    0x22, 0x88, 0x0F, 0x8C, 0x0C, 0x8F, 0x09, 0x91, 0x07, 0x86, 0x05, 0x86,
    0x05, 0x85, 0x09, 0x84, 0x05, 0x84, 0x0B, 0x84, 0x04, 0x83, 0x0D, 0x83,
    0x03, 0x84, 0x0D, 0x83, 0x03, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83, 0x02, 0x83, 0x0F, 0x83,
    0x02, 0x84, 0x0D, 0x84, 0x03, 0x83, 0x0D, 0x84, 0x03, 0x84, 0x0B, 0x85,
    0x04, 0x84, 0x09, 0x86, 0x04, 0x86, 0x05, 0x88, 0x05, 0x94, 0x06, 0x8E,
    0x00, 0x83, 0x07, 0x8B, 0x02, 0x83, 0x0A, 0x86, 0x04, 0x83, 0x16, 0x83,
    0x15, 0x83, 0x16, 0x83, 0x03, 0x83, 0x0E, 0x83, 0x03, 0x83, 0x0D, 0x84,
    0x03, 0x84, 0x0C, 0x83, 0x05, 0x83, 0x0B, 0x84, 0x05, 0x84, 0x09, 0x85,
    0x06, 0x85, 0x05, 0x86, 0x08, 0x90, 0x0A, 0x8E, 0x0C, 0x8B, 0x11, 0x86,
    0x7F, 0x7F, 0x33};

PROGMEM const unsigned char chr_f64_3A[] = {
    0x7F, 0x1A, 0x84, 0x09, 0x84, 0x09, 0x84, 0x09, 0x84, 0x09, 0x84, 0x7F,
    0x3D, 0x84, 0x09, 0x84, 0x09, 0x84, 0x09, 0x84, 0x09, 0x84, 0x7F, 0x74};

PROGMEM const unsigned char chr_f64_61[] = {
    0x7F, 0x7B, 0x88, 0x0F, 0x8D, 0x0A, 0x90, 0x09, 0x91, 0x07, 0x85, 0x06,
    0x85, 0x06, 0x84, 0x0A, 0x84, 0x05, 0x83, 0x0C, 0x83, 0x05, 0x83, 0x0C,
    0x83, 0x05, 0x83, 0x0C, 0x83, 0x16, 0x83, 0x15, 0x84, 0x0E, 0x8B, 0x09,
    0x90, 0x07, 0x92, 0x06, 0x8D, 0x01, 0x83, 0x05, 0x88, 0x07, 0x83, 0x05,
    0x84, 0x0B, 0x83, 0x04, 0x84, 0x0C, 0x83, 0x04, 0x83, 0x0D, 0x83, 0x04,
    0x83, 0x0D, 0x83, 0x04, 0x83, 0x0C, 0x84, 0x04, 0x83, 0x0B, 0x85, 0x04,
    0x84, 0x09, 0x86, 0x05, 0x84, 0x06, 0x8A, 0x03, 0x8F, 0x01, 0x84, 0x04,
    0x8D, 0x02, 0x84, 0x05, 0x8A, 0x05, 0x83, 0x07, 0x86, 0x7F, 0x7F, 0x35};

PROGMEM const unsigned char chr_f64_6D[] = {
    0x7F, 0x7F, 0x7F, 0x05, 0x86, 0x07, 0x86, 0x0A, 0x83, 0x02, 0x89, 0x04,
    0x8A, 0x08, 0x83, 0x00, 0x8C, 0x02, 0x8C, 0x07, 0x83, 0x00, 0x8D, 0x00,
    0x8E, 0x06, 0x87, 0x05, 0x89, 0x05, 0x84, 0x06, 0x85, 0x08, 0x86, 0x08,
    0x84, 0x05, 0x84, 0x0A, 0x84, 0x0A, 0x83, 0x05, 0x84, 0x0A, 0x84, 0x0A,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B,
    0x83, 0x05, 0x83, 0x0B, 0x83, 0x0B, 0x83, 0x7F, 0x7F, 0x7F, 0x7A};

PROGMEM const unsigned char chr_f64_70[] = {
    0x7F, 0x7F, 0x10, 0x86, 0x0C, 0x83, 0x02, 0x8B, 0x09, 0x83, 0x01, 0x8D,
    0x08, 0x83, 0x00, 0x8F, 0x07, 0x88, 0x05, 0x86, 0x06, 0x86, 0x09, 0x84,
    0x06, 0x85, 0x0B, 0x84, 0x05, 0x84, 0x0D, 0x83, 0x05, 0x84, 0x0D, 0x83,
    0x05, 0x84, 0x0D, 0x84, 0x04, 0x83, 0x0F, 0x83, 0x04, 0x83, 0x0F, 0x83,
    0x04, 0x83, 0x0F, 0x83, 0x04, 0x83, 0x0F, 0x83, 0x04, 0x83, 0x0F, 0x83,
    0x04, 0x83, 0x0F, 0x83, 0x04, 0x83, 0x0F, 0x83, 0x04, 0x83, 0x0F, 0x83,
    0x04, 0x83, 0x0E, 0x84, 0x04, 0x84, 0x0D, 0x83, 0x05, 0x84, 0x0C, 0x84,
    0x05, 0x85, 0x0B, 0x84, 0x05, 0x86, 0x09, 0x84, 0x06, 0x88, 0x05, 0x86,
    0x06, 0x83, 0x00, 0x8F, 0x07, 0x83, 0x01, 0x8D, 0x08, 0x83, 0x02, 0x8B,
    0x09, 0x83, 0x04, 0x86, 0x0C, 0x83, 0x18, 0x83, 0x18, 0x83, 0x18, 0x83,
    0x18, 0x83, 0x18, 0x83, 0x18, 0x83, 0x18, 0x83, 0x18, 0x83, 0x18, 0x83,
    0x32};
PROGMEM const unsigned char* const chrtbl_f64[96] =  // character pointer table
    {chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_2D, chr_f64_2E, chr_f64_20, chr_f64_30, chr_f64_31,
     chr_f64_32, chr_f64_33, chr_f64_34, chr_f64_35, chr_f64_36, chr_f64_37,
     chr_f64_38, chr_f64_39, chr_f64_3A, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_61,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_6D,
     chr_f64_20, chr_f64_20, chr_f64_70, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20,
     chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20, chr_f64_20};
