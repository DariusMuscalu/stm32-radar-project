
#include <stdint.h>

#ifndef __FONT_H_
#define __FONT_H_

#define ZEILE_1 0
#define ZEILE_2 8
#define ZEILE_3 16
#define ZEILE_4 24
#define ZEILE_5 32
#define ZEILE_6 40
#define ZEILE_7 48
#define ZEILE_8 56

struct FONT_DEF {
	uint8_t u8Width; /* Character width for storage         */
	uint8_t u8Height; /* Character height for storage        */
	uint8_t u8FirstChar; /* The first character available       */
	uint8_t u8LastChar; /* The last character available        */
	const uint8_t *au8FontTable; /* Font table start address in memory  */
};

extern const struct FONT_DEF Font_5x7;
extern const uint8_t lcdfont5x7[];

#endif
