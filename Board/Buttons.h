#ifndef __BUTTONS_ADAFRUITU4_H__
#define __BUTTONS_ADAFRUITU4_H__

	/* Macros: */
		#define __INCLUDE_FROM_BUTTONS_H

	/* Includes: */
		#include <LUFA/Common/Common.h>

		#if (BOARD == BOARD_ADAFRUITU4)
			#define BUTTONS_BUTTON1  0
			static inline void       Buttons_Init(void) {};
			static inline uint_reg_t Buttons_GetStatus(void) { return 0; };
		#endif
#endif
