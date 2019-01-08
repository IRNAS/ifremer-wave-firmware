/* DEBUG LIBRARY 
Change value of DEBUG to adjust level of print-out, defoult in 1:
-1 - no printout
0 - only critical error
1 - important calculation values
2 - Additional measurments and values
3 - General debug print
*/
#ifndef _DEBUG_PRINT_H_
#define _DEBUG_PRINT_H_

#include <Arduino.h>
#include <stdarg.h>

//Define error logging level
#define DEBUG 1
#define STM32_BOARD
#pragma region Logging functions

//Error
#if (DEBUG >= 0)

#define ERR_PRINTLN(x){ 
#ifdef STM32_BOARD
	Serial1.println(x);
#else
	Serial.println(x);
#endif // STM32_BOARD
}
#define ERR_PRINT(x){ 
#ifdef STM32_BOARD
	Serial1.print(x);
#else
	Serial.print(x);
#endif // STM32_BOARD
}
#else
#define ERR_PRINTLN(x)
#define ERR_PRINT(x)
#endif

//Error
#if (DEBUG >= 1)
#define INFO_PRINTLN(x)){ 
#ifdef STM32_BOARD
Serial1.println(x);
#else
Serial.println(x);
#endif // STM32_BOARD
}
#define INFO_PRINT(x){ 
#ifdef STM32_BOARD
Serial1.print(x);
#else
Serial.print(x);
#endif // STM32_BOARD
}
#else
#define INFO_PRINTLN(x)
#define INFO_PRINT(x)
#endif

//Print
#if (DEBUG >= 2)
#define PRINTOUT_PRINTLN(x)){ 
#ifdef STM32_BOARD
Serial1.println(x);
#else
Serial.println(x);
#endif // STM32_BOARD
}
#define PRINTOUT_PRINT(x){ 
#ifdef STM32_BOARD
Serial1.print(x);
#else
Serial.print(x);
#endif // STM32_BOARD
}
#else
#define PRINTOUT_PRINTLN(x)
#define PRINTOUT_PRINT(x)
#endif

//Debug
#if (DEBUG >= 3)
#define DEBUG_PRINTLN(x)){ 
#ifdef STM32_BOARD
Serial1.println(x);
#else
Serial.println(x);
#endif // STM32_BOARD
}
#define DEBUG_PRINT(x){ 
#ifdef STM32_BOARD
Serial1.print(x);
#else
Serial.print(x);
#endif // STM32_BOARD
}
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

#pragma endregion

void LOG(int, const char*, ...);

#endif 
