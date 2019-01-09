/* DEBUG LIBRARY 
Change value of DEBUG to adjust level of print-out, default in 1:
-1 - no printout
0 - only critical error
1 - important calculation values
2 - Additional measurements and values
3 - General debug print
*/
#ifndef _DEBUG_PRINT_H_
#define _DEBUG_PRINT_H_

#include <Arduino.h>
#include <stdarg.h>

//Define error logging level
#define DEBUG 1
#define STM32_BOARD

#ifdef STM32_BOARD
#define PRINTLN(x){ Serial1.println (x); }
#define PRINT(x){ Serial1.print (x);}
#else
#define PRINTLN(x){ Serial.println (x); }
#define PRINT(x){ Serial.print (x);}
#endif // STM32_BOARD


#pragma region Logging functions

//Error
#if (DEBUG >= 0)
#define ERR_PRINTLN(x){ PRINTLN(x); }
#define ERR_PRINT(x){ PRINT(x);}
#else
#define ERR_PRINTLN(x)
#define ERR_PRINT(x)
#endif

//Error
#if (DEBUG >= 1)
#define INFO_PRINTLN(x){ PRINTLN(x); }
#define INFO_PRINT(x){ PRINT(x);}
#else
#define INFO_PRINTLN(x)
#define INFO_PRINT(x)
#endif

//Print
#if (DEBUG >= 2)
#define PRINTOUT_PRINTLN(x){ PRINTLN(x); }
#define PRINTOUT_PRINT(x){ PRINT(x);}
#else
#define PRINTOUT_PRINTLN(x)
#define PRINTOUT_PRINT(x)
#endif

//Debug
#if (DEBUG >= 3)
#define DEBUG_PRINTLN(x){ PRINTLN(x);}
#define DEBUG_PRINT(x){ PRINT(x);}
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

#pragma endregion

void LOG(int, const char*, ...);

#endif 

