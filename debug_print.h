#ifndef _DEBUG_PRINT_H_
#define _DEBUG_PRINT_H_

#include <Arduino.h>
#include <stdarg.h>

//Define error logging level
#define DEBUG 1
#pragma region Logging functions

//Error
#if (DEBUG >= 0)
#define ERR_PRINTLN(x){ Serial1.println (x); }
#define ERR_PRINT(x){ Serial1.print (x);}
#else
#define ERR_PRINTLN(x)
#define ERR_PRINT(x)
#endif

//Error
#if (DEBUG >= 1)
#define INFO_PRINTLN(x){ Serial1.println (x); }
#define INFO_PRINT(x){ Serial1.print (x);}
#else
#define INFO_PRINTLN(x)
#define INFO_PRINT(x)
#endif

//Print
#if (DEBUG >= 2)
#define PRINTOUT_PRINTLN(x){ Serial1.println (x); }
#define PRINTOUT_PRINT(x){ Serial1.print (x);}
#else
#define PRINTOUT_PRINTLN(x)
#define PRINTOUT_PRINT(x)
#endif

//Debug
#if (DEBUG >= 3)
#define DEBUG_PRINTLN(x){ Serial1.println (x);}
#define DEBUG_PRINT(x){ Serial1.print (x);}
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

#pragma endregion

void LOG(int, const char*, ...);

#endif 
