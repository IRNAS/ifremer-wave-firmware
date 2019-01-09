#include "debug_print.h"

void LOG(int level, const char* text, ...)
{
	//Combine text
	char msg[100];
	va_list  args;
	va_start(args, text);
	vsprintf(msg, text, args);
	va_end(args);

	//Write level specifier at the start of the new line
	switch (level) {
	case 0:
	{
		//ERR_PRINT(millis());
		ERR_PRINT("  ERROR: ");
		ERR_PRINTLN(msg);
		return;
	}
	case 1:
	{
		INFO_PRINT(millis());
		INFO_PRINT(" INFO: ");
		INFO_PRINTLN(msg);
		return;
	}
	case 2:
	{
		PRINTOUT_PRINT(millis());
		PRINTOUT_PRINT("  PRINT: ");
		PRINTOUT_PRINTLN(msg);
		return;
	}
	case 3:
	{
		//DEBUG_PRINT(millis());
		DEBUG_PRINT("  DEBUG: ");
		DEBUG_PRINTLN(msg);
		return;
	}
	}
}
