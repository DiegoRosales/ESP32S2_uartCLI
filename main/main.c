#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FreeRTOS_CLI.h"

#include "esp32_uart_driver.h"

#define cliNEW_LINE "\n\r"
#define APPEND_NEWLINE(BUFFER) strcat( BUFFER, cliNEW_LINE )


static BaseType_t prv_xEchoCMD( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

// Echo Command
static const CLI_Command_Definition_t prv_xEchoCMD_definition =
{
    "echo",
    "\r\necho:\r\n Simple echo command.\r\n",
    prv_xEchoCMD, /* The function to run. */
    -1 /* The user can enter any number of commands. */
};


//////////////////////////////////////////////////////
// CLI Command Implementations
//////////////////////////////////////////////////////

static BaseType_t prv_xEchoCMD( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( uxParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( pcWriteBuffer, "The parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							uxParameterNumber,		/* Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);

		if( pcParameter != NULL )
		{
			/* Return the parameter string. */
			memset( pcWriteBuffer, 0x00, xWriteBufferLen );
			sprintf( pcWriteBuffer, "%d: ", ( int ) uxParameterNumber );
			strncat( pcWriteBuffer, ( char * ) pcParameter, ( size_t ) xParameterStringLength );
			APPEND_NEWLINE(pcWriteBuffer);

            uxParameterNumber = 0;
			/* There might be more parameters to return after this one. */
			xReturn = pdFALSE;
			//uxParameterNumber++;
		}
		else
		{
			/* No more parameters were found.  Make sure the write buffer does
			not contain a valid string. */
			pcWriteBuffer[ 0 ] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			uxParameterNumber = 0;
		}
	}

	return xReturn;
}

void app_main(void) {
  printf("Initializing FreeRTOS Console");

  FreeRTOS_CLIRegisterCommand( &prv_xEchoCMD_definition );

  vUARTCommandConsoleStart( mainUART_COMMAND_CONSOLE_STACK_SIZE, mainUART_COMMAND_CONSOLE_TASK_PRIORITY );

}


