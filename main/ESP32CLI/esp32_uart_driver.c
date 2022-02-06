//////////////////////////////////////////////////////////////////
// These are the functions required to enable a CLI interface
// over UART using the ESP32-S2 platform
// Many of these are taken from the Demo source code of FreeRTOS
//////////////////////////////////////////////////////////////////

// C includes
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* FreeRTOS includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* Example includes. */
#include "FreeRTOS_CLI.h"

/* Demo application includes. */
#include "serial_driver.h"

#include "esp32_uart_driver.h"

/* Static Functions */
static void prvUARTCommandConsoleTask( void *pvParameters );

/* Const messages output by the command console. */
static const char * const pcWelcomeMessage = "FreeRTOS command server for the ESP32-S2.\r\nType Help to view a list of registered commands.\r\n\r\n>> ";
static const char * const pcNewLine = "\r\n";
#if( EXEC_LAST_CMD_ON_EMPTY_RETURN == 1 )
    static const char * const pcEndOfOutputMessage = "\r\n[Press ENTER to execute the previous command again]\r\n>> ";
#else
    static const char * const pcEndOfOutputMessage = "\r\n>> ";
#endif

/* Used to guard access to the UART in case messages are sent to the UART from
more than one task. */
static SemaphoreHandle_t xTxMutex = NULL;

// This task comes from the Zynq CLI demo from the FreeRTOS source code
// -------------
// This is responsible for receiving all the inputs from the UART console and
// stitching it together to form the command that will be passed to the FreeRTOS CLI routines
// -------------
// This task should be called from the scheduler
static void prvUARTCommandConsoleTask( void *pvParameters )
{
    signed char    cRxedChar[ cmdQUEUE_LENGTH ];          // Character received
    static char    cInputString[ cmdMAX_INPUT_SIZE ];     // This array will hold the characters received in a string
    static char    cEscParamBytes[ cmdMAX_INPUT_SIZE ];   // This array will hold the escape command parameter bytes
    static char    cEscInterBytes[ cmdMAX_INPUT_SIZE ];   // This array will hold the escape command intermeiate bytes
    static char    cEscInitialByte;                       // This will hold the escape command initial byte
    static char    cEscFinalByte;                         // This will hold the escape command final byte
    static char    cLastInputString[ cmdMAX_INPUT_SIZE ]; // This will hold the last command in case the user presses ENTER without any input (i.e. repeats the last command)
    static char    cTmpInputString[ cmdMAX_INPUT_SIZE ];  // This will hold a temporary input string
    uint8_t        ucHistoryIndex      = 0;               // Idex pointer for cLastInputString
    uint8_t        ucInputIndex        = 0;               // Idex pointer for cInputString
    uint8_t        ucParamBytesIndex   = 0;               // Idex pointer for cInputString
    uint8_t        ucInterBytesIndex   = 0;               // Idex pointer for cInputString
    uint8_t        ucCurrInputStrinLen = 0;               // Idex pointer for cInputString
    uint8_t        ucEscCount          = 0;               // Escape Character Count
    uint8_t        ucReceivingEsc      = 0;               // Escape Character Count
    uint8_t        ucRxedCharStrlen    = 0;               // Length of the received string
    uint32_t       uiCharCnt           = 0;               // Char count
    char           *pcOutputString;                       // Pointer to the buffer that will be used to store the CLI application output
    BaseType_t     xReturned;                             // This will hold the exit state of the CLI command (pdFALSE == Command is done generating output strings)
    xComPortHandle xPort;                                 // UART Port
    char           *clear_screen = (char *) malloc(27);

    ( void ) pvParameters; // We are not using the pvParameters right now. Using this to avoid warnings.

    sprintf( clear_screen, "%c[2J\n\r",27 );

    /* Initialize the Input String */
    memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
    memset( cEscParamBytes, 0x00, cmdMAX_INPUT_SIZE );
    memset( cEscInterBytes, 0x00, cmdMAX_INPUT_SIZE );

    /* Obtain the address of the output buffer.  Note there is no mutual
    exclusion on this buffer as it is assumed only one command console interface
    will be used at any one time. */
    pcOutputString = FreeRTOS_CLIGetOutputBuffer();

    /* Initialise the UART. */
    xPort = xSerialPortInitMinimal( configCLI_BAUD_RATE, cmdQUEUE_LENGTH );

    /* Send the welcome message. */
    vSerialPutString( xPort, ( signed char * ) clear_screen, ( unsigned short ) strlen( clear_screen ) );
    vSerialPutString( xPort, ( signed char * ) pcWelcomeMessage, ( unsigned short ) strlen( pcWelcomeMessage ) );

    for( ;; )
    {
        /* Wait for the next character.  The while loop is used in case
        INCLUDE_vTaskSuspend is not set to 1 - in which case portMAX_DELAY will
        be a genuine block time rather than an infinite block time. */
        memset( cRxedChar, 0x00, cmdQUEUE_LENGTH );
        while( xSerialGetChar( xPort, cRxedChar, portMAX_DELAY ) != pdPASS );
        ucRxedCharStrlen = strlen((const char *) cRxedChar);

        /* Ensure exclusive access to the UART Tx. */
        if( xSemaphoreTake( xTxMutex, cmdMAX_MUTEX_WAIT ) == pdPASS )
        {
            ucCurrInputStrinLen = strlen(cInputString);
            for ( uiCharCnt = 0; uiCharCnt < ucRxedCharStrlen ; uiCharCnt++) {

                /* Echo the character back unless it's a backspace and there's nothing there. */
                if ( ((cmdIS_BACKSPACE(cRxedChar[uiCharCnt])  && ucInputIndex > 0) || (cRxedChar[uiCharCnt] >= 0x20)) && (ucReceivingEsc == 0) && (ucCurrInputStrinLen == ucInputIndex) )  {
                    xSerialPutChar( xPort, cRxedChar[uiCharCnt], portMAX_DELAY );
                }

                /* Was it the end of the line? */
                if( cRxedChar[uiCharCnt] == '\n' || cRxedChar[uiCharCnt] == '\r' )
                {


                    /* See if the command is empty, indicating that the last command
                    is to be executed again. */
                    if( ucInputIndex == 0 )
                    {
                        #if( EXEC_LAST_CMD_ON_EMPTY_RETURN == 1 )
                            /* Copy the last command back into the input string. */
                            strcpy( cInputString, cLastInputString );
                            // Printout the command
                            vSerialPutString( xPort, ( signed char * ) cInputString, ( unsigned short ) strlen( cInputString ) );
                        #else
                            goto printEndOfOutputMessage;
                        #endif
                    }

            /* Just to space the output from the input. */
                    vSerialPutString( xPort, ( signed char * ) pcNewLine, ( unsigned short ) strlen( pcNewLine ) );

                    /* Pass the received command to the command interpreter.  The
                    command interpreter is called repeatedly until it returns
                    pdFALSE (indicating there is no more output) as it might
                    generate more than one string. */
                    memset(pcOutputString, '\0', configCOMMAND_INT_MAX_OUTPUT_SIZE);
                    do
                    {

                        /* Get the next output string from the command interpreter. */
                        xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

                        /* Write the generated string to the UART. */
                        if ( pcOutputString[ 0 ] != 0x0 ) vSerialPutString( xPort, ( signed char * ) pcOutputString, ( unsigned short ) strlen( pcOutputString ) );

                    } while( xReturned != pdFALSE );

                    /* All the strings generated by the input command have been
                    sent.  Clear the input string ready to receive the next command.
                    Remember the command that was just processed first in case it is
                    to be processed again. */
                    strcpy( cLastInputString, cInputString );
                    ucInputIndex = 0;
                    memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
                    // Reset history
                    ucHistoryIndex = 0;

                    printEndOfOutputMessage: vSerialPutString( xPort, ( signed char * ) pcEndOfOutputMessage, ( unsigned short ) strlen( pcEndOfOutputMessage ) );
                }
                else
                {
                    if( cmdIS_BACKSPACE(cRxedChar[uiCharCnt]) )
                    {
                        /* Backspace was pressed.  Erase the last character in the
                        string - if any. */
                        if( ucInputIndex > 0 )
                        {
                            if ( ucCurrInputStrinLen > ucInputIndex ) {
                                // Go back one index
                                ucInputIndex--;
                                xSerialPutChar( xPort, '\b', portMAX_DELAY );

                                // Shift left all chars from the new index to the end
                                for(uint8_t i = ucInputIndex; i < ucCurrInputStrinLen; i++) {
                                    cInputString[ i ] = cInputString[i + 1];
                                }

                                // Save the current console pointer
                                cmdSAVE_CURSOR(xPort);
                                // Print the new shifted string
                                vSerialPutString( xPort, ( signed char * ) &cInputString[ucInputIndex], ( ucCurrInputStrinLen - ucInputIndex ) );
                                xSerialPutChar( xPort, ' ', portMAX_DELAY );
                                // Restore the console pointer
                                cmdRESTORE_CURSOR(xPort);
                            } else {
                                ucInputIndex--;
                                cInputString[ ucInputIndex ] = '\0';
                                xSerialPutChar( xPort, ' ', portMAX_DELAY );
                                xSerialPutChar( xPort, '\b', portMAX_DELAY );
                            }
                        }
                    }
                    else if ( cmdIS_DEL(cRxedChar[uiCharCnt]) ) {
                        if ( ucCurrInputStrinLen > ucInputIndex ) {
                            for(uint8_t i = ucInputIndex; i < ucCurrInputStrinLen; i++) {
                                cInputString[ i ] = cInputString[ i + 1];
                            }
                            cmdSAVE_CURSOR(xPort);
                            vSerialPutString( xPort, ( signed char * ) &cInputString[ucInputIndex], ( ucCurrInputStrinLen - ucInputIndex ) );
                            xSerialPutChar( xPort, ' ', portMAX_DELAY );
                            cmdRESTORE_CURSOR(xPort);
                        }
                    }
                    // Start processing the Escape Character
                    // 3 Parts: 1) Recieve the esc char. 2) Get the first byte. 3) Get subsequent bytes until the end and execute the command
                    else if ( cmdIS_ESCAPE(cRxedChar[uiCharCnt]) ) // Part 1 - Recieve the esc char.
                    {
                        // Initialize the escape sequences
                        ucReceivingEsc = 1;
                        memset( cEscParamBytes, 0x00, cmdMAX_INPUT_SIZE );
                        memset( cEscInterBytes, 0x00, cmdMAX_INPUT_SIZE );
                        cEscInitialByte   = 0x00;
                        cEscFinalByte     = 0x00;
                        ucParamBytesIndex = 0;
                        ucInterBytesIndex = 0;
                    }
                    else if (ucReceivingEsc == 1) // Part 2 - Get the first byte.
                    {
                        // Get the first byte
                        if (cRxedChar[uiCharCnt] >= 0x40 && cRxedChar[uiCharCnt] <= 0x5f) {
                            cEscInitialByte = cRxedChar[uiCharCnt];
                            ucReceivingEsc = 2;
                        } else {
                            ucReceivingEsc = 0;
                        }
                    }
                    else if (ucReceivingEsc == 2) // Part 3 - Get subsequent bytes until the end and execute the command
                    {
                        if (cEscInitialByte == cmdASCII_CTRL_SEQ_INTRODUCER) { // Check for Control Sequence Introducer
                            // Get the subsequent bytes
                            if( cRxedChar[uiCharCnt] >=  0x30 && cRxedChar[uiCharCnt] <= 0x3f) {        // Parameter bytes
                                cEscParamBytes[ ucParamBytesIndex ] = cRxedChar[uiCharCnt];
                                ucParamBytesIndex++;
                            } else if( cRxedChar[uiCharCnt] >=  0x20 && cRxedChar[uiCharCnt] <= 0x2f) { // Intermediate bytes
                                cEscInterBytes[ ucInterBytesIndex ] = cRxedChar[uiCharCnt];
                                ucInterBytesIndex++;
                            } else if( cRxedChar[uiCharCnt] >=  0x40 && cRxedChar[uiCharCnt] <= 0x7e) { // Final byte
                                cEscFinalByte  = cRxedChar[uiCharCnt];
                                ucReceivingEsc = 0;
                            }

                            // Execute the command
                            if ( ucReceivingEsc == 0 ) {
                                if ( cEscFinalByte == 'A' && ucHistoryIndex < 1 ) // Up arrow
                                {
                                    // Save the current string
                                    memset( cTmpInputString, 0x00, sizeof(char)*cmdMAX_INPUT_SIZE );
                                    strcpy( cTmpInputString, cInputString );
                                    // Copy the previous string to the current one
                                    memset( cInputString, 0x00, sizeof(char)*cmdMAX_INPUT_SIZE );
                                    strcpy( cInputString, cLastInputString );

                                    // Clear the current line
                                    if ( ucInputIndex > 0 ) {
                                        // Move cursor all the way to the beginning
                                        cmdMOVE_CURSOR_LEFT_POS(xPort, ucInputIndex)
                                        // Delete the line
                                        cmdERASE_LINE_FROM_CURSOR(xPort);
                                        // Print the new line
                                    }

                                    // Print the previous command
                                    vSerialPutString(xPort, (signed char *) cInputString, strlen(cInputString));

                                    // Get the current cursor position
                                    ucInputIndex        = strlen(cInputString);
                                    ucCurrInputStrinLen = strlen(cInputString);

                                    // Set the history pointer // TODO: Add more than 1
                                    ucHistoryIndex = 1;
                                } // End Up arrow
                                else if ( cEscFinalByte == 'B' && ucHistoryIndex > 0 ) // Down arrow
                                {
                                    // Restore from the cTmpInputString
                                    memset( cInputString, 0x00, sizeof(char)*cmdMAX_INPUT_SIZE );
                                    strcpy( cInputString, cTmpInputString );

                                    // Clear the current line
                                    if ( ucInputIndex > 0 ) {
                                        // Move cursor all the way to the beginning
                                        cmdMOVE_CURSOR_LEFT_POS(xPort, ucInputIndex)
                                        // Delete the line
                                        cmdERASE_LINE_FROM_CURSOR(xPort);
                                        // Print the new line
                                    }

                                    // Print the saved command
                                    vSerialPutString(xPort, (signed char *) cInputString, strlen(cInputString));

                                    // Get the current cursor position
                                    ucInputIndex        = strlen(cInputString);
                                    ucCurrInputStrinLen = strlen(cInputString);
                                    // Decrease the history pointer // TODO: Add more than 1
                                    ucHistoryIndex -= 1;
                                } // End Down arrow
                                else if ( cEscFinalByte == 'C' ) // Right arrow
                                {
                                    if ( ucInputIndex < ucCurrInputStrinLen ) {
                                        cmdMOVE_CURSOR_RIGHT(xPort)
                                        ucInputIndex++;
                                    }
                                } // End Right arrow
                                else if ( cEscFinalByte == 'D' ) // Left Arrow
                                {
                                    if ( ucInputIndex > 0 ) {
                                        cmdMOVE_CURSOR_LEFT(xPort)
                                        ucInputIndex--;
                                    }
                                } // End Left Arrow
                                else if ( cEscFinalByte == '~' )  // VT Sequence
                                {
                                    if (ucParamBytesIndex == 0) goto giveMutexSemaphore; // Must have a parameter

                                    if ( cEscParamBytes[0] == '3' ) { // Delete
                                        if ( ucCurrInputStrinLen > ucInputIndex ) {
                                            for(uint8_t i = ucInputIndex; i < ucCurrInputStrinLen; i++) {
                                                cInputString[ i ] = cInputString[ i + 1];
                                            }
                                            cmdSAVE_CURSOR(xPort);
                                            vSerialPutString( xPort, ( signed char * ) &cInputString[ucInputIndex], ( ucCurrInputStrinLen - ucInputIndex ) );
                                            xSerialPutChar( xPort, ' ', portMAX_DELAY );
                                            cmdRESTORE_CURSOR(xPort);
                                        }
                                    }
                                } // End VT Sequence
                            }
                        }
                    }
                    else
                    {
                        /* A character was entered.  Add it to the string entered so
                        far.  When a \n is entered the complete string will be
                        passed to the command interpreter. */
                        #if( INPUT_IS_ASCII_ONLY == 1 )
                            if( ( cRxedChar[uiCharCnt] >= ' ' ) && ( cRxedChar[uiCharCnt] <= '~' ) )
                            {
                                if( ucInputIndex < cmdMAX_INPUT_SIZE )
                                {
                                    cInputString[ ucInputIndex ] = cRxedChar[uiCharCnt];
                                    ucInputIndex++;
                                }
                            }
                        #else
                            if( (ucInputIndex < cmdMAX_INPUT_SIZE) && (ucEscCount == 0) )
                            {

                                // If the cursor is in the middle of the string, move the right side of the string to accomodate the new char
                                if ( ucCurrInputStrinLen > ucInputIndex ) {
                                    for (uint8_t i = ucCurrInputStrinLen; i > ucInputIndex; i--) {
                                        cInputString[ i ] = cInputString[ i-1 ];
                                    }
                                    cInputString[ ucInputIndex ] = cRxedChar[uiCharCnt];
                                    xSerialPutChar( xPort, cRxedChar[uiCharCnt], portMAX_DELAY );
                                    vSerialPutString( xPort, ( signed char * ) cmdANSI_ESC_SAVE_CURSOR, strlen(cmdANSI_ESC_SAVE_CURSOR) );
                                    vSerialPutString( xPort, ( signed char * ) &cInputString[ucInputIndex+1], ( ucCurrInputStrinLen - ucInputIndex + 1) );
                                    vSerialPutString( xPort, ( signed char * ) cmdANSI_ESC_RESTORE_CURSOR, strlen(cmdANSI_ESC_RESTORE_CURSOR) );
                                } else {
                                    cInputString[ ucInputIndex ] = cRxedChar[uiCharCnt];
                                }
                                ucInputIndex++;
                            }
                        #endif
                    }
                }
            }

            /* Must ensure to give the mutex back. */
            giveMutexSemaphore: xSemaphoreGive( xTxMutex );
        }
    }
}

// This task comes from the Zynq CLI demo from the FreeRTOS source code
// -------------
// This function creates the prvUARTCommandConsoleTask
// This is intended to be called in the initialization phases
void vUARTCommandConsoleStart( configSTACK_DEPTH_TYPE usStackSize, UBaseType_t uxPriority )
{
    /* Create the semaphore used to access the UART Tx. */
    xTxMutex = xSemaphoreCreateMutex();
    configASSERT( xTxMutex );

    /* Create that task that handles the console itself. */
    xTaskCreate(     prvUARTCommandConsoleTask, /* The task that implements the command console. */
                    "CLI",                      /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
                    usStackSize,                /* The size of the stack allocated to the task. */
                    NULL,                       /* The parameter is not used, so NULL is passed. */
                    uxPriority,                 /* The priority allocated to the task. */
                    NULL );                     /* A handle is not required, so just pass NULL. */
}
