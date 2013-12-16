#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"

#include "Types.h"

#include <cross_studio_io.h>

#include "lcd/lcd-320x240.h"
#include "lcd/touchscreen.h"

static void MainTask1( void *pvParameters );
static void MainTask2( void *pvParameters );

int main( void )
{
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_FSMC, ENABLE );
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC3, ENABLE );
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
						  RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
						  RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF |
						  RCC_APB2Periph_GPIOG, ENABLE );
  /* Initialize LCD */
  LCD_Init();
  while( !LCD_LcdIdOk() );

  ts_init();

  void tscal_ui_cb(int x, int y);

  ts_calibrate( &tscal_ui_cb );

  int Cntr = 0;

  int radius = 5;

  while( 1 )
  {
    int x,y;
    int pressed;

    ts_poll(&x,&y,&pressed);

    if( pressed )
      LCD_SetPixel( x, y, LCD_COLOR_WHITE );
  }

  xTaskCreate( MainTask1, ( signed char * ) "Main1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
  xTaskCreate( MainTask2, ( signed char * ) "Main2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );

  vTaskStartScheduler();

  for( ;; );
}

void tscal_ui_cb(int x, int y)
{
  if( x <= 0 )
	  x+=2;
  else
	  x-=2;

  if( y <= 0 )
	  y+=2;
  else
	  y-=2;

  LCD_SetPixel( x, y, LCD_COLOR_WHITE );
  //LCD_DrawCircle( x, y, 5, LCD_COLOR_WHITE );
}

/*-----------------------------------------------------------*/

/* Described at the top of this file. */
void MainTask1( void *pvParameters )
{
  while( true )
  {
    vTaskDelay( 500 );
    debug_printf( "task 1\n" );
    vTaskDelay( 500 );
  }
}

/* Described at the top of this file. */
void MainTask2( void *pvParameters )
{
  while( true )
  {
    vTaskDelay( 500 );
    debug_printf( "task 2\n" );
    vTaskDelay( 500 );
  }
}

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
  /* This function will get called if a task overflows its stack.   If the
  parameters are corrupt then inspect pxCurrentTCB to find which was the
  offending task. */

  ( void ) pxTask;
  ( void ) pcTaskName;

  for( ;; );
}
/*-----------------------------------------------------------*/

void assert_failed( unsigned char *pucFile, unsigned long ulLine )
{
  ( void ) pucFile;
  ( void ) ulLine;

  for( ;; );
}
