// TO TEST LCD
# define F_CPU 16000000UL

#include "../../include/MCAL/STD_TYPES.h"
#include "../../include/MCAL/BIT_MATH.h"
#include <util/delay.h>

#include "../../include/MCAL/DIO_INT.h"
#include "../../include/HAL/SS_INT.h"
#include "../../include/HAL/LCD_INT.h"

int main(void)
{
    // Initialize the LCD
    LCD_init();
    
    // Clear the display
    LCD_clearDisplay();
    
    // Try writing a few different characters
    LCD_writeChar('2'); // Test with 'A'
    _delay_ms(500);  // Wait to visually confirm
    
    LCD_writeChar('B'); // Test with 'B'
    _delay_ms(500);
    
    LCD_writeStr("TEST123"); // Test with a string
    _delay_ms(1000); // Give enough time to display the string
    
    LCD_writeChar(' ');
    _delay_ms(500);

    LCD_writeStr("Hello");  // Test with another string
    _delay_ms(1000);
}

// TO TEST ADC


// # define F_CPU 16000000UL
// #include "../../include/MCAL/STD_TYPES.h"
// #include "../../include/MCAL/BIT_MATH.h"

// #include "../../include/MCAL/DIO_INT.h"
// #include "../../include/HAL/SS_INT.h"
// #include "../../include/HAL/LCD_INT.h"
// #include "../../include/HAL/KPD_INT.h"
// #include "../../include/MCAL/EXT_INT.h"
// #include "../../include/MCAL/GI_INT.h"
// #include "../../include/MCAL/ADC_INT.h"
// #include <util/delay.h>




// int main()
// {	
// 	ADC_init();
// 	LCD_init();
//     while (1) 
//     {	
// 		u16 digital = ADC_read(ADC_CH1);
// 		f32 analogVoltage = ADC_convertToAnalog(digital);
// 		f32 cel = analogVoltage*100;
// 		LCD_clearDisplay();
// 		LCD_writeFloatNum(cel);
// 		_delay_ms(1000);	
		
//     }
// }

