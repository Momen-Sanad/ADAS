#include "../../include/MCAL/ADC_INT.h"
#include "../../include/MCAL/BIT_MATH.h"

void ADC_init()
{
    // Enable ADC
    SET_BIT(ADCSRA, ADEN);
    
    // Set the reference voltage to AVCC (5V)
    SET_BIT(ADMUX, REFS0);
    CLEAR_BIT(ADMUX, REFS1);
    
    // Optionally, set the prescaler for ADC speed (ADPS2, ADPS1, ADPS0)
    // For example, a prescaler of 64
    // SET_BIT(ADCSRA, ADPS2);
    // SET_BIT(ADCSRA, ADPS1);
    // CLEAR_BIT(ADCSRA, ADPS0);
}


u16 ADC_read(u8 channelNum)
{
    // Select the channel
    if(channelNum == ADC_CH0)
    {
        CLEAR_BIT(ADMUX, MUX0);
        CLEAR_BIT(ADMUX, MUX1);
        CLEAR_BIT(ADMUX, MUX2);
        CLEAR_BIT(ADMUX, MUX3);
        CLEAR_BIT(ADMUX, MUX4);
    }
    else if(channelNum == ADC_CH1)
    {
        SET_BIT(ADMUX, MUX0);
        CLEAR_BIT(ADMUX, MUX1);
        CLEAR_BIT(ADMUX, MUX2);
        CLEAR_BIT(ADMUX, MUX3);
        CLEAR_BIT(ADMUX, MUX4);
    }

    // Start conversion
    SET_BIT(ADCSRA, ADSC);

    // Wait for conversion to finish
    while (GET_BIT(ADCSRA, ADSC) == 1); 

    // Combine the result from ADCL and ADCH (10-bit result)
    u16 result = ADCL;  // Lower byte
    result |= (ADCH << 8);  // Upper byte (shifted)

    // Return the 10-bit result
    return (ADCL | (ADCH << 8));  // Combine ADCL and ADCH to form the full 10-bit result
}


f32 ADC_convertToAnalog(u16 digital)
{
	return digital * 5.0/1024;	
}