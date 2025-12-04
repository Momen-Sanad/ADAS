#include "../../include/MCAL/WATCHDOG_INT.h"
#include "../../include/MCAL/BIT_MATH.h"

void WATCHDOG_init(u8 prescaler)
{
    // Enable change of WDT settings (must be followed by ONE write within 4 cycles)
    WDTCR = (1 << WDTOE) | (1 << WDE);

    // Set new prescaler (WDE is NOT enabled here — only timeout configuration)
    WDTCR = (prescaler & 0x07);
}

void WATCHDOG_start()
{
    // Enable watchdog (requires timed sequence)
    WDTCR = (1 << WDTOE) | (1 << WDE);
    WDTCR = (1 << WDE);    // WDE=1, prescaler unchanged
}

void WATCHDOG_reset()
{
    __asm__ __volatile__("wdr");  // safer version
}

void WATCHDOG_disable()
{
    // Required 4-cycle timed sequence
    WDTCR = (1 << WDTOE) | (1 << WDE);
    WDTCR = 0x00;           // Disable WDT
}