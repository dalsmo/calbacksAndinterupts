
/** INCLUDES ******************************************************************/

#include <xc.h>             /* contains Vector Name/Number Macros */
#include <sys/attribs.h>    /* contains __ISR() Macros */
#include <timer.h>
#include <stdio.h>

/** CONFIGURATION *************************************************************/

// Key Settings:
// OSCILLATOR: 8MHz XT Oscillator w. PLL
// SYSCLK = 80 MHz (set in config bits), PBCLK = 10 MHz (= SYSCLK/8 - set in config bits))
// JTAG PORT: Disabled
// WATCHDOG TIMER: Disabled
// DEBUG/PGM PINS: PGEC2/PGED2

// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7     // SRS Select (SRS Priority 7)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc (XT,HS,EC) w. PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


/** VARIABLES *****************************************************************/


/** LOCAL MACROS **************************************************************/

#define SYS_CLK_FREQUENCY  (80000000ull)       // Fsys = 80 MHz
#define PB3_CLK_FREQUENCY SYS_CLK_FREQUENCY/8   // Fpb = 10 MHz


/** LOCAL PROTOTYPES ********************************************************/

void InitializeSystem(void);            // Initialize hardware and global variables

/** main() ********************************************************************/

typedef enum {
    STATE_INIT,
    STATE_ACTIVE,
    STATE_PASSIVE,
} main_state_t;

int cb(void *ctx) {
    return 0;
}

static void testFunction(){
    printf("inside the test function.");
}

int main(void)
{
    InitializeSystem();
    initialise_Timer();
    
    // register a callback that will return after 10*10ms
    register_Callback(testFunction,10);
    
    static main_state_t state = STATE_INIT;
    // Include examples on how the timer can be used here in the main function

    while(1){
        switch (state)
        {
        case STATE_INIT:
            //callback_register(&cb, 1, NULL);
            /* Init stuff here */
            state = STATE_ACTIVE;
            break;

        case STATE_ACTIVE:
            /* Do active part here */
            state = STATE_PASSIVE;
            break;

        case STATE_PASSIVE:
            /* Do passive part here */
            break;

        default:
            /* Default handler should not be hit in this case */
            break;
        }
    };

    
} // main()


void InitializeSystem(void) // left from example code, unsure what it does
{
    // PIC32MX CPU Speed Optimizations (Cache/Wait States/Peripheral Bus Clks)
    // On reset, and after c-startup:
    // - Prefetch Buffer is disabled,
    // - I Cache is disabled,
    // - PFM wait States set to max setting (7 = too slow!!!)
    // - Data Memory SRAM wait states set to max setting (1 = too slow!!!)
    
    // PBCLK - already set to SYSCLK/8 via config settings
    
    // Data Memory SRAM wait states: Default Setting = 1; set it to 0
    BMXCONbits.BMXWSDRM = 0;

    // Flash PM Wait States: MX Flash runs at 2 wait states @ 80 MHz
    CHECONbits.PFMWS = 2;

    // Prefetch-cache: Enable prefetch for cacheable PFM instructions
    CHECONbits.PREFEN = 1;

    // PIC32MX695-Specific
    // JTAG: Disable on PORTA
    DDPCONbits.JTAGEN = 0;
    
}



void __ISR (_TIMER_2_VECTOR, IPL7SRS) T2Interrupt(void)
{
    //Call the test function
    execute_Callback();

	// Reset interrupt flag
	IFS0bits.T2IF = 0;
}
