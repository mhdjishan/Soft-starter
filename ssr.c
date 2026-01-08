#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"
#include "stdio.h"
#include "math.h"

#define PWM_PERIOD 10000 //2khz
#define BLANKING_PERIOD 880000 //1ms = 1000us = value 100000 //150 degree blanking which is get by 8.3333333ms
#define ON_PERIOD 900000 // 9ms
#define dead_band 10 //100 nanoseconds no of cycles * (1/100MHz)
#define TRIAC_MIN_LIMIT 100
volatile uint16_t currentDuty = 5000; //volatile is used since it is modified in interrupt , 75% duty cycle
volatile uint32_t triac_angle = 800000;
volatile bool pwm_flag = false;

//rms calculation new

// 2048 counts = 325V. Scale = 325/2048 = 0.15869
const float VOLTS_PER_COUNT = 0.15869f; 
volatile uint32_t sum_squares_ac =0;
volatile float final_rms = 0.0f;
 
volatile bool rms_ready = false;

//ADC timer_isr variables
volatile uint32_t temp_sum_squares = 0;
volatile uint16_t sample_counter = 0;
volatile bool data_ready_flag = false;

void initEPWM1(void);
void initZVCInput(void);
void initBlankingTimer(void);
void pwm_timer(void);
void initADC(void);
void initSCIA(void);
void initSamplingTimer(void);


void sendSCIText(char *msg);
void readADC(uint16_t *result1);

__interrupt void xint1_ISR(void);
__interrupt void cpuTimer0_ISR(void);
__interrupt void cpuTimer1_ISR(void);
__interrupt void cpuTimer2_ISR(void);


void main(void)
{
    // Initialize device clock and peripherals
    Device_init();
    // Disable pin locks and enable internal pull-ups.
    Device_initGPIO();
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    Interrupt_initModule();
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();

    Interrupt_register(INT_XINT1, &xint1_ISR);
    Interrupt_register(INT_TIMER0, &cpuTimer0_ISR);
    Interrupt_register(INT_TIMER1, &cpuTimer1_ISR);
    Interrupt_register(INT_TIMER2, &cpuTimer2_ISR);


    // PinMux and Peripheral Initialization
    Board_init();
    initEPWM1();
    initZVCInput();
    initBlankingTimer();
    pwm_timer();
    initADC();
    initSCIA();
    initSamplingTimer();

    
    // C2000Ware Library initialization
    C2000Ware_libraries_init();
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    EINT;
    ERTM;

    EPWM_forceTripZoneEvent(EPWM6_BASE, EPWM_TZ_FORCE_EVENT_OST); // to force pwm low initially (no pwm at the beginning)
   
    // other variables
    uint16_t zvc = 0;
    char buffer2[100];
    
    while(1)
    {
        if(data_ready_flag)
        {
            
           float mean_square = (float)sum_squares_ac / 200.0f; 
           float rms_counts = sqrtf(mean_square);
           final_rms = rms_counts * VOLTS_PER_COUNT;
            if(final_rms >= 180.0f)
            {
                pwm_flag = true;
            }
            else if (final_rms < 170.0f)
            {
                pwm_flag = false;
                triac_angle = 800000;
                EPWM_forceTripZoneEvent(EPWM6_BASE, EPWM_TZ_FORCE_EVENT_OST);
                Interrupt_enable(INT_XINT1);
            }
            if(pwm_flag == true)
            {   
                //if Total Time: 500ms
                //Loop Duration: ~20ms
                //Total Steps: 500 / 20 = 25 iterations
                //Total Reduction: 800,000 counts
                //New Step Size: 800,000 / 25 = 32,000

                if (triac_angle > 16000) 
                {
                    triac_angle -= 16000; // Decrease the delay
                    CPUTimer_setPeriod(CPUTIMER0_BASE, triac_angle);
                }
                else 
                {
                    Interrupt_disable(INT_XINT1); 
                    CPUTimer_stopTimer(CPUTIMER0_BASE);
                    CPUTimer_stopTimer(CPUTIMER1_BASE);
                    EPWM_clearTripZoneFlag(EPWM6_BASE, EPWM_TZ_FLAG_OST);     // Cap at zero (full power)
                }
            }
            sprintf(buffer2, "####****&&&&!!!!@@@@$$$$--Sin RMS: %d \r\n", (int)final_rms); //  
         sendSCIText(buffer2);
         sprintf(buffer2, "- RMS count: %d \r\n", (int)rms_counts); //  
         sendSCIText(buffer2);
            data_ready_flag = false;
        } 

        
        sprintf(buffer2, "Duty Count: %d\r\n", currentDuty);
        sendSCIText(buffer2);
        zvc=GPIO_readPin(12);
        sprintf(buffer2, "gpio12 zvc detection: %d\r\n ----------- ------------ \r\n", zvc );
        sendSCIText(buffer2);
    }
}
// ISR
__interrupt void xint1_ISR(void)
{
    //Force PWMs LOW immediately = Blanking tome start
    
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);

    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_startTimer(CPUTIMER1_BASE);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
__interrupt void cpuTimer0_ISR(void)
{  
     if (pwm_flag == true)
    {
    //   if (GPIO_readPin(12) == 0)
    //     {
        EPWM_clearTripZoneFlag(EPWM6_BASE, EPWM_TZ_FLAG_OST);
        //}
    }
   
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
__interrupt void cpuTimer1_ISR(void)
{
    // if (GPIO_readPin(12) == 0)
    //     {
        EPWM_forceTripZoneEvent(EPWM6_BASE, EPWM_TZ_FORCE_EVENT_OST);
        // }
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//interrupts that run every 100us that timmer 3
__interrupt void cpuTimer2_ISR(void)
{
    uint16_t raw = 0 ;
    int16_t adj;


    readADC(&raw);
    adj = (int16_t)raw - 1985; // here is your offset value
    temp_sum_squares += (uint32_t)((int32_t)adj * (int32_t)adj);
    sample_counter++;
    
    

    if(sample_counter >= 200) 
    {
        sum_squares_ac = temp_sum_squares; // Transfer to main variable
        sample_counter = 0;
        temp_sum_squares = 0;
        data_ready_flag = true; 
                 // Tell main loop to calculate RMS
    }
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void readADC(uint16_t *result1)
{
    // Force BOTH SOC0 and SOC1 to start
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0 );
    while (!ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1));
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    *result1 = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0); // Result of SOC0 (ADCIN6)
}    

//adc initializationfor output and input
void initADC(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN6, 15);// look here to know the pins
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
   
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

// for serial communication UART
void initSCIA(void)
{
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(28, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(29, GPIO_QUAL_ASYNC);
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200, SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE);
    SCI_enableFIFO(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
    SCI_enableTxModule(SCIA_BASE);
    SCI_enableRxModule(SCIA_BASE);
}
//for serial communication to print
void sendSCIText(char *msg)
{
     while (*msg)
    {
        SCI_writeCharBlockingFIFO(SCIA_BASE, *msg++);
    }
}

void initEPWM1(void)
{
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_11_EPWM6B);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    EPWM_setClockPrescaler(EPWM6_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM6_BASE, PWM_PERIOD);
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_setPhaseShift(EPWM6_BASE, 0);
   
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, currentDuty);

    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDelayCount(EPWM6_BASE, dead_band);
    EPWM_setFallingEdgeDelayCount(EPWM6_BASE, dead_band);

    EPWM_setTripZoneAction(EPWM6_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM6_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_clearTripZoneFlag(EPWM6_BASE, EPWM_TZ_FLAG_OST);
}


void initZVCInput(void)
{
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(12, GPIO_QUAL_ASYNC);
    GPIO_setInterruptPin(12, GPIO_INT_XINT1);
    Interrupt_enable(INT_XINT1);
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_BOTH_EDGES);
    GPIO_enableInterrupt(GPIO_INT_XINT1);
}

void initBlankingTimer(void)
{
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_setPeriod(CPUTIMER0_BASE, triac_angle);
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);
}


void pwm_timer(void)
{
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_setPeriod(CPUTIMER1_BASE, ON_PERIOD);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_enable(INT_TIMER1);
}

void initSamplingTimer(void) {
    // 100MHz / 10,000 = 10kHz (100us)
    CPUTimer_setPeriod(CPUTIMER2_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);
    CPUTimer_setPeriod(CPUTIMER2_BASE, 10000); 
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);
    CPUTimer_enableInterrupt(CPUTIMER2_BASE);
    Interrupt_enable(INT_TIMER2);
    CPUTimer_startTimer(CPUTIMER2_BASE);

}

//
// End of File
//
