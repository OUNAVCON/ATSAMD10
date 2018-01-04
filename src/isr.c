#include "samd10d13as.h"

/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;
extern uint32_t _estack;

/** \cond DOXYGEN_SHOULD_SKIP_THIS */
int main(void);
/** \endcond */

void __libc_init_array(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Exception Table */
__attribute__ ((section(".vectors")))
const DeviceVectors exception_table = {

        /* Configure Initial Stack Pointer, using linker-generated symbols */
        (void*) (0x00000000),
        (void*) main,
        (void*) Dummy_Handler,
        (void*) Dummy_Handler,
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) Dummy_Handler,
        (void*) (0UL), /* Reserved */
        (void*) (0UL), /* Reserved */
        (void*) Dummy_Handler,
        (void*) Dummy_Handler,

        /* Configurable interrupts */
        (void*) Dummy_Handler,             /*  0 Power Manager */
        (void*) Dummy_Handler,        /*  1 System Control */
        (void*) Dummy_Handler,            /*  2 Watchdog Timer */
        (void*) Dummy_Handler,            /*  3 Real-Time Counter */
        (void*) Dummy_Handler,            /*  4 External Interrupt Controller */
        (void*) Dummy_Handler,        /*  5 Non-Volatile Memory Controller */
        (void*) Dummy_Handler,           /*  6 Direct Memory Access Controller */
        (void*) (0UL), /* Reserved */
        (void*) Dummy_Handler,          /*  8 Event System Interface */
        (void*) Dummy_Handler,        /*  9 Serial Communication Interface 0 */
        (void*) Dummy_Handler,        /* 10 Serial Communication Interface 1 */
        (void*) Dummy_Handler,        /* 11 Serial Communication Interface 2 */
        (void*) Dummy_Handler,           /* 12 Timer Counter Control */
        (void*) Dummy_Handler,            /* 13 Basic Timer Counter 0 */
        (void*) Dummy_Handler,            /* 14 Basic Timer Counter 1 */
        (void*) Dummy_Handler,            /* 15 Analog Digital Converter */
        (void*) Dummy_Handler,             /* 16 Analog Comparators */
        (void*) Dummy_Handler,            /* 17 Digital Analog Converter */
        (void*) Dummy_Handler             /* 18 Peripheral Touch Controller */
};

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
void Reset_Handler(void)
{
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
        while (1) {
        }
}
