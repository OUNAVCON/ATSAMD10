#ifndef _SAMD10D13AS_
#define _SAMD10D13AS_

/**
 * \ingroup SAMD10_definitions
 * \addtogroup SAMD10D13AS_definitions SAMD10D13AS definitions
 * This file defines all structures and symbols for SAMD10D13AS:
 *   - registers and bitfields
 *   - peripheral base address
 *   - peripheral ID
 *   - PIO definitions
*/
/*@{*/

#ifdef __cplusplus
 extern "C" {
#endif

#if !(defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
#include <stdint.h>
#ifndef __cplusplus
typedef volatile const uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile const uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile const uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
#else
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile       uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
#endif
typedef volatile       uint32_t WoReg;   /**< Write only 32-bit register (volatile unsigned int) */
typedef volatile       uint16_t WoReg16; /**< Write only 16-bit register (volatile unsigned int) */
typedef volatile       uint32_t WoReg8;  /**< Write only  8-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */
#if !defined(_UL)
#define _U(x)          x ## U            /**< C code: Unsigned integer literal constant value */
#define _L(x)          x ## L            /**< C code: Long integer literal constant value */
#define _UL(x)         x ## UL           /**< C code: Unsigned Long integer literal constant value */
#endif
#else
#if !defined(_UL)
#define _U(x)          x                 /**< Assembler: Unsigned integer literal constant value */
#define _L(x)          x                 /**< Assembler: Long integer literal constant value */
#define _UL(x)         x                 /**< Assembler: Unsigned Long integer literal constant value */
#endif
#endif

/* ************************************************************************** */
/**  CMSIS DEFINITIONS FOR SAMD10D13AS */
/* ************************************************************************** */
/** \defgroup SAMD10D13AS_cmsis CMSIS Definitions */
/*@{*/

/** Interrupt Number Definition */
typedef enum IRQn
{
  /******  Cortex-M0+ Processor Exceptions Numbers ******************************/
  NonMaskableInt_IRQn      = -14,/**<  2 Non Maskable Interrupt                 */
  HardFault_IRQn           = -13,/**<  3 Cortex-M0+ Hard Fault Interrupt        */
  SVCall_IRQn              = -5, /**< 11 Cortex-M0+ SV Call Interrupt           */
  PendSV_IRQn              = -2, /**< 14 Cortex-M0+ Pend SV Interrupt           */
  SysTick_IRQn             = -1, /**< 15 Cortex-M0+ System Tick Interrupt       */
  /******  SAMD10D13AS-specific Interrupt Numbers ***********************/
  PM_IRQn                  =  0, /**<  0 SAMD10D13AS Power Manager (PM) */
  SYSCTRL_IRQn             =  1, /**<  1 SAMD10D13AS System Control (SYSCTRL) */
  WDT_IRQn                 =  2, /**<  2 SAMD10D13AS Watchdog Timer (WDT) */
  RTC_IRQn                 =  3, /**<  3 SAMD10D13AS Real-Time Counter (RTC) */
  EIC_IRQn                 =  4, /**<  4 SAMD10D13AS External Interrupt Controller (EIC) */
  NVMCTRL_IRQn             =  5, /**<  5 SAMD10D13AS Non-Volatile Memory Controller (NVMCTRL) */
  DMAC_IRQn                =  6, /**<  6 SAMD10D13AS Direct Memory Access Controller (DMAC) */
  EVSYS_IRQn               =  8, /**<  8 SAMD10D13AS Event System Interface (EVSYS) */
  SERCOM0_IRQn             =  9, /**<  9 SAMD10D13AS Serial Communication Interface 0 (SERCOM0) */
  SERCOM1_IRQn             = 10, /**< 10 SAMD10D13AS Serial Communication Interface 1 (SERCOM1) */
  SERCOM2_IRQn             = 11, /**< 11 SAMD10D13AS Serial Communication Interface 2 (SERCOM2) */
  TCC0_IRQn                = 12, /**< 12 SAMD10D13AS Timer Counter Control (TCC0) */
  TC1_IRQn                 = 13, /**< 13 SAMD10D13AS Basic Timer Counter 1 (TC1) */
  TC2_IRQn                 = 14, /**< 14 SAMD10D13AS Basic Timer Counter 2 (TC2) */
  ADC_IRQn                 = 15, /**< 15 SAMD10D13AS Analog Digital Converter (ADC) */
  AC_IRQn                  = 16, /**< 16 SAMD10D13AS Analog Comparators (AC) */
  DAC_IRQn                 = 17, /**< 17 SAMD10D13AS Digital Analog Converter (DAC) */
  PTC_IRQn                 = 18, /**< 18 SAMD10D13AS Peripheral Touch Controller (PTC) */

  PERIPH_COUNT_IRQn        = 19  /**< Number of peripheral IDs */
} IRQn_Type;

/*
 * \brief Configuration of the Cortex-M0+ Processor and Core Peripherals
 */

#define LITTLE_ENDIAN          1        
#define __CM0PLUS_REV          1         /*!< Core revision r0p1 */
#define __MPU_PRESENT          0         /*!< MPU present or not */
#define __NVIC_PRIO_BITS       2         /*!< Number of bits used for Priority Levels */
#define __VTOR_PRESENT         1         /*!< VTOR present or not */
#define __Vendor_SysTickConfig 0         /*!< Set to 1 if different SysTick Config is used */

/**
 * \brief CMSIS includes
 */

#include <core_cm0plus.h>
#include "system_samd10.h"

/*@}*/
typedef struct _DeviceVectors
{
  /* Stack pointer */
  void* pvStack;

  /* Cortex-M handlers */
  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
  void* pfnReservedM12;
  void* pfnReservedM11;
  void* pfnReservedM10;
  void* pfnReservedM9;
  void* pfnReservedM8;
  void* pfnReservedM7;
  void* pfnReservedM6;
  void* pfnSVC_Handler;
  void* pfnReservedM4;
  void* pfnReservedM3;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;

  /* Peripheral handlers */
  void* pfnPM_Handler;                    /*  0 Power Manager */
  void* pfnSYSCTRL_Handler;               /*  1 System Control */
  void* pfnWDT_Handler;                   /*  2 Watchdog Timer */
  void* pfnRTC_Handler;                   /*  3 Real-Time Counter */
  void* pfnEIC_Handler;                   /*  4 External Interrupt Controller */
  void* pfnNVMCTRL_Handler;               /*  5 Non-Volatile Memory Controller */
  void* pfnDMAC_Handler;                  /*  6 Direct Memory Access Controller */
  void* pfnReserved7;
  void* pfnEVSYS_Handler;                 /*  8 Event System Interface */
  void* pfnSERCOM0_Handler;               /*  9 Serial Communication Interface 0 */
  void* pfnSERCOM1_Handler;               /* 10 Serial Communication Interface 1 */
  void* pfnSERCOM2_Handler;               /* 11 Serial Communication Interface 2 */
  void* pfnTCC0_Handler;                  /* 12 Timer Counter Control */
  void* pfnTC1_Handler;                   /* 13 Basic Timer Counter 1 */
  void* pfnTC2_Handler;                   /* 14 Basic Timer Counter 2 */
  void* pfnADC_Handler;                   /* 15 Analog Digital Converter */
  void* pfnAC_Handler;                    /* 16 Analog Comparators */
  void* pfnDAC_Handler;                   /* 17 Digital Analog Converter */
  void* pfnPTC_Handler;                   /* 18 Peripheral Touch Controller */
} DeviceVectors;

/* Cortex-M0+ processor handlers */
void Reset_Handler               ( void );
void NMI_Handler                 ( void );
void HardFault_Handler           ( void );
void SVC_Handler                 ( void );
void PendSV_Handler              ( void );
void SysTick_Handler             ( void );

/* Peripherals handlers */
void PM_Handler                  ( void );
void SYSCTRL_Handler             ( void );
void WDT_Handler                 ( void );
void RTC_Handler                 ( void );
void EIC_Handler                 ( void );
void NVMCTRL_Handler             ( void );
void DMAC_Handler                ( void );
void EVSYS_Handler               ( void );
void SERCOM0_Handler             ( void );
void SERCOM1_Handler             ( void );
void SERCOM2_Handler             ( void );
void TCC0_Handler                ( void );
void TC1_Handler                 ( void );
void TC2_Handler                 ( void );
void ADC_Handler                 ( void );
void AC_Handler                  ( void );
void DAC_Handler                 ( void );
void PTC_Handler                 ( void );


/* ************************************************************************** */
/**  PERIPHERAL ID DEFINITIONS FOR SAMD10D13AS */
/* ************************************************************************** */
/** \defgroup SAMD10D13AS_id Peripheral Ids Definitions */
/*@{*/

// Peripheral instances on HPB0 bridge
#define ID_PAC0           0 /**< \brief Peripheral Access Controller 0 (PAC0) */
#define ID_PM             1 /**< \brief Power Manager (PM) */
#define ID_SYSCTRL        2 /**< \brief System Control (SYSCTRL) */
#define ID_GCLK           3 /**< \brief Generic Clock Generator (GCLK) */
#define ID_WDT            4 /**< \brief Watchdog Timer (WDT) */
#define ID_RTC            5 /**< \brief Real-Time Counter (RTC) */
#define ID_EIC            6 /**< \brief External Interrupt Controller (EIC) */

// Peripheral instances on HPB1 bridge
#define ID_PAC1          32 /**< \brief Peripheral Access Controller 1 (PAC1) */
#define ID_DSU           33 /**< \brief Device Service Unit (DSU) */
#define ID_NVMCTRL       34 /**< \brief Non-Volatile Memory Controller (NVMCTRL) */
#define ID_PORT          35 /**< \brief Port Module (PORT) */
#define ID_DMAC          36 /**< \brief Direct Memory Access Controller (DMAC) */
#define ID_MTB           38 /**< \brief Cortex-M0+ Micro-Trace Buffer (MTB) */
#define ID_SBMATRIX      39 /**< \brief HSB Matrix (SBMATRIX) */

// Peripheral instances on HPB2 bridge
#define ID_PAC2          64 /**< \brief Peripheral Access Controller 2 (PAC2) */
#define ID_EVSYS         65 /**< \brief Event System Interface (EVSYS) */
#define ID_SERCOM0       66 /**< \brief Serial Communication Interface 0 (SERCOM0) */
#define ID_SERCOM1       67 /**< \brief Serial Communication Interface 1 (SERCOM1) */
#define ID_SERCOM2       68 /**< \brief Serial Communication Interface 2 (SERCOM2) */
#define ID_TCC0          69 /**< \brief Timer Counter Control (TCC0) */
#define ID_TC1           70 /**< \brief Basic Timer Counter 1 (TC1) */
#define ID_TC2           71 /**< \brief Basic Timer Counter 2 (TC2) */
#define ID_ADC           72 /**< \brief Analog Digital Converter (ADC) */
#define ID_AC            73 /**< \brief Analog Comparators (AC) */
#define ID_DAC           74 /**< \brief Digital Analog Converter (DAC) */
#define ID_PTC           75 /**< \brief Peripheral Touch Controller (PTC) */

#define ID_PERIPH_COUNT  76 /**< \brief Max number of peripheral IDs */
/*@}*/

/* ************************************************************************** */
/**  BASE ADDRESS DEFINITIONS FOR SAMD10D13AS */
/* ************************************************************************** */
/** \defgroup SAMD10D13AS_base Peripheral Base Address Definitions */
/*@{*/

/*
 * Calibration data.
 * 34:27 ADC Linearity CALIB offset: 0x28 bits: 0-7
 * 37:35 ADC_Bias_Cal CALIB offset: 0x28 bits: 8-10
 * 44:38 OSC 32k Cal OSC32K offset 0x18, bits: 16-22
 * 63:58 DFLL 48M Course Cal //Loaded into DFLL48M Value offset 0x28
 */
#define CALIBRATION                   (0x00806020) //This is a 128bit value.
#define SERIAL_NUMBER_0               (0x0080A00C)
#define SERIAL_NUMBER_1               (0x0080A040)
#define SERIAL_NUMBER_2               (0x0080A044)
#define SERIAL_NUMBER_3               (0x0080A048)
//#if defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)
#define AC                            (0x42002400) /**< \brief (AC) APB Base Address */
#define ADC                           (0x42002000) /**< \brief (ADC) APB Base Address */
#define DAC                           (0x42002800) /**< \brief (DAC) APB Base Address */
#define DMAC                          (0x41004800) /**< \brief (DMAC) APB Base Address */
#define DSU                           (0x41002000) /**< \brief (DSU) APB Base Address */
#define EIC                           (0x40001800) /**< \brief (EIC) APB Base Address */
#define EVSYS                         (0x42000400) /**< \brief (EVSYS) APB Base Address */
#define GCLK                          (0x40000C00) /**< \brief (GCLK) APB Base Address */
#define SBMATRIX                      (0x41007000) /**< \brief (SBMATRIX) APB Base Address */
#define MTB                           (0x41006000) /**< \brief (MTB) APB Base Address */
#define NVMCTRL                       (0x41004000) /**< \brief (NVMCTRL) APB Base Address */
#define NVMCTRL_CAL                   (0x00800000) /**< \brief (NVMCTRL) CAL Base Address */
#define NVMCTRL_LOCKBIT               (0x00802000) /**< \brief (NVMCTRL) LOCKBIT Base Address */
#define NVMCTRL_OTP1                  (0x00806000) /**< \brief (NVMCTRL) OTP1 Base Address */
#define NVMCTRL_OTP2                  (0x00806008) /**< \brief (NVMCTRL) OTP2 Base Address */
#define NVMCTRL_OTP4                  (0x00806020) /**< \brief (NVMCTRL) OTP4 Base Address */
#define NVMCTRL_TEMP_LOG              (0x00806030) /**< \brief (NVMCTRL) TEMP_LOG Base Address */
#define NVMCTRL_USER                  (0x00804000) /**< \brief (NVMCTRL) USER Base Address */
#define PAC0                          (0x40000000) /**< \brief (PAC0) APB Base Address */
#define PAC1                          (0x41000000) /**< \brief (PAC1) APB Base Address */
#define PAC2                          (0x42000000) /**< \brief (PAC2) APB Base Address */
#define PM                            (0x40000400) /**< \brief (PM) APB Base Address */
#define PORT                          (0x41004400) /**< \brief (PORT) APB Base Address */
#define PORT_IOBUS                    (0x60000000) /**< \brief (PORT) IOBUS Base Address */
#define PTC                           (0x42002C00) /**< \brief (PTC) APB Base Address */
#define RTC                           (0x40001400) /**< \brief (RTC) APB Base Address */
#define SERCOM0                       (0x42000800) /**< \brief (SERCOM0) APB Base Address */
#define SERCOM1                       (0x42000C00) /**< \brief (SERCOM1) APB Base Address */
#define SERCOM2                       (0x42001000) /**< \brief (SERCOM2) APB Base Address */
#define SYSCTRL                       (0x40000800) /**< \brief (SYSCTRL) APB Base Address */
#define TC1                           (0x42001800) /**< \brief (TC1) APB Base Address */
#define TC2                           (0x42001C00) /**< \brief (TC2) APB Base Address */
#define TCC0                          (0x42001400) /**< \brief (TCC0) APB Base Address */
#define WDT                           (0x40001000) /**< \brief (WDT) APB Base Address */
/*@}*/

#define PAC0_WPCLR   (*(volatile uint32_t *)(PAC0+0x00))
#define PAC0_WPSET   (*((volatile uint32_t *)PAC0+0x04))

#define PAC1_WPCLR   (*(volatile uint32_t *)(PAC1+0x00))
#define PAC1_WPSET   (*((volatile uint32_t *)PAC1+0x04))

#define PAC2_WPCLR   (*(volatile uint32_t *)(PAC2+0x00))
#define PAC2_WPSET   (*((volatile uint32_t *)PAC2+0x04))

#define DSU_CTRL     (*((volatile uint8_t *)DSU+0x0000))
#define DSU_STATUSA  (*((volatile uint8_t *)DSU+0x0001))
#define DSU_STATUSB  (*((volatile uint8_t *)DSU+0x0002))
#define DSU_ADDR     (*((volatile uint32_t *)DSU+0x0004))
#define DSU_LENGTH   (*((volatile uint32_t *)DSU+0x0008))
#define DSU_DATA     (*((volatile uint32_t *)DSU+0x000C))
#define DSU_DCC0     (*((volatile uint32_t *)DSU+0x0010))
#define DSU_DCC1     (*((volatile uint32_t *)DSU+0x0014))
#define DSU_DID      (*((volatile uint32_t *)DSU+0x0018))
#define DSU_DCFG0    (*((volatile uint32_t *)DSU+0x00F0))
#define DSU_DCFG1    (*((volatile uint32_t *)DSU+0x00F4))
#define DSU_ENTRY0   (*((volatile uint32_t *)DSU+0x1000))
#define DSU_ENTRY1   (*((volatile uint32_t *)DSU+0x1004))
#define DSU_END      (*((volatile uint32_t *)DSU+0x1008))
#define DSU_MEMTYPE  (*((volatile uint32_t *)DSU+0x1FCC))
#define DSU_PID4     (*((volatile uint32_t *)DSU+0x1FD0))
#define DSU_PID5     (*((volatile uint32_t *)DSU+0x1FD4))
#define DSU_PID6     (*((volatile uint32_t *)DSU+0x1FD8))
#define DSU_PID7     (*((volatile uint32_t *)DSU+0x1FDC))
#define DSU_PID0     (*((volatile uint32_t *)DSU+0x1FE0))
#define DSU_PID1     (*((volatile uint32_t *)DSU+0x1FE4))
#define DSU_PID2     (*((volatile uint32_t *)DSU+0x1FE8))
#define DSU_PID3     (*((volatile uint32_t *)DSU+0x1FEC))
#define DSU_CID0     (*((volatile uint32_t *)DSU+0x1FF0))
#define DSU_CID1     (*((volatile uint32_t *)DSU+0x1FF4))
#define DSU_CID2     (*((volatile uint32_t *)DSU+0x1FF8))
#define DSU_CID3     (*((volatile uint32_t *)DSU+0x1FFC))

#define GCLK_CTRL    (*((volatile uint8_t  *)GCLK+0x0000))
#define GCLK_STATUS  (*((volatile uint8_t  *)GCLK+0x0001))
#define GCLK_CLKCTRL (*((volatile uint16_t *)GCLK+0x0002))
#define GCLK_GENCTRL (*((volatile uint32_t *)GCLK+0x0004))
#define GCLK_GENDIV  (*((volatile uint32_t *)GCLK+0x0008))

#define PM_CTRL      (*((volatile uint8_t  *)PM+0x0000))
#define PM_SLEEP     (*((volatile uint8_t  *)PM+0x0001))
#define PM_EXTCTRL   (*((volatile uint8_t  *)PM+0x0002))
#define PM_CPUSEL    (*((volatile uint8_t  *)PM+0x0008))
#define PM_APBASEL   (*((volatile uint8_t  *)PM+0x0009))
#define PM_APBBSEL   (*((volatile uint8_t  *)PM+0x000A))
#define PM_APBCSEL   (*((volatile uint8_t  *)PM+0x000B))
#define PM_AHBMASK   (*((volatile uint32_t *)PM+0x0014))
#define PM_APBAMASK  (*((volatile uint32_t *)PM+0x0018))
#define PM_APBBMASK  (*((volatile uint32_t *)PM+0x001C))
#define PM_APBCMASK  (*((volatile uint32_t *)PM+0x0020))
#define PM_INTENCLR  (*((volatile uint8_t  *)PM+0x0034))
#define PM_INTENSET  (*((volatile uint8_t  *)PM+0x0035))
#define PM_NTFLAG    (*((volatile uint8_t  *)PM+0x0036))
#define PM_RCAUSE    (*((volatile uint8_t  *)PM+0x0038))

#define SYSCTRL_INTENCLR   (*((volatile uint32_t *)SYSCTRL+0x0000))
#define SYSCTRL_INTENSET   (*((volatile uint32_t *)SYSCTRL+0x0004))
#define SYSCTRL_INTFLAG    (*((volatile uint32_t *)SYSCTRL+0x0008))
#define SYSCTRL_PCLKSR     (*((volatile uint32_t *)SYSCTRL+0x000C))
#define SYSCTRL_XOSC       (*((volatile uint16_t *)SYSCTRL+0x0010))
#define SYSCTRL_XOSC32K    (*((volatile uint16_t *)SYSCTRL+0x0014))
#define SYSCTRL_OSC32K     (*((volatile uint32_t *)SYSCTRL+0x0018))
#define SYSCTRL_OSCULP32K  (*((volatile uint8_t  *)SYSCTRL+0x001C))
#define SYSCTRL_OSC8M      (*((volatile uint32_t *)SYSCTRL+0x0020))
#define SYSCTRL_FPLLCTRL   (*((volatile uint16_t *)SYSCTRL+0x0024))
#define SYSCTRL_DFLLVAL    (*((volatile uint32_t *)SYSCTRL+0x0028))
#define SYSCTRL_DFLLMUL    (*((volatile uint32_t *)SYSCTRL+0x002C))
#define SYSCTRL_DFLLSYNC   (*((volatile uint8_t  *)SYSCTRL+0x0030))
#define SYSCTRL_BOD33      (*((volatile uint32_t *)SYSCTRL+0x0034))
#define SYSCTRL_REF        (*((volatile uint32_t *)SYSCTRL+0x0040))
#define SYSCTRL_DPLLCTRLA  (*((volatile uint8_t  *)SYSCTRL+0x0044))
#define SYSCTRL_DPLLRATIO  (*((volatile uint32_t *)SYSCTRL+0x0048))
#define SYSCTRL_DPLLCTRLB  (*((volatile uint32_t *)SYSCTRL+0x004C))
#define SYSCTRL_DPLLSTATUS (*((volatile uint8_t  *)SYSCTRL+0x0050))

#define WDT_CTRL           (*((volatile uint8_t *)WDT+0x0000))
#define WDT_CONFIG         (*((volatile uint8_t *)WDT+0x0001))
#define WDT_INTENCLR       (*((volatile uint8_t *)WDT+0x0004))
#define WDT_INTENSET       (*((volatile uint8_t *)WDT+0x0005))
#define WDT_INTFLAG        (*((volatile uint8_t *)WDT+0x0006))
#define WDT_STATUS         (*((volatile uint8_t *)WDT+0x0007))
#define WDT_CLEAR          (*((volatile uint8_t *)WDT+0x0008))


//Mode O
#define RTC_CTRL           (*((volatile uint16_t *)RTC+0x0000))
#define RTC_READREQ        (*((volatile uint16_t *)RTC+0x0002))
#define RTC_EVCTRL         (*((volatile uint16_t *)RTC+0x0004))
#define RTC_INTENCLR       (*((volatile uint8_t  *)RTC+0x0006))
#define RTC_INTENSET       (*((volatile uint8_t  *)RTC+0x0007))
#define RTC_INTFLAG        (*((volatile uint8_t  *)RTC+0x0008))
#define RTC_STATUS         (*((volatile uint8_t  *)RTC+0x000A))
#define RTC_DBGCTRL        (*((volatile uint8_t  *)RTC+0x000B))
#define RTC_FREQCORR       (*((volatile uint8_t  *)RTC+0x000C))
#define RTC_COUNT          (*((volatile uint32_t *)RTC+0x0010))
#define RTC_COMP0          (*((volatile uint32_t *)RTC+0x0018))

#define DMAC_CTRL          (*((volatile uint16_t *)DMAC+0x0000))
#define DMAC_CRCTRL        (*((volatile uint16_t *)DMAC+0x0002))
#define DMAC_CRCDATAIN     (*((volatile uint32_t *)DMAC+0x0004))
#define DMAC_CRCCHKSUM     (*((volatile uint32_t *)DMAC+0x0008))
#define DMAC_CRCSTATUS     (*((volatile uint8_t  *)DMAC+0x000C))
#define DMAC_DBGCTRL       (*((volatile uint8_t  *)DMAC+0x000D))
#define DMAC_QOSCTRL       (*((volatile uint8_t  *)DMAC+0x000E))
#define DMAC_SWTRIGCTRL    (*((volatile uint32_t *)DMAC+0x0010))
#define DMAC_PRICTRL0      (*((volatile uint32_t *)DMAC+0x0014))
#define DMAC_INTPEND       (*((volatile uint16_t *)DMAC+0x0020))
#define DMAC_INTSTATUS     (*((volatile uint32_t *)DMAC+0x0024))
#define DMAC_BUSYCH        (*((volatile uint32_t *)DMAC+0x0028))
#define DMAC_PENDCH        (*((volatile uint32_t *)DMAC+0x002C))
#define DMAC_ACTIVCE       (*((volatile uint32_t *)DMAC+0x0030))
#define DMAC_BASEADDR      (*((volatile uint32_t *)DMAC+0x0034))
#define DMAC_WRBADDR       (*((volatile uint32_t *)DMAC+0x0038))
#define DMAC_CHID          (*((volatile uint8_t  *)DMAC+0x003F))
#define DMAC_CHCTRLA       (*((volatile uint8_t  *)DMAC+0x0040))
#define DMAC_CHCTRLB       (*((volatile uint32_t *)DMAC+0x0044))
#define DMAC_CHINTENCLR    (*((volatile uint8_t  *)DMAC+0x004C))
#define DMAC_CHINTENSET    (*((volatile uint8_t  *)DMAC+0x004D))
#define DMAC_CHINTFLAG     (*((volatile uint8_t  *)DMAC+0x004E))
#define DMAC_CHSTATUS      (*((volatile uint8_t  *)DMAC+0x004F

//#define DMAC_SRAM_BTCTRL   (*((voltaile uint16_t *)DMAC_SRAM+0x0000)) //What is this
//#define DMAC_SRAM_BTCNT    (*((volatile uint16_t *)DMAC_SRAM+0x0002))
//#define DMAC_SRAM_SRCADDR  (*((volatile uint32_t *)DMAC_SRAM+0x0004))
//#define DMAC_SRAM_DSTADDR  (*((volatile uint32_t *)DMAC_SRAM+0x0008))
//#define DMAC_SRAM_DESCADDR (*((volatile uint32_t *)DMAC_SRAM+0x000C))

#define EIC_CTRL           (*((volatile uint8_t  *)EIC+0x0000))
#define EIC_STATUS         (*((volatile uint8_t  *)EIC+0x0001))
#define EIC_NMICTRL        (*((volatile uint8_t  *)EIC+0x0002))
#define EIC_NMIFLAG        (*((volatile uint8_t  *)EIC+0x0003))
#define EIC_EVCTRL         (*((volatile uint32_t *)EIC+0x0004))
#define EIC_INTENCLR       (*((volatile uint32_t *)EIC+0x0008))
#define EIC_INTENSET       (*((volatile uint32_t *)EIC+0x000C))
#define EIC_INTFLAG        (*((volatile uint32_t *)EIC+0x0010))
#define EIC_WAKEUP         (*((volatile uint32_t *)EIC+0x0014))
#define EIC_CONFIG0        (*((volatile uint32_t *)EIC+0x0018))

#define NVMCTRL_CTRLA      (*((volatile uint16_t *)NVM+0x0000))
#define NVMCTRL_CTRLB      (*((volatile uint32_t *)NVM+0x0004))
#define NVMCTRL_PARAM      (*((volatile uint32_t *)NVM+0x0008))
#define NVMCTRL_INTENSET   (*((volatile uint8_t  *)NVM+0x000C))
#define NVMCTRL_INTENCLR   (*((volatile uint8_t  *)NVM+0x0010))
#define NVMCTRL_INTFLAG    (*((volatile uint8_t  *)NVM+0x0014))
#define NVMCTRL_STATUS     (*((volatile uint16_t *)NVM+0x0018))
#define NVMCTRL_ADDR       (*((volatile uint32_t *)NVM+0x001C))
#define NVMCTRL_LOCK       (*((volatile uint16_t *)NVM+0x0020))

#define PORTA            0
#define PORTB            1
#define PORTC            2
#define PORTD            3
#define PORTE            4
#define PORTF            5


#define PORT_BASE          0x80

#define PORT_DIR(p)        (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x00)))
#define PORT_DIRCLR(p)     (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x04)))
#define PORT_DIRSET(p)     (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x08)))
#define PORT_DIRTGL(p)     (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x0C)))
#define PORT_OUT(p)        (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x10)))
#define PORT_OUTCLR(p)     (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x14)))
#define PORT_OUTSET(p)     (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x18)))
#define PORT_OUTTGL(p)     (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x1C)))
#define PORT_IN(p)         (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x20)))
#define PORT_CTRL(p)       (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x24)))
#define PORT_WRCONFIG(p)   (*((volatile uint32_t *)(PORT + PORT_BASE*p + 0x28)))
#define PORT_MUX(portV,pin) (*((volatile uint8_t *)(PORT + PORT_BASE*portV + pin + 0x30)))
#define PORT_PINCFG(portV,pin)   (*((volatile uint8_t *)(PORT + PORT_BASE*portV + pin + 0x40)))



/* ************************************************************************** */
/**  PORT DEFINITIONS FOR SAMD10D13AS */
/* ************************************************************************** */
/** \defgroup SAMD10D13AS_port PORT Definitions */
/*@{*/

#include "pio/samd10d13as.h"
/*@}*/

/* ************************************************************************** */
/**  MEMORY MAPPING DEFINITIONS FOR SAMD10D13AS */
/* ************************************************************************** */

#define FLASH_SIZE            _UL(0x00002000) /* 8 kB */
#define FLASH_PAGE_SIZE       64
#define FLASH_NB_OF_PAGES     128
#define FLASH_USER_PAGE_SIZE  64
#define HMCRAMC0_SIZE         _UL(0x00001000) /* 4 kB */

#define FLASH_ADDR            _UL(0x00000000) /**< FLASH base address */
#define FLASH_USER_PAGE_ADDR  _UL(0x00800000) /**< FLASH_USER_PAGE base address */
#define HMCRAMC0_ADDR         _UL(0x20000000) /**< HMCRAMC0 base address */
#define HPB0_ADDR             _UL(0x40000000) /**< HPB0 base address */
#define HPB1_ADDR             _UL(0x41000000) /**< HPB1 base address */
#define HPB2_ADDR             _UL(0x42000000) /**< HPB2 base address */
#define PPB_ADDR              _UL(0xE0000000) /**< PPB base address */

#define DSU_DID_RESETVALUE    _UL(0x10020104)

/* ************************************************************************** */
/**  ELECTRICAL DEFINITIONS FOR SAMD10D13AS */
/* ************************************************************************** */


#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* SAMD10D13AS_H */
