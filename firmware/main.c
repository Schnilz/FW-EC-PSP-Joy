#include "ch32v003fun.h"
#include "gamepad_config.h"
#include "rv003usb.h"
#include <stdio.h>
#include <string.h>

void init(void);

volatile unsigned long uptime_10ms = 0;
volatile unsigned long last_usb_packet_10ms = 0;

#define STANDBY_AFTER_DURATION_10MS 10

// #define PC3_STATUS_LED

void enter_sleep() {
  // enable power interface module clock
  RCC->APB1PCENR |= RCC_APB1Periph_PWR;

  // enable low speed oscillator (LSI)
  RCC->RSTSCKR |= RCC_LSION;
  while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {
  };

  // enable AutoWakeUp event
  EXTI->EVENR |= EXTI_Line9;
  EXTI->FTENR |= EXTI_Line9;

  // configure AWU prescaler
  PWR->AWUPSC |= PWR_AWU_Prescaler_61440;

  // configure AWU window comparison value
  PWR->AWUWR &= ~0x3f;
  PWR->AWUWR |= 30; // 1 - 63

  // enable AWU
  PWR->AWUCSR |= (1 << 1);

  // select standby on power-down
  PWR->CTLR |= PWR_CTLR_PDDS;

  // peripheral interrupt controller send to deep sleep
  PFIC->SCTLR |= (1 << 2);

  __WFE();
  init();
}

int main() {
  init();
  while (1) {
    Delay_Ms(10);
#if defined(PC3_STATUS_LED)
    funDigitalWrite(PC3, 1);
#endif
    uptime_10ms++;
    if (last_usb_packet_10ms + STANDBY_AFTER_DURATION_10MS < uptime_10ms) {
      enter_sleep();
      uptime_10ms = 0;
      last_usb_packet_10ms = STANDBY_AFTER_DURATION_10MS;
#if defined(PC3_STATUS_LED)
      funDigitalWrite(PC3, 0);
#endif
    }
  };
}

#define ADC_NUMCHLS 2
volatile uint16_t adc_buffer[ADC_NUMCHLS];

/*
 * initialize adc for DMA
 */
void init(void) {
  SystemInit();
  Delay_Ms(1); // Ensures USB re-enumeration after bootloader or reset; Spec
               // demand >2.5µs ( TDDIS )
  // ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
  RCC->CFGR0 &= ~(0x1F << 11);

  // Enable GPIOA, GPIOC and ADC
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
                    RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1;

#if defined(PC3_STATUS_LED)
  funPinMode(PC3, GPIO_CFGLR_OUT_2Mhz_OD);
  funDigitalWrite(PC3, 1);
#endif
  funPinMode(PC5, GPIO_CFGLR_IN_PUPD);
  funPinMode(PC7, GPIO_CFGLR_IN_PUPD);
  funDigitalWrite(PC5, 1);
  funDigitalWrite(PC7, 1);

  // PA1 is analog input chl 1
  funPinMode(PA1, GPIO_CFGLR_IN_ANALOG);
  // PA2 is analog input chl 0
  funPinMode(PA2, GPIO_CFGLR_IN_ANALOG);

  // Reset the ADC to init all regs
  RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

  // Set up four conversions on chl 1, 0
  ADC1->RSQR1 = (ADC_NUMCHLS - 1) << 20; // four chls in the sequence
  ADC1->RSQR2 = 0;
  ADC1->RSQR3 = (1 << (5 * 1)) | (0 << (5 * 2));

  // set sampling time for chl 1, 0
  // 0:7 => 3/9/15/30/43/57/73/241 cycles
  ADC1->SAMPTR2 = (6 << (3 * 0)) | (6 << (3 * 1));

  // turn on ADC
  ADC1->CTLR2 |= ADC_ADON;

  // Reset calibration
  ADC1->CTLR2 |= ADC_RSTCAL;
  while (ADC1->CTLR2 & ADC_RSTCAL)
    ;

  // Calibrate
  ADC1->CTLR2 |= ADC_CAL;
  while (ADC1->CTLR2 & ADC_CAL)
    ;

  // Turn on DMA
  RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

  // DMA1_Channel1 is for ADC
  DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
  DMA1_Channel1->MADDR = (uint32_t)adc_buffer;
  DMA1_Channel1->CNTR = ADC_NUMCHLS;
  DMA1_Channel1->CFGR = DMA_M2M_Disable | DMA_Priority_VeryHigh |
                        DMA_MemoryDataSize_HalfWord |
                        DMA_PeripheralDataSize_HalfWord | DMA_MemoryInc_Enable |
                        DMA_Mode_Circular | DMA_DIR_PeripheralSRC;

  // Turn on DMA channel 1
  DMA1_Channel1->CFGR |= DMA_CFGR1_EN;

  // enable scanning
  ADC1->CTLR1 |= ADC_SCAN;

  // Enable continuous conversion and DMA
  ADC1->CTLR2 |= ADC_CONT | ADC_DMA | ADC_EXTSEL;

  // start conversion
  ADC1->CTLR2 |= ADC_SWSTART;

  usb_setup();
}

void usb_handle_user_in_request(struct usb_endpoint *e, uint8_t *scratchpad,
                                int endp, uint32_t sendtok,
                                struct rv003usb_internal *ist) {
  if (endp) {

#if defined(PC3_STATUS_LED)
    funDigitalWrite(PC3, 0);
#endif
    static uint8_t tsajoystick[3] = {0x00, 0x00, 0x00};
    for (int ax = 0; ax < 2; ax += 1) {
      if (adc_buffer[ax] > 10 && adc_buffer[ax] < ((1 << 10) - 11)) {
        tsajoystick[ax] = (int8_t)(adc_buffer[ax] >> 2); // 10bit to 8bit
      } else {
        tsajoystick[ax] = (GAMEPAD_LOGICAL_MAX - GAMEPAD_LOGICAL_MIN) / 2;
      }
    }

    tsajoystick[2] = (GPIOC->INDR & (1 << 5)) ? 0b00 : 0b01;
    tsajoystick[2] |= (GPIOC->INDR & (1 << 7)) ? 0b00 : 0b10;
    usb_send_data(tsajoystick, 3, 0, sendtok);

#if defined(PC3_STATUS_LED)
    funDigitalWrite(PC3, 1);
#endif
    last_usb_packet_10ms = uptime_10ms;
  } else {
    // If it's a control transfer, nak it.
    usb_send_empty(sendtok);
  }
}
