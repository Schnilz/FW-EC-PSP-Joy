#include "ch32v003fun.h"
#include "gamepad_config.h"
#include "rv003usb.h"
#include <stdio.h>
#include <string.h>

void init(void);

/* Board drivers */
#include "ch32v003_flash.h"
#include "lib_i2c.h"
#include "system_interface.h"

/* Static functions definition */
static int setup_imu();
static void init(void);

/* Static variables */
static inv_imu_device_t imu_dev; /* Driver structure */
volatile unsigned long uptime_10ms = 0;
volatile unsigned long last_usb_packet_10ms = 0;
volatile unsigned long calibration_button_timout = 0;

volatile bool calibration_mode = false;

#define ADC_NUMCHLS 2
const int adc_half_value = (1 << 10) / 2;
volatile uint16_t adc_buffer[ADC_NUMCHLS];
uint16_t adc_buffer_min[ADC_NUMCHLS] = {0xFFFF - 1, 0xFFFF - 1};
uint16_t adc_buffer_max[ADC_NUMCHLS] = {1, 1};
uint16_t adc_buffer_deadzone_min[ADC_NUMCHLS] = {0xFFFF - 1, 0xFFFF - 1};
uint16_t adc_buffer_deadzone_max[ADC_NUMCHLS] = {1, 1};
uint16_t zero_offset[ADC_NUMCHLS] = {0, 0};

#define STANDBY_AFTER_DURATION_10MS 10
#define CALIBRATION_MODE_BUTTON_10MS 800

#define NONVOLATILE_START_ADDR FLASH_PRECALCULATE_NONVOLATILE_ADDR(0)
#define NONVLOATILE_VAR_ADDR FLASH_PRECALCULATE_NONVOLATILE_ADDR(10)

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)

static volatile uint8_t tsajoystick[3] = {0x00, 0x00, 0x00};

// #define STATUS_LED
// #define STATUS_LED_PIN PC3

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
  PWR->AWUWR |= 63; // 1 - 63

  // enable AWU
  PWR->AWUCSR |= (1 << 1);

  // select standby on power-down
  PWR->CTLR |= PWR_CTLR_PDDS;

  // peripheral interrupt controller send to deep sleep
  PFIC->SCTLR |= (1 << 2);

  __WFE();
  init();
}

// I2C Scan Callback example function. Prints the address which responded
static bool i2c_init_ok = false;
static int found_i2c_dev = 0;
static uint8_t i2c_addr = 0;
void i2c_scan_callback(const uint8_t addr) {
  found_i2c_dev += 1;
  i2c_addr = addr;
  // printf("Address: 0x%02X Responded.\n", addr);
}

bool imu_self_test();

static inline uint8_t fast_lerp_u16_to_u8(uint16_t value, uint16_t min_value,
                                          uint16_t max_value,
                                          uint8_t min_target,
                                          uint8_t max_target) {
  if (value <= min_value)
    return min_target;
  if (value >= max_value)
    return max_target;

  uint32_t in_range = (uint32_t)(max_value - min_value);
  uint32_t out_range = (uint32_t)(max_target - min_target);

  uint32_t scaled = (uint32_t)(value - min_value) * out_range;

  // add half of denominator for rounding (instead of truncation)
  uint32_t mapped = (scaled + in_range / 2) / in_range;

  return (uint8_t)(min_target + mapped);
}

void calc_values() {

  bool both_in_deadzone = true;
  for (int ax = 0; ax < 2; ax += 1) {
    const int16_t deadzone_width =
        (adc_buffer_deadzone_max[ax] - adc_buffer_deadzone_min[ax]);
    const int16_t deadzone_tolerance =
        deadzone_width * 0.15; /* 15% deadzone tolerance*/
    const bool out_of_deadzone =
        (adc_buffer[ax] < adc_buffer_deadzone_min[ax] - deadzone_tolerance ||
         adc_buffer[ax] > adc_buffer_deadzone_max[ax] + deadzone_tolerance);
    if (out_of_deadzone) {
      both_in_deadzone = false;
    }
  }

  for (int ax = 0; ax < 2; ax += 1) {
    const unsigned int mapped_axis = ax ? 0 : 1;
    const uint8_t gamepad_mid = (GAMEPAD_LOGICAL_MAX + GAMEPAD_LOGICAL_MIN) / 2;
    if (adc_buffer[ax] > 10 && adc_buffer[ax] < ((1 << 10) - 11) &&
        !both_in_deadzone) {
      const bool lower = (adc_buffer[ax] < zero_offset[ax]);
      const uint16_t min = lower ? adc_buffer_min[ax] : zero_offset[ax];
      const uint16_t max = lower ? zero_offset[ax] : adc_buffer_max[ax];
      tsajoystick[mapped_axis] = fast_lerp_u16_to_u8(
          adc_buffer[ax], min, max, lower ? GAMEPAD_LOGICAL_MIN : gamepad_mid,
          lower ? gamepad_mid : GAMEPAD_LOGICAL_MAX);
    } else {
      tsajoystick[mapped_axis] = gamepad_mid;
    }
  }
  tsajoystick[2] = ((GPIOC->INDR & (1 << 5)) ? 0 : 1 << 0) |
                   ((GPIOC->INDR & (1 << 6)) ? 0 : 1 << 1);
}

int main() {
  init();
  while (1) {
    calc_values();
    Delay_Ms(5);
    calc_values();
    Delay_Ms(5);
#if defined(STATUS_LED)
    funDigitalWrite(STATUS_LED_PIN, 1);
#endif
    uptime_10ms++;

    const bool button1_pressed = !(GPIOC->INDR & (1 << 5));
    const bool button2_pressed = !(GPIOC->INDR & (1 << 6));

    if (calibration_mode) {
      if (button1_pressed) {
        for (int ax = 0; ax < 2; ax += 1) {
          if (adc_buffer[ax] > 10 && adc_buffer[ax] < ((1 << 10) - 11)) {
            adc_buffer_deadzone_min[ax] =
                MIN(adc_buffer_deadzone_min[ax], adc_buffer[ax]);
            adc_buffer_deadzone_max[ax] =
                MAX(adc_buffer_deadzone_max[ax], adc_buffer[ax]);
          }
        }
        for (int ax = 0; ax < 2; ax += 1) {
          zero_offset[ax] =
              ((adc_buffer_deadzone_min[ax] + adc_buffer_deadzone_max[ax]) / 2);
        }
      } else {
        for (int ax = 0; ax < 2; ax += 1) {
          if (adc_buffer[ax] > 10 && adc_buffer[ax] < ((1 << 10) - 11)) {
            adc_buffer_min[ax] = MIN(adc_buffer_min[ax], adc_buffer[ax]);
            adc_buffer_max[ax] = MAX(adc_buffer_max[ax], adc_buffer[ax]);
          }
        }
      }
    }

    if (button1_pressed && button2_pressed) {
      if (calibration_button_timout++ > CALIBRATION_MODE_BUTTON_10MS) {
        if (calibration_mode) {
          flash_unlock();
          flash_erase_page(
              NONVOLATILE_START_ADDR); // Erase a 64-bit block of flash memory
          flash_erase_page(NONVOLATILE_START_ADDR + 64);
          for (int ax = 0; ax < 2; ax += 1) {
            flash_program_16(NONVOLATILE_START_ADDR + ax * 8 + 0,
                             adc_buffer_deadzone_max[ax]);
            flash_program_16(NONVOLATILE_START_ADDR + ax * 8 + 2,
                             adc_buffer_deadzone_min[ax]);
            flash_program_16(NONVOLATILE_START_ADDR + ax * 8 + 4,
                             adc_buffer_max[ax]);
            flash_program_16(NONVOLATILE_START_ADDR + ax * 8 + 6,
                             adc_buffer_min[ax]);
          }
          flash_lock();

          /* to bootloader */
          FLASH->BOOT_MODEKEYR = 0x45670123;
          FLASH->BOOT_MODEKEYR = 0xCDEF89AB;
          FLASH->STATR = 0x4000;
          RCC->RSTSCKR |= 0x1000000;
          PFIC->CFGR = 0xBEEF0080;
        }
        calibration_mode = true;
        calibration_button_timout = 0;
        for (int ax = 0; ax < 2; ax += 1) {
          adc_buffer_min[ax] = 0xFFFF - 1;
          adc_buffer_max[ax] = 1;
          adc_buffer_deadzone_min[ax] = 0xFFFF - 1;
          adc_buffer_deadzone_max[ax] = 1;
        }
      }
    } else {
      calibration_button_timout = 0;
    }
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

  flash_set_latency();
  for (int ax = 0; ax < 2; ax += 1) {
    adc_buffer_deadzone_max[ax] =
        flash_read_16_bits(NONVOLATILE_START_ADDR + ax * 8 + 0);
    adc_buffer_deadzone_min[ax] =
        flash_read_16_bits(NONVOLATILE_START_ADDR + ax * 8 + 2);
    adc_buffer_max[ax] =
        flash_read_16_bits(NONVOLATILE_START_ADDR + ax * 8 + 4);
    adc_buffer_min[ax] =
        flash_read_16_bits(NONVOLATILE_START_ADDR + ax * 8 + 6);

    zero_offset[ax] =
        ((adc_buffer_deadzone_min[ax] + adc_buffer_deadzone_max[ax]) / 2);
  }

  // flash_erase_page(NONVOLATILE_START_ADDR);
  // zero_offset = flash_read_16_bits(NONVLOATILE_VAR_ADDR);
#if defined(STATUS_LED)
  funPinMode(STATUS_LED_PIN, GPIO_CFGLR_OUT_2Mhz_OD);
  funDigitalWrite(STATUS_LED_PIN, 1);
#endif
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
  ADC1->RSQR3 = 0;
  ADC1->RSQR3 = (1 << (5 * 0)) | (0 << (5 * 1));

  // set sampling time for chl 0, 1
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
  // i2c_init_ok = !i2c_init(I2C_CLK_400KHZ);
  // Delay_Ms(100);
  //  i2c_scan(i2c_scan_callback);
  //  i2c_addr = i2c_addr == 0x68 ? 1 : 0;
  // while(1) {
  // i2c_addr |= setup_imu() == 0 ? 1 << 1 : 0;
  // Delay_Ms(1000);
  //}  // imu_self_test();
}

// static int setup_imu() {
//   int rc = 0;
//   inv_imu_serif_t imu_serif;
//   uint8_t whoami;

//   /* Initialize serial interface between MCU and IMU */
//   imu_serif.context = 0; /* no need */
//   imu_serif.read_reg = si_io_imu_read_reg;
//   imu_serif.write_reg = si_io_imu_write_reg;
//   imu_serif.max_read =
//       8; /* maximum number of bytes allowed per serial read */
//   imu_serif.max_write =
//       8; /* maximum number of bytes allowed per serial write */
//   imu_serif.serif_type = UI_I2C;

//   /* Init device */
//   rc |= inv_imu_init(&imu_dev, &imu_serif, NULL);
//   SI_CHECK_RC(rc);

//   return rc;
// #if SERIF_TYPE == UI_SPI4
//   /* Configure slew-rate to 19 ns (required when using EVB) */
//   rc |= inv_imu_set_spi_slew_rate(&imu_dev,
//                                   DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_19_NS);
//   SI_CHECK_RC(rc);
// #endif

//   /* Check WHOAMI */
//   rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
//   SI_CHECK_RC(rc);
//   if (whoami != INV_IMU_WHOAMI) {
//     INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
//     INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
//     INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
//     return INV_ERROR;
//   }

//   return rc;
// }

// bool imu_self_test() {
//
//   int rc = 0;
//   int retries = 0;
//   do {
//     int rc = 0;
//     static int iter = 0;
//     inv_imu_selftest_output_t out;
//     inv_imu_selftest_parameters_t params;
//
//     rc |= inv_imu_init_selftest_parameters_struct(&imu_dev, &params);
//     /* Update `params` if needed here */
//     rc |= inv_imu_run_selftest(&imu_dev, params, &out);
//     SI_CHECK_RC(rc);
//
//     /* Print self-test status */
//     INV_MSG(INV_MSG_LEVEL_INFO, "[%u] Accel self-test %s", iter,
//             out.accel_status == 1 ? "OK" : "KO");
//     if (out.accel_status != 1) {
//       INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Accel X: %s",
//               out.ax_status == 1 ? "OK" : "KO");
//       INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Accel Y: %s",
//               out.ay_status == 1 ? "OK" : "KO");
//       INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Accel Z: %s",
//               out.az_status == 1 ? "OK" : "KO");
//       rc |= INV_ERROR;
//       i2c_addr |=1 << 2;
//     }
//
// #if INV_IMU_IS_GYRO_SUPPORTED
//     INV_MSG(INV_MSG_LEVEL_INFO, "[%u] Gyro self-test %s", iter,
//             out.gyro_status == 1 ? "OK" : "KO");
//     if (out.gyro_status != 1) {
//       INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Gyro X: %s",
//               out.gx_status == 1 ? "OK" : "KO");
//       INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Gyro Y: %s",
//               out.gy_status == 1 ? "OK" : "KO");
//       INV_MSG(INV_MSG_LEVEL_VERBOSE, "  - Gyro Z: %s",
//               out.gz_status == 1 ? "OK" : "KO");
//       rc |= INV_ERROR;
//       i2c_addr |=1 << 3;
//     }
//
//     /* Check incomplete state */
//     if (out.gyro_status & 0x2) {
//       INV_MSG(INV_MSG_LEVEL_ERROR, "[%u] Gyro self-test are incomplete.",
//       iter); rc |= INV_ERROR; i2c_addr |=1 << 4;
//     }
// #endif
//
//     iter++;
//
//     /* Print empty line to ease readability */
//     INV_MSG(INV_MSG_LEVEL_INFO, "");
//   } while (rc == 0 && retries++ < 4);
//   return rc;
// }

void usb_handle_user_in_request(struct usb_endpoint *e, uint8_t *scratchpad,
                                int endp, uint32_t sendtok,
                                struct rv003usb_internal *ist) {
  if (endp) {

#if defined(PC3_STATUS_LED)
    funDigitalWrite(PC3, 0);
#endif
    if (calibration_mode) {
      tsajoystick[2] = 0;
    }

    tsajoystick[2] = ((GPIOC->INDR & (1 << 5)) ? 0 : (1 << 0)) |
                     ((GPIOC->INDR & (1 << 6)) ? 0 : (1 << 1));
    usb_send_data((const void *)tsajoystick, sizeof(tsajoystick), 0, sendtok);

#if defined(PC3_STATUS_LED)
    funDigitalWrite(PC3, 1);
#endif
    last_usb_packet_10ms = uptime_10ms;
  } else {
    // If it's a control transfer, nak it.
    usb_send_empty(sendtok);
  }
}
