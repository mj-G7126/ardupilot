/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  ArduPilot bootloader. This implements the same protocol originally
  developed for PX4, but builds on top of the ChibiOS HAL

  It does not use the full AP_HAL API in order to keep the firmware
  size below the maximum of 16kByte required for F4 based
  boards. Instead it uses the ChibiOS APIs directly
 */

#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include "support.h"
#include "bl_protocol.h"
#include "can.h"

#define SPI_BUFFERS_SIZE    8U  

static uint8_t txbuf[SPI_BUFFERS_SIZE];
static uint8_t rxbuf[SPI_BUFFERS_SIZE];

extern "C"
{
    int main(void);
}

struct boardinfo board_info;

#ifndef HAL_BOOTLOADER_TIMEOUT
#define HAL_BOOTLOADER_TIMEOUT 5000
#endif

#ifndef HAL_STAY_IN_BOOTLOADER_VALUE
#define HAL_STAY_IN_BOOTLOADER_VALUE 0
#endif

#define LINE_PE14 PAL_LINE(GPIOE, 14)

int main(void)
{
    board_info.board_type = APJ_BOARD_ID;
    board_info.board_rev = 0;
    board_info.fw_size = (BOARD_FLASH_SIZE - FLASH_BOOTLOADER_LOAD_KB) * 1024;
    if (BOARD_FLASH_SIZE > 1024 && check_limit_flash_1M())
    {
        board_info.fw_size = (1024 - FLASH_BOOTLOADER_LOAD_KB) * 1024;
    }

    bool try_boot = false;
    uint32_t timeout = HAL_BOOTLOADER_TIMEOUT;

#ifdef HAL_BOARD_AP_PERIPH_ZUBAXGNSS
    // setup remapping register for ZubaxGNSS
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~AFIO_MAPR_SWJ_CFG;
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    AFIO->MAPR = mapr | AFIO_MAPR_CAN_REMAP_REMAP2 | AFIO_MAPR_SPI3_REMAP;
#endif

#ifndef NO_FASTBOOT
    enum rtc_boot_magic m = check_fast_reboot();
    if (stm32_was_watchdog_reset())
    {
        try_boot = true;
        timeout = 0;
    }
    else if (m == RTC_BOOT_HOLD)
    {
        timeout = 0;
    }
    else if (m == RTC_BOOT_FAST)
    {
        try_boot = true;
        timeout = 0;
    }
#if HAL_USE_CAN == TRUE
    else if ((m & 0xFFFFFF00) == RTC_BOOT_CANBL)
    {
        try_boot = false;
        timeout = 10000;
        can_set_node_id(m & 0xFF);
    }
    can_check_update();
    if (!can_check_firmware())
    {
        // bad firmware CRC, don't try and boot
        timeout = 0;
        try_boot = false;
    }
    else if (timeout != 0)
    {
        // fast boot for good firmware
        try_boot = true;
        timeout = 1000;
    }
#endif

    // if we fail to boot properly we want to pause in bootloader to give
    // a chance to load new app code
    set_fast_reboot(RTC_BOOT_OFF);
#endif

#ifdef HAL_GPIO_PIN_STAY_IN_BOOTLOADER
    // optional "stay in bootloader" pin
    if (palReadLine(HAL_GPIO_PIN_STAY_IN_BOOTLOADER) == HAL_STAY_IN_BOOTLOADER_VALUE)
    {
        try_boot = false;
        timeout = 0;
    }
#endif

    if (try_boot)
    {
        jump_to_app();
    }

#if defined(BOOTLOADER_DEV_LIST)
    init_uarts();
#endif
#if HAL_USE_CAN == TRUE
    can_start();
#endif
    flash_init();
    spiInit();


    // SPI1 기본 설정   PAL_MODE_ALTERNATE 번호 체크 요망
    palSetPadMode(GPIOA, 5,
                  PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // New SCK
    palSetPadMode(GPIOA, 6,
                  PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); //New MISO
    palSetPadMode(GPIOA, 7,
                  PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // New MOSI
    palSetPadMode(GPIOD, 7,
                  PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); // New CS
    
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &spi_ms5611);
    spiSelect(&SPID1);

    *rxbuf = 0x00;  //RX 버퍼 수신 값을 확인하기 위해서 0으로 초기화 했음
    *txbuf = 0x1E;
    
    spiExchange(&SPID1, SPI_BUFFERS_SIZE, &txbuf, &rxbuf); //SPI 송신 및 수신 동작 하는 함수
    //prev = *rxbuf;

    *txbuf = 0xAE;

    spiExchange(&SPID1, SPI_BUFFERS_SIZE, &txbuf, &rxbuf);

    *txbuf = 0x44;

    spiExchange(&SPID1, SPI_BUFFERS_SIZE, &txbuf, &rxbuf);

    palClearLine(LINE_PE14);
    palSetLineMode(LINE_PE14, PAL_MODE_INPUT_PULLUP); // AUX 1번 핀을 풀업 상태로 기본 설정

   *txbuf = 0xA0;
    spiStartExchange(&SPID1,SPI_BUFFERS_SIZE, &txbuf, &rxbuf);

    while (!palReadLine(LINE_PE14)) // AUX 1번 핀을 GND와 쇼트 시킨 상태에서 전원을 연결하면 진단 모드로 진입, 빼는 순간 초기화 진행
    {

        chThdSleepMilliseconds(1000);
        spiReceive(&SPID1, SPI_BUFFERS_SIZE, &rxbuf);
        uprintf(" C1 : %d, ",*rxbuf); // Serial 통신상 메시지 출력 함수

        chThdSleepMilliseconds(1000);
        *txbuf = 0x40;
        spiSend(&SPID1, SPI_BUFFERS_SIZE, &txbuf);

        chThdSleepMilliseconds(1000);
        *txbuf = 0x00;
        spiSend(&SPID1, SPI_BUFFERS_SIZE, &txbuf);

        chThdSleepMilliseconds(1000);
        spiReceive(&SPID1,SPI_BUFFERS_SIZE, &rxbuf);
        uprintf(" ADC : %d, ",*rxbuf);

        chThdSleepMilliseconds(1000);
        *txbuf = 0xAE;
        spiSend(&SPID1, SPI_BUFFERS_SIZE, &txbuf);

        chThdSleepMilliseconds(1000);
        spiReceive(&SPID1, SPI_BUFFERS_SIZE, &rxbuf);
        uprintf(" CRC : %d, ",*rxbuf);

        *txbuf = 0xAE;
        chThdSleepMilliseconds(1000);
        spiExchange(&SPID1, SPI_BUFFERS_SIZE, &txbuf, &rxbuf);
        uprintf(" CRC2 : %d, ",*rxbuf);

        *txbuf = 0x00;
        spiExchange(&SPID1, SPI_BUFFERS_SIZE, &txbuf, &rxbuf);
        uprintf(" Exchange Fuction ADC : %d",*rxbuf);
        spiReceive(&SPID1, SPI_BUFFERS_SIZE, &rxbuf);
        uprintf(". %d \n\r",*rxbuf);

    }

#if defined(BOOTLOADER_DEV_LIST)
    while (true)
    {
        bootloader(timeout);
        jump_to_app();
    }
#else
    // CAN only
    while (true)
    {
        uint32_t t0 = AP_HAL::millis();
        while (timeout == 0 || AP_HAL::millis() - t0 <= timeout)
        {
            can_update();
            chThdSleep(chTimeMS2I(1));
        }
        jump_to_app();
    }
#endif
}
