/*
 * hw_config.h
 *
 *  Created on: May 17, 2015
 *      Author: david_s5
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

/****************************************************************************
 * 10-8--2016:
 *  To simplify the ripple effect on the tools, we will be using
 *  /dev/serial/by-id/<asterisk>PX4<asterisk> to locate PX4 devices. Therefore
 *  moving forward all Bootloaders must contain the prefix "PX4 BL "
 *  in the USBDEVICESTRING
 *  This Change will be made in an upcoming BL release
 ****************************************************************************/
/*
 * Define usage to configure a bootloader
 *
 *
 * Constant                example          Usage
 * APP_LOAD_ADDRESS     0x08004000            - The address in Linker Script, where the app fw is org-ed
 * BOOTLOADER_DELAY     5000                  - Ms to wait while under USB pwr or bootloader request
 * BOARD_FMUV2
 * INTERFACE_USB        1                     - (Optional) Scan and use the USB interface for bootloading
 * INTERFACE_USART      1                     - (Optional) Scan and use the Serial interface for bootloading
 * USBDEVICESTRING      "PX4 BL FMU v2.x"     - USB id string
 * USBPRODUCTID         0x0011                - PID Should match defconfig
 * BOOT_DELAY_ADDRESS   0x000001a0            - (Optional) From the linker script from Linker Script to get a custom
 *                                               delay provided by an APP FW
 * BOARD_TYPE           9                     - Must match .prototype boad_id
 * _FLASH_KBYTES        (*(uint16_t *)0x1fff7a22) - Run time flash size detection
 * BOARD_FLASH_SECTORS  ((_FLASH_KBYTES == 0x400) ? 11 : 23) - Run time determine the physical last sector
 * BOARD_FLASH_SECTORS   11                   - Hard coded zero based last sector
 * BOARD_FLASH_SIZE     (_FLASH_KBYTES*1024)-   Total Flash size of device, determined at run time.
 *                         (1024 * 1024)      - Hard coded Total Flash of device - The bootloader and app reserved will be deducted
 *                                              programmatically
 *
 * BOARD_FIRST_FLASH_SECTOR_TO_ERASE  2        - Optional sectors index in the flash_sectors table  (F4 only), to begin erasing.
 *                                               This is to allow sectors to be reserved for app fw usage. That will NOT be erased
 *                                               during a FW upgrade.
 *                                               The default is 0, and selects the first sector to be erased, as the 0th entry in the
 *                                               flash_sectors table. Which is the second physical sector of FLASH in the device.
 *                                               The first physical sector of FLASH is used by the bootloader, and is not defined
 *                                               in the table.
 *
 * APP_RESERVATION_SIZE (BOARD_FIRST_FLASH_SECTOR_TO_ERASE * 16 * 1024) - Number of bytes reserved by the APP FW. This number plus
 *                                                                        BOOTLOADER_RESERVATION_SIZE  will be deducted from
 *                                                                        BOARD_FLASH_SIZE to determine the size of the App FW
 *                                                                        and hence the address space of FLASH to erase and program.
 * USBMFGSTRING            "PX4 AP"            - Optional USB MFG string (default is '3D Robotics' if not defined.)
 * SERIAL_BREAK_DETECT_DISABLED                -  Optional prevent break selection on Serial port from entering or staying in BL
 *
 * * Other defines are somewhat self explanatory.
 */

/* Boot device selection list*/
#define USB0_DEV       0x01
#define SERIAL0_DEV    0x02
#define SERIAL1_DEV    0x04

#define APP_LOAD_ADDRESS               0x08020000
#define BOOTLOADER_DELAY               5000
#define INTERFACE_USB                  1
#define INTERFACE_USB_CONFIG           "/dev/ttyACM0"
#define BOARD_VBUS                     MK_GPIO_INPUT(GPIO_OTGFS_VBUS)

//#define USE_VBUS_PULL_DOWN
#define INTERFACE_USART                1
#define INTERFACE_USART_CONFIG         "/dev/ttyS0,115200"
#define BOOT_DELAY_ADDRESS             0x000001a0
#define BOARD_TYPE                     54
#define _FLASH_KBYTES                  (*(uint32_t *)0x1FF1E880)
#define BOARD_FLASH_SECTORS            (15)
#define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

#define OSC_FREQ                       16

#define BOARD_PIN_LED_ACTIVITY         GPIO_nLED_BLUE // BLUE
#define BOARD_PIN_LED_BOOTLOADER       GPIO_nLED_GREEN // GREEN
#define BOARD_LED_ON                   0
#define BOARD_LED_OFF                  1

#define SERIAL_BREAK_DETECT_DISABLED   1

/*
 * Uncommenting this allows to force the bootloader through
 * a PWM output pin. As this can accidentally initialize
 * an ESC prematurely, it is not recommended. This feature
 * has not been used and hence defaults now to off.
 *
 * # define BOARD_FORCE_BL_PIN_OUT         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN14)
 * # define BOARD_FORCE_BL_PIN_IN          (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
 *
 * # define BOARD_POWER_PIN_OUT           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
 * # define BOARD_POWER_ON                 1
 * # define BOARD_POWER_OFF                0
 * # undef  BOARD_POWER_PIN_RELEASE        // Leave pin enabling Power - un comment to release (disable power)
 *
*/

#if !defined(ARCH_SN_MAX_LENGTH)
# define ARCH_SN_MAX_LENGTH 12
#endif

#if !defined(APP_RESERVATION_SIZE)
#  define APP_RESERVATION_SIZE 0
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 1
#endif

#if !defined(USB_DATA_ALIGN)
# define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB USB0_DEV|SERIAL0_DEV|SERIAL1_DEV
#endif

#endif /* HW_CONFIG_H_ */
