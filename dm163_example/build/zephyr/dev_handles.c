#include <zephyr/device.h>
#include <zephyr/toolchain.h>

/* 1 : /soc/rcc@40021000:
 * Supported:
 *    - /soc/pin-controller@48000000/gpio@48001c00
 *    - /soc/pin-controller@48000000/gpio@48001800
 *    - /soc/pin-controller@48000000/gpio@48001400
 *    - /soc/pin-controller@48000000/gpio@48001000
 *    - /soc/pin-controller@48000000/gpio@48000c00
 *    - /soc/pin-controller@48000000/gpio@48000800
 *    - /soc/pin-controller@48000000/gpio@48000400
 *    - /soc/pin-controller@48000000/gpio@48000000
 *    - /soc/serial@40004c00
 *    - /soc/serial@40013800
 *    - /soc/i2c@40005c00
 *    - /soc/i2c@40005800
 *    - /soc/i2c@40005400
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_9[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, DEVICE_HANDLE_ENDS };

/* 2 : /soc/interrupt-controller@40010400:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_51[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 3 : /soc/pin-controller@48000000/gpio@48001c00:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_96[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 4 : /soc/pin-controller@48000000/gpio@48001800:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_95[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 5 : /soc/pin-controller@48000000/gpio@48001400:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_94[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 6 : /soc/pin-controller@48000000/gpio@48001000:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_109[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 7 : /soc/pin-controller@48000000/gpio@48000c00:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 * Supported:
 *    - /soc/i2c@40005800/hts221@5f
 *    - /soc/i2c@40005800/lsm6dsl@6a
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_88[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 17, 20, DEVICE_HANDLE_ENDS };

/* 8 : /soc/pin-controller@48000000/gpio@48000800:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 * Supported:
 *    - /soc/i2c@40005800/vl53l0x@29
 *    - /dm163
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_20[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 21, 22, DEVICE_HANDLE_ENDS };

/* 9 : /soc/pin-controller@48000000/gpio@48000400:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 * Supported:
 *    - /dm163
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_19[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 22, DEVICE_HANDLE_ENDS };

/* 10 : /soc/pin-controller@48000000/gpio@48000000:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 * Supported:
 *    - /dm163
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_18[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 22, DEVICE_HANDLE_ENDS };

/* 11 : /soc/serial@40004c00:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_64[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 12 : /soc/serial@40013800:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_69[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 13 : /soc/i2c@40005c00:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_50[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 14 : /soc/i2c@40005800:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 * Supported:
 *    - /soc/i2c@40005800/hts221@5f
 *    - /soc/i2c@40005800/lis3mdl-magn@1e
 *    - /soc/i2c@40005800/lps22hb-press@5d
 *    - /soc/i2c@40005800/lsm6dsl@6a
 *    - /soc/i2c@40005800/vl53l0x@29
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_87[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 17, 18, 19, 20, 21, DEVICE_HANDLE_ENDS };

/* 15 : /soc/i2c@40005400:
 * Direct Dependencies:
 *    - /soc/rcc@40021000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_47[] = { 1, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 16 : /leds:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_36[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 17 : /soc/i2c@40005800/hts221@5f:
 * Direct Dependencies:
 *    - /soc/pin-controller@48000000/gpio@48000c00
 *    - /soc/i2c@40005800
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_89[] = { 7, 14, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 18 : /soc/i2c@40005800/lis3mdl-magn@1e:
 * Direct Dependencies:
 *    - /soc/i2c@40005800
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_90[] = { 14, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 19 : /soc/i2c@40005800/lps22hb-press@5d:
 * Direct Dependencies:
 *    - /soc/i2c@40005800
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_91[] = { 14, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 20 : /soc/i2c@40005800/lsm6dsl@6a:
 * Direct Dependencies:
 *    - /soc/pin-controller@48000000/gpio@48000c00
 *    - /soc/i2c@40005800
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_92[] = { 7, 14, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 21 : /soc/i2c@40005800/vl53l0x@29:
 * Direct Dependencies:
 *    - /soc/pin-controller@48000000/gpio@48000800
 *    - /soc/i2c@40005800
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_93[] = { 8, 14, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 22 : /dm163:
 * Direct Dependencies:
 *    - /soc/pin-controller@48000000/gpio@48000800
 *    - /soc/pin-controller@48000000/gpio@48000400
 *    - /soc/pin-controller@48000000/gpio@48000000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_21[] = { 8, 9, 10, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };
