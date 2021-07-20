/**
 * @file tap_esc_params.c
 * Parameters for esc version
 *
 */

/**
 * Required esc firmware version.
 *
 * @group ESC
 * @min 0
 * @max 65535
 */
PARAM_DEFINE_INT32(ESC_FW_VER, 0);

/**
 * Required esc bootloader version.
 *
 * @group ESC
 * @min 0
 * @max 65535
 */
PARAM_DEFINE_INT32(ESC_BL_VER, 0);

/**
 * Required esc hardware version
 *
 * @group ESC
 * @min 0
 * @max 65535
 */
PARAM_DEFINE_INT32(ESC_HW_VER, 0);
