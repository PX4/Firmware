
/**
 * Satellite radio read interval. Only required to be nonzero if data is not sent using a ring call.
 *
 * @unit s
 * @min 0
 * @max 5000
 * @group Iridium SBD
 */
PARAM_DEFINE_INT32(ISBD_READ_INT, 0);

/**
 * Iridium SBD session timeout
 *
 * @unit s
 * @min 0
 * @max 300
 * @group Iridium SBD
 */
PARAM_DEFINE_INT32(ISBD_SBD_TIMEOUT, 60);

/**
 * Time the Iridium driver will wait for additional mavlink messages to combine them into one SBD message
 *
 * Value 0 turns the functionality off
 *
 * @unit ms
 * @min 0
 * @max 500
 * @group Iridium SBD
 */
PARAM_DEFINE_INT32(ISBD_STACK_TIME, 0);
