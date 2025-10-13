/**
 * Initialize the fans.
 */
void fans_init();

/**
 * Set the speed of the fans
 * @param speed uint8_t percent to set fans, from 0 to 100
 */
void fans_set_speed(uint8_t percent);

/**
 * Set the duty cycle of the fans
 * @param duty_cycle uint8_t duty cycle to set fans to, from 0 to 255
 */
void fans_set_duty_cycle(uint8_t duty_cycle);