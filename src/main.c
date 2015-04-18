#include <ch.h>
#include <hal.h>

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("led");
  while (1) {
    palSetPad(GPIOD, GPIOD_LED3);
    chThdSleepMilliseconds(500);
    palClearPad(GPIOD, GPIOD_LED3);
    chThdSleepMilliseconds(500);
  }
}

int main(void) {
    halInit();
    chSysInit();

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    while (1) {
        chThdSleepMilliseconds(1000);
    }
    return 0;
}
