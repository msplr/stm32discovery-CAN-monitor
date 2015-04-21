#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include <shell.h>
#include <serial-can-bridge/serial_can_bridge.h>
#include <serial-datagram/serial_datagram.h>
#include "usbcfg.h"

static const CANConfig can1_config = {
    .mcr = (1 << 6)  /* Automatic bus-off management enabled. */
         | (1 << 2), /* Message are prioritized by order of arrival. */

#if defined(BOARD_ST_STM32F3_DISCOVERY)
    /* APB Clock is 36 Mhz
       36MHz / 2 / (1tq + 10tq + 7tq) = 1MHz => 1Mbit */
    .btr = (1 << 0)  /* Baudrate prescaler (10 bits) */
         | (9 << 16)/* Time segment 1 (3 bits) */
         | (6 << 20) /* Time segment 2 (3 bits) */
         | (0 << 24) /* Resync jump width (2 bits) */
#elif defined(BOARD_ST_STM32F4_DISCOVERY)
    /* APB Clock is 42 Mhz
       42MHz / 2 / (1tq + 12tq + 8tq) = 1MHz => 1Mbit */
    .btr = (1 << 0)  /* Baudrate prescaler (10 bits) */
         | (11 << 16)/* Time segment 1 (3 bits) */
         | (7 << 20) /* Time segment 2 (3 bits) */
         | (0 << 24) /* Resync jump width (2 bits) */
#endif
#if 0
         | (1 << 30) /* Loopback mode enabled */
#endif
};

SerialUSBDriver SDU1;
BaseSequentialStream* stdout;

#define CAN_BRIDGE_RX_STACKSIZE 512
#define CAN_BRIDGE_PRIO         NORMALPRIO

void can_bridge(BaseAsynchronousChannel *chp);
msg_t can_bridge_rx_thread(void *p);

#define CAN_RX_QUEUE_SIZE   512
#define CAN_TX_QUEUE_SIZE   512

memory_pool_t can_rx_pool;
memory_pool_t can_tx_pool;
mailbox_t can_rx_queue;
mailbox_t can_tx_queue;
msg_t rx_mbox_buf[CAN_RX_QUEUE_SIZE];
msg_t tx_mbox_buf[CAN_TX_QUEUE_SIZE];

struct can_frame rx_pool_buf[CAN_RX_QUEUE_SIZE];
struct can_frame tx_pool_buf[CAN_TX_QUEUE_SIZE];

static THD_WORKING_AREA(can_tx_thread_wa, 256);
static THD_FUNCTION(can_tx_thread, arg) {
    (void)arg;
    chRegSetThreadName("CAN tx");
    while (1) {
        struct can_frame *framep;
        msg_t m = chMBFetch(&can_tx_queue, (msg_t *)&framep, TIME_INFINITE);
        if (m != MSG_OK) {
            continue;
        }
        CANTxFrame txf;
        uint32_t id = framep->id;
        txf.RTR = 0;
        if (id & CAN_FRAME_EXT_FLAG) {
            txf.EID = id & CAN_FRAME_EXT_ID_MASK;
            txf.IDE = 1;
        } else {
            txf.SID = id & CAN_FRAME_STD_ID_MASK;
            txf.IDE = 0;
        }

        if (id & CAN_FRAME_RTR_FLAG) {
            txf.RTR = 1;
        }

        txf.DLC = framep->dlc;
        txf.data32[0] = framep->data.u32[0];
        txf.data32[1] = framep->data.u32[1];

        chPoolFree(&can_tx_pool, framep);
        canTransmit(&CAND1, CAN_ANY_MAILBOX, &txf, MS2ST(100));
    }
    return 0;
}

static THD_WORKING_AREA(can_rx_thread_wa, 256);
static THD_FUNCTION(can_rx_thread, arg) {
    (void)arg;
    chRegSetThreadName("CAN rx");
    while (1) {
        CANRxFrame rxf;
        msg_t m = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxf, MS2ST(1000));
        if (m != MSG_OK) {
            continue;
        }
        // if (!can_id_passes_filter(id)) {
        //     continue;
        // }
        struct can_frame *f = (struct can_frame *)chPoolAlloc(&can_rx_pool);
        if (f == NULL) {
            continue;
        }
        if (rxf.IDE) {
            f->id = rxf.EID | CAN_FRAME_EXT_FLAG;
        } else {
            f->id = rxf.SID;
        }
        if (rxf.RTR) {
            f->id |= CAN_FRAME_RTR_FLAG;
        }
        f->dlc = rxf.DLC;
        f->data.u32[0] = rxf.data32[0];
        f->data.u32[1] = rxf.data32[1];
        if (chMBPost(&can_rx_queue, (msg_t)f, TIME_IMMEDIATE) != MSG_OK) {
            // couldn't post message: drop data & free the memory
            chPoolFree(&can_rx_pool, f);
        }
    }
    return 0;
}

void can_init(void)
{
    // rx queue
    chMBObjectInit(&can_rx_queue, rx_mbox_buf, CAN_RX_QUEUE_SIZE);
    chPoolObjectInit(&can_rx_pool, sizeof(struct can_frame), NULL);
    chPoolLoadArray(&can_rx_pool, rx_pool_buf, sizeof(rx_pool_buf)/sizeof(struct can_frame));

    // tx queue
    chMBObjectInit(&can_tx_queue, tx_mbox_buf, CAN_TX_QUEUE_SIZE);
    chPoolObjectInit(&can_tx_pool, sizeof(struct can_frame), NULL);
    chPoolLoadArray(&can_tx_pool, tx_pool_buf, sizeof(tx_pool_buf)/sizeof(struct can_frame));

#if defined(BOARD_ST_STM32F3_DISCOVERY)
    // CAN gpio init
    iomode_t mode = PAL_STM32_MODE_ALTERNATE | PAL_STM32_OTYPE_PUSHPULL
        | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING
        | PAL_STM32_ALTERNATE(9);
    palSetPadMode(GPIOB, GPIOB_PIN8, mode); // RX
    palSetPadMode(GPIOB, GPIOB_PIN9, mode); // TX
    canStart(&CAND1, &can1_config);
#elif defined(BOARD_ST_STM32F4_DISCOVERY)
    // CAN1 gpio init
    iomode_t mode = PAL_STM32_MODE_ALTERNATE | PAL_STM32_OTYPE_PUSHPULL
        | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING
        | PAL_STM32_ALTERNATE(9);
    palSetPadMode(GPIOD, GPIOD_PIN0, mode); // RX
    palSetPadMode(GPIOD, GPIOD_PIN1, mode); // TX
    canStart(&CAND1, &can1_config);
    // canSTM32SetFilters(uint32_t can2sb, uint32_t num, const CANFilter *cfp);
#endif
}

char hex4(uint8_t b)
{
    b &= 0x0f;
    if (b < 10) {
        return '0' + b;
    } else {
        return 'a' - 10 + b;
    }
}

void print_can_frame(BaseSequentialStream *out, struct can_frame *f)
{
    char buf[25];
    int i;
    for (i = 0; i < f->dlc; i++) {
        buf[3*i] = hex4(f->data.u8[i]>>4);
        buf[3*i+1] = hex4(f->data.u8[i]);
        buf[3*i+2] = ' ';
    }
    buf[3*i] = 0;
    if (!(f->id & CAN_FRAME_RTR_FLAG)) {
        if (f->id & CAN_FRAME_EXT_FLAG) {
            chprintf(out, "%08x:E %s\n", f->id & CAN_FRAME_EXT_ID_MASK, buf);
        } else {
            chprintf(out, "%03x: %s\n", f->id & CAN_FRAME_STD_ID_MASK, buf);
        }
    } else {
        if (f->id & CAN_FRAME_EXT_FLAG) {
            chprintf(out, "%08x:E %u\n", f->id & CAN_FRAME_EXT_ID_MASK, f->dlc);
        } else {
            chprintf(out, "%03x: !%u\n", f->id & CAN_FRAME_STD_ID_MASK, f->dlc);
        }
    }
}

static void cmd_candump(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    while (1) {
        if (palReadPad(GPIOA, GPIOA_BUTTON)) {
            return;
        }
        struct can_frame *framep;
        msg_t m = chMBFetch(&can_rx_queue, (msg_t *)&framep, MS2ST(100));
        if (m != MSG_OK) {
            continue;
        }
        print_can_frame(chp, framep);
        chPoolFree(&can_rx_pool, framep);
    }
}

const ShellCommand shell_commands[] = {
  {"candump", cmd_candump},
  {NULL, NULL}
};

int main(void) {
    halInit();
    chSysInit();

    // USB Serial Driver
    usbStop(serusbcfg.usbp);
    chThdSleepMilliseconds(100);
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
    stdout = (BaseSequentialStream *)&SDU1;

    bool run_bridge = false;
    if (!palReadPad(GPIOA, GPIOA_BUTTON)) {
        run_bridge = true;
    }

    while (SDU1.config->usbp->state != USB_ACTIVE) {
        chThdSleepMilliseconds(100);
    }

    can_init();
    chThdCreateStatic(can_tx_thread_wa, sizeof(can_tx_thread_wa), NORMALPRIO, can_tx_thread, NULL);
    chThdCreateStatic(can_rx_thread_wa, sizeof(can_rx_thread_wa), NORMALPRIO, can_rx_thread, NULL);

    if (run_bridge) {
        LED_BLUE(1);
        can_bridge((BaseAsynchronousChannel *)&SDU1);
        while (1) {
            chThdSleepMilliseconds(100);
        }
    }

    LED_GREEN(1);
    shellInit();
    static thread_t *shelltp = NULL;
    static ShellConfig shell_cfg;
    shell_cfg.sc_channel = (BaseSequentialStream*)&SDU1;
    shell_cfg.sc_commands = shell_commands;
    while (true) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                shelltp = shellCreate(&shell_cfg, THD_WORKING_AREA_SIZE(2048), NORMALPRIO);
            }
        } else if (chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);
            shelltp = NULL;
        }
        chThdSleepMilliseconds(500);
    }
    return 0;
}

void can_bridge(BaseAsynchronousChannel *chp)
{
    chThdCreateFromHeap(NULL, /* Use system heap */
                        CAN_BRIDGE_RX_STACKSIZE,
                        CAN_BRIDGE_PRIO,
                        can_bridge_rx_thread,
                        (void *)chp);

    int len;
    serial_datagram_rcv_handler_t rcv;

    static uint8_t buf[32];
    static char datagram_buf[64];

    serial_datagram_rcv_handler_init(
        &rcv,
        datagram_buf,
        sizeof(datagram_buf),
        can_bridge_datagram_rcv_cb,
        NULL);

    while (1) {
        len = chnReadTimeout(chp, buf, sizeof(buf), MS2ST(10));
        if (len == 0) {
            continue;
        }
        serial_datagram_receive(&rcv, buf, len);
    }
}

void can_interface_send(struct can_frame *frame)
{
    struct can_frame *tx = (struct can_frame *)chPoolAlloc(&can_tx_pool);
    if (tx == NULL) {
        return;
    }
    tx->id = frame->id;
    tx->dlc = frame->dlc;
    tx->data.u32[0] = frame->data.u32[0];
    tx->data.u32[1] = frame->data.u32[1];
    if (chMBPost(&can_tx_queue, (msg_t)tx, MS2ST(100)) != MSG_OK) {
        // couldn't post, free memory
        chPoolFree(&can_tx_pool, tx);
    }
    return;
}

void serial_write(void *arg, const void *p, size_t len)
{
    BaseAsynchronousChannel *chp = (BaseAsynchronousChannel *)arg;
    chnWrite(chp, p, len);
}

msg_t can_bridge_rx_thread(void *p)
{
    chRegSetThreadName("can_bridge_rx");

    struct can_frame *framep;

    static uint8_t outbuf[32];
    size_t outlen;

    /* Wait as long as the thread is not finished (semaphore not taken) */
    while (1) {
        msg_t m = chMBFetch(&can_rx_queue, (msg_t *)&framep, MS2ST(100));
        if (m != MSG_OK) {
            continue;
        }

        outlen = sizeof(outbuf);
        if (can_bridge_frame_write(framep, outbuf, &outlen)) {
            serial_datagram_send(outbuf, outlen, serial_write, p);
        }

        chPoolFree(&can_rx_pool, framep);
    }

    return MSG_OK;
}
