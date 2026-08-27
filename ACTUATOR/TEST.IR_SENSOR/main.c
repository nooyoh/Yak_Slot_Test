/*
 * TEST.IR_SENSOR.c
 *
 * Created: 2026-08-27
 * Author : kccistc
 *
 * IR sensor detection-rate test for ATmega128A.
 *
 * Pill is dropped 20 times, detection rate is measured.
 *   - Real IR sensor is not connected yet, so the serial terminal
 *     stands in for it:
 *       ENTER (CR)   -> IR sensor "detected"  : success++ and attempt++
 *       SPACEBAR     -> not detected (told by operator) : attempt++ only
 *   - When attempt count reaches 20, the detection rate is printed
 *     and the test stops.
 *
 * Serial : UART0, 9600 8N1  (connect a USB-serial terminal such as
 *          Tera Term / PuTTY, local echo optional)
 */

#ifndef F_CPU
#define F_CPU 16000000UL      /* adjust to your board crystal */
#endif

#include <avr/io.h>
#include <util/delay.h>

#define BAUD        9600UL
#define UBRR_VALUE  ((F_CPU / (16UL * BAUD)) - 1UL)

#define TOTAL_ATTEMPTS  20

#define KEY_ENTER_CR    '\r'
#define KEY_ENTER_LF    '\n'
#define KEY_SPACE       ' '

/* ---------- UART0 ---------- */
static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);            /* enable RX and TX      */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);          /* 8 data, no parity, 1 stop */
}

static void uart_tx(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = (uint8_t)c;
}

static char uart_rx(void)
{
    while (!(UCSR0A & (1 << RXC0)))
        ;
    return (char)UDR0;
}

static void uart_print(const char *s)
{
    while (*s)
        uart_tx(*s++);
}

static void uart_print_uint(uint16_t v)
{
    char buf[6];
    int8_t i = 0;

    if (v == 0) {
        uart_tx('0');
        return;
    }
    while (v > 0 && i < (int8_t)sizeof(buf)) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0)
        uart_tx(buf[--i]);
}

/* ---------- application ---------- */
static void print_status(uint16_t success, uint16_t attempt)
{
    /* prints  "success/attempt = s/a" */
    uart_print("count  ");
    uart_print_uint(success);
    uart_tx('/');
    uart_print_uint(attempt);
    uart_print("\r\n");
}

static void print_result(uint16_t success, uint16_t attempt)
{
    /* rate in tenths of a percent to keep one decimal without float */
    uint16_t rate_x10 = (uint16_t)(((uint32_t)success * 1000UL) / attempt);

    uart_print("\r\n===== TEST DONE =====\r\n");
    uart_print("detected : ");
    uart_print_uint(success);
    uart_print("\r\n");
    uart_print("attempts : ");
    uart_print_uint(attempt);
    uart_print("\r\n");
    uart_print("detection rate : ");
    uart_print_uint(rate_x10 / 10);
    uart_tx('.');
    uart_print_uint(rate_x10 % 10);
    uart_print(" %\r\n");
    uart_print("=====================\r\n");
}

int main(void)
{
    uint16_t success = 0;
    uint16_t attempt = 0;
    char c;
    char prev = 0;

    uart_init();

    uart_print("\r\n=== IR SENSOR DETECTION-RATE TEST ===\r\n");
    uart_print("SPACE = missed   (attempt only)\r\n");
    uart_print("drop the pill 20 times...\r\n\r\n");

    while (attempt < TOTAL_ATTEMPTS)
    {
        c = uart_rx();

        /* swallow the LF of a CR+LF pair so ENTER counts once */
        if (c == KEY_ENTER_LF && prev == KEY_ENTER_CR)
        {
            prev = c;
            continue;
        }
        prev = c;

        if (c == KEY_ENTER_CR || c == KEY_ENTER_LF)
        {
            /* IR sensor detected the pill */
            success++;
            attempt++;
            uart_print("[DETECTED] ");
            print_status(success, attempt);
        }
        else if (c == KEY_SPACE)
        {
            /* operator reports the pill was dropped but not detected */
            attempt++;
            uart_print("[MISSED]   ");
            print_status(success, attempt);
        }
        /* any other key is ignored */
    }

    print_result(success, attempt);

    while (1)
    {
    }
}
