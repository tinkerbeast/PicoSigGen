#include "shell.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/multicore.h"

//#define INPUT_MAX_LEN 1024

char* shell_get_input(void) {
    static char input[INPUT_MAX_LEN];
    return input;
}

static
int shell_get_line_(uart_inst_t* uart, char* buf, int len) {    
    char ch = '\0';    
    int i = 0;
    for (; i < len && ch != '\r'; ++i) {
        ch = uart_getc(uart);
        uart_putc(uart, ch);
        buf[i] = ch;
    }
    buf[i] = '\0'; // overwrite \r or last char
    uart_putc(uart, '\n');
    return i - 1;
}


//#define COMMANDS_LEN 1
//#define COMMAND_MAGIC 0xabcd1234
//enum eCommands {
//    SETPINS = 0
//};


static
int shell_validate_command_(const char* buf, int len) {
    static const char* valid_commands[COMMANDS_LEN] = {
        "set_pins",
        "sig_sine",
        "sig_square",
        "sig_ramp",
        "set_freq",
        "set_arbitrary",
        "sig_arbitrary",
        "CREATE_ARBITRARY"
    };
    
    char input_copy[len];
    strncpy(input_copy, buf, len);
    char* context = NULL;
    char* first_word = strtok_r(input_copy, " ", &context);

    for (int i = 0; i < COMMANDS_LEN; ++i) {
        if (strncasecmp(valid_commands[i], first_word, len) == 0) {
            return i;
        }
    }    
    return -1;
}



semaphore_t command_done_;

void shell_set_command_done(void) {
    sem_release(&command_done_);
}


static
uart_inst_t* const UART_ID = uart0;

void shell_run(void) {
    // Uart hardware specs.
    const int UART_RX_PIN = 17;
    const int UART_TX_PIN = 16;    
    // Uart hardware config sepcs.
    const int cUART_CONFIG_BAUDRATE = 115200;
    // Init semaphore.
    sem_init(&command_done_, 0, 1);
    // Init uart0 on proper pins.
    uart_init(UART_ID, cUART_CONFIG_BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    // Init default led (used to blink tx/rx in uart).
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // Start shell loop.
    sem_acquire_blocking(&command_done_); // wait on main thread to give go.
    uart_puts(UART_ID, "Shell started: build=" __TIMESTAMP__ "\r\n");
    char* input = shell_get_input();
    while(true) {
        // Get User Input.
        uart_puts(UART_ID, "myshell> ");
        int len = shell_get_line_(UART_ID, input, INPUT_MAX_LEN);
        if (len == 0) continue;
        // Validate user input.
        int rc = shell_validate_command_(input, len);
        if (rc == -1) {
            char* context = NULL;
            char* first_word = strtok_r(input, " ", &context);
            uart_puts(UART_ID, first_word);
            uart_puts(UART_ID, ": command not found\r\n");
            continue;
        }
        // Send signal to other core and wait back for signal.
        multicore_fifo_push_blocking(rc);
        sem_acquire_blocking(&command_done_);
    }
}
