#include "shell.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"






static inline
uint32_t gpio_array_to_mask(int n, const uint gpios[n]) {
  // Each pin value is a Pi Pico GPIO number. Since PIO gives us only 29 usable
  // GPIO pins (GP0 to GP28) we can specify the set of pins with a 32 bit mask.
  // Note: Internall pico has 36 GPIO pins
  uint32_t pin_mask = 0;
  for (int i = 0; i < n; ++i) {
      pin_mask |= (1u << gpios[i]);
  }
  return pin_mask;
}

//
int mypio_gpio_out_pio_config(PIO pio, uint sm,
                            int n, const uint gpios[n], bool initially_on) {
   // Get pin mask from gpio array
   uint32_t pin_mask = gpio_array_to_mask(n, gpios);
   // Mark pio pins as out a for gpio   
   pio_sm_set_pins_with_mask(pio, sm, initially_on, pin_mask); // sets inital value to pins
   pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask); // 1 is out, 0 is in
   for (int i = 0; i < n; ++i) {
     pio_gpio_init(pio, gpios[i]); // This has to do with GPIO muxing (peripheral to IO pad mapping)
                                   // See RP2040 datasheet "2.19.2. Function Selec"
   }   
   //
   return 0;
}


/**
 * NOTE: Order in which pins are shifted out - Order of gpios[i] does not matter. It will always shift out in min to max order.
 * // NOTE: number represents number of bits to consume
 *  // TODO: shift right = shift from right side.
 */
int mypio_gpio_out_sm_config(pio_sm_config* c, int n, const uint gpios[n],
                          bool osr_autopull, uint osr_shift_count, bool osr_shift_right) {
    // Determine pin range
    int min_pin = 32;
    int max_pin = -1;
    for (int i = 0; i < n; ++i) {
        min_pin = min_pin < gpios[i] ? min_pin : gpios[i];
        max_pin = max_pin > gpios[i] ? max_pin : gpios[i];
    }
    int pin_count = (max_pin - min_pin) + 1;
    // Set pin range
    sm_config_set_out_pins(c, min_pin, pin_count);
    // Set autopull configs
    sm_config_set_out_shift(c, osr_shift_right, osr_autopull, osr_shift_count);
    //
    return 0;
}








const double cBASE_FREQ_HZ = 5000000; // (125Mhz / (1 cycle program * clk divider))
#define WAVE_MAX_LEN 131072 // 128 KiB

static
uint8_t wave[WAVE_MAX_LEN];
static
int wave_len = 0;

void command_setpins(PIO pio, uint sm, uint8_t bits) {
  pio_sm_put_blocking(pio, sm, bits);
}

int command_sine(PIO pio, uint sm, int iterations, double freq_hz) {
  // Convert frequency to sample count and validate.
  int sample_count = round(cBASE_FREQ_HZ / freq_hz);
  if (sample_count >= WAVE_MAX_LEN) return -1;
  // Form the wave.
  for (int x = 0; x < sample_count; ++x) {
    double sine_x = sin(2 * M_PI * x / sample_count);
    wave[x] = round((sine_x * 127) + 128);
  }
  // Output the wave.
  for (int i = 0; i < iterations; ++i) {
    for (int j = 0; j < sample_count; ++j) {
      pio_sm_put_blocking(pio, sm, wave[j]);
    }
  }
  pio_sm_put_blocking(pio, sm, 0);
  return 0;
}

int command_square(PIO pio, uint sm, int iterations, double freq_hz, double percent_duty) {
  percent_duty /= 100;
  // Convert frequency to sample count and validate.
  int sample_count = round(cBASE_FREQ_HZ / freq_hz);  
  int high_sample_count = round(sample_count * percent_duty);
  int low_sample_count = sample_count - high_sample_count;
  
  for (int i = 0; i < iterations; ++i) {
    for (int j = 0; j < high_sample_count; ++j) {
      pio_sm_put_blocking(pio, sm, 255);
    }
    for (int j = 0; j < low_sample_count; ++j) {
      pio_sm_put_blocking(pio, sm, 0);
    }
  }
  pio_sm_put_blocking(pio, sm, 0);
  return 0;
}

int command_ramp(PIO pio, uint sm, int iterations, double freq_hz, 
    double percent_offset, double percent_rampup) {
  percent_offset /= 100;
  percent_rampup /= 100;
  // Convert frequency to sample count and validate.
  int sample_count = round(cBASE_FREQ_HZ / freq_hz);
  if (sample_count >= WAVE_MAX_LEN) return -1;
  int offset_sample_count = floor(sample_count * percent_offset);
  int rampup_sample_count = round(sample_count * percent_rampup);
  int rampdown_sample_count = sample_count - (offset_sample_count + rampup_sample_count);
  printf("command_ramp sample_count=%d offset_sample_count=%d rampup_sample_count=%d rampdown_sample_count=%d\r\n", 
      sample_count, offset_sample_count, rampup_sample_count, rampdown_sample_count);
  // Form the wave - Set 0 for offset period.
  for (int x = 0; x < offset_sample_count; ++x) {
    wave[x] = 0;
  }
  // Form the wave - draw up ramp
  // TODO: Use Bresenham line drawing algorithm for ramp
  int x0 = offset_sample_count;  
  int del_y = 255 - 0;
  int del_x = rampup_sample_count;
  double m = ((double)del_y) / del_x;
  double c = -1.0 * m * x0; // since y0 = 0
  for (int x = x0; x < x0 + del_x; ++x) {
    wave[x] = round(m * x + c);
  }
  // Form the wave - draw down ramp
  x0 += del_x;
  del_y = 0 - 255;
  del_x = rampdown_sample_count;
  m = ((double)del_y) / del_x;
  c = 255 - 1.0 * m * x0; // since y0 = 255
  for (int x = x0; x < sample_count; ++x) {
    wave[x] = round(m * x + c);
  }

  // Output the wave.
  for (int i = 0; i < iterations; ++i) {
    for (int j = 0; j < sample_count; ++j) {
      pio_sm_put_blocking(pio, sm, wave[j]);
    }
  }
  pio_sm_put_blocking(pio, sm, 0);
  return 0;  
}


#include "parallel_out.pio.h"
// parallel_out_OUT_BITCOUNT
// parallel_out_program
// parallel_out_program_get_default_config


int main() {
    // NOTE: clk_sys by default is 125000 setting it explicitly 
    //       because signal-gen depends on frequency.
    set_sys_clock_khz(125000, true);
    // Startup defaults
    stdio_init_all();
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    for (int i = 0; i < 10; ++i) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
    
    // Launch shell.
    multicore_launch_core1(shell_run);
    sleep_ms(10); // TODO: better sync mechanism

    // Startup debug info.
    float cycles_per_sec = clock_get_hz(clk_sys);
    printf("DEBUG: clock_kHz=%f\r\n", cycles_per_sec/1000.0f);
    
    // Allocate PIO resources
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &parallel_out_program);
    pio_sm_config c = parallel_out_program_get_default_config(offset);
    printf("PIO init: Allocated pio resources pio=%d offset=%d sm=%d\r\n", pio, offset, sm);
    // Configs and validations    
    int n = 8;
    uint gpios[8] = {8, 9, 10, 11, 12, 13, 14, 15};
    if (parallel_out_OUT_BITCOUNT != n) goto my_exit_main;
    // Configure PIO for my programme    
    mypio_gpio_out_sm_config(&c, n, gpios, true, n, true);
    // clock does 125'000'000 cycles per second
    // loop is 19 cycles
    // clock is divided by 1.0
    // so per pio_sm_put_blocking is - 125'000'000 / (1cycle * 1.0) = 5000000 Hz
    sm_config_set_clkdiv(&c, 25.0);
    mypio_gpio_out_pio_config(pio, sm, n, gpios, false);
    //sm_config_set_clkdiv_int_frac(&c, 62500u, 0);
    // Enable the PIO
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    printf("PIO init: pio enabled\r\n");

    

    
    // Start loop and continue shell.
    shell_set_command_done();
    while (true) {
        int command = multicore_fifo_pop_blocking();
        char* input = shell_get_input();
        char* temp[128];
        switch (command) {
            case SET_PINS: {
                int bits = -1;
                sscanf(input, "%s %d", temp, &bits);
                printf("SETPINS bits=%d\r\n", bits);
                command_setpins(pio, sm, bits);
                break;
            }
            case SIG_SINE: {
                int iterations = 1;
                double freq_khz = 1;
                sscanf(input, "%s %d %lf", temp, &iterations, &freq_khz);
                printf("SIG_SINE iterations=%d freq_khz=%f\r\n", iterations, freq_khz);
                int rc = command_sine(pio, sm, iterations, freq_khz * 1000);
                if (rc < 0) printf("ERROR: SIG_SINE freq_khz=%f not supported\r\n", freq_khz);
                break;
            }
            case SIG_SQUARE: {
                int iterations = 1;
                double freq_khz = 1;
                double duty = 50;
                sscanf(input, "%s %d %lf %lf", temp, &iterations, &freq_khz, &duty);
                printf("SIG_SQUARE iterations=%d freq_khz=%f duty=%f\r\n", iterations, freq_khz, duty);
                int rc = command_square(pio, sm, iterations, freq_khz * 1000, duty);
                if (rc < 0) printf("ERROR: SIG_SQUARE freq_khz=%f not supported\r\n", freq_khz);
                break;
            }
            case SIG_RAMP: {
                int iterations = 1;
                double freq_khz = 1;
                double percent_offset = 0.0;
                double percent_rampup = 50;
                sscanf(input, "%s %d %lf %lf %lf", temp, &iterations, &freq_khz, &percent_offset, &percent_rampup);
                printf("SIG_RAMP iterations=%d freq_khz=%f percent_offset=%f percent_rampup=%f\r\n", iterations, freq_khz, percent_offset, percent_rampup);
                int rc = command_ramp(pio, sm, iterations, freq_khz * 1000, percent_offset, percent_rampup);
                if (rc < 0) printf("ERROR: SIG_RAMP freq_khz=%f not supported\r\n", freq_khz);
                break;
            }
            case SET_FREQ: {
                double freq_hz = 5000000; // defalut 5 Mhz
                sscanf(input, "%s %lf", temp, &freq_hz);
                double divider = 125000000 / freq_hz;
                printf("SET_FREQ freq_hz=%f divider=%f\r\n", freq_hz, divider);
                if (divider < 25.0) {
                  printf("ERROR: SET_FREQ freq_hz=%f not supported\r\n", freq_hz);
                } else {
                  pio_sm_restart(pio, sm);
                  sm_config_set_clkdiv(&c, divider);
                  mypio_gpio_out_pio_config(pio, sm, n, gpios, false);
                  pio_sm_init(pio, sm, offset, &c);
                  pio_sm_set_enabled(pio, sm, true);
                }
                break;
            }
            case SET_ARBITRARY: {
              char* context = NULL;
              char* token = strtok_r(input, " ", &context); // Ignorw first word which is the command.              
              printf("SET_ARBITRARY first_token=%s\r\n", token);
              token = strtok_r(NULL, " ", &context);              
              int i = 0;
              while (token != NULL) {                
                wave[i] = atoi(token); // TODO: bounds check.
                printf("%d, ", wave[i]);
                token = strtok_r(NULL, " ", &context);
                ++i;
              }
              printf("\r\n");
              wave_len = i;
              printf("SET_ARBITRARY wave_len=%d\r\n", wave_len);
              break;
            }
            case SIG_ARBITRARY: {
              int iterations = 1;
              sscanf(input, "%s %d", temp, &iterations);
              printf("SIG_ARBITRARY iterations=%d\r\n", iterations);
              for (int i = 0; i < iterations; ++i) {
                for (int j = 0; j < wave_len; ++j) {
                  pio_sm_put_blocking(pio, sm, wave[j]);
                }
              }
              pio_sm_put_blocking(pio, sm, 0);
              break;
            }
            case CREATE_ARBITRARY: {
              int k = 0;
              for  (int i = 0; i < 255; i += 32) {
                for  (int j = 0; j < 8192; ++j) {
                  wave[k] = i;
                  ++k;
                }
              }
              wave_len = k;        
              printf("CREATE_ARBITRARY wave_len=%d\r\n", wave_len);
              break;
            }
            default: {
                printf("Invalid command\r\n");
                break;
            }
        }
        shell_set_command_done();        
    }

my_exit_main:

    printf("DEBUG: Raspi-pico exited main\r\n");    
    sleep_ms(5000);

    return EXIT_SUCCESS;    
}


