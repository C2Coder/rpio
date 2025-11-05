#include "entry.pio.h"
#include "hardware/pio.h"
#include <hardware/watchdog.h>
#include <itf.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <rpio.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Framebuffer configuration
#define FB_WIDTH 64
#define FB_HEIGHT 64
#define FB_PIXELS (FB_WIDTH * FB_HEIGHT)
#define FB_RGB_BYTES (FB_PIXELS * 3)

#define DATA_BASE_PIN 0
#define DATA_N_PINS 6
#define ROWSEL_BASE_PIN 6
#define ROWSEL_N_PINS 5
#define CLK_PIN 11
#define STROBE_PIN 12
#define OEN_PIN 13

// Double-buffered framebuffers (RGB888 packed into a uint32_t as 0x00RRGGBB)
// idx 0 and 1 are ping-pong buffers. Producer writes to `fb_write_idx`,
// when a full frame is ready it signals core1 with the buffer index and
// switches to the other buffer.
static volatile uint32_t framebuffer[2][FB_PIXELS];

// Indexes for buffer management (values 0 or 1)
static volatile int fb_write_idx = 0;   // buffer the producer writes into
static volatile int fb_display_idx = 1; // buffer the display/core1 reads from

static uint8_t fb_partial[3];
static size_t fb_partial_len = 0;
static size_t fb_write_pos = 0;

void fb_update_from_rgb_chunk(const uint8_t *src, size_t len) {
    printf("fb_update_from_rgb_chunk: len=%u\n", (unsigned)len);
    if (!src || len == 0)
        return;

    size_t idx = 0;
    // local copy of write index to reduce volatile accesses
    int w = fb_write_idx;

    while (idx < len) {
        // Fill partial pixel if present
        if (fb_partial_len > 0) {
            while (fb_partial_len < 3 && idx < len) {
                fb_partial[fb_partial_len++] = src[idx++];
            }

            if (fb_partial_len == 3) {
                uint8_t r = fb_partial[0];
                uint8_t g = fb_partial[1];
                uint8_t b = fb_partial[2];
                if (fb_write_pos < FB_PIXELS) {
                    framebuffer[w][fb_write_pos++] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
                }
                fb_partial_len = 0;
                if (fb_write_pos >= FB_PIXELS) {
                    // finished a full frame in buffer `w`
                    multicore_fifo_push_blocking((uint32_t)w);
                    // switch write buffer
                    w = 1 - w;
                    fb_write_idx = w;
                    fb_write_pos = 0;
                    fb_partial_len = 0;
                }
            }
        }

        // Fast path: convert as many full pixels from src as possible
        while (idx + 3 <= len && fb_write_pos < FB_PIXELS) {
            uint8_t r = src[idx + 0];
            uint8_t g = src[idx + 1];
            uint8_t b = src[idx + 2];
            framebuffer[w][fb_write_pos++] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
            idx += 3;
        }

        // If we filled the write buffer, signal and switch
        if (fb_write_pos >= FB_PIXELS) {
            multicore_fifo_push_blocking((uint32_t)w);
            w = 1 - w;
            fb_write_idx = w;
            fb_write_pos = 0;
            fb_partial_len = 0;
        }

        // Stash any trailing bytes (<3) for the next call
        while (idx < len) {
            fb_partial[fb_partial_len++] = src[idx++];
        }
    }
}

static void task_loop() {

    PIO pio = pio0;
    uint sm_data = 0;
    uint sm_row = 1;

    int current = fb_display_idx;
    uint32_t loop_counter = 0;
    uint data_prog_offs = pio_add_program(pio, &hub75_data_rgb888_program);
    uint row_prog_offs = pio_add_program(pio, &hub75_row_program);
    hub75_data_rgb888_program_init(pio, sm_data, data_prog_offs, DATA_BASE_PIN, CLK_PIN);
    hub75_row_program_init(pio, sm_row, row_prog_offs, ROWSEL_BASE_PIN, ROWSEL_N_PINS, STROBE_PIN);

    static uint32_t gc_row[2][FB_WIDTH];
    while (true) {
        // If producer signalled a new buffer, grab it (non-blocking check)
        if (multicore_fifo_rvalid()) {
            uint32_t new_buf = multicore_fifo_pop_blocking();
            if (new_buf <= 1) {
                fb_display_idx = (int)new_buf;
                current = fb_display_idx;
                // small debug
                printf("core1: switched to buffer %u\n", (unsigned)new_buf);
            }
        }

        // Drive the display using framebuffer[current]. Replace the placeholder
        // below with your actual driver call (e.g. drv_hub75_display(framebuffer[current])).
        if ((loop_counter & 0x3f) == 0) { // throttle prints
            uint32_t pix = framebuffer[current][0];
            printf("core1: driving buffer %d, first pix 0x%06x\n", current, pix & 0x00ffffff);
        }
        loop_counter++;

        for (int rowsel = 0; rowsel < (1 << ROWSEL_N_PINS); ++rowsel) {
            for (int x = 0; x < FB_WIDTH; ++x) {
                gc_row[0][x] = framebuffer[current][rowsel * FB_WIDTH + x];
                gc_row[1][x] = framebuffer[current][((1u << ROWSEL_N_PINS) + rowsel) * FB_WIDTH + x];
            }
            for (int bit = 0; bit < 8; ++bit) {
                hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, bit);
                for (int x = 0; x < FB_WIDTH; ++x) {
                    pio_sm_put_blocking(pio, sm_data, gc_row[0][x]);
                    pio_sm_put_blocking(pio, sm_data, gc_row[1][x]);
                }
                // Dummy pixel per lane
                pio_sm_put_blocking(pio, sm_data, 0);
                pio_sm_put_blocking(pio, sm_data, 0);
                // SM is finished when it stalls on empty TX FIFO
                hub75_wait_tx_stall(pio, sm_data);
                // Also check that previous OEn pulse is finished, else things can get out of sequence
                hub75_wait_tx_stall(pio, sm_row);

                // Latch row data, pulse output enable for new row.
                pio_sm_put_blocking(pio, sm_row, rowsel | (100u * (1u << bit) << 5));
            }
        }
    }
}

extern void misc_cmd(uint8_t cmd);

uint8_t i = 0;
static void cmd_loop() {
    while (true) {

        const size_t CHUNK = 256 * 3; // choose a small chunk that your interface can handle
        uint8_t buf[CHUNK];

        itf_read(buf, CHUNK);
        fb_update_from_rgb_chunk(buf, CHUNK);

        i += 1;
        if ((i & 0x1f) == 0)
            printf("received chunk #%u\n", i);

        /*uint8_t ctype;;
        itf_read(&ctype, 1);
        printf("cmd type: [%u]\n", ctype);

        if (ctype == rpio_ctype_nop)
            continue;

        uint8_t cmd;
        itf_read(&cmd, 1);

        printf("cmd: [%d]\n", cmd);

        switch (ctype) {
        case rpio_ctype_misc:
            misc_cmd(cmd);
            break;
*/
        // case rpio_ctype_fb:
        //     fb_cmd(cmd);
        //     break;

        // case rpio_ctype_hub75:
        //     hub75_cmd(cmd);
        //     break;
        /*
                default:
                    break; // FIXME: handle on itf? reset?
                }*/
    }
}

int main() {
    // setup interface
    stdio_init_all();

#ifdef RPIO_INTERFACE_SPI
    itf_spi_init(1000 * 1000);
#endif

    // setup loops

    multicore_launch_core1(&task_loop);
    cmd_loop();
}
