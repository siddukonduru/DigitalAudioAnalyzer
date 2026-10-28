/*
 * AudioAnalyzer_PC rev A firmware
 * RP2040 + 2x PCM1865 -> USB Audio Class 2 microphone, 6 channels, 24-bit, 48 kHz.
 *
 * Pins (RP2040 GPIO):
 *   18 I2C1 SDA, 19 I2C1 SCL          PCM1865 #1 (U3) @0x4A, #2 (U4) @0x4B
 *   20 BCK (in), 22 LRCK (in)         from U3 (master, 24.576 MHz crystal); BCK = 256 fs, LRCK 1/256 duty
 *   24 DOUT of U3 (ch1..4), 25 DOUT of U4 (ch5,6)   TDM, 4 x 32-bit slots, offset 1 BCK
 *   21, 23  unused SCK lines (inputs)
 *   8  status LED (on = streaming, slow blink = idle, fast blink = ADC fault)
 *   4/5 UART1 TX/RX debug 115200
 *
 * USB channel order: 1=AIN1 2=AIN2 3=AIN3 4=AIN4 (U3) 5=AIN5 6=AIN6 (U4)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/usb.h"
#include "hardware/structs/scb.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "tusb.h"
#include "pcm1865.h"
#include "tdm_rx.pio.h"

#define FW_VERSION "2.6"

#define PIN_SDA     18
#define PIN_SCL     19
#define PIN_BCK     20
#define PIN_SCK2    21
#define PIN_LRCK    22
#define PIN_SCK1    23
#define PIN_D0      24
#define PIN_D1      25
#define PIN_LED     8
#define EPNUM_AUDIO_IN_NUM 1   /* must match EPNUM_AUDIO_IN in usb_descriptors.c */

#define RING_WORDS  2048                     /* per ADC: 512 frames of 4 slots = 10.7 ms */
#define RING_BYTES  (RING_WORDS * 4)
#define FRAMES_PER_PKT  48                   /* 1 ms at 48 kHz */

static uint32_t ring0[RING_WORDS] __attribute__((aligned(RING_BYTES)));
static uint32_t ring1[RING_WORDS] __attribute__((aligned(RING_BYTES)));
static int dma_ch[2];
static uint rd_idx[2];
static PIO pio = pio0;
static uint sm[2];
static uint pio_offset;

static volatile bool streaming = false;
static volatile bool restart_req = false;
/* crash recorder: watchdog scratch registers survive a watchdog reset (cleared by unplugging)
 *   scratch[0] = last breadcrumb, scratch[1] = number of watchdog resets, scratch[2] = hard-fault PC, scratch[3] = fault LR */
#define CRUMB(x) (watchdog_hw->scratch[0] = (x))
static bool safe_mode = false;            /* after a crash: enumerate and answer the debug port, but never stream */   /* set from USB callbacks, handled in the main loop */
static volatile int8_t itf_event = -1;      /* last alt setting selected by the host, for the log */
static bool adc_ok = false;
static uint32_t sampFreq = AUDIO_SAMPLE_RATE;
static uint8_t clkValid = 1;
static audio_control_range_4_n_t(1) sampleFreqRng;

static uint8_t usb_pkt[FRAMES_PER_PKT * AUDIO_N_CHANNELS * AUDIO_BYTES_PER_SAMPLE];

/* ------------------------------------------------------------------ DMA */
static void dma_irq_handler(void) {
    for (int i = 0; i < 2; i++) {
        if (dma_channel_get_irq0_status(dma_ch[i])) {
            dma_channel_acknowledge_irq0(dma_ch[i]);
            /* ring wrap keeps the write address inside the buffer; just re-arm the count */
            dma_channel_set_trans_count(dma_ch[i], RING_WORDS, true);
        }
    }
}

static void capture_start(void) {
    /* v1.4: stop the DMA *safely*. Before, the channels were aborted with their IRQ still enabled; the abort raises the
     * completion IRQ (RP2040-E13), whose handler re-triggered the channel, so dma_channel_abort() waited forever for a
     * paced channel whose PIO was already stopped -> main loop (and USB) hung as soon as Windows opened the stream. */
    CRUMB(0x100);
    for (int i = 0; i < 2; i++) dma_channel_set_irq0_enabled(dma_ch[i], false);
    CRUMB(0x101);
    pio_sm_set_enabled(pio, sm[0], false);
    pio_sm_set_enabled(pio, sm[1], false);
    CRUMB(0x102);
    dma_channel_abort(dma_ch[0]);
    CRUMB(0x103);
    dma_channel_abort(dma_ch[1]);
    CRUMB(0x104);
    dma_channel_acknowledge_irq0(dma_ch[0]);
    dma_channel_acknowledge_irq0(dma_ch[1]);
    pio_sm_clear_fifos(pio, sm[0]);
    pio_sm_clear_fifos(pio, sm[1]);
    pio_sm_restart(pio, sm[0]);
    pio_sm_restart(pio, sm[1]);
    pio_sm_exec(pio, sm[0], pio_encode_jmp(pio_offset));
    pio_sm_exec(pio, sm[1], pio_encode_jmp(pio_offset));

    CRUMB(0x105);
    uint32_t *rings[2] = {ring0, ring1};
    for (int i = 0; i < 2; i++) {
        dma_channel_config c = dma_channel_get_default_config(dma_ch[i]);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_ring(&c, true, 13);          /* 8192-byte ring on the write side */
        channel_config_set_dreq(&c, pio_get_dreq(pio, sm[i], false));
        dma_channel_configure(dma_ch[i], &c, rings[i], &pio->rxf[sm[i]], RING_WORDS, false);
        dma_channel_set_irq0_enabled(dma_ch[i], true);
        rd_idx[i] = 0;
    }
    CRUMB(0x106);
    dma_channel_start(dma_ch[0]);
    dma_channel_start(dma_ch[1]);
    CRUMB(0x107);
    /* both state machines start together and lock onto the same LRCK pulse -> frame aligned */
    pio_enable_sm_mask_in_sync(pio, (1u << sm[0]) | (1u << sm[1]));
}

static inline uint wr_index(int i) {
    uint32_t base = (uint32_t) (i == 0 ? ring0 : ring1);
    return (uint) ((dma_hw->ch[dma_ch[i]].write_addr - base) / 4) & (RING_WORDS - 1);
}

static inline uint available_frames(int i) {
    uint avail = (wr_index(i) - rd_idx[i]) & (RING_WORDS - 1);
    return avail / 4;
}

static inline void put24(uint8_t *p, uint32_t w) {
    /* 32-bit slot, MSB first: 24 valid bits in the top -> little-endian 3-byte sample */
    p[0] = (uint8_t) (w >> 8);
    p[1] = (uint8_t) (w >> 16);
    p[2] = (uint8_t) (w >> 24);
}

/* called from the main loop: move complete 1 ms blocks from the rings to the USB FIFO */
static void audio_task(void) {
    if (safe_mode) return;
    while (available_frames(0) >= FRAMES_PER_PKT && available_frames(1) >= FRAMES_PER_PKT) {
        uint8_t *p = usb_pkt;
        for (int f = 0; f < FRAMES_PER_PKT; f++) {
            const uint32_t *a = &ring0[rd_idx[0]];
            const uint32_t *b = &ring1[rd_idx[1]];
            put24(p + 0, a[0]);   /* ch1 = ADC1L = AIN1 */
            put24(p + 3, a[1]);   /* ch2 = ADC1R = AIN2 */
            put24(p + 6, a[3]);   /* ch3 = ADC2R = VIN3 = AIN3 */
            put24(p + 9, a[2]);   /* ch4 = ADC2L = VIN4 = AIN4 */
            put24(p + 12, b[0]);  /* ch5 = U4 ADC1L = AIN5 */
            put24(p + 15, b[1]);  /* ch6 = U4 ADC1R = AIN6 */
            p += 18;
            rd_idx[0] = (rd_idx[0] + 4) & (RING_WORDS - 1);
            rd_idx[1] = (rd_idx[1] + 4) & (RING_WORDS - 1);
        }
        if (streaming) {
            CRUMB(0x300);
            tud_audio_write(usb_pkt, sizeof(usb_pkt));
            CRUMB(0x301);
        }
    }
    /* if the reader fell far behind (USB stalled), drop old data */
    for (int i = 0; i < 2; i++) {
        if (available_frames(i) > (RING_WORDS / 4) - 64) rd_idx[i] = wr_index(i) & ~3u;
    }
}

/* ------------------------------------------------------------------ USB serial debug port */
/* Everything printed goes to UART1 (test points) AND to the USB virtual COM port. The last 4 KB are kept so the
 * boot log can be replayed with the 'l' command after the port is opened on the PC. */
#define LOG_SZ 4096
static char log_ring[LOG_SZ];
static uint32_t log_head = 0;      /* total chars ever written */

static void cdc_out_chars(const char *buf, int len) {
    for (int i = 0; i < len; i++) log_ring[(log_head++) % LOG_SZ] = buf[i];
    if (tud_cdc_connected()) {
        tud_cdc_write(buf, (uint32_t) len);
        tud_cdc_write_flush();
    }
}

static stdio_driver_t cdc_stdio = {
    .out_chars = cdc_out_chars,
    .out_flush = NULL,
    .in_chars = NULL,
    .crlf_enabled = true,
};

void pcm1865_idle_hook(void);
static void log_replay(void) {
    uint32_t n = log_head < LOG_SZ ? log_head : LOG_SZ;
    uint32_t start = log_head - n;
    tud_cdc_write_str("---- boot log ----\r\n");
    for (uint32_t i = 0; i < n; i++) {
        char c = log_ring[(start + i) % LOG_SZ];
        tud_cdc_write_char(c);
        if ((i & 63) == 63) {
            tud_cdc_write_flush();
            pcm1865_idle_hook();      /* v2.6: 4 KB replay took ~15-20 ms -> keep USB audio flowing */
        }
    }
    tud_cdc_write_str("---- end ----\r\n");
    tud_cdc_write_flush();
}

static bool init_adcs(void);
static void audio_task(void);
/* v2.4: keep USB audio flowing during slow work (I2C, debug commands): called from pcm1865 I2C helpers and wait loops */
static bool in_service = false;
void pcm1865_idle_hook(void) {
    if (!streaming || in_service) return;
    in_service = true;
    tud_task();
    audio_task();
    watchdog_update();
    in_service = false;
}
static void print_panic_record(void);
static int8_t pga_db_x2 = 0;

static void handle_command(char *cmd) {
    while (*cmd == ' ') cmd++;
    switch (cmd[0]) {
        case 'h': case '?':
            printf("commands: s=status  l=boot log  r=re-init ADCs  g <dB>=PGA gain -12..40 (both ADCs)  v=version  h=help\n");
            break;
        case 'v':
            printf("AudioAnalyzer_PC fw v%s built %s %s\n", FW_VERSION, __DATE__, __TIME__);
            break;
        case 's': {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            printf("uptime %lus  adc_ok=%d  streaming=%d  usb_fifo=%u bytes (flow control keeps ~half of 14112)  pga=%+.1f dB\n", (unsigned long) (now / 1000), adc_ok,
                   streaming, tu_fifo_count(tud_audio_get_ep_in_ff()), pga_db_x2 / 2.0);
            printf("safe_mode=%d  crash record: resets=%lu crumb=0x%03lx pc=0x%08lx lr=0x%08lx (pc bit31 = stall)\n", safe_mode,
                   (unsigned long) watchdog_hw->scratch[1], (unsigned long) watchdog_hw->scratch[0],
                   (unsigned long) watchdog_hw->scratch[2], (unsigned long) watchdog_hw->scratch[3]);
            print_panic_record();
            pcm1865_dump_status(i2c1, PCM1865_ADDR_ADC1, "ADC1");
            pcm1865_dump_status(i2c1, PCM1865_ADDR_ADC2, "ADC2");
            uint edges;
            {   /* count frames written by the DMA in 20 ms (same number as LRCK periods), servicing USB meanwhile */
                uint32_t w0 = dma_hw->ch[dma_ch[0]].transfer_count;
                absolute_time_t t_end = make_timeout_time_ms(20);
                uint32_t moved = 0, prev = w0;
                while (!time_reached(t_end)) {
                    uint32_t c = dma_hw->ch[dma_ch[0]].transfer_count;
                    moved += (prev >= c) ? (prev - c) : (prev + RING_WORDS - c);
                    prev = c;
                    pcm1865_idle_hook();
                }
                edges = (moved / 4) * 2;
            }
            printf("LRCK edges in 20 ms: %u (expect ~1920)  frames in ring: %u / %u\n", edges, available_frames(0), available_frames(1));
            break;
        }
        case 'l':
            log_replay();
            break;
        case 'r':
            printf("re-initialising ADCs...\n");
            watchdog_update();
            adc_ok = init_adcs();
            capture_start();
            printf("ADC status: %s\n", adc_ok ? "OK" : "FAULT");
            break;
        case 'g': {
            int db = atoi(cmd + 1);
            if (db < -12) db = -12;
            if (db > 40) db = 40;
            pga_db_x2 = (int8_t) (db * 2);
            pcm1865_set_pga(i2c1, PCM1865_ADDR_ADC1, pga_db_x2);
            pcm1865_set_pga(i2c1, PCM1865_ADDR_ADC2, pga_db_x2);
            printf("PGA set to %+d dB on both ADCs (full scale is now %.1f Vrms; set the same in the app)\n", db, 66.0 / powf(10.0f, db / 20.0f));
            break;
        }
        case 0:
            break;
        default:
            printf("unknown command '%c' (h for help)\n", cmd[0]);
    }
}

static void cdc_task(void) {
    static char line[64];
    static int n = 0;
    if (!tud_cdc_available()) return;
    char c;
    while (tud_cdc_read(&c, 1) == 1) {
        if (c == '\r' || c == '\n') {
            line[n] = 0;
            if (n) handle_command(line);
            n = 0;
        } else if (n < (int) sizeof(line) - 1) {
            line[n++] = c;
        }
    }
}

/* ------------------------------------------------------------------ LED */
static void led_task(void) {
    static uint32_t last = 0;
    static bool on = false;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t period = safe_mode ? 250 : (!adc_ok ? 100 : (streaming ? 0 : 800));
    if (period == 0) { gpio_put(PIN_LED, 1); return; }
    if (now - last >= period) { last = now; on = !on; gpio_put(PIN_LED, on); }
}

static bool init_adcs(void) {
    bool p1 = pcm1865_probe(i2c1, PCM1865_ADDR_ADC1);
    bool p2 = pcm1865_probe(i2c1, PCM1865_ADDR_ADC2);
    printf("I2C probe: ADC1(0x4A)=%s ADC2(0x4B)=%s\n", p1 ? "ok" : "MISSING", p2 ? "ok" : "MISSING");

    bool ok1 = p1 && pcm1865_init_master_xtal(i2c1, PCM1865_ADDR_ADC1);
    bool ok2 = p2 && pcm1865_init_slave_bck(i2c1, PCM1865_ADDR_ADC2);
    if (pga_db_x2) {
        pcm1865_set_pga(i2c1, PCM1865_ADDR_ADC1, pga_db_x2);
        pcm1865_set_pga(i2c1, PCM1865_ADDR_ADC2, pga_db_x2);
    }
    pcm1865_dump_status(i2c1, PCM1865_ADDR_ADC1, "ADC1");
    pcm1865_dump_status(i2c1, PCM1865_ADDR_ADC2, "ADC2");

    /* LRCK activity check */
    uint edges = 0; bool last = gpio_get(PIN_LRCK);
    absolute_time_t t_end = make_timeout_time_ms(20);
    while (!time_reached(t_end)) { bool v = gpio_get(PIN_LRCK); if (v != last) { edges++; last = v; } }
    printf("LRCK edges in 20 ms: %u (expect ~1920)\n", edges);
    return ok1 && ok2 && edges > 100;
}

/* ------------------------------------------------------------------ panic recorder (survives the watchdog reset) */
#define PANIC_MAGIC 0x50414E43u
typedef struct { uint32_t magic; const char *fmt; uint32_t arg; uint32_t caller; uint32_t sp; uint32_t stack[12]; } panic_rec_t;
static panic_rec_t __uninitialized_ram(panic_rec);

void __attribute__((used, noreturn)) panic_record(const char *fmt, uint32_t arg, uint32_t *sp) {
    systick_hw->csr = 0;                          /* stop the stall recorder so it can't overwrite this */
    /* RAM did not survive the reset on this board, so put the essentials in the watchdog scratch registers */
    watchdog_hw->scratch[0] = 0xDEAD0000u | ((uint32_t) fmt & 0xFFFFu);   /* low half of the message pointer */
    watchdog_hw->scratch[2] = sp[0];              /* who called panic() */
    {
        uint32_t nxt = 0;
        for (int i = 1; i < 96; i++) { uint32_t v = sp[i]; if (v >= 0x10000000u && v < 0x10100000u && (v & 1u)) { nxt = v; break; } }
        watchdog_hw->scratch[3] = nxt;            /* next return address up the stack */
    }
    panic_rec.magic = PANIC_MAGIC;
    panic_rec.fmt = fmt;
    panic_rec.arg = arg;
    panic_rec.caller = sp[0];          /* lr pushed by the SDK's panic() forwarder = who called panic() */
    panic_rec.sp = (uint32_t) sp;
    int n = 0;                         /* collect further return addresses from the stack (call chain) */
    for (int i = 1; i < 96 && n < 12; i++) {
        uint32_t v = sp[i];
        if (v >= 0x10000000u && v < 0x10100000u && (v & 1u)) panic_rec.stack[n++] = v;
    }
    while (n < 12) panic_rec.stack[n++] = 0;
    while (1) { }                      /* watchdog resets us */
}
void __attribute__((naked, noreturn)) crash_panic(const char *fmt, ...) {
    __asm volatile("mov r2, sp\n"     /* sp -> [lr pushed by panic()] */
                   "b panic_record\n");
}

static void print_panic_record(void) {
    if (panic_rec.magic != PANIC_MAGIC) return;
    printf("!!! PANIC RECORD: \"%s\" arg=0x%08lx caller=0x%08lx sp=0x%08lx\n!!! stack:", panic_rec.fmt ? panic_rec.fmt : "(null)",
           (unsigned long) panic_rec.arg, (unsigned long) panic_rec.caller, (unsigned long) panic_rec.sp);
    for (int i = 0; i < 12; i++) printf(" %08lx", (unsigned long) panic_rec.stack[i]);
    printf("\n");
}

/* ------------------------------------------------------------------ stall recorder
 * SysTick (highest priority, ~134 ms) watches the main-loop counter; if it stops advancing for > 0.5 s, the code
 * location SysTick interrupted (= where the loop is stuck) goes into scratch[2]/[3] before the watchdog resets. */
static volatile uint32_t loop_count;
static uint32_t st_last_count, st_same;
void __attribute__((used)) systick_record(uint32_t *frame) {
    if (loop_count != st_last_count) { st_last_count = loop_count; st_same = 0; return; }
    if (++st_same >= 4) {
        watchdog_hw->scratch[2] = frame[6] | 0x80000000u;   /* bit31 marks "stall" (vs hard fault) */
        watchdog_hw->scratch[3] = frame[5];
    }
}
void __attribute__((naked)) isr_systick(void) {
    __asm volatile(
        "movs r0, #4\n"
        "mov r1, lr\n"
        "tst r0, r1\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "b systick_record\n"
        "1:\n"
        "mrs r0, msp\n"
        "b systick_record\n");
}
static void stall_recorder_start(void) {
    systick_hw->rvr = 0x00FFFFFF;
    systick_hw->cvr = 0;
    *(volatile uint32_t *) 0xE000ED20u &= 0x00FFFFFFu;   /* SHPR3: SysTick priority 0 (highest) */
    systick_hw->csr = 0x7;                    /* processor clock, interrupt, enable */
}

/* ------------------------------------------------------------------ hard fault recorder */
void __attribute__((used)) hardfault_record(uint32_t *frame) {
    watchdog_hw->scratch[2] = frame[6];   /* stacked PC */
    watchdog_hw->scratch[3] = frame[5];   /* stacked LR */
    while (1) { }                         /* watchdog resets us */
}
void __attribute__((naked)) isr_hardfault(void) {
    __asm volatile(
        "movs r0, #4\n"
        "mov r1, lr\n"
        "tst r0, r1\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "b hardfault_record\n"
        "1:\n"
        "mrs r0, msp\n"
        "b hardfault_record\n");
}

/* ------------------------------------------------------------------ main */
int main(void) {
    stdio_init_all();
    stdio_set_driver_enabled(&cdc_stdio, true);
    gpio_init(PIN_LED); gpio_set_dir(PIN_LED, GPIO_OUT); gpio_put(PIN_LED, 1);
    for (int p = PIN_BCK; p <= PIN_D1; p++) { gpio_init(p); gpio_set_dir(p, GPIO_IN); }
    /* the unused SCKI inputs of both ADCs are held low so they never float (ADC1 clocks from its crystal,
     * ADC2 from the BCK PLL) */
    gpio_set_dir(PIN_SCK1, GPIO_OUT); gpio_put(PIN_SCK1, 0);
    gpio_set_dir(PIN_SCK2, GPIO_OUT); gpio_put(PIN_SCK2, 0);

    sleep_ms(50);
    printf("\nAudioAnalyzer_PC fw v%s built %s %s\n", FW_VERSION, __DATE__, __TIME__);

    i2c_init(i2c1, 400 * 1000);   /* v2.4: 400 kHz (PCM1865 fast mode) -> shorter register access */
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    adc_ok = init_adcs();
    printf("ADC status: %s\n", adc_ok ? "OK" : "FAULT");

    /* PIO + DMA */
    pio_offset = pio_add_program(pio, &tdm_rx_program);
    sm[0] = pio_claim_unused_sm(pio, true);
    sm[1] = pio_claim_unused_sm(pio, true);
    tdm_rx_program_init(pio, sm[0], pio_offset, PIN_D0);
    tdm_rx_program_init(pio, sm[1], pio_offset, PIN_D1);
    dma_ch[0] = dma_claim_unused_channel(true);
    dma_ch[1] = dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    capture_start();

    sampleFreqRng.wNumSubRanges = 1;
    sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bRes = 0;

    tud_init(BOARD_TUD_RHPORT);
    printf("USB started\n");

    if (watchdog_enable_caused_reboot()) {
        watchdog_hw->scratch[1]++;
        safe_mode = true;
        printf("!!! CRASH RECORD: previous run was reset by the watchdog (resets so far: %lu)\n", (unsigned long) watchdog_hw->scratch[1]);
        printf("!!! last breadcrumb = 0x%03lx   hardfault pc = 0x%08lx lr = 0x%08lx\n", (unsigned long) watchdog_hw->scratch[0],
               (unsigned long) watchdog_hw->scratch[2], (unsigned long) watchdog_hw->scratch[3]);
        print_panic_record();
        printf("!!! SAFE MODE: audio streaming disabled until the board is unplugged and plugged back in\n");
    } else {
        for (int i = 0; i < 4; i++) watchdog_hw->scratch[i] = 0;
        panic_rec.magic = 0;
    }
    stall_recorder_start();
    watchdog_enable(1500, true);      /* never wedge the PC's USB/audio stack: reset instead */

    uint32_t last_stat = 0;
    while (true) {
        watchdog_update();
        loop_count++;
        CRUMB(0x200);
        tud_task();
        CRUMB(0x201);
        if (!streaming && !restart_req) stdio_set_driver_enabled(&stdio_uart, true);
        if (itf_event >= 0) {
            printf("host set streaming alt %d\n", itf_event);
            itf_event = -1;
        }
        if (restart_req && !safe_mode) {
            restart_req = false;
            stdio_set_driver_enabled(&stdio_uart, false);   /* UART printf blocks ~0.1 ms/char: CDC only while streaming */
            capture_start();          /* restart capture frame-aligned, empty rings */
            CRUMB(0x202);
            tud_audio_clear_ep_in_ff();
            CRUMB(0x203);
            streaming = true;
        }
        restart_req = false;
        CRUMB(0x210);
        audio_task();
        CRUMB(0x220);
        cdc_task();
        CRUMB(0x230);
        led_task();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!streaming && now - last_stat > 5000) {
            last_stat = now;
            printf("[%lus] streaming=%d fifo=%u\n", (unsigned long) (now / 1000), streaming,
                   tu_fifo_count(tud_audio_get_ep_in_ff()));
        }
    }
}

/* ------------------------------------------------------------------ TinyUSB callbacks */
void tud_mount_cb(void) { printf("USB mounted\n"); }
void tud_umount_cb(void) { streaming = false; }
void tud_suspend_cb(bool remote_wakeup_en) { (void) remote_wakeup_en; streaming = false; }
void tud_resume_cb(void) {}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport;
    uint8_t alt = TU_U16_LOW(p_request->wValue);
    itf_event = (int8_t) alt;
    if (alt == 1) {
        streaming = false;            /* main loop restarts capture, clears the FIFO, then sets streaming */
        restart_req = true;
    } else {
        streaming = false;
    }
    return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport; (void) p_request;
    streaming = false;
    /* v1.9 FIX: with ISO endpoint alloc/activate (pico-sdk 2.1.1 TinyUSB) closing the stream does NOT disarm the ISO IN
     * buffer. Windows switches alt 1 -> 0 -> 1 while probing the microphone; on the second activation TinyUSB arms the
     * buffer again and the RP2040 driver panics ("ep 81 was already available") -> watchdog reset loop. Disarm it here. */
    usb_dpram->ep_buf_ctrl[EPNUM_AUDIO_IN_NUM].in = 0;
    return true;
}

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void) rhport; (void) p_request; (void) pBuff; return false;
}
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void) rhport; (void) p_request; (void) pBuff; return false;
}
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void) rhport; (void) pBuff;
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    if (entityID == 4 && ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) return true;   /* fixed 48 kHz, accept silently */
    return false;
}
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport; (void) p_request; return false;
}
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport; (void) p_request; return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    if (entityID == 1) {   /* input terminal */
        if (ctrlSel == AUDIO_TE_CTRL_CONNECTOR) {
            audio_desc_channel_cluster_t ret;
            ret.bNrChannels = AUDIO_N_CHANNELS;
            ret.bmChannelConfig = (audio_channel_config_t) 0;
            ret.iChannelNames = 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void *) &ret, sizeof(ret));
        }
        return false;
    }
    if (entityID == 4) {   /* clock source */
        if (ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            if (p_request->bRequest == AUDIO_CS_REQ_CUR)
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));
            if (p_request->bRequest == AUDIO_CS_REQ_RANGE)
                return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));
            return false;
        }
        if (ctrlSel == AUDIO_CS_CTRL_CLK_VALID)
            return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));
        return false;
    }
    return false;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting) {
    (void) rhport; (void) itf; (void) ep_in; (void) cur_alt_setting;
    return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting) {
    (void) rhport; (void) n_bytes_copied; (void) itf; (void) ep_in; (void) cur_alt_setting;
    return true;
}
