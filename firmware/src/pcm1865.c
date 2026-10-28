#include "hardware/watchdog.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pcm1865.h"

extern void pcm1865_idle_hook(void);   /* main.c: services USB audio while streaming */

bool pcm_write(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    bool ok = i2c_write_timeout_us(i2c, addr, buf, 2, false, 20000) == 2;
    pcm1865_idle_hook();
    return ok;
}

bool pcm_read(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *val) {
    if (i2c_write_timeout_us(i2c, addr, &reg, 1, true, 20000) != 1) return false;
    bool ok = i2c_read_timeout_us(i2c, addr, val, 1, false, 20000) == 1;
    pcm1865_idle_hook();
    return ok;
}

bool pcm1865_probe(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t v;
    return pcm_read(i2c, addr, PCM_REG_PAGE, &v);
}

static void pcm_reset(i2c_inst_t *i2c, uint8_t addr) {
    pcm_write(i2c, addr, PCM_REG_PAGE, 0xFE);     /* software reset */
    sleep_ms(20);
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
}

/* common part: inputs, PGA, TDM 4ch 32-bit slots, offset 1 BCK */
static void pcm_common(i2c_inst_t *i2c, uint8_t addr) {
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
    /* differential inputs: ADC1L=VIN1P/1M, ADC1R=VIN2P/2M, ADC2L=VIN4P/4M, ADC2R=VIN3P/3M */
    pcm_write(i2c, addr, PCM_REG_ADC1L_SEL, 0x10);
    pcm_write(i2c, addr, PCM_REG_ADC1R_SEL, 0x10);
    pcm_write(i2c, addr, PCM_REG_ADC2L_SEL, 0x20);
    pcm_write(i2c, addr, PCM_REG_ADC2R_SEL, 0x20);
    /* PGA 0 dB (external 47k/4.7k dividers set full scale ~66 Vrms) */
    pcm_write(i2c, addr, PCM_REG_PGA_CH1L, 0x00);
    pcm_write(i2c, addr, PCM_REG_PGA_CH1R, 0x00);
    pcm_write(i2c, addr, PCM_REG_PGA_CH2L, 0x00);
    pcm_write(i2c, addr, PCM_REG_PGA_CH2R, 0x00);
    /* RX 32-bit, TDM LRCK 1/256 duty, TX 32-bit, TDM/DSP format */
    pcm_write(i2c, addr, PCM_REG_PCM_CFG, (0u << 6) | (1u << 4) | (0u << 2) | 3u);
    pcm_write(i2c, addr, PCM_REG_TDM_TX_SEL, 0x01);      /* 4ch TDM: ch1L, ch1R, ch2L, ch2R */
    pcm_write(i2c, addr, PCM_REG_TDM_TX_OFFSET, 0x01);   /* MSB one BCK after LRCK edge (I2S-like) */
    pcm_write(i2c, addr, PCM_REG_POWER_CTRL, 0x70);      /* run */
    /* mic bias output (pin 5, unconnected on this board) off: page 3 reg 0x15 PDZ = 0 */
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x03);
    pcm_write(i2c, addr, 0x15, 0x00);
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
}

static bool wait_run(i2c_inst_t *i2c, uint8_t addr, int ms) {
    uint8_t st = 0;
    for (int i = 0; i < ms / 10; i++) {
        sleep_ms(10);
        watchdog_update();
        if (pcm_read(i2c, addr, PCM_REG_DEVICE_STATUS, &st) && (st & 0x0F) == 0x0F) return true;
    }
    return false;
}

static void pcm_restart(i2c_inst_t *i2c, uint8_t addr) {
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
    pcm_write(i2c, addr, PCM_REG_POWER_CTRL, 0x71);   /* STBY = 1 */
    sleep_ms(5);
    pcm_write(i2c, addr, PCM_REG_POWER_CTRL, 0x70);   /* STBY = 0 */
}

static void pcm_dump_clk(i2c_inst_t *i2c, uint8_t addr, const char *tag) {
    uint8_t r[16] = {0}, st = 0, pw = 0;
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
    for (int i = 0; i < 16; i++) pcm_read(i2c, addr, (uint8_t) (0x20 + i), &r[i]);
    pcm_read(i2c, addr, PCM_REG_DEVICE_STATUS, &st);
    pcm_read(i2c, addr, PCM_REG_POWER_CTRL, &pw);
    printf("  %s 0x%02X: status=0x%02X pwr=0x%02X  r20..2F:", tag, addr, st, pw);
    for (int i = 0; i < 16; i++) printf(" %02X", r[i]);
    printf("\n");
}

/* ADC1 is the clock master (24.576 MHz crystal = 512 fs). v2.2 showed it RUNs as master in I2S but stays in STAND-BY
 * in TDM with the 1/256-duty LRCK pulse. v2.3 tries TDM with a 50 % duty LRCK (DSP long frame sync) instead; the
 * RP2040 PIO only needs the LRCK rising edge, so it captures either. ADC2 follows the chosen LRCK mode. */
uint8_t pcm1865_lrck_mode = 1u << 4;      /* PCM_CFG bit 4: 1 = 1/256 duty, 0 = 50 % duty */

bool pcm1865_init_master_xtal(i2c_inst_t *i2c, uint8_t addr) {
    struct { const char *name; uint8_t lrck_mode; uint8_t bck_div; uint8_t lrck_div; } cfg[] = {
        {"TDM, LRCK 50%, BCK 256fs", 0x00, 1, 255},
        {"TDM, LRCK 50%, BCK 128fs", 0x00, 3, 127},
        {"TDM, LRCK 1/256, BCK 256fs (old)", 0x10, 1, 255},
    };
    for (unsigned k = 0; k < sizeof(cfg) / sizeof(cfg[0]); k++) {
        pcm_reset(i2c, addr);
        pcm_write(i2c, addr, PCM_REG_PAGE, 0x03);
        pcm_write(i2c, addr, 0x12, 0x40);            /* crystal oscillator on */
        pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
        pcm_common(i2c, addr);
        pcm_write(i2c, addr, PCM_REG_POWER_CTRL, 0x71);   /* standby while configuring */
        pcm_write(i2c, addr, PCM_REG_PCM_CFG, (uint8_t) (cfg[k].lrck_mode | 0x03));   /* 32-bit, TDM */
        pcm_write(i2c, addr, PCM_REG_BCK_DIV, cfg[k].bck_div);
        pcm_write(i2c, addr, PCM_REG_LRCK_DIV, cfg[k].lrck_div);
        pcm_write(i2c, addr, PCM_REG_CLK_CTRL, PCM_CLK_SCK_XI_SEL1 | PCM_CLK_MST_MODE | PCM_CLK_CLKDET_EN); /* 0x91 */
        sleep_ms(10);
        pcm_write(i2c, addr, PCM_REG_POWER_CTRL, 0x70);   /* run */
        bool ok = wait_run(i2c, addr, 400);
        printf("ADC1 master %s: %s\n", cfg[k].name, ok ? "RUN" : "no");
        pcm_dump_clk(i2c, addr, "ADC1");
        if (ok) { pcm1865_lrck_mode = cfg[k].lrck_mode; return true; }
    }
    printf("ADC1: no TDM config reached RUN (left on the last one)\n");
    return false;
}

bool pcm1865_init_slave_bck(i2c_inst_t *i2c, uint8_t addr) {
    pcm_reset(i2c, addr);
    pcm_common(i2c, addr);
    pcm_write(i2c, addr, PCM_REG_PCM_CFG, (uint8_t) (pcm1865_lrck_mode | 0x03));   /* follow ADC1's LRCK mode */
    /* slave, no SCK: auto clock detector configures the PLL from BCK (256 fs supported, Table 11) */
    pcm_write(i2c, addr, PCM_REG_CLK_CTRL, PCM_CLK_CLKDET_EN);                 /* 0x01 */
    bool ok = wait_run(i2c, addr, 800);
    pcm_dump_clk(i2c, addr, "ADC2(auto)");
    if (!ok) {
        /* manual fallback: PLL ref = BCK, all clocks from PLL */
        printf("ADC2: auto PLL did not run, trying manual PLL config\n");
        pcm_write(i2c, addr, PCM_REG_CLK_CTRL, 0x00);
        pcm_write(i2c, addr, PCM_REG_PLL_CTRL, 0x00);
        pcm_write(i2c, addr, PCM_REG_PLL_P, 0);      /* P=1 */
        pcm_write(i2c, addr, PCM_REG_PLL_R, 0);      /* R=1 */
        pcm_write(i2c, addr, PCM_REG_PLL_J, 8);      /* 12.288 MHz * 8 = 98.304 MHz */
        pcm_write(i2c, addr, PCM_REG_PLL_D_LSB, 0);
        pcm_write(i2c, addr, PCM_REG_PLL_D_MSB, 0);
        pcm_write(i2c, addr, PCM_REG_DSP1_CLK_DIV, 1);  /* 98.304/2 = 49.152 (1024 fs) */
        pcm_write(i2c, addr, PCM_REG_DSP2_CLK_DIV, 3);  /* 98.304/4 = 24.576 (512 fs) */
        pcm_write(i2c, addr, PCM_REG_ADC_CLK_DIV, 7);   /* 98.304/8 = 12.288 (256 fs) */
        pcm_write(i2c, addr, PCM_REG_PLL_CTRL, PCM_PLL_REF_BCK | PCM_PLL_EN);
        pcm_write(i2c, addr, PCM_REG_CLK_CTRL, PCM_CLK_SCK_SRC_PLL | PCM_CLK_ADC_SRC_PLL | PCM_CLK_DSP2_SRC_PLL | PCM_CLK_DSP1_SRC_PLL);
        ok = wait_run(i2c, addr, 800);
    }
    return ok;
}

void pcm1865_set_pga(i2c_inst_t *i2c, uint8_t addr, int8_t g) {
    if (g < -24) g = -24;
    if (g > 80) g = 80;
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
    for (uint8_t r = PCM_REG_PGA_CH1L; r <= PCM_REG_PGA_CH2R; r++) pcm_write(i2c, addr, r, (uint8_t) g);
}

void pcm1865_dump_status(i2c_inst_t *i2c, uint8_t addr, const char *name) {
    uint8_t st = 0, fs = 0, dv = 0, ck = 0, pll = 0, sup = 0, clk = 0;
    pcm_write(i2c, addr, PCM_REG_PAGE, 0x00);
    pcm_read(i2c, addr, PCM_REG_DEVICE_STATUS, &st);
    pcm_read(i2c, addr, PCM_REG_FS_STATUS, &fs);
    pcm_read(i2c, addr, PCM_REG_DIV_STATUS, &dv);
    pcm_read(i2c, addr, PCM_REG_CLK_STATUS, &ck);
    pcm_read(i2c, addr, PCM_REG_PLL_CTRL, &pll);
    pcm_read(i2c, addr, PCM_REG_SUPPLY_STATUS, &sup);
    pcm_read(i2c, addr, PCM_REG_CLK_CTRL, &clk);
    printf("%s @0x%02X: state=%s(0x%X) fs=0x%02X bckratio=0x%02X clkstat=0x%02X pll=0x%02X(%s) supply=0x%02X clkctrl=0x%02X\n",
           name, addr, (st & 0xF) == 0xF ? "RUN" : "not-run", st & 0xF, fs & 7, dv, ck, pll,
           (pll & PCM_PLL_LOCK) ? "locked" : "unlocked", sup, clk);
}
