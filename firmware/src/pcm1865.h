#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* PCM1865 register map (page 0) */
#define PCM_REG_PAGE            0x00
#define PCM_REG_PGA_CH1L        0x01
#define PCM_REG_PGA_CH1R        0x02
#define PCM_REG_PGA_CH2L        0x03
#define PCM_REG_PGA_CH2R        0x04
#define PCM_REG_PGA_CTRL        0x05
#define PCM_REG_ADC1L_SEL       0x06
#define PCM_REG_ADC1R_SEL       0x07
#define PCM_REG_ADC2L_SEL       0x08
#define PCM_REG_ADC2R_SEL       0x09
#define PCM_REG_AUXADC_SEL      0x0A
#define PCM_REG_PCM_CFG         0x0B
#define PCM_REG_TDM_TX_SEL      0x0C
#define PCM_REG_TDM_TX_OFFSET   0x0D
#define PCM_REG_TDM_RX_OFFSET   0x0E
#define PCM_REG_GPIO10_CTRL     0x10
#define PCM_REG_GPIO32_CTRL     0x11
#define PCM_REG_GPIO10_DIR      0x12
#define PCM_REG_GPIO32_DIR      0x13
#define PCM_REG_CLK_CTRL        0x20
#define PCM_REG_DSP1_CLK_DIV    0x21
#define PCM_REG_DSP2_CLK_DIV    0x22
#define PCM_REG_ADC_CLK_DIV     0x23
#define PCM_REG_PLL_SCK_DIV     0x25
#define PCM_REG_BCK_DIV         0x26
#define PCM_REG_LRCK_DIV        0x27
#define PCM_REG_PLL_CTRL        0x28
#define PCM_REG_PLL_P           0x29
#define PCM_REG_PLL_R           0x2A
#define PCM_REG_PLL_J           0x2B
#define PCM_REG_PLL_D_LSB       0x2C
#define PCM_REG_PLL_D_MSB       0x2D
#define PCM_REG_POWER_CTRL      0x70
#define PCM_REG_FILTER_MUTE     0x71
#define PCM_REG_DEVICE_STATUS   0x72
#define PCM_REG_FS_STATUS       0x73
#define PCM_REG_DIV_STATUS      0x74
#define PCM_REG_CLK_STATUS      0x75
#define PCM_REG_SUPPLY_STATUS   0x78

/* CLK_CTRL bits */
#define PCM_CLK_SCK_XI_SEL1     (1u << 7)
#define PCM_CLK_SCK_XI_SEL0     (1u << 6)
#define PCM_CLK_SCK_SRC_PLL     (1u << 5)
#define PCM_CLK_MST_MODE        (1u << 4)
#define PCM_CLK_ADC_SRC_PLL     (1u << 3)
#define PCM_CLK_DSP2_SRC_PLL    (1u << 2)
#define PCM_CLK_DSP1_SRC_PLL    (1u << 1)
#define PCM_CLK_CLKDET_EN       (1u << 0)

/* PLL_CTRL bits */
#define PCM_PLL_LOCK            (1u << 4)
#define PCM_PLL_REF_BCK         (1u << 1)   /* 0 = SCK, 1 = BCK */
#define PCM_PLL_EN              (1u << 0)

/* CLK_STATUS bits (0x75) */
#define PCM_CLKST_LRCKHLT       (1u << 6)
#define PCM_CLKST_BCKHLT        (1u << 5)
#define PCM_CLKST_SCKHLT        (1u << 4)
#define PCM_CLKST_LRCKERR       (1u << 2)
#define PCM_CLKST_BCKERR        (1u << 1)
#define PCM_CLKST_SCKERR        (1u << 0)

#define PCM1865_ADDR_ADC1       0x4A    /* MS/AD = GND  (U3, clock master, crystal on XI/XO) */
#define PCM1865_ADDR_ADC2       0x4B    /* MS/AD = 3V3  (U4, slave, PLL locked to BCK) */

bool pcm_write(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val);
bool pcm_read(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *val);
bool pcm1865_probe(i2c_inst_t *i2c, uint8_t addr);

/* Configure U3 as TDM master (4 ch, 32-bit slots, BCK=256fs, LRCK pulse) from its 24.576 MHz crystal.
 * Returns true if the device reports a running clock. */
bool pcm1865_init_master_xtal(i2c_inst_t *i2c, uint8_t addr);
/* Configure U4 as TDM slave, PLL referenced to BCK (no SCK). Returns true if PLL locked. */
bool pcm1865_init_slave_bck(i2c_inst_t *i2c, uint8_t addr);
/* gain_db_x2: -24 (= -12 dB) .. +80 (= +40 dB) in 0.5 dB steps, applied to all four channels */
void pcm1865_set_pga(i2c_inst_t *i2c, uint8_t addr, int8_t gain_db_x2);
void pcm1865_dump_status(i2c_inst_t *i2c, uint8_t addr, const char *name);
