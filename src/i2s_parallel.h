#ifndef I2S_PARALLEL_H
#define I2S_PARALLEL_H

#include <stdint.h>

#include <esp_err.h>
#include <soc/i2s_struct.h>
#include <rom/lldesc.h>

typedef enum {
    I2S_PARALLEL_BITS_8  =  8, // BUG: Doesn't work.
    I2S_PARALLEL_BITS_16 = 16,
    I2S_PARALLEL_BITS_32 = 32,
} i2s_parallel_cfg_bits_t;

typedef struct {
    void *memory;
    size_t size;
} i2s_parallel_buffer_desc_t;

typedef struct {
    int gpio_bus[24];
    int gpio_clk;
    int clkspeed_hz;
    i2s_parallel_cfg_bits_t bits;
    int desccount_a;
    int desccount_b;
    lldesc_t *lldesc_a;
    lldesc_t *lldesc_b;
} i2s_parallel_config_t;

esp_err_t i2s_parallel_setup(i2s_dev_t *dev, const i2s_parallel_config_t *cfg);
void i2s_parallel_flip_to_buffer(i2s_dev_t *dev, int bufid);
void i2s_parallel_link_dma_desc(volatile lldesc_t *dmadesc, volatile lldesc_t *prevdmadesc, void *memory, size_t size);
void i2s_parallel_stop(i2s_dev_t *dev);

typedef int (*i2s_parallel_callback_t)(void);
void i2s_parallel_set_shiftcomplete_cb(i2s_parallel_callback_t f);


#endif
