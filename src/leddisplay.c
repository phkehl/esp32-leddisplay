/*!
    \file
    \brief HUB75 LED display driver using I2S parallel mode

    Based on:
    - https://esp32.com/viewtopic.php?t=3188
    - https://github.com/ESP32DE/I2S_parallel_example_drive_a_64x32_display
    - https://github.com/pixelmatix/esp32_I2sParallelDmaLedMatrix
    - https://github.com/pixelmatix/SmartMatrix/tree/teensylc
    - https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA

    Copyright:

    - Copyright 2017 Espressif Systems (Shanghai) PTE LTD
    - Copyright 2018 Louis Beaudoin (Pixelmatix)
    - Copyright 2019 Philippe Kehl (flipflip at oinkzwurgl dot org)

    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
    in compliance with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software distributed under the
    License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
    express or implied.  See the License for the specific language governing permissions and
    limitations under the License.

    The following text is from the original driver example from the esp forum (see above):

      This is example code to driver a p3(2121)64*32 -style RGB LED display. These types of displays
      do not have memory and need to be refreshed continuously. The display has 2 RGB inputs, 4
      inputs to select the active line, a pixel clock input, a latch enable input and an
      output-enable input. The display can be seen as 2 64x16 displays consisting of the upper half
      and the lower half of the display. Each half has a separate RGB pixel input, the rest of the
      inputs are shared.

      Each display half can only show one line of RGB pixels at a time: to do this, the RGB data for
      the line is input by setting the RGB input pins to the desired value for the first pixel,
      giving the display a clock pulse, setting the RGB input pins to the desired value for the
      second pixel, giving a clock pulse, etc. Do this 64 times to clock in an entire row. The
      pixels will not be displayed yet: until the latch input is made high, the display will still
      send out the previously clocked in line. Pulsing the latch input high will replace the
      displayed data with the data just clocked in.

      The 4 line select inputs select where the currently active line is displayed: when provided
      with a binary number (0-15), the latched pixel data will immediately appear on this
      line. Note: While clocking in data for a line, the *previous* line is still displayed, and
      these lines should be set to the value to reflect the position the *previous* line is supposed
      to be on.

      Finally, the screen has an OE input, which is used to disable the LEDs when latching new data
      and changing the state of the line select inputs: doing so hides any artifacts that appear at
      this time. The OE line is also used to dim the display by only turning it on for a limited
      time every line.

      All in all, an image can be displayed by 'scanning' the display, say, 100 times per
      second. The slowness of the human eye hides the fact that only one line is showed at a time,
      and the display looks like every pixel is driven at the same time.

      Now, the RGB inputs for these types of displays are digital, meaning each red, green and blue
      subpixel can only be on or off. This leads to a color palette of 8 pixels, not enough to
      display nice pictures. To get around this, we use binary code modulation.

      Binary code modulation is somewhat like PWM, but easier to implement in our case. First, we
      define the time we would refresh the display without binary code modulation as the 'frame
      time'. For, say, a four-bit binary code modulation, the frame time is divided into 15 ticks of
      equal length.

      We also define 4 subframes (0 to 3), defining which LEDs are on and which LEDs are off during
      that subframe. (Subframes are the same as a normal frame in non-binary-coded-modulation mode,
      but are showed faster.)  From our (non-monochrome) input image, we take the (8-bit: bit 7 to
      bit 0) RGB pixel values. If the pixel values have bit 7 set, we turn the corresponding LED on
      in subframe 3. If they have bit 6 set, we turn on the corresponding LED in subframe 2, if bit
      5 is set subframe 1, if bit 4 is set in subframe 0.

      Now, in order to (on average within a frame) turn a LED on for the time specified in the pixel
      value in the input data, we need to weigh the subframes. We have 15 pixels: if we show
      subframe 3 for 8 of them, subframe 2 for 4 of them, subframe 1 for 2 of them and subframe 1
      for 1 of them, this 'automatically' happens. (We also distribute the subframes evenly over the
      ticks, which reduces flicker.)

      In this code, we use the I2S peripheral in parallel mode to achieve this. Essentially, first
      we allocate memory for all subframes. This memory contains a sequence of all the signals
      (2xRGB, line select, latch enable, output enable) that need to be sent to the display for that
      subframe.  Then we ask the I2S-parallel driver to set up a DMA chain so the subframes are sent
      out in a sequence that satisfies the requirement that subframe x has to be sent out for (2^x)
      ticks. Finally, we fill the subframes with image data.

      We use a frontbuffer/backbuffer technique here to make sure the display is refreshed in one go
      and drawing artifacts do not reach the display.  In practice, for small displays this is not
      really necessarily.

      Finally, the binary code modulated intensity of a LED does not correspond to the intensity as
      seen by human eyes. To correct for that, a luminance correction is used. See val2pwm.c for
      more info.

      Note: Because every subframe contains one bit of grayscale information, they are also referred
      to as 'bitplanes' by the code below.

    Note that the text above may not be fully true anymore for this implementation.

    \todo study SmartMatrix changes (https://github.com/pixelmatix/SmartMatrix/tree/teensylc),
      specifically https://github.com/pixelmatix/SmartMatrix/commit/e3f6784304da95bfe1fc63a68b4acf05546c0e78
    \todo https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA

*/

/* *********************************************************************************************** */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_heap_caps.h>

#include "val2pwm.h"
#include "i2s_parallel.h"

#include "leddisplay.h"

/* *********************************************************************************************** */
// local logging and debugging

#define LOGNAME "leddisplay"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)    ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)

/* *********************************************************************************************** */
// some useful macros

#ifdef BIT
#  undef BIT
#endif
#define BIT(bit) (1<<(bit))
#define STRINGIFY(x) _STRINGIFY(x)
#define _STRINGIFY(x) #x
#define NUMOF(x) (sizeof(x)/sizeof(*(x)))

/* *********************************************************************************************** */
// I2S bus bits (corresponds to the GPIOs)

// display panel upper half
#define BIT_R1   BIT(0)   // CONFIG_LEDDISPLAY_R1_GPIO
#define BIT_G1   BIT(1)   // CONFIG_LEDDISPLAY_G1_GPIO
#define BIT_B1   BIT(2)   // CONFIG_LEDDISPLAY_B1_GPIO

// display panel lower half
#define BIT_R2   BIT(3)   // CONFIG_LEDDISPLAY_R2_GPIO
#define BIT_G2   BIT(4)   // CONFIG_LEDDISPLAY_G2_GPIO
#define BIT_B2   BIT(5)   // CONFIG_LEDDISPLAY_B2_GPIO

// display panel control signals (latch, output enable)
#define BIT_LAT  BIT(6)   // CONFIG_LEDDISPLAY_LAT_GPIO
#define BIT_OE   BIT(7)   // CONFIG_LEDDISPLAY_OE_GPIO

// row address
#define BIT_A    BIT(8)   // CONFIG_LEDDISPLAY_A_GPIO
#define BIT_B    BIT(9)   // CONFIG_LEDDISPLAY_B_GPIO
#define BIT_C    BIT(10)  // CONFIG_LEDDISPLAY_C_GPIO
#define BIT_D    BIT(11)  // CONFIG_LEDDISPLAY_D_GPIO
#define BIT_E    BIT(12)  // CONFIG_LEDDISPLAY_E_GPIO

// CONFIG_LEDDISPLAY_E_CLK

/* *********************************************************************************************** */
// configuration (see also leddisplay.h)

#if CONFIG_LEDDISPLAY_TYPE_32X16_4SCAN     // doesn't work
#  warning CONFIG_LEDDISPLAY_TYPE_32X16_4SCAN does not work
#  define LEDDISPLAY_ROWS_IN_PARALLEL      4

#elif CONFIG_LEDDISPLAY_TYPE_32X16_8SCAN   // tested, works
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_32X32_8SCAN   // doesn't work
#  warning CONFIG_LEDDISPLAY_TYPE_32X32_8SCAN does not work
#  define LEDDISPLAY_ROWS_IN_PARALLEL      4

#elif CONFIG_LEDDISPLAY_TYPE_32X32_16SCAN  // tested, works
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_64X32_8SCAN   // doesn't work
#  warning CONFIG_LEDDISPLAY_TYPE_64X32_8SCAN does not work
#  define LEDDISPLAY_ROWS_IN_PARALLEL      4

#elif CONFIG_LEDDISPLAY_TYPE_64X32_16SCAN  // tested, works
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_64X64_32SCAN  // not tested
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2
#  define LEDDISPLAY_NEED_E_GPIO 1
#  if CONFIG_LEDDISPLAY_E_GPIO < 0
#    error Need CONFIG_LEDDISPLAY_E_GPIO > 0!
#  endif
#else
#  error This CONFIG_LEDDISPLAY_TYPE is not implemented!
#endif

#if CONFIG_LEDDISPLAY_I2S_FREQ_13MHZ
#  define I2S_CLOCK_SPEED  13333334
#elif CONFIG_LEDDISPLAY_I2S_FREQ_16MHZ
#  define I2S_CLOCK_SPEED  16000000
#elif CONFIG_LEDDISPLAY_I2S_FREQ_20MHZ
#  define I2S_CLOCK_SPEED  20000000
#elif CONFIG_LEDDISPLAY_I2S_FREQ_26MHZ
#  define I2S_CLOCK_SPEED  26666667
#else
#  error This CONFIG_LEDDISPLAY_I2S_FREQ is not implemented!
#endif

#define NUM_FRAME_BUFFERS         2
//#define OE_OFF_CLKS_AFTER_LATCH   1
#define COLOR_DEPTH_BITS          8
#define PIXELS_PER_LATCH          ((LEDDISPLAY_WIDTH * LEDDISPLAY_HEIGHT) / LEDDISPLAY_HEIGHT)
#define ROWS_PER_FRAME            (LEDDISPLAY_HEIGHT / LEDDISPLAY_ROWS_IN_PARALLEL)

/* *********************************************************************************************** */

// RGB data for two rows of pixels, and address and control signals
typedef struct row_bit_s
{
    uint16_t pixel[LEDDISPLAY_WIDTH];
} row_bit_t;
// Note: sizeof(data) must be multiple of 32 bits, as DMA linked list buffer address pointer must be word-aligned

// row data for each bitplane
typedef struct row_data_s
{
    row_bit_t rowbits[COLOR_DEPTH_BITS];
} row_data_t;

// full frame (display)
typedef struct frame_s
{
    row_data_t rowdata[ROWS_PER_FRAME];
} frame_t;

/* *********************************************************************************************** */

// pixel data (bitplanes) is organized from LSB to MSB sequentially by row, from row 0 to row
// matrixHeight/matrixRowsInParallel (two rows of pixels are refreshed in parallel)
static frame_t *s_frames;

static uint32_t s_current_frame;
static int s_lsb_msb_transition_bit;

// DMA memory linked list descriptors
lldesc_t *s_dmadesc_a;
lldesc_t *s_dmadesc_b;

// brightness level (value for data calculation, and percent used in API)
static int s_brightness_val;
static int s_brightness_percent;

// flush complete semaphore
SemaphoreHandle_t s_shift_complete_sem;
static IRAM_ATTR int s_shift_complete_sem_cb(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_shift_complete_sem, &xHigherPriorityTaskWoken );
    return xHigherPriorityTaskWoken;
}

void leddisplay_pixel_update(int block)
{
    i2s_parallel_flip_to_buffer(&I2S1, s_current_frame);
    s_current_frame = (s_current_frame + 1) % NUM_FRAME_BUFFERS;

    // wait until buffer is no longer used (I2S will continue using buffer until it's done and only
    // then switch to the new one)
    if (block != 0)
    {
        xSemaphoreTake(s_shift_complete_sem, portMAX_DELAY);
    }
    // 100 / portTICK_PERIOD_MS
}

esp_err_t leddisplay_init(void)
{
    esp_err_t res = ESP_OK;

    INFO(STRINGIFY(LEDDISPLAY_WIDTH) "x" STRINGIFY(LEDDISPLAY_HEIGHT) " (" STRINGIFY(COLOR_DEPTH_BITS) "bits)");

    DEBUG("GPIOs:"
        " R1="  STRINGIFY(CONFIG_LEDDISPLAY_R1_GPIO)
        " G1="  STRINGIFY(CONFIG_LEDDISPLAY_G1_GPIO)
        " B1="  STRINGIFY(CONFIG_LEDDISPLAY_B1_GPIO)
        " R2="  STRINGIFY(CONFIG_LEDDISPLAY_R2_GPIO)
        " G2="  STRINGIFY(CONFIG_LEDDISPLAY_G2_GPIO)
        " B2="  STRINGIFY(CONFIG_LEDDISPLAY_B2_GPIO)
        " A="   STRINGIFY(CONFIG_LEDDISPLAY_A_GPIO)
        " B="   STRINGIFY(CONFIG_LEDDISPLAY_B_GPIO)
        " C="   STRINGIFY(CONFIG_LEDDISPLAY_C_GPIO)
        " D="   STRINGIFY(CONFIG_LEDDISPLAY_D_GPIO)
#if LEDDISPLAY_NEED_E_GPIO
        " E="   STRINGIFY(CONFIG_LEDDISPLAY_E_GPIO)
#endif
        " LAT=" STRINGIFY(CONFIG_LEDDISPLAY_LAT_GPIO)
        " OE="  STRINGIFY(CONFIG_LEDDISPLAY_OE_GPIO)
        " CLK=" STRINGIFY(CONFIG_LEDDISPLAY_CLK_GPIO));

    // set default brightness 75%
    leddisplay_set_brightness(75);

    // allocate memory for the frame buffers, initialise frame buffers
    if (res == ESP_OK)
    {
        DEBUG("frame buffers: size=%u (available total=%u, largest=%u)", NUM_FRAME_BUFFERS * sizeof(frame_t),
            heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        s_frames = (frame_t *)heap_caps_malloc(NUM_FRAME_BUFFERS * sizeof(frame_t), MALLOC_CAP_DMA);
        if (s_frames == NULL)
        {
            WARNING("framebuf alloc");
            res = ESP_ERR_NO_MEM;
        }
        // clear frame buffers
        else
        {
            const int old_brightness = leddisplay_set_brightness(0);

            s_current_frame = 1;
            leddisplay_pixel_fill_rgb(0, 0, 0);
            s_current_frame = 0;
            leddisplay_pixel_fill_rgb(0, 0, 0);

            leddisplay_set_brightness(old_brightness);
        }
    }

    // calculate the lowest LSBMSB_TRANSITION_BIT value that will fit in memory and achieves the minimal refresh rate
    int numDescriptorsPerRow = 0;
    int refreshRate = 0;
    bool ramOkay = false;
    bool refreshOkay = false;
    if (res == ESP_OK)
    {
        int largestBlockFree = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        int totalFree        = heap_caps_get_free_size(MALLOC_CAP_DMA);

        s_lsb_msb_transition_bit = 0;

        while (1)
        {
            ramOkay = false;
            refreshOkay = false;

            // calculate memory requirements for this value of s_lsb_msb_transition_bit
            numDescriptorsPerRow = 1;
            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                numDescriptorsPerRow += (1 << (i - s_lsb_msb_transition_bit - 1));
            }
            int ramRequired = numDescriptorsPerRow * ROWS_PER_FRAME * NUM_FRAME_BUFFERS * sizeof(lldesc_t);

            // calculate achievable refresh rate for this value of s_lsb_msb_transition_bit
            int psPerClock = 1000000000000UL / I2S_CLOCK_SPEED;
            int nsPerLatch = (PIXELS_PER_LATCH * psPerClock) / 1000;
            // add time to shift out LSBs + LSB-MSB transition bit - this ignores fractions...
            int nsPerRow = COLOR_DEPTH_BITS * nsPerLatch;
            // add time to shift out MSBs
            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                nsPerRow += (1 << (i - s_lsb_msb_transition_bit - 1)) * (COLOR_DEPTH_BITS - i) * nsPerLatch;
            }
            int nsPerFrame = nsPerRow * ROWS_PER_FRAME;
            refreshRate = 1000000000UL / nsPerFrame;

            // check if that satisfies our requirements
            if ( (ramRequired < largestBlockFree) && (ramRequired < (totalFree - CONFIG_LEDDISPLAY_PRESERVE_RAM_SIZE)) )
            {
                ramOkay = true;
            }
            if (refreshRate >= CONFIG_LEDDISPLAY_MIN_FRAME_RATE)
            {
                refreshOkay = true;
            }

            // log summary
            DEBUG("lsb_msb_transition_bit=%d: ramRequired=%u available=%u largest=%u %s, refreshRate=%d %s",
                s_lsb_msb_transition_bit, ramRequired, totalFree, largestBlockFree, ramOkay ? ":-)" : ":-(",
                refreshRate, refreshOkay ? ":-)" : ":-(");

            // stop if we're satisfied
            if (ramOkay && refreshOkay)
            {
                break;
            }
            // try again if we can do more
            if ( s_lsb_msb_transition_bit < (COLOR_DEPTH_BITS - 1) )
            {
                s_lsb_msb_transition_bit++;
            }
            // give up
            else
            {
                break;
            }
        }

        // are we happy?
        if (ramOkay && refreshOkay)
        {
            DEBUG("finally: lsb_msb_transition_bit=%d/%d, rows=%d, RAM=%d, refresh=%d", s_lsb_msb_transition_bit, COLOR_DEPTH_BITS - 1,
                ROWS_PER_FRAME, NUM_FRAME_BUFFERS * numDescriptorsPerRow * ROWS_PER_FRAME * sizeof(lldesc_t), refreshRate);
        }
        // give up if we could not meet the RAM and refresh rate requirements
        else
        {
            if (!ramOkay)
            {
                WARNING("desc alloc");
                res = ESP_ERR_NO_MEM;
            }
            if (!refreshOkay)
            {
                WARNING("refresh");
                res = ESP_FAIL;
            }
        }
    }

    // malloc the DMA linked list descriptors that i2s_parallel will need
    int desccount = numDescriptorsPerRow * ROWS_PER_FRAME;
    if (res == ESP_OK)
    {
        s_dmadesc_a = (lldesc_t *)heap_caps_malloc(desccount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (s_dmadesc_a == NULL)
        {
            WARNING("desc a alloc");
            res = ESP_ERR_NO_MEM;
        }
        s_dmadesc_b = (lldesc_t *)heap_caps_malloc(desccount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (s_dmadesc_b == NULL)
        {
            WARNING("desc b alloc");
            res = ESP_ERR_NO_MEM;
        }
    }

    //heap_caps_print_heap_info(MALLOC_CAP_DMA);

    // fill DMA linked lists for both frames
    if (res == ESP_OK)
    {
        lldesc_t *prevdmadesca = NULL;
        lldesc_t *prevdmadescb = NULL;
        int currentDescOffset = 0;
        for (int j = 0; j < ROWS_PER_FRAME; j++)
        {
            // first set of data is LSB through MSB, single pass - all color bits are displayed once, which takes care of everything below and inlcluding LSBMSB_TRANSITION_BIT
            // TODO: size must be less than DMA_MAX - worst case for SmartMatrix Library: 16-bpp with 256 pixels per row would exceed this, need to break into two
            i2s_parallel_link_dma_desc(&s_dmadesc_a[currentDescOffset], prevdmadesca, &(s_frames[0].rowdata[j].rowbits[0].pixel), sizeof(row_bit_t) * COLOR_DEPTH_BITS);
            prevdmadesca = &s_dmadesc_a[currentDescOffset];
            i2s_parallel_link_dma_desc(&s_dmadesc_b[currentDescOffset], prevdmadescb, &(s_frames[1].rowdata[j].rowbits[0].pixel), sizeof(row_bit_t) * COLOR_DEPTH_BITS);
            prevdmadescb = &s_dmadesc_b[currentDescOffset];
            currentDescOffset++;
            //DEBUG("row %d:", j);

            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                // binary time division setup: we need 2 of bit (LSBMSB_TRANSITION_BIT + 1) four of (LSBMSB_TRANSITION_BIT + 2), etc
                // because we sweep through to MSB each time, it divides the number of times we have to sweep in half (saving linked list RAM)
                // we need 2^(i - LSBMSB_TRANSITION_BIT - 1) == 1 << (i - LSBMSB_TRANSITION_BIT - 1) passes from i to MSB
                //DEBUG("buffer %d: repeat %d times, size: %d, from %d - %d", nextBufdescIndex, 1<<(i - LSBMSB_TRANSITION_BIT - 1), (COLOR_DEPTH_BITS - i), i, COLOR_DEPTH_BITS-1);
                for (int k = 0; k < (1 << (i - s_lsb_msb_transition_bit - 1)); k++)
                {
                    i2s_parallel_link_dma_desc(&s_dmadesc_a[currentDescOffset], prevdmadesca, &(s_frames[0].rowdata[j].rowbits[i].pixel), sizeof(row_bit_t) * (COLOR_DEPTH_BITS - i));
                    prevdmadesca = &s_dmadesc_a[currentDescOffset];
                    i2s_parallel_link_dma_desc(&s_dmadesc_b[currentDescOffset], prevdmadescb, &(s_frames[1].rowdata[j].rowbits[i].pixel), sizeof(row_bit_t) * (COLOR_DEPTH_BITS - i));
                    prevdmadescb = &s_dmadesc_b[currentDescOffset];
                    currentDescOffset++;
                    //DEBUG("i %d, j %d, k %d", i, j, k);
                }
            }
        }
        // end markers
        s_dmadesc_a[desccount - 1].eof = 1;
        s_dmadesc_b[desccount - 1].eof = 1;
        s_dmadesc_a[desccount - 1].qe.stqe_next = (lldesc_t *)&s_dmadesc_a[0];
        s_dmadesc_b[desccount - 1].qe.stqe_next = (lldesc_t *)&s_dmadesc_b[0];
    }

    // flush complete semaphore
    if (res == ESP_OK)
    {
#if CONFIG_SUPPORT_STATIC_ALLOCATION
        static StaticSemaphore_t sem;
        s_shift_complete_sem = xSemaphoreCreateBinaryStatic(&sem);
#else
        s_shift_complete_sem = xSemaphoreCreateBinary();
#endif
        i2s_parallel_set_shiftcomplete_cb(s_shift_complete_sem_cb);
    }

    // initialise parallel I2S
    if (res == ESP_OK)
    {
        i2s_parallel_config_t cfg =
        {
            .gpio_bus =
            {
                CONFIG_LEDDISPLAY_R1_GPIO,   //  0 BIT_R1
                CONFIG_LEDDISPLAY_G1_GPIO,   //  1 BIT_G1
                CONFIG_LEDDISPLAY_B1_GPIO,   //  2 BIT_B1
                CONFIG_LEDDISPLAY_R2_GPIO,   //  3 BIT_R2
                CONFIG_LEDDISPLAY_G2_GPIO,   //  4 BIT_G2
                CONFIG_LEDDISPLAY_B2_GPIO,   //  5 BIT_B2
                CONFIG_LEDDISPLAY_LAT_GPIO,  //  6 BIT_LAT
                CONFIG_LEDDISPLAY_OE_GPIO,   //  7 BIT_OE
                CONFIG_LEDDISPLAY_A_GPIO,    //  8 BIT_A
                CONFIG_LEDDISPLAY_B_GPIO,    //  9 BIT_B
                CONFIG_LEDDISPLAY_C_GPIO,    // 10 BIT_C
                CONFIG_LEDDISPLAY_D_GPIO,    // 11 BIT_D
#if LEDDISPLAY_NEED_E_GPIO
                CONFIG_LEDDISPLAY_E_GPIO,    // 12 BIT_E
#else
                -1,
#endif
                -1,                          // 13
                -1,                          // 14
                -1,                          // 15
            },
            .gpio_clk    = CONFIG_LEDDISPLAY_CLK_GPIO,
            .clkspeed_hz = I2S_CLOCK_SPEED,
            .bits        = I2S_PARALLEL_BITS_16,
            .desccount_a = desccount,
            .desccount_b = desccount,
            .lldesc_a    = s_dmadesc_a,
            .lldesc_b    = s_dmadesc_b,
        };

        esp_err_t res2 = i2s_parallel_setup(&I2S1, &cfg);
        if (res2 != ESP_OK)
        {
            WARNING("i2s fail (%d, %s)", res, esp_err_to_name(res));
            res = ESP_FAIL;
        }
    }

    if (res == ESP_OK)
    {
        INFO("init done");
    }
    // clean up on error
    else
    {
        WARNING("init fail: %s (%d)", esp_err_to_name(res), res);
        leddisplay_shutdown();
    }
    return res;
}

void leddisplay_shutdown(void)
{
    INFO("shutdown");
    i2s_parallel_stop(&I2S1);
    if (s_frames != NULL)
    {
        heap_caps_free(s_frames);
        s_frames = NULL;
    }
    if (s_dmadesc_a != NULL)
    {
        heap_caps_free(s_dmadesc_a);
        s_dmadesc_a = NULL;
    }
    if (s_dmadesc_b != NULL)
    {
        heap_caps_free(s_dmadesc_b);
        s_dmadesc_b = NULL;
    }
#if CONFIG_SUPPORT_STATIC_ALLOCATION
#else
    vSemaphoreDelete(s_shift_complete_sem);
#endif


}

/* *********************************************************************************************** */

int leddisplay_set_brightness(int brightness)
{
    const int last_brightness_percent = s_brightness_percent;

    if (brightness <= 0)
    {
        s_brightness_val = 0;
        s_brightness_percent = 0;
    }
    else if (brightness >= 100)
    {
        s_brightness_val = LEDDISPLAY_WIDTH;
        s_brightness_percent = 100;
    }
    else
    {
        s_brightness_percent = brightness;

        // scale brightness percent to value for this display: 0..100% --> 0..LEDDISPLAY_WIDTH
        const int brightness_val = ((((1000 * LEDDISPLAY_WIDTH) * brightness) + 500) / 1000) / 100;

#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT

        const int f = 256 / LEDDISPLAY_WIDTH;
        s_brightness_val = val2pwm(brightness_val * f) / f;

#elif CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED

        const int f = 256 / LEDDISPLAY_WIDTH;
        const int lut = val2pwm(brightness_val * f) / f;
        if (lut <= 0)
        {
            s_brightness_val = 1;
        }
        else
        {
            s_brightness_val = lut;
        }

#else
        s_brightness_val = brightness_val;
#endif
    }

    return last_brightness_percent;
}

int leddisplay_get_brightness(void)
{
    return s_brightness_percent;
}

/* *********************************************************************************************** */

void leddisplay_pixel_xy_rgb(uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue)
{
    if ( (x_coord >= LEDDISPLAY_WIDTH) || (y_coord >= LEDDISPLAY_HEIGHT) )
    {
        return;
    }

    // What half of the HUB75 panel are we painting to?
    bool paint_top_half = true;
    if ( y_coord > (ROWS_PER_FRAME - 1) ) // co-ords start at zero, y_coord = 15 = 16 (rows per frame)
    {
        y_coord -= ROWS_PER_FRAME; // if it's 16, subtract 16. Array position 0 again.
        paint_top_half = false;
    }

#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
    red   = val2pwm(red);
    green = val2pwm(green);
    blue  = val2pwm(blue);
#endif

    row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

    for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
    {
        // the destination for the pixel bitstream
        row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

        int v = 0; // the output bitstream

        // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
        // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
        int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

        if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
        if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
        if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
        if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
        if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif

        // need to disable OE after latch to hide row transition
        if (x_coord == 0)
        {
            v |= BIT_OE;
        }

        // drive latch while shifting out last bit of RGB data
        // need to turn off OE one clock before latch, otherwise can get ghosting
        if (x_coord == (PIXELS_PER_LATCH - 1))
        {
            v |= (BIT_LAT | BIT_OE);
        }

        // turn off OE after brightness value is reached when displaying MSBs
        // MSBs always output normal brightness
        // LSB (bitplane_ix == 0) outputs normal brightness as MSB from previous row is being displayed
        if ( ((bitplane_ix == 0) || (bitplane_ix > s_lsb_msb_transition_bit)) && (x_coord >= s_brightness_val) )
        {
            v |= BIT_OE; // for Brightness
        }

        // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output
        // after data is shifted, so need to set OE to fractional brightness
        if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
        {
            // divide brightness in half for each bit below s_lsb_msb_transition_bit
            int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
            if (x_coord >= lsbBrightness)
            {
                v |= BIT_OE; // for Brightness
            }
        }

        // When using the Adafruit drawPixel, we only have one pixel co-ordinate and colour to draw
        // (duh) so we can't paint a top and bottom half (or whatever row split the panel is) at the
        // same time.  Need to be smart and check the DMA buffer to see what the other half thinks
        // (pun intended) and persist this when we refresh.
        // The DMA buffer order has also been reversed (refer to the last code in this function) so
        // we have to check for this and check the correct position of the uint16_t data.
        int16_t tmp_x_coord = x_coord;
        if ((x_coord % 2) != 0)
        {
            tmp_x_coord -= 1;
        }
        else
        {
            tmp_x_coord += 1;
        } // end reordering

        uint8_t mask = BIT(bitplane_ix); // 8 bit color

        // need to copy what the RGB status is for the bottom pixels
        if (paint_top_half)
        {

           // Set the color of the pixel of interest
           if (green & mask) { v |= BIT_G1; }
           if (blue  & mask) { v |= BIT_B1; }
           if (red   & mask) { v |= BIT_R1; }

           // Persist what was painted to the other half of the frame equiv. pixel
           if (rowbits->pixel[tmp_x_coord] & BIT_R2) { v |= BIT_R2; }
           if (rowbits->pixel[tmp_x_coord] & BIT_G2) { v |= BIT_G2; }
           if (rowbits->pixel[tmp_x_coord] & BIT_B2) { v |= BIT_B2; }
        }
        // do it the other way around
        else
        {
            // color to set
            if (red   & mask) { v |= BIT_R2; }
            if (green & mask) { v |= BIT_G2; }
            if (blue  & mask) { v |= BIT_B2; }

            // copy
            if (rowbits->pixel[tmp_x_coord] & BIT_R1) { v |= BIT_R1; }
            if (rowbits->pixel[tmp_x_coord] & BIT_G1) { v |= BIT_G1; }
            if (rowbits->pixel[tmp_x_coord] & BIT_B1) { v |= BIT_B1; }

        } // paint


        // 16 bit parallel mode
        // save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
        if( (x_coord % 2) != 0)
        {
            rowbits->pixel[tmp_x_coord] = v;
        }
        else
        {
            rowbits->pixel[tmp_x_coord] = v;
        } // end reordering

    } // color depth loop (8)
}


void leddisplay_pixel_fill_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
#if 0
    for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
    {
        for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
        {
            leddisplay_pixel_xy_rgb(x, y, red, green, blue);
        }
    }
#else
#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
    red   = val2pwm(red);
    green = val2pwm(green);
    blue  = val2pwm(blue);
#endif
    for (unsigned int y_coord = 0; y_coord < ROWS_PER_FRAME; y_coord++) // half height - 16 iterations
    {
        row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

        for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
        {
            uint16_t mask = (1 << bitplane_ix); // 24 bit color

            // the destination for the pixel bitstream
            row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

            for (int x_coord = 0; x_coord < LEDDISPLAY_WIDTH; x_coord++) // row pixel width 64 iterations
            {
                int v = 0; // the output bitstream

                // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
                // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
                int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

                if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
                if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
                if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
                if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
                if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif
                // need to disable OE after latch to hide row transition
                if (x_coord == 0) { v |= BIT_OE; }

                // drive latch while shifting out last bit of RGB data
                // need to turn off OE one clock before latch, otherwise can get ghosting
                if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE); }

                // turn off OE after brightness value is reached when displaying MSBs
                // MSBs always output normal brightness
                // LSB (!bitplane_ix) outputs normal brightness as MSB from previous row is being displayed
                if ( ((bitplane_ix > s_lsb_msb_transition_bit) || !bitplane_ix) && (x_coord >= s_brightness_val) )
                {
                    v |= BIT_OE; // For Brightness
                }

                // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output after data is shifted, so need to set OE to fractional brightness
                if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
                {
                    // divide brightness in half for each bit below s_lsb_msb_transition_bit
                    int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
                    if (x_coord >= lsbBrightness) { v |= BIT_OE; } // For Brightness
                }

                // top and bottom half colours
                if (red    & mask) { v |= (BIT_R1 | BIT_R2); }
                if (green  & mask) { v |= (BIT_G1 | BIT_G2); }
                if (blue   & mask) { v |= (BIT_B1 | BIT_B2); }

                // 16 bit parallel mode
                // Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
                if ((x_coord % 2) != 0)
                {
                    rowbits->pixel[x_coord - 1] = v;
                }
                else
                {
                    rowbits->pixel[x_coord + 1] = v;
                } // end reordering

            } // end x_coord iteration
        } // colour depth loop (8)
    } // end row iteration
#endif
}

/* *********************************************************************************************** */

inline void leddisplay_frame_xy_rgb(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue)
{
    if ( (x_coord >= LEDDISPLAY_WIDTH) || (y_coord >= LEDDISPLAY_HEIGHT) )
    {
        return;
    }
    p_frame->yx[y_coord][x_coord][0] = red;
    p_frame->yx[y_coord][x_coord][1] = green;
    p_frame->yx[y_coord][x_coord][2] = blue;
}

inline void leddisplay_frame_fill_rgb(leddisplay_frame_t *p_frame, uint8_t red, uint8_t green, uint8_t blue)
{
    if ( (red == green) && (red == blue) )
    {
        memset(p_frame, red, sizeof(*p_frame));
    }
    else
    {
        for (uint16_t ix = 0; ix < NUMOF(p_frame->ix); ix++)
        {
            p_frame->ix[ix][0] = red;
            p_frame->ix[ix][1] = green;
            p_frame->ix[ix][2] = blue;
        }
    }
}

inline void leddisplay_frame_clear(leddisplay_frame_t *p_frame)
{
    memset(p_frame, 0, sizeof(*p_frame));
}

#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
#  define _VAL2PWM(v) val2pwm(v)
#else
#  define _VAL2PWM(v) (v)
#endif

void leddisplay_frame_update(const leddisplay_frame_t *p_frame)
{
    // if necessary, block until current framebuffer memory becomes available
    xSemaphoreTake(s_shift_complete_sem, portMAX_DELAY);

#if 0
    for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
    {
        for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
        {
            const uint8_t *p_rgb = p_frame->yx[y][x];
            leddisplay_pixel_xy_rgb(x, y, _VAL2PWM(p_rgb[0]), _VAL2PWM(p_rgb[1]), _VAL2PWM(p_rgb[2]));
        }
    }
#else
    for (unsigned int y_coord = 0; y_coord < ROWS_PER_FRAME; y_coord++) // half height - 16 iterations
    {
        row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

        for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
        {
            uint16_t mask = (1 << bitplane_ix); // 24 bit color

            // the destination for the pixel bitstream
            row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

            for (int x_coord = 0; x_coord < LEDDISPLAY_WIDTH; x_coord++) // row pixel width 64 iterations
            {
                int v = 0; // the output bitstream

                // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
                // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
                int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

                if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
                if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
                if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
                if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
                if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif
                // need to disable OE after latch to hide row transition
                if (x_coord == 0) { v |= BIT_OE; }

                // drive latch while shifting out last bit of RGB data
                // need to turn off OE one clock before latch, otherwise can get ghosting
                if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE); }

                // turn off OE after brightness value is reached when displaying MSBs
                // MSBs always output normal brightness
                // LSB (!bitplane_ix) outputs normal brightness as MSB from previous row is being displayed
                if ( ((bitplane_ix > s_lsb_msb_transition_bit) || !bitplane_ix) && (x_coord >= s_brightness_val) )
                {
                    v |= BIT_OE; // For Brightness
                }

                // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output after data is shifted, so need to set OE to fractional brightness
                if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
                {
                    // divide brightness in half for each bit below s_lsb_msb_transition_bit
                    int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
                    if (x_coord >= lsbBrightness) { v |= BIT_OE; } // For Brightness
                }

                // top half
                const uint8_t *p_rgb_top = p_frame->yx[y_coord][x_coord];
                if (_VAL2PWM(p_rgb_top[0]) & mask) { v |= BIT_R1; }
                if (_VAL2PWM(p_rgb_top[1]) & mask) { v |= BIT_G1; }
                if (_VAL2PWM(p_rgb_top[2]) & mask) { v |= BIT_B1; }

                // bottom half
                const uint8_t *p_rgb_bot = p_frame->yx[y_coord + ROWS_PER_FRAME][x_coord];
                if (_VAL2PWM(p_rgb_bot[0]) & mask) { v |= BIT_R2; }
                if (_VAL2PWM(p_rgb_bot[1]) & mask) { v |= BIT_G2; }
                if (_VAL2PWM(p_rgb_bot[2]) & mask) { v |= BIT_B2; }

                // 16 bit parallel mode
                // Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
                if ((x_coord % 2) != 0)
                {
                    rowbits->pixel[x_coord - 1] = v;
                }
                else
                {
                    rowbits->pixel[x_coord + 1] = v;
                } // end reordering

            } // end x_coord iteration
        } // colour depth loop (8)
    } // end row iteration
#endif

    leddisplay_pixel_update(0);
}


/* *********************************************************************************************** */
