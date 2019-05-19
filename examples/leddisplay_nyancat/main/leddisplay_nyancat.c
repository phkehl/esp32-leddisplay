/* *********************************************************************************************** */
/* leddisplay test

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/* *********************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <soc/rtc.h>
#include <esp_timer.h>

#include <leddisplay.h>

#include "mon.h"
#include "nyan_64x32.h"

/* *********************************************************************************************** */

// local logging
#define LOGNAME "leddisplay_nyancat"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)    ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)

// useful macros
#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define TICKS2MS(ticks)           ((ticks) * portTICK_PERIOD_MS)
#define osTicks()                 xTaskGetTickCount()
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));
#define osDelayUntil(prev, inc)   vTaskDelayUntil(prev, MS2TICKS(inc));
#define NUMOF(x)                  (sizeof(x)/sizeof(*(x)))

/* *********************************************************************************************** */

// forward declarations
static void sLeddisplayTestTask(void *pParam);
static void sAnimNyan(leddisplay_frame_t *p_frame, uint32_t delay, int frame);
static void sDumpMemInfo(void);

void app_main(void)
{
    monStart();
    //monSetPeriod(1000);

    osSleep(2000);
    printf("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");

    INFO("Hello, hello, good morning, good evening!");

    rtc_cpu_freq_config_t cpuCfg;
    rtc_clk_cpu_freq_get_config(&cpuCfg);
    DEBUG("CPU: %uMHz", cpuCfg.freq_mhz);

    BaseType_t res = xTaskCreatePinnedToCore(
        sLeddisplayTestTask, "leddisplay_test", 8192 / sizeof(StackType_t), NULL, 10, NULL, 1);
    assert(res == pdPASS);
}

static void sLeddisplayTestTask(void *pParam)
{
    INFO("testing start...");
    const uint32_t delay = 100;
    static leddisplay_frame_t sDispFrame;

    // run tests forever
    while (true)
    {
        /* ***** initialise display ************************************************************** */

        INFO("init display");
        sDumpMemInfo();
        if (leddisplay_init()!= ESP_OK)
        {
            ERROR(":-(");
            osSleep(2000);
            continue;
        }
        sDumpMemInfo();
        osSleep(1000);

        // -----------------------------------------------------------------------------------------

        INFO("random fill (frame)");
        {
            int n = 200;
            while (n--)
            {
                esp_fill_random(&sDispFrame, sizeof(sDispFrame));
                leddisplay_frame_update(&sDispFrame);
                osSleep(delay / 10);
                n--;
            }
        }

        // -----------------------------------------------------------------------------------------

        INFO("animation (frame)");
        {
            int n = 15;
            while (n > 0)
            {
                sAnimNyan(&sDispFrame, delay, -1);
                n--;
            }
        }

        // -----------------------------------------------------------------------------------------

        leddisplay_frame_clear(&sDispFrame);
        leddisplay_frame_update(&sDispFrame);

        // -----------------------------------------------------------------------------------------

        INFO("animation + brightness (frame)");
        {
            const int oldBrightness = leddisplay_get_brightness();
            const int delta = 1;
            int n = 100 * 2 / delta;
            int aniFrame = 0;
            int brightness = 0;
            int dir = delta;
            while (n > 0)
            {
                leddisplay_set_brightness(brightness);
                sAnimNyan(&sDispFrame, 2*delay, aniFrame);

                n--;
                aniFrame++;
                aniFrame %= 12;

                brightness += dir;
                if (brightness >= 100)
                {
                    dir = -delta;
                    brightness -= 2 * delta;
                }
                else if (brightness <= 0)
                {
                    dir = +delta;
                    brightness += 2 * delta;
                }
            }
            leddisplay_set_brightness(oldBrightness);
        }

        // -----------------------------------------------------------------------------------------

        INFO("shutdown display");
        sDumpMemInfo();
        leddisplay_shutdown();
        sDumpMemInfo();
        printf("\r\n\r\n\r\n");
        osSleep(2000);
    }
}


/* *********************************************************************************************** */

typedef struct anim_frame_s
{
    uint8_t yx[32][64][3];
} anim_frame_t;

static void sAnimNyan(leddisplay_frame_t *p_frame, uint32_t delay, int frame)
{
    // get animation frame
    int nFrames;
    const uint8_t *animData = get_nyan_64x32(&nFrames);

    //DEBUG("test %s delay=%u frame=%d nFrames=%d brightness=%d",
    //    p_frame == NULL ? "pixel" : "frame", delay, frame, nFrames, leddisplay_get_brightness());

    // clear display
    leddisplay_pixel_fill_rgb(0, 0, 0);

    // which frames to play?
    int startFrame = (frame < 0) || (frame >= nFrames) ? 0             : frame;
    int endFrame   = (frame < 0) || (frame >= nFrames) ? (nFrames - 1) : frame;

    // offset animation if display is smaller than the animation
    const int16_t nxOffs = LEDDISPLAY_WIDTH  < 64 ? (64 - LEDDISPLAY_WIDTH)  / 2 : 0;
    const int16_t nyOffs = LEDDISPLAY_HEIGHT < 32 ? (32 - LEDDISPLAY_HEIGHT) / 2 : 0;

    // play each frame
    uint32_t prevTick = xTaskGetTickCount();
    for (frame = startFrame; frame <= endFrame; frame++)
    {
        // copy frame data to display buffer
        const anim_frame_t *data = (const anim_frame_t  *)&animData[64 * 32 * 3 * frame];

        // pixel based
        if (p_frame == NULL)
         {
            for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
            {
                for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
                {
                    const uint8_t *rgb = data->yx[(nyOffs + y) % 32][(nxOffs + x) % 64];
                    leddisplay_pixel_xy_rgb(x, y, rgb[0], rgb[1], rgb[2]);
                }
            }
        }
        // frame based
        else
        {
            for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
            {
                for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
                {
                    const uint8_t *rgb = data->yx[(nyOffs + y) % 32][(nxOffs + x) % 64];
                    leddisplay_frame_xy_rgb(p_frame, x, y, rgb[0], rgb[1], rgb[2]);
                }
            }
        }

        // flush display buffer to display
        if (p_frame == NULL)
        {
            leddisplay_pixel_update(true);
        }
        else
        {
            leddisplay_frame_update(p_frame);
        }

        // delay
        if (delay > 0)
        {
            vTaskDelayUntil(&prevTick, delay / portTICK_PERIOD_MS);
            //vTaskDelay();
        }
    }
}

/* *********************************************************************************************** */

static void sDumpMemInfo(void)
{
    DEBUG("heap: EXEC free=%u (min=%u, largest=%u), 32BIT free=%u (min=%u, largest=%u), 8BIT free=%u (min=%u, largest=%u), DMA free=%u (min=%u, largest=%u)",
        heap_caps_get_free_size(MALLOC_CAP_EXEC), heap_caps_get_minimum_free_size(MALLOC_CAP_EXEC), heap_caps_get_largest_free_block(MALLOC_CAP_EXEC),
        heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT),
        heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
        heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_minimum_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
}

/* *********************************************************************************************** */
