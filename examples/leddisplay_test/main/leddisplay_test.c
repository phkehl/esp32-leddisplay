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

#include "nyan_64x32.h"

/* *********************************************************************************************** */

// local logging
#define LOGNAME "leddisplay_test"
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
static void sHsvToRgb(const uint8_t hue, const uint8_t sat, const uint8_t val, uint8_t *red, uint8_t *green, uint8_t *blue);
static void sLedfxConcentricHueFlow(leddisplay_frame_t *pFrame, const bool init, const int8_t step, uint8_t *r0);
static void sLedfxPlasma(leddisplay_frame_t *pFrame, const bool init, float *r0);
static void sAnimNyan(leddisplay_frame_t *p_frame, uint32_t delay, int frame);
static void sTicTocInit(const uint32_t reg, const char *name);
static void sTic(const uint32_t reg);
static void sToc(const uint32_t reg);
static void sTicTocStats(const uint32_t reg);
static void sDumpMemInfo(void);

void app_main(void)
{
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
        leddisplay_init();
        sDumpMemInfo();

        /* ***** pixel based API ***************************************************************** */

        INFO("fill red (pixel)");
        leddisplay_pixel_fill_rgb(255, 0, 0);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill blue (pixel)");
        leddisplay_pixel_fill_rgb(0, 255, 0);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill green (pixel)");
        leddisplay_pixel_fill_rgb(0, 0, 255);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill red+green (pixel)");
        leddisplay_pixel_fill_rgb(255, 255, 0);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill red+blue (pixel)");
        leddisplay_pixel_fill_rgb(255, 0, 255);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill green+blue (pixel)");
        leddisplay_pixel_fill_rgb(0, 255, 255);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill red+green+blue (pixel)");
        leddisplay_pixel_fill_rgb(255, 255, 255);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("some pixels (pixel)");
        leddisplay_pixel_fill_rgb(0, 0, 0);
        leddisplay_pixel_xy_rgb(1, 2, 255, 0, 0);
        leddisplay_pixel_xy_rgb(3, 4, 0, 255, 0);
        leddisplay_pixel_xy_rgb(5, 6, 0, 0, 255);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("individual pixels (pixel)");
        {
            int n = 25;
            sTicTocInit(0, "setpixel");
            sTicTocInit(1, "update");
            uint8_t hue = 0;
            while (n > 0)
            {
                uint8_t red = 0, green = 0, blue = 0;
                sHsvToRgb(hue, 255, 255, &red, &green, &blue);
                sTic(0);
                for (uint16_t y = 0; y < CONFIG_LEDDISPLAY_HEIGHT; y++)
                {
                    for (uint16_t x = 0; x < CONFIG_LEDDISPLAY_WIDTH; x++)
                    {
                        leddisplay_pixel_xy_rgb(x, y, red, green, blue);
                    }
                }
                sToc(0);
                sTic(1);
                leddisplay_pixel_update(0);
                sToc(1);
                n--;
                hue += 10;
                osSleep(2 * delay);
            }
            sTicTocStats(0);
            sTicTocStats(1);
        }

        // -----------------------------------------------------------------------------------------

        INFO("clear pixels (pixel)");
        leddisplay_pixel_fill_rgb(0, 0, 0);
        leddisplay_pixel_update(0);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("hue flow fx (pixel)");
        {
            uint32_t now = osTicks();
            uint8_t r0;
            const uint8_t step = 4;
            sLedfxConcentricHueFlow(NULL, true, 0, &r0);
            leddisplay_pixel_update(0);
            osDelayUntil(&now, delay);
            int n = 256 / step;
            while (n > 0)
            {
                sLedfxConcentricHueFlow(NULL, false, step, &r0);
                leddisplay_pixel_update(0);
                osDelayUntil(&now, delay);
                n--;
            }
        }

        // -----------------------------------------------------------------------------------------

        INFO("plasma fx (pixel)");
        {
            float r0;
            sLedfxPlasma(NULL, true, &r0);
            leddisplay_pixel_update(0);
            osSleep(10);
            int n = 70;
            sTicTocInit(0, "plasma");
            sTicTocInit(1, "update");
            while (n > 0)
            {
                sTic(0);
                sLedfxPlasma(NULL, false, &r0);
                sToc(0);
                sTic(1);
                leddisplay_pixel_update(0);
                sToc(1);
                osSleep(10);
                n--;
            }
            sTicTocStats(0);
            sTicTocStats(1);
        }

        // -----------------------------------------------------------------------------------------

        INFO("animation (pixel)");
        {
            int n = 7;
            while (n > 0)
            {
                sAnimNyan(NULL, delay, -1);
                n--;
            }
        }

        // -----------------------------------------------------------------------------------------

        INFO("animation + brightness (pixel)");
        {
            const int oldBrightness = leddisplay_get_brightness();
            const int delta = 2;
            int n = CONFIG_LEDDISPLAY_WIDTH * 2 / delta;
            int aniFrame = 0;
            int brightness = 0;
            int dir = delta;
            while (n > 0)
            {
                leddisplay_set_brightness(brightness);
                sAnimNyan(NULL, 2*delay, aniFrame);

                n--;
                aniFrame++;
                aniFrame %= 12;

                brightness += dir;
                if (brightness > CONFIG_LEDDISPLAY_WIDTH)
                {
                    dir = -delta;
                    brightness -= 2 * delta;
                }
                else if (brightness < 0)
                {
                    dir = +delta;
                    brightness += 2 * delta;
                }
            }
            DEBUG("reset brightness");
            leddisplay_set_brightness(oldBrightness);
        }


        /* ***** frame based API ***************************************************************** */

        INFO("fill red (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 255, 0, 0);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill blue (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 0, 255, 0);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill green (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 0, 0, 255);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill red+green (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 255, 255, 0);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill red+blue (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 255, 0, 255);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill green+blue (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 0, 255, 255);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("fill red+green+blue (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 255, 255, 255);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("some pixels (frame)");
        leddisplay_frame_fill_rgb(&sDispFrame, 0, 0, 0);
        leddisplay_frame_xy_rgb(&sDispFrame, 1, 2, 255, 0, 0);
        leddisplay_frame_xy_rgb(&sDispFrame, 3, 4, 0, 255, 0);
        leddisplay_frame_xy_rgb(&sDispFrame, 5, 6, 0, 0, 255);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("individual pixels (frame)");
        {
            int n = 25;
            sTicTocInit(0, "setpixel");
            sTicTocInit(1, "update");
            uint8_t hue = 0;
            while (n > 0)
            {
                uint8_t red = 0, green = 0, blue = 0;
                sHsvToRgb(hue, 255, 255, &red, &green, &blue);
                sTic(0);
                for (uint16_t y = 0; y < CONFIG_LEDDISPLAY_HEIGHT; y++)
                {
                    for (uint16_t x = 0; x < CONFIG_LEDDISPLAY_WIDTH; x++)
                    {
                        leddisplay_frame_xy_rgb(&sDispFrame, x, y, red, green, blue);
                    }
                }
                sToc(0);
                sTic(1);
                leddisplay_frame_update(&sDispFrame);
                sToc(1);
                n--;
                hue += 10;
                osSleep(2 * delay);
            }
            sTicTocStats(0);
            sTicTocStats(1);
        }

        // -----------------------------------------------------------------------------------------

        INFO("clear pixels (frame)");
        leddisplay_frame_clear(&sDispFrame);
        leddisplay_frame_update(&sDispFrame);
        osSleep(10 * delay);

        // -----------------------------------------------------------------------------------------

        INFO("hue flow fx (frame)");
        {
            uint32_t now = osTicks();
            uint8_t r0;
            const uint8_t step = 4;
            sLedfxConcentricHueFlow(&sDispFrame, true, 0, &r0);
            leddisplay_frame_update(&sDispFrame);
            osDelayUntil(&now, delay);
            int n = 256 / step;
            while (n > 0)
            {
                sLedfxConcentricHueFlow(&sDispFrame, false, step, &r0);
                leddisplay_frame_update(&sDispFrame);
                osDelayUntil(&now, delay);
                n--;
            }
        }

        // -----------------------------------------------------------------------------------------

        INFO("plasma fx (frame)");
        {
            float r0;
            sLedfxPlasma(&sDispFrame, true, &r0);
            leddisplay_frame_update(&sDispFrame);
            osSleep(10);
            int n = 70;
            sTicTocInit(0, "plasma");
            sTicTocInit(1, "update");
            while (n > 0)
            {
                sTic(0);
                sLedfxPlasma(&sDispFrame, false, &r0);
                sToc(0);
                sTic(1);
                leddisplay_frame_update(&sDispFrame);
                sToc(1);
                osSleep(10);
                n--;
            }
            sTicTocStats(0);
            sTicTocStats(1);
        }

        // -----------------------------------------------------------------------------------------

        INFO("animation (frame)");
        {
            int n = 7;
            while (n > 0)
            {
                sAnimNyan(&sDispFrame, delay, -1);
                n--;
            }
        }

        // -----------------------------------------------------------------------------------------

        INFO("animation + brightness (frame)");
        {
            const int oldBrightness = leddisplay_get_brightness();
            const int delta = 2;
            int n = CONFIG_LEDDISPLAY_WIDTH * 2 / delta;
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
                if (brightness > CONFIG_LEDDISPLAY_WIDTH)
                {
                    dir = -delta;
                    brightness -= 2 * delta;
                }
                else if (brightness < 0)
                {
                    dir = +delta;
                    brightness += 2 * delta;
                }
            }
            DEBUG("reset brightness");
            leddisplay_set_brightness(oldBrightness);
        }


        /* ***** shutdown display *************************************************************** */

        INFO("shutdown display");
        sDumpMemInfo();
        leddisplay_shutdown();
        sDumpMemInfo();
        printf("\r\n\r\n\r\n");
        osSleep(5000);
    }
}


/* *********************************************************************************************** */

// classic HSV2RGB code (#HSV2RGB_METHOD 1 and 2) à la Wikipedia
static void sHsvToRgb(const uint8_t hue, const uint8_t sat, const uint8_t val, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    const uint32_t s = (6 * (uint32_t)hue) >> 8;               /* the segment 0..5 (360/60 * [0..255] / 256) */
    const uint32_t t = (6 * (uint32_t)hue) & 0xff;             /* within the segment 0..255 (360/60 * [0..255] % 256) */
    const uint32_t l = ((uint32_t)val * (255 - (uint32_t)sat)) >> 8; /* lower level */
    const uint32_t r = ((uint32_t)val * (uint32_t)sat * t) >> 16;    /* ramp */
    switch (s)
    {
        case 0: *red = (uint8_t)val;        *green = (uint8_t)(l + r);    *blue = (uint8_t)l;          break;
        case 1: *red = (uint8_t)(val - r);  *green = (uint8_t)val;        *blue = (uint8_t)l;          break;
        case 2: *red = (uint8_t)l;          *green = (uint8_t)val;        *blue = (uint8_t)(l + r);    break;
        case 3: *red = (uint8_t)l;          *green = (uint8_t)(val - r);  *blue = (uint8_t)val;        break;
        case 4: *red = (uint8_t)(l + r);    *green = (uint8_t)l;          *blue = (uint8_t)val;        break;
        case 5: *red = (uint8_t)val;        *green = (uint8_t)l;          *blue = (uint8_t)(val - r);  break;
    }
}

static void sLedfxConcentricHueFlow(leddisplay_frame_t *pFrame, const bool init, const int8_t step, uint8_t *r0)
{
    if (init)
    {
        *r0 = 0;
    }
    else
    {
        (*r0) += step;
    }

    const int16_t x0 = CONFIG_LEDDISPLAY_WIDTH / 2;
    const int16_t y0 = CONFIG_LEDDISPLAY_HEIGHT / 2;
    const int16_t hueMax = 256/2;
    const uint8_t sat = 255;
    const uint8_t val = 255;
    int16_t dX = x0 + 1;
    while (dX--)
    {
        int16_t dY = y0 + 1;
        while (dY--)
        {
            const uint8_t hue = (uint8_t)(((dX*dX) + (dY*dY))
               * hueMax / ((x0*x0) + (y0*y0))) + *r0;
            //DEBUG("ledfxConcentricHueFlow() %2i %2i %2i %2i %3u -> %2i %2i   %2i %2i  %2i %2i  %2i %2i",
            //      dx, dy, X0, Y0, hue,
            //      X0 + dx, dy, X0 - dx, dy, dx, Y0 + dy, dx, Y0 - dy);

            uint8_t red = 0, green = 0, blue = 0;
            sHsvToRgb(hue, sat, val, &red, &green, &blue);
            if (pFrame == NULL)
            {
                leddisplay_pixel_xy_rgb(x0 + dX, y0 + dY, red, green, blue);
                leddisplay_pixel_xy_rgb(x0 - dX, y0 + dY, red, green, blue);
                leddisplay_pixel_xy_rgb(x0 + dX, y0 - dY, red, green, blue);
                leddisplay_pixel_xy_rgb(x0 - dX, y0 - dY, red, green, blue);
            }
            else
            {
                leddisplay_frame_xy_rgb(pFrame, x0 + dX, y0 + dY, red, green, blue);
                leddisplay_frame_xy_rgb(pFrame, x0 - dX, y0 + dY, red, green, blue);
                leddisplay_frame_xy_rgb(pFrame, x0 + dX, y0 - dY, red, green, blue);
                leddisplay_frame_xy_rgb(pFrame, x0 - dX, y0 - dY, red, green, blue);
            }
        }
    }
}

// The formulas used in this function are based on code floating the internet in various code
// (google "colorduino", "shiftbrite", et al.). While the original source is never properly
// referenced it all seems to be attributed to: copyright (c) 2011 Sam C. Lin, 2009 Ben Combee,
// 2009 Ken Corey, 2008 Windell H. Oskay \todo find original reference for this
static inline float sDist(const float a, const float b, const float c, const float d)
{
    // based on: see header file
    const float cma = c - a;
    const float dmb = d - b;
    return sqrtf( (cma * cma) + (dmb * dmb) );
}
static void sLedfxPlasma(leddisplay_frame_t *pFrame, const bool init, float *r0)
{
    const uint8_t sat = 255;
    const uint8_t val = 255;

    if (init)
    {
        *r0 = 128000.0f; // pallete shift, FIXME: 128000, obviously.. :-/
    }

    for (uint16_t y = 0; y < CONFIG_LEDDISPLAY_HEIGHT; y++)
    {
        for (uint16_t x = 0; x < CONFIG_LEDDISPLAY_WIDTH; x++)
        {
            // based on source: see progPlasma() docu
            const float value =
                sinf( sDist((float)x + *r0, y, 128.0f, 128.0) * (1.0f / 8.0f)) +
                sinf( sDist(x, y, 64.0f, 64.0f) * (1.0f / 8.0f) ) +
                sinf( sDist(x, (float)y + (*r0 / 7.0f), 192.0f, 64.0f) * (1.0f / 7.0f) ) +
                sinf( sDist(x, y, 192.0f, 100.0f) * (1.0 / 8.0) );
            const uint8_t hue = (uint8_t)((uint32_t)(value * 128.0) & 0xff);

            uint8_t red = 0, green = 0, blue = 0;
            sHsvToRgb(hue, sat, val, &red, &green, &blue);
            if (pFrame == NULL)
            {
                leddisplay_pixel_xy_rgb(x, y, red, green, blue);
            }
            else
            {
                leddisplay_frame_xy_rgb(pFrame, x, y, red, green, blue);
            }
        }
    }
    (*r0) -= 0.25; // smooth (the original code used -= 1)
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

    const uint16_t x_max = CONFIG_LEDDISPLAY_WIDTH  < 64 ? CONFIG_LEDDISPLAY_WIDTH  : 64;
    const uint16_t y_max = CONFIG_LEDDISPLAY_HEIGHT < 32 ? CONFIG_LEDDISPLAY_HEIGHT : 32;

    // which frames to play?
    int startFrame = (frame < 0) || (frame >= nFrames) ? 0             : frame;
    int endFrame   = (frame < 0) || (frame >= nFrames) ? (nFrames - 1) : frame;

    // play each frame
    uint32_t prevTick = xTaskGetTickCount();
    for (frame = startFrame; frame <= endFrame; frame++)
    {
        // copy frame data to display buffer
        const anim_frame_t *data = (const anim_frame_t  *)&animData[64 * 32 * 3 * frame];

        // pixel based
        if (p_frame == NULL)
         {
            for (uint16_t x = 0; x < x_max; x++)
            {
                for (uint16_t y = 0; y < y_max; y++)
                {
                    const uint8_t *rgb = data->yx[y][x];
                    leddisplay_pixel_xy_rgb(x, y, rgb[0], rgb[1], rgb[2]);
                }
            }
        }
        // frame based
        else
        {
            for (uint16_t x = 0; x < x_max; x++)
            {
                for (uint16_t y = 0; y < y_max; y++)
                {
                    const uint8_t *rgb = data->yx[y][x];
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

typedef struct TICTOC_s
{
    const char *name;
    uint32_t last;
    uint32_t meas[100];
    uint32_t ix;
} TICTOC_t;

static TICTOC_t sTicToc[5];

static void sTicTocInit(const uint32_t reg, const char *name)
{
    if (reg > (NUMOF(sTicToc) - 1))
    {
        return;
    }
    memset(&sTicToc[reg], 0, sizeof(sTicToc[reg]));
    sTicToc[reg].name = name;
}

static void sTic(const uint32_t reg)
{
    if (reg > (NUMOF(sTicToc) - 1))
    {
        return;
    }
    sTicToc[reg].last = esp_timer_get_time();
}

static void sToc(const uint32_t reg)
{
    if (reg > (NUMOF(sTicToc) - 1))
    {
        return;
    }
    sTicToc[reg].meas[ sTicToc[reg].ix ] = esp_timer_get_time() - sTicToc[reg].last;
    sTicToc[reg].ix++;
    sTicToc[reg].ix %= NUMOF(sTicToc[reg].meas);
}

static void sTicTocStats(const uint32_t reg)
{
    if (reg > (NUMOF(sTicToc) - 1))
    {
        return;
    }

    uint64_t sum = 0;
    uint32_t n = 0;
    uint32_t min = UINT32_MAX;
    uint32_t max = 0;
    for (int ix = 0; ix < NUMOF(sTicToc[reg].meas); ix++)
    {
        if (sTicToc[reg].meas[ix] > 0)
        {
            n++;
            sum += sTicToc[reg].meas[ix];
            if (sTicToc[reg].meas[ix] < min) { min = sTicToc[reg].meas[ix]; }
            if (sTicToc[reg].meas[ix] > max) { max = sTicToc[reg].meas[ix]; }
        }
    }
    if (n > 0)
    {
        uint32_t avg = sum / n;
        double rate = 1000000.0 / (double)avg;
        DEBUG("tictoc[%u]=%s: n=%u, avg=%u, rate=%.1f, min=%u, max=%u",
            reg, sTicToc[reg].name == NULL ? "???" : sTicToc[reg].name,
            n, avg, rate, min, max);
    }
    else
    {
        DEBUG("tictoc[%u]=%s: no meas",
            reg, sTicToc[reg].name == NULL ? "???" : sTicToc[reg].name);
    }

    memset(&sTicToc[reg], 0, sizeof(sTicToc[reg]));
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
