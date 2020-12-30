/*!
    \file
    \brief HUB75 LED display driver using I2S parallel mode (see \ref LEDDISPLAY)

    \defgroup LEDDISPLAY LEDDISPLAY

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

    See src/leddisplay.c for details.

    @{
*/

#ifndef __LEDDISPLAY_H__
#define __LEDDISPLAY_H__

#include <stdint.h>
#include <sdkconfig.h>
#include <esp_err.h>

// configuration (see also leddisplay.c)
#if CONFIG_LEDDISPLAY_TYPE_32X16_4SCAN || CONFIG_LEDDISPLAY_TYPE_32X16_8SCAN
#  define LEDDISPLAY_WIDTH                32
#  define LEDDISPLAY_HEIGHT               16

#elif CONFIG_LEDDISPLAY_TYPE_32X32_8SCAN || CONFIG_LEDDISPLAY_TYPE_32X32_16SCAN
#  define LEDDISPLAY_WIDTH                32
#  define LEDDISPLAY_HEIGHT               32

#elif CONFIG_LEDDISPLAY_TYPE_64X32_8SCAN || CONFIG_LEDDISPLAY_TYPE_64X32_16SCAN
#  define LEDDISPLAY_WIDTH                64
#  define LEDDISPLAY_HEIGHT               32

#elif CONFIG_LEDDISPLAY_TYPE_64X64_32SCAN
#  define LEDDISPLAY_WIDTH                64
#  define LEDDISPLAY_HEIGHT               64

#else
#  error This CONFIG_LEDDISPLAY_TYPE is not implemented!
#endif

/* *********************************************************************************************** */
/*!
    \name display functions

    Example:

\code{.c}
    #include <leddisplay.h>
    leddisplay_init();
\endcode

    @{
*/

//! initialise the LED display
/*!
    Call this before calling any other leddisplay_*() function.

    \returns #ESP_OK on success, or on error: #ESP_ERR_NO_MEM, #ESP_FAIL
*/
esp_err_t leddisplay_init(void);

//! shutdown the LED display
void leddisplay_shutdown(void);

//! set global brightness level
/*
    \param[in] brightness  global brightness level, range 0..100 [%]
    \returns the previously set global brightness level
*/
int leddisplay_set_brightness(int brightness);

//! get global brightness levell
/*!
    \returns the currently set global brightness level (0..100 [%])
*/
int leddisplay_get_brightness(void);

//@}

/* *********************************************************************************************** */
/*!
    \name pixel based functions

    These functions operate directly on the internal buffers, which is relatively expensive on CPU
    usage. At 160MHz CPU speed it takes about 20ms to set all pixels on a 64x32 display.

    Example:

\code{.c}
    #include <leddisplay.h>
    leddisplay_init();
    leddisplay_pixel_fill_rgb(0, 0, 0);        // fill black, i.e. clear frame
    leddisplay_pixel_fill_rgb(255, 0, 0);      // fill red at full brightness
    leddisplay_pixel_xy_rgb(10, 5, 0, 255, 0); // set green pixel at x=10 / y=5
    leddisplay_pixel_update();                 // display
\endcode

    @{
*/

//! set pixel to colour
/*!
    Top-left of display is x=0 / y=0.

    \param[in] x_coord  x coordinate
    \param[in] y_coord  y coordinate
    \param[in] red      red value
    \param[in] green    green value
    \param[in] blue     blue value
*/
void leddisplay_pixel_xy_rgb(uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue);

//! fill all pixels with colour
/*!
    \param[in] red    red value
    \param[in] green  green value
    \param[in] blue   blue value
*/
void leddisplay_pixel_fill_rgb(uint8_t red, uint8_t green, uint8_t blue);

//! update display with current frame
/*!
    Flushes the frame to the display.

    \param[in] block   waits for framebuffer to become available again if non-zero

    \note After this function returns the frame memory will be invalid and you will have to draw the
          next frame fully from scratch.
*/
void leddisplay_pixel_update(int block);

//@}

/* *********************************************************************************************** */
/*!
    \name frame based functions

    These functions operate on a user supplied buffer and are more CPU efficient at the cost of
    additional memory. At 160MHz CPU speed it takes about 8ms to process a 64x32 frame into the DMA
    buffers.

    Example:

\code{.c}
    #include <leddisplay.h>
    leddisplay_init();
    static leddisplay_frame_t frame;
    leddisplay_frame_clear(&frame);            // clear frame (i.e. fill black)
    leddisplay_frame_fill_rgb(255, 0, 0);      // fill red at full brightness
    leddisplay_frame_xy_rgb(10, 5, 0, 255, 0); // set green pixel at x=10 / y=5
    leddisplay_frame_update(&frame);           // display
\endcode

    @{
*/

//! frame type
typedef union leddisplay_frame_u
{
    //! access RGB pixel by coordinates
    uint8_t yx[LEDDISPLAY_HEIGHT][LEDDISPLAY_WIDTH][3];
    //! access RGB pixel by index
    uint8_t ix[LEDDISPLAY_HEIGHT * LEDDISPLAY_WIDTH][3];
    //! raw frame data
    uint8_t raw[LEDDISPLAY_HEIGHT * LEDDISPLAY_WIDTH * 3];
} leddisplay_frame_t;

//! set pixel to colour
/*!
    Top-left of display is x=0 / y=0.

    \param[in,out] p_frame  pointer to frame memory
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value
*/
void leddisplay_frame_xy_rgb(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue);

//! fill frame with colour
/*!
    \param[in,out] p_frame  pointer to frame memory
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value
*/
void leddisplay_frame_fill_rgb(leddisplay_frame_t *p_frame, uint8_t red, uint8_t green, uint8_t blue);

//! clear frame
/*!
    \param[in,out] p_frame  pointer to frame memory

    This is equivalient ot leddisplay_frame_xy_rgb(p_frame, 0, 0, 0) but faster.
*/
void leddisplay_frame_clear(leddisplay_frame_t *p_frame);

//! update display with frame
/*!
    Renders a frame to the display. The RGB data must be of the size #LEDDISPLAY_WIDTH *
    #LEDDISPLAY_HEIGHT * 3. This will block as necessary until the frame buffer memory
    becomes available.

    \param[in] frame  RGB data for one frame
*/
void leddisplay_frame_update(const leddisplay_frame_t *p_frame);

//@}

/* *********************************************************************************************** */
//@}
#endif // __LEDDISPLAY_H__
