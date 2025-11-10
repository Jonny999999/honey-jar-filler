#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "hx711.h"   // external HX711 component
#include "sdkconfig.h"

// Define GPIO pins the module is connected
#ifndef CONFIG_HX711_DT_GPIO
#define CONFIG_HX711_DT_GPIO  GPIO_NUM_12
#endif

#ifndef CONFIG_HX711_SCK_GPIO
#define CONFIG_HX711_SCK_GPIO GPIO_NUM_13
#endif


// Struct that keeps both the HX711 device and calibration data
typedef struct {
    hx711_t dev;         // underlying hx711 driver handle
    int32_t offset_raw;  // tare offset (raw counts at empty)
    float   scale_cpg;   // counts per gram
    bool    calibrated;  // true after scale is known
} scale_hx711_t;


/**
 * @brief Initialize HX711 interface and prepare for reading.
 *
 *  - sets pins (DT, SCK)
 *  - sets gain (Channel A, gain 128)
 *  - waits once for data ready so the chip is alive
 *
 * DOES NOT tare or calibrate.
 */
esp_err_t scale_hx711_init(scale_hx711_t *s);


/**
 * @brief Tare (zero) the scale.
 *
 * Reads 'samples' raw values with no load and stores the average as offset_raw.
 * After this, weight with no jar should read ~0 g (once you also set scale).
 *
 * Typical samples: 16 or 32 for smooth offset.
 */
esp_err_t scale_hx711_tare(scale_hx711_t *s, uint16_t samples);


/**
 * @brief Calibrate/compute scale factor.
 *
 * Place a known reference mass (grams_on_scale grams),
 * then call this. It will:
 *  - read 'samples' raw average
 *  - subtract the stored offset_raw
 *  - compute scale_cpg = counts_per_gram
 *
 * After this, s->calibrated = true.
 */
esp_err_t scale_hx711_calibrate(scale_hx711_t *s,
                                float grams_on_scale,
                                uint16_t samples);


/**
 * @brief Read current raw ADC average.
 *
 * Returns averaged HX711 reading in *raw_out.
 * This is the unscaled integer (24-bit signed).
 */
esp_err_t scale_hx711_read_raw(scale_hx711_t *s,
                               uint16_t samples,
                               int32_t *raw_out);


/**
 * @brief Read current weight in grams.
 *
 * - reads averaged raw
 * - subtracts offset_raw
 * - divides by scale_cpg
 *
 * If not yet calibrated, grams_out will still be computed using whatever
 * scale_cpg is (default 1.0f) and calibrated=false means "not meaningful".
 */
esp_err_t scale_hx711_read_grams(scale_hx711_t *s,
                                 uint16_t samples,
                                 int32_t *raw_out,
                                 float *grams_out,
                                 bool  *is_valid);
