//
// Copyright (c) 2020 dushin.net
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// References
// https://docs.espressif.com/projects/esp-idf/en/v4.4.4/api-reference/peripherals/adc.html
//

#include "atomvm_adc.h"

#include <context.h>
#include <defaultatoms.h>
#include <interop.h>
#include <nifs.h>
#include <term.h>

// #define ENABLE_TRACE
#include <trace.h>

#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#include <stdlib.h>

#define TAG "adc_resource"

#define CHECK_ERROR(ctx, err, msg)                                                      \
if (UNLIKELY(err != ESP_OK)) {                                                          \
    ESP_LOGE(TAG, msg ": err: %i.", err);                                               \
    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2)) != MEMORY_GC_OK)) {             \
        return OUT_OF_MEMORY_ATOM;                                                      \
    }                                                                                   \
    return create_error_tuple(ctx, esp_err_to_term(ctx->global, err));                  \
}

static ErlNifResourceType *adc_resource_type;

struct ADCResource
{
    adc_unit_t adc_num;
    adc_oneshot_unit_handle_t adc_handle;
};

#define TAG "atomvm_adc"
#define DEFAULT_SAMPLES 64
#define DEFAULT_VREF 1100

static adc_unit_t adc_unit_from_pin(int pin_val)
{
    switch (pin_val) {
#if CONFIG_IDF_TARGET_ESP32
        case 32:
        case 33:
        case 34:
        case 35:
        case 36:
        case 37:
        case 38:
        case 39:
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
#elif CONFIG_IDF_TARGET_ESP32C3
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
#endif
            return ADC_UNIT_1;
#ifdef CONFIG_AVM_ADC2_ENABLE
#if CONFIG_IDF_TARGET_ESP32
        case 0:
        case 2:
        case 4:
        case 12:
        case 13:
        case 14:
        case 15:
        case 25:
        case 26:
        case 27:
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
#elif CONFIG_IDF_TARGET_ESP32C3
        case 5:
#endif
            return ADC_UNIT_2;
#endif
        default:
            return ADC_UNIT_2 + 1;
    }
}

static adc_channel_t get_channel(avm_int_t pin_val)
{
    switch (pin_val) {
#if CONFIG_IDF_TARGET_ESP32
        case 32:
            return ADC_CHANNEL_4;
        case 33:
            return ADC_CHANNEL_5;
        case 34:
            return ADC_CHANNEL_6;
        case 35:
            return ADC_CHANNEL_7;
        case 36:
            return ADC_CHANNEL_0;
        case 37:
            return ADC_CHANNEL_1;
        case 38:
            return ADC_CHANNEL_2;
        case 39:
            return ADC_CHANNEL_3;
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        case 1:
            return ADC_CHANNEL_0;
        case 2:
            return ADC_CHANNEL_1;
        case 3:
            return ADC_CHANNEL_2;
        case 4:
            return ADC_CHANNEL_3;
        case 5:
            return ADC_CHANNEL_4;
        case 6:
            return ADC_CHANNEL_5;
        case 7:
            return ADC_CHANNEL_6;
        case 8:
            return ADC_CHANNEL_7;
        case 9:
            return ADC_CHANNEL_8;
        case 10:
            return ADC_CHANNEL_9;
#elif CONFIG_IDF_TARGET_ESP32C3
        case 0:
            return ADC1_CHANNEL_0;
        case 1:
            return ADC1_CHANNEL_1;
        case 2:
            return ADC1_CHANNEL_2;
        case 3:
            return ADC1_CHANNEL_3;
        case 4:
            return ADC1_CHANNEL_4;
#endif
#ifdef CONFIG_AVM_ADC2_ENABLE
#if CONFIG_IDF_TARGET_ESP32
        case 0:
            return ADC_CHANNEL_1;
        case 2:
            return ADC_CHANNEL_2;
        case 4:
            return ADC_CHANNEL_0;
        case 12:
            return ADC_CHANNEL_5;
        case 13:
            return ADC_CHANNEL_4;
        case 14:
            return ADC_CHANNEL_6;
        case 15:
            return ADC_CHANNEL_3;
        case 25:
            return ADC_CHANNEL_8;
        case 26:
            return ADC_CHANNEL_9;
        case 27:
            return ADC_CHANNEL_7;
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        case 11:
            return ADC_CHANNEL_0;
        case 12:
            return ADC_CHANNEL_1;
        case 13:
            return ADC_CHANNEL_2;
        case 14:
            return ADC_CHANNEL_3;
        case 15:
            return ADC_CHANNEL_4;
        case 16:
            return ADC_CHANNEL_5;
        case 17:
            return ADC_CHANNEL_6;
        case 18:
            return ADC_CHANNEL_7;
        case 19:
            return ADC_CHANNEL_8;
        case 20:
            return ADC_CHANNEL_9;
#elif CONFIG_IDF_TARGET_ESP32C3
        case 5:
            return ADC_CHANNEL_0;
#endif
#endif
        default:
            return ADC_CHANNEL_9 + 1;
    }
}

static const AtomStringIntPair bit_width_table[] = {
    { ATOM_STR("\xA", "bit_defult"), (ADC_BITWIDTH_DEFAULT) },
    { ATOM_STR("\x5", "bit_9"), ADC_BITWIDTH_9 },
    { ATOM_STR("\x6", "bit_10"), ADC_BITWIDTH_10 },
    { ATOM_STR("\x6", "bit_11"), ADC_BITWIDTH_11 },
    { ATOM_STR("\x6", "bit_12"), ADC_BITWIDTH_12 },
    { ATOM_STR("\x6", "bit_13"), ADC_BITWIDTH_13 },
    SELECT_INT_DEFAULT(ADC_BITWIDTH_13 + 1)
};

static const AtomStringIntPair attenuation_table[] = {
    { ATOM_STR("\x4", "db_0"), ADC_ATTEN_DB_0 },
    { ATOM_STR("\x6", "db_2_5"), ADC_ATTEN_DB_2_5 },
    { ATOM_STR("\x4", "db_6"), ADC_ATTEN_DB_6 },
    { ATOM_STR("\x5", "db_12"), ADC_ATTEN_DB_12 },
    SELECT_INT_DEFAULT(ADC_ATTEN_DB_12 + 1)
};

static const char *const invalid_pin_atom   = ATOM_STR("\xb", "invalid_pin");
static const char *const invalid_unit_adc_atom  = ATOM_STR("\xb", "invalid_unit_adc");
static const char *const invalid_width_atom = ATOM_STR("\xd", "invalid_width");
static const char *const invalid_db_atom    = ATOM_STR("\xa", "invalid_db");
static const char *const default_db   = ATOM_STR("\x5", "bit_12");
static const char *const default_width   = ATOM_STR("\xa", "bit_defult");
static const char *const error_read = ATOM_STR("\xA", "error_read");
#ifdef CONFIG_AVM_ADC2_ENABLE
static const char *const timeout_atom = ATOM_STR("\x7", "timeout");
#endif

#define ADC_ATOMSTR (ATOM_STR("\x4", "$adc"))

static term create_pair(Context *ctx, term term1, term term2)
{
    term ret = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(ret, 0, term1);
    term_put_tuple_element(ret, 1, term2);

    return ret;
}

static term create_error_tuple(Context *ctx, term reason)
{
    return create_pair(ctx, ERROR_ATOM, reason);
}

static bool is_adc_resource(GlobalContext *global, term t)
{
    bool ret = term_is_tuple(t)
        && term_get_tuple_arity(t) == 3
        && globalcontext_is_term_equal_to_atom_string(global, term_get_tuple_element(t, 0), ADC_ATOMSTR)
        && term_is_binary(term_get_tuple_element(t, 1))
        && term_is_reference(term_get_tuple_element(t, 2));

    return ret;
}

static bool to_adc_resource(term adc_resource, struct ADCResource **rsrc_obj, Context *ctx)
{
    if (!is_adc_resource(ctx->global, adc_resource)) {
        return false;
    }
    void *rsrc_obj_ptr;
    if (UNLIKELY(!enif_get_resource(erl_nif_env_from_context(ctx), term_get_tuple_element(adc_resource, 1), adc_resource_type, &rsrc_obj_ptr))) {
        return false;
    }
    *rsrc_obj = (struct ADCResource *) rsrc_obj_ptr;

    return true;
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

//
// adc:init_nif/1
//
    static term nif_adc_init(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    GlobalContext *global = ctx->global;
    term adc_unit_value = argv[0];

    adc_unit_t adc_num = ADC_UNIT_1;
    term peripheral = interop_kv_get_value(opts, ATOM_STR("\xA", "peripheral"), global);
    if (!term_is_invalid_term(peripheral)) {
        if (!term_is_integer(peripheral)) {
            ESP_LOGE(TAG, "Invalid parameter: peripheral is not an integer");
            RAISE_ERROR(BADARG_ATOM);
        }
        adc_num = term_to_int32(peripheral) -1;
        if (adc_num < 0 || adc_num > ADC_UNIT_2) {
            ESP_LOGE(TAG, "Invalid parameter: adc_num out of range");
            RAISE_ERROR(BADARG_ATOM);
        }
    }

    adc_oneshot_unit_handle_t adc_handle;

    if (adc_num == ADC_UNIT_1) {
        //-------------ADC1 Init---------------//
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        esp_err_t err = adc_oneshot_new_unit(&init_config, &adc_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ADC parameters.  err=%i", err);
            if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2)) != MEMORY_GC_OK)) {
                ESP_LOGW(TAG, "Failed to allocate memory: %s:%i.\n", __FILE__, __LINE__);
                return OUT_OF_MEMORY_ATOM;
            } else {
                return create_error_tuple(ctx, term_from_int(err));
            }
        }
        ESP_LOGE(TAG, "ADC driver installed using ADC port %i\n, ", adc_num);
    }
#ifdef CONFIG_AVM_ADC2_ENABLE
    else {
         //-------------ADC2 Init---------------//
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_2,
        };
        esp_err_t err = adc_oneshot_new_unit(&init_config, &adc_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ADC parameters.  err=%i", err);
            if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2)) != MEMORY_GC_OK)) {
                ESP_LOGW(TAG, "Failed to allocate memory: %s:%i.\n", __FILE__, __LINE__);
                return OUT_OF_MEMORY_ATOM;
            } else {
                return create_error_tuple(ctx, term_from_int(err));
            }
        }
         ESP_LOGE(TAG, "ADC driver installed using ADC port %i\n, ", adc_num);
    }
#endif

    //
    // allocate and initialize the Nif resource
    //

    struct ADCResource *rsrc_obj = enif_alloc_resource(adc_resource_type, sizeof(struct ADCResource));
    if (IS_NULL_PTR(rsrc_obj)) {
        adc_driver_delete(adc_num);
        ESP_LOGW(TAG, "Failed to allocate memory: %s:%i.\n", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    rsrc_obj->adc_handle = adc_handle;
    rsrc_obj->adc_num = adc_num;


    if (UNLIKELY(memory_ensure_free(ctx, TERM_BOXED_RESOURCE_SIZE) != MEMORY_GC_OK)) {
        adc_driver_delete(adc_num);
        enif_release_resource(rsrc_obj);
        ESP_LOGW(TAG, "Failed to allocate memory: %s:%i.\n", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    term obj = enif_make_resource(erl_nif_env_from_context(ctx), rsrc_obj);
    enif_release_resource(rsrc_obj);

    //
    // Return result
    //

    // {'$adc', Resource :: resource(), Ref :: reference()} :: adc()
    size_t requested_size = TUPLE_SIZE(3) + REF_SIZE;
    if (UNLIKELY(memory_ensure_free_with_roots(ctx, requested_size, 1, &obj, MEMORY_CAN_SHRINK) != MEMORY_GC_OK)) {
        adc_driver_delete(adc_num);
        ESP_LOGW(TAG, "Failed to allocate memory: %s:%i.\n", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }

    term adc_term = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(adc_term, 0, globalcontext_make_atom(global, ADC_ATOMSTR));
    term_put_tuple_element(adc_term, 1, obj);
    uint64_t ref_ticks = globalcontext_get_ref_ticks(ctx->global);
    term ref = term_from_ref_ticks(ref_ticks, &ctx->heap);
    term_put_tuple_element(adc_term, 2, ref);

    return adc_term;
}

//
// a2d:close_nif/1
//

static term nif_adc_close(Context *ctx, int argc, term argv[])
{
    TRACE("nif_close\n");
    UNUSED(argc);

    //
    // extract the resource
    //
    term adc_resource = argv[0];
    struct ADCResource *rsrc_obj;

    if (UNLIKELY(!to_adc_resource(adc_resource, &rsrc_obj, ctx))) {
        ESP_LOGE(TAG, "Failed to convert adc_resource");
        RAISE_ERROR(BADARG_ATOM);
    }

    esp_err_t err;

    err = adc_driver_delete(rsrc_obj->adc_enum);
    CHECK_ERROR(ctx, err, "nif_close; Failed to delete driver");

    return OK_ATOM;
}

static term nif_config_channel_bitwidth_atten(Context *ctx, int argc, term argv[])
{
    TRACE("config_channel_bitwidth_atten_nif\n");
    UNUSED(argc);
    GlobalContext *global = ctx->global;

    //
    // extract the resource
    //
    term adc_resource = argv[0];
    struct ADCResource *rsrc_obj;
    if (UNLIKELY(!to_adc_resource(adc_resource, &rsrc_obj, ctx))) {
        ESP_LOGE(TAG, "Failed to convert adc_resource");
        RAISE_ERROR(BADARG_ATOM);
    }

    term pin = argv[1];
    VALIDATE_VALUE(pin, term_is_integer);
    adc_channel_t channel = get_channel(term_to_int(pin));
    if (UNLIKELY(channel == ADC_CHANNEL_9 + 1)) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_pin_atom));
        }
    }

    term config_options = argv[2];
    VALIDATE_VALUE(config_options, term_is_list);

    term bitwidth = interop_kv_get_value_default(config_options, ATOM_STR("\x8", "bitwidth"), default_width, ctx->global);
    VALIDATE_VALUE(bitwidth, term_is_atom);
    adc_bits_width_t bit_width = interop_atom_term_select_int(bit_width_table, bitwidth, ctx->global);

    term attenuation = interop_kv_get_value_default(config_options, ATOM_STR("\x5", "atten"), default_db, ctx->global);
    VALIDATE_VALUE(attenuation, term_is_atom);
    adc_atten_t atten = interop_atom_term_select_int(attenuation_table, attenuation, ctx->global);

    if (UNLIKELY(bit_width == ADC_BITWIDTH_13 + 1)) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_width_atom));
        }
    }

    if (UNLIKELY(atten == ADC_ATTEN_DB_12 + 1)) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_db_atom));
        }
    }

    adc_unit_t adc_unit = adc_unit_from_pin(term_to_int(pin));
    if (UNLIKELY(adc_unit != rsrc_obj->adc_num)) {
        int Pin = term_to_int(pin);
        TRACE("Pin %i is not a valid adc pin.\n", Pin);
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_pin_atom));
        }
    }

    //-------------ADC Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = bit_width,
        .atten = atten,
    };
    esp_err_t err = adc_oneshot_config_channel(rsrc_obj->adc_handle, channel, &config);

    CHECK_ERROR(ctx, err, "config_channel_bitwidth_atten_nif; adc_oneshot_config_channel");

    return OK_ATOM;
}

static term nif_config_channel_calibration(Context *ctx, int argc, term argv[])
{
    TRACE("config_channel_calibration_nif\n");
    UNUSED(argc);
    GlobalContext *global = ctx->global;

    //
    // extract the resource
    //
    term adc_resource = argv[0];
    struct ADCResource *rsrc_obj;
    if (UNLIKELY(!to_adc_resource(adc_resource, &rsrc_obj, ctx))) {
        ESP_LOGE(TAG, "Failed to convert adc_resource");
        RAISE_ERROR(BADARG_ATOM);
    }

    term pin = argv[1];
    VALIDATE_VALUE(pin, term_is_integer);
    adc_channel_t channel = get_channel(term_to_int(pin));
    if (UNLIKELY(channel == ADC_CHANNEL_9 + 1)) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_pin_atom));
        }
    }

    term config_options = argv[2];
    VALIDATE_VALUE(config_options, term_is_list);

    term attenuation = interop_kv_get_value_default(config_options, ATOM_STR("\x5", "atten"), default_db, ctx->global);
    VALIDATE_VALUE(attenuation, term_is_atom);
    adc_atten_t atten = interop_atom_term_select_int(attenuation_table, attenuation, ctx->global);

    if (UNLIKELY(atten == ADC_ATTEN_DB_12 + 1)) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_db_atom));
        }
    }

    adc_unit_t adc_unit = adc_unit_from_pin(term_to_int(pin));
    if (UNLIKELY(adc_unit != rsrc_obj->adc_num)) {
        int Pin = term_to_int(pin);
        TRACE("Pin %i is not a valid adc pin.\n", Pin);
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_pin_atom));
        }
    }

    //-------------ADC Calibration Init---------------//
    adc_cali_handle_t adc_cali_chan_handle = NULL;
    bool do_calibration = adc_calibration_init(rsrc_obj->adc_enum, channel, atten, &adc_cali_chan_handle);

    esp_err_t err;

    if(do_calibration)
        err = ESP_OK;
    else
        err = ESP_FAIL;

    CHECK_ERROR(ctx, err, "config_channel_calibration_nif; ADC Calibration");

    return OK_ATOM;
}

static term nif_adc_take_reading(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    GlobalContext *global = ctx->global;

    //
    // extract the resource
    //
    term adc_resource = argv[0];
    struct ADCResource *rsrc_obj;
    if (UNLIKELY(!to_adc_resource(adc_resource, &rsrc_obj, ctx))) {
        ESP_LOGE(TAG, "Failed to convert adc_resource");
        RAISE_ERROR(BADARG_ATOM);
    }

    term pin = argv[1];
    VALIDATE_VALUE(pin, term_is_integer);
    adc_channel_t channel = get_channel(term_to_int(pin));
    if (UNLIKELY(channel == ADC_CHANNEL_9 + 1)) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_pin_atom));
        }
    }

    term config_options = argv[2];
    VALIDATE_VALUE(config_options, term_is_list);

    term samples = interop_kv_get_value_default(config_options, ATOM_STR("\x7", "samples"), DEFAULT_SAMPLES, ctx->global);
    avm_int_t samples_val = term_to_int(samples);
    term raw = interop_kv_get_value_default(config_options, ATOM_STR("\x3", "raw"), FALSE_ATOM, ctx->global);
    term voltage = interop_kv_get_value_default(config_options, ATOM_STR("\x7", "voltage"), FALSE_ATOM, ctx->global);

    adc_unit_t adc_unit = adc_unit_from_pin(term_to_int(pin));
    if (UNLIKELY(adc_unit != rsrc_obj->adc_num)) {
        int Pin = term_to_int(pin);
        TRACE("Pin %i is not a valid adc pin.\n", Pin);
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, invalid_pin_atom));
        }
    }

    uint32_t AdcRawValueChannel = 0;
    uint32_t AdcVoltageChannel = 0;

    uint32_t adc_reading = 0;

    esp_err_t err;

    
    for (avm_int_t i = 0; i < samples_val; ++i) {
            err = adc_oneshot_read(rsrc_obj->adc_handle, channel, &AdcRawValueChannel)
            adc_reading += AdcRawValueChannel;
        }

    if (UNLIKELY(err != ESP_OK)) {
            if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
                RAISE_ERROR(OUT_OF_MEMORY_ATOM);
            } else {
                return create_pair(ctx, ERROR_ATOM, globalcontext_make_atom(ctx->global, error_read));
            }
        }

    adc_reading /= samples_val;
    TRACE("take_reading adc_reading: %i\n", adc_reading);

    raw = raw == TRUE_ATOM ? term_from_int32(adc_reading) : UNDEFINED_ATOM;
    if (voltage == TRUE_ATOM) {
        adc_cali_raw_to_voltage(rsrc_obj->adc_handle, channel, &AdcVoltageChannel);
        voltage = term_from_int32(AdcVoltageChannel);
    } else {
        voltage = UNDEFINED_ATOM;
    };

    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    } else {
        return create_pair(ctx, raw, voltage);
    }
}

static const struct Nif adc_init_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_init
};
static const struct Nif adc_close_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_close
};
static const struct Nif config_channel_bitwidth_atten_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_config_channel_bitwidth_atten
};
static const struct Nif config_channel_calibration_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_config_channel_calibration
};
static const struct Nif adc_take_reading_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_take_reading
};

//
// entrypoints
//

static void adc_resource_dtor(ErlNifEnv *caller_env, void *obj)
{
    UNUSED(caller_env);
    struct ADCResource *rsrc_obj = (struct ADCResource *) obj;

    esp_err_t err = adc_driver_delete(rsrc_obj->adc_num);
    if (UNLIKELY(err != ESP_OK)) {
        TRACE("Failed to delete driver in resource d'tor.  err=%i", err);
    }
}

static const ErlNifResourceTypeInit ADCResourceTypeInit = {
    .members = 1,
    .dtor = adc_resource_dtor,
};

//
// Component Nif Entrypoints
//

void atomvm_adc_init(GlobalContext *global)
{

    ErlNifEnv env;
    erl_nif_env_partial_init_from_globalcontext(&env, global);
    adc_resource_type = enif_init_resource_type(&env, "adc_resource", &ADCResourceTypeInit, ERL_NIF_RT_CREATE, NULL);

}

const struct Nif *atomvm_adc_get_nif(const char *nifname)
{

    TRACE("Locating nif %s ...", nifname);
    if (strcmp("adc:nif_init/1", nifname) == 0) {
        TRACE("Resolved platform nif %s ...\n", nifname);
        return &adc_init_nif;
    }
    if (strcmp("adc:nif_closef/1", nifname) == 0) {
        TRACE("Resolved platform nif %s ...\n", nifname);
        return &adc_close_nif;
    }
    if (strcmp("adc:nif_config_channel_bitwidth_atten/3", nifname) == 0) {
        TRACE("Resolved platform nif %s ...\n", nifname);
        return &config_channel_bitwidth_atten_nif;
    }
    if (strcmp("adc:nif_config_channel_calibration/3", nifname) == 0) {
        TRACE("Resolved platform nif %s ...\n", nifname);
        return &config_channel_calibration_nif;
    }
    if (strcmp("adc:nif_take_reading/3", nifname) == 0) {
        TRACE("Resolved platform nif %s ...\n", nifname);
        return &adc_take_reading_nif;
    }
    return NULL;
}

#include <sdkconfig.h>
#ifdef CONFIG_AVM_ADC_ENABLE
REGISTER_NIF_COLLECTION(atomvm_adc, atomvm_adc_init, NULL, atomvm_adc_get_nif)
#endif
