/*
 * Copyright (c) 2021 IAW
 *
 * Based on adc_mcux_adc12.c, which is:
 * Copyright (c) 2017-2018, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_adc12

#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <fsl_adc.h>
#include <fsl_power.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_lpc_adc12);
#include "adc_context.h"

struct lpc_adc12_config
{
    ADC_Type* base;
    void (*irq_config_func)(const struct device* dev);
    const struct pinctrl_dev_config *pincfg;
};

struct lpc_adc12_data
{
    const struct device* dev;
    struct adc_context ctx;
    uint16_t* buffer;
    uint16_t* repeat_buffer;
    uint32_t channels;
    uint8_t channel_id;
};

static int lpc_adc12_init(const struct device* dev);

static inline void adc_context_enable_timer(struct adc_context* ctx)
{
}

static inline void adc_context_disable_timer(struct adc_context* ctx)
{
}

static int lpc_adc12_channel_setup(const struct device* dev, const struct adc_channel_cfg* channel_cfg)
{
    if(channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT)
    {
        LOG_ERR("Unsupported channel acquisition time");
        return -ENOTSUP;
    }

    if(channel_cfg->differential)
    {
        LOG_ERR("Differential channels are not supported");
        return -ENOTSUP;
    }

    if(channel_cfg->gain != ADC_GAIN_1)
    {
        LOG_ERR("Unsupported channel gain %d", channel_cfg->gain);
        return -ENOTSUP;
    }

    if(channel_cfg->reference != ADC_REF_VDD_1)
    {
        LOG_ERR("Unsupported channel reference");
        return -ENOTSUP;
    }

    return 0;
}

static int lpc_adc12_start_read(const struct device* dev, const struct adc_sequence* sequence)
{
    struct lpc_adc12_data* data = (struct lpc_adc12_data*)dev->data;
    int error;

    data->buffer = sequence->buffer;
    adc_context_start_read(&data->ctx, sequence);
    error = adc_context_wait_for_completion(&data->ctx);

    return error;
}

static int lpc_adc12_read_async(const struct device* dev, const struct adc_sequence* sequence, struct k_poll_signal* async)
{
    struct lpc_adc12_data* data = (struct lpc_adc12_data*)dev->data;
    int error;

    adc_context_lock(&data->ctx, async ? true : false, async);
    error = lpc_adc12_start_read(dev, sequence);
    adc_context_release(&data->ctx, error);

    return error;
}

static int lpc_adc12_read(const struct device* dev, const struct adc_sequence* sequence)
{
    return lpc_adc12_read_async(dev, sequence, NULL);
}

static void lpc_adc12_start_channel(const struct device* dev)
{
    const struct lpc_adc12_config* config = (const struct lpc_adc12_config*)dev->config;
    struct lpc_adc12_data* data = (struct lpc_adc12_data*)dev->data;
    ADC_Type* base = config->base;
    adc_conv_seq_config_t adc_conv_seq;
    adc_result_info_t adc_result;
    bool res;

    data->channel_id = find_lsb_set(data->channels) - 1;
    adc_conv_seq.channelMask = (1U << data->channel_id);
    adc_conv_seq.triggerMask = 0U;
    adc_conv_seq.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adc_conv_seq.enableSingleStep = false;
    adc_conv_seq.enableSyncBypass = false;
    adc_conv_seq.interruptMode = kADC_InterruptForEachSequence;
    adc_result.result = base->SEQ_GDAT[data->channel_id];

    ADC_ClearStatusFlags(base, kADC_ConvSeqAInterruptFlag);
    ADC_SetConvSeqAConfig(base, &adc_conv_seq);
    ADC_EnableConvSeqA(base, true);
    ADC_DoSoftwareTriggerConvSeqA(base);
    k_busy_wait(1);
    res = ADC_GetChannelConversionResult(base, data->channel_id, &adc_result);
    ADC_EnableConvSeqA(base, false);

    if(res == true)
    {
        *data->buffer++ = adc_result.result;
        data->channels &= ~BIT(data->channel_id);
    }

    if(res == true && data->channels)
    {
        lpc_adc12_start_channel(dev);
    }
    else
    {
        adc_context_on_sampling_done(&data->ctx, dev);

        if(res == false)
        {
            RESET_PeripheralReset(kADC0_RST_SHIFT_RSTn);
            lpc_adc12_init(dev);
        }
    }
}

static void adc_context_start_sampling(struct adc_context* ctx)
{
    struct lpc_adc12_data* data = (struct lpc_adc12_data*)CONTAINER_OF(ctx, struct lpc_adc12_data, ctx);

    data->channels = ctx->sequence.channels;
    data->repeat_buffer = data->buffer;

    lpc_adc12_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context* ctx, bool repeat_sampling)
{
    struct lpc_adc12_data* data = (struct lpc_adc12_data*)CONTAINER_OF(ctx, struct lpc_adc12_data, ctx);

    if(repeat_sampling)
    {
        data->buffer = data->repeat_buffer;
    }
}

static int lpc_adc12_init(const struct device* dev)
{
    const struct lpc_adc12_config* config = (const struct lpc_adc12_config*)dev->config;
    struct lpc_adc12_data* data = (struct lpc_adc12_data*)dev->data;
    ADC_Type* base = (ADC_Type*)config->base;
    adc_config_t adc_config;
    uint32_t frequency = 0;
    int rc;

    base->CTRL |= ADC_CTRL_BYPASSCAL_MASK;
    frequency = CLOCK_GetFreq(kCLOCK_BusClk);
    if(ADC_DoOffsetCalibration(base, frequency) != true)
    {
        LOG_ERR("Failed to calibrate the ADC");
        return -EIO;
    }

    adc_config.clockMode = kADC_ClockSynchronousMode;
    adc_config.clockDividerNumber = 3;
    adc_config.enableBypassCalibration = false;
    adc_config.sampleTimeNumber = 0U;
    adc_config.resolution = kADC_Resolution12bit;

    ADC_Init(base, &adc_config);
    data->dev = dev;
    rc = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
    adc_context_unlock_unconditionally(&data->ctx);

    return rc;
}

static const struct adc_driver_api lpc_adc12_driver_api =
{
    .channel_setup = lpc_adc12_channel_setup,
    .read = lpc_adc12_read,
#ifdef CONFIG_ADC_ASYNC
    .read_async = lpc_adc12_read_async,
#endif
};

#define ACD12_LPC_INIT(n)\
    static void lpc_adc12_config_func_##n(const struct device *dev);\
    PINCTRL_DT_INST_DEFINE(n);\
    static const struct lpc_adc12_config lpc_adc12_config_##n = {\
        .base = (ADC_Type *)DT_INST_REG_ADDR(n),\
        .irq_config_func = lpc_adc12_config_func_##n,\
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),\
    };\
    static struct lpc_adc12_data lpc_adc12_data_##n = {\
        ADC_CONTEXT_INIT_LOCK(lpc_adc12_data_##n, ctx),\
        ADC_CONTEXT_INIT_SYNC(lpc_adc12_data_##n, ctx),\
    };\
    DEVICE_DT_INST_DEFINE(n, &lpc_adc12_init,\
                        NULL, &lpc_adc12_data_##n,\
                        &lpc_adc12_config_##n, POST_KERNEL,\
                        CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
                        &lpc_adc12_driver_api);\
    static void lpc_adc12_config_func_##n(const struct device *dev)	{}

DT_INST_FOREACH_STATUS_OKAY(ACD12_LPC_INIT)
