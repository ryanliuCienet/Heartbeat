#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_CHANNEL_ID 1
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT

#define SAMP_SIZE 5
#define RISE_THRESHOLD 5
#define SAMPLING_PERIOD_MS 17  // ~16.667ms for 60Hz noise elimination

static const struct adc_channel_cfg m_1st_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput1
#endif
};

static int16_t m_sample_buffer[1];
static struct adc_sequence sequence = {
    .channels = BIT(ADC_CHANNEL_ID),
    .buffer = m_sample_buffer,
    .buffer_size = sizeof(m_sample_buffer),
    .resolution = ADC_RESOLUTION,
};

void print_adc_config(const struct adc_sequence *seq) {
    printk("ADC Configuration:\n");
    printk("  Channels: 0x%x\n", seq->channels);
    printk("  Buffer size: %d\n", seq->buffer_size);
    printk("  Resolution: %d\n", seq->resolution);
    if (seq->oversampling) {
        printk("  Oversampling: %d\n", seq->oversampling);
    }
    if (seq->calibrate) {
        printk("  Calibration enabled\n");
    }
}

void main(void)
{
    int ret;
    const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
    
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return;
    }

    ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
    if (ret != 0) {
        printk("Error in adc setup: %d\n", ret);
        return;
    }

    print_adc_config(&sequence);

    int n = 0, rise_count = 0;
    int64_t ptr = 0, last_beat = 0;
    bool rising = false;
    float last = 0, reader = 0, reads[SAMP_SIZE] = {0}, now = 0, start = 0, sum = 0;
    float first = 0, second = 0, third = 0, before = 0, print_value = 0;

    while (1) {
        n = 0;
        start = k_uptime_get_32();
        reader = 0;

        // Sample for about 16.667ms
        while (k_uptime_get_32() - start < SAMPLING_PERIOD_MS) {
            ret = adc_read(adc_dev, &sequence);
            if (ret != 0) {
                printk("Error in adc read: %d\n", ret);
                printk("ADC value: %d\n", m_sample_buffer[0]);
                k_msleep(1000);  // Wait for 1 second before retrying
                continue;
            }
            reader += m_sample_buffer[0];
            n++;
        }

        if (n == 0) {
            printk("No samples collected in this period\n");
            continue;
        }

        reader /= n;
        sum -= reads[ptr];
        sum += reader;
        reads[ptr] = reader;
        last = sum / SAMP_SIZE;

        // Check for a rising curve (a heart beat)
        if (last > before) {
            rise_count++;
            if (!rising && rise_count > RISE_THRESHOLD) {
                rising = true;
                first = k_uptime_get_32() - last_beat;
                last_beat = k_uptime_get_32();

                print_value = 60000. / (0.4 * first + 0.3 * second + 0.3 * third);

                printk("Heartbeat = %d, ADC value = %d\n", (int)print_value, (int)last);

                third = second;
                second = first;
            }
        } else {
            rising = false;
            rise_count = 0;
        }

        before = last;
        ptr++;
        ptr %= SAMP_SIZE;

        k_msleep(10);  // Small delay to prevent busy-waiting
    }
}