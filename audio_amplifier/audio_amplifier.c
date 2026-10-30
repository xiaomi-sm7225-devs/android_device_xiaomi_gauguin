/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Load the factory speaker calibration into the CS35L41 CSPL firmware,
 * like the stock audio HAL does in audio_extn_cirrus_boot_thread().
 */

#define LOG_TAG "audio_amplifier"
//#define LOG_NDEBUG 0

#include <endian.h>
#include <errno.h>
#include <hardware/audio_amplifier.h>
#include <hardware/hardware.h>
#include <log/log.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tinyalsa/asoundlib.h>
#include <unistd.h>

#define UNUSED __attribute__((unused))

#define SND_CARD 0
#define CSPL_RETRIES 100
#define CSPL_RETRY_DELAY_US 100000

/* Range the stock audio HAL accepts, (cal_r * 5.85714 / 8192) ohm */
#define CAL_R_MIN 7693
#define CAL_R_MAX 11888

typedef struct cspl_amp {
    const char* prefix;
    const char* cal_r_path;
    uint32_t cal_r;
    bool calibrated;
    bool pending;
} cspl_amp_t;

/* Stock calls the RCV amp left and the SPK amp right */
static cspl_amp_t cspl_amps[] = {
        {
                .prefix = "",
                .cal_r_path = "/mnt/vendor/persist/audio/cs35l41_cal_right.bin",
        },
        {
                .prefix = "RCV ",
                .cal_r_path = "/mnt/vendor/persist/audio/cs35l41_cal.bin",
        },
};

#define NUM_CSPL_AMPS (sizeof(cspl_amps) / sizeof(cspl_amps[0]))

/* The calibration files hold a single native endian 4 byte word */
static bool read_cal_value(const char* path, uint32_t* value) {
    FILE* f = fopen(path, "rb");
    bool ret;

    if (!f) return false;

    ret = fread(value, sizeof(*value), 1, f) == 1;
    if (!ret) ALOGE("%s: Failed to read %s", __func__, path);

    fclose(f);

    if (ret && (*value < CAL_R_MIN || *value > CAL_R_MAX)) {
        ALOGE("%s: Invalid calibration %u in %s", __func__, *value, path);
        ret = false;
    }
    return ret;
}

static struct mixer_ctl* get_amp_ctl(struct mixer* mixer, const cspl_amp_t* amp,
                                     const char* ctl) {
    char name[64];

    snprintf(name, sizeof(name), "%s%s", amp->prefix, ctl);
    return mixer_get_ctl_by_name(mixer, name);
}

static bool set_amp_ctl(struct mixer* mixer, const cspl_amp_t* amp, const char* ctl, int value) {
    struct mixer_ctl* mixer_ctl = get_amp_ctl(mixer, amp, ctl);

    if (!mixer_ctl || mixer_ctl_set_value(mixer_ctl, 0, value)) {
        ALOGE("%s: Failed to set %s%s", __func__, amp->prefix, ctl);
        return false;
    }
    return true;
}

/* CSPL controls are 4 byte big endian words */
static bool get_cspl_ctl(struct mixer* mixer, const cspl_amp_t* amp, const char* ctl,
                         uint32_t* value) {
    char name[64];
    struct mixer_ctl* mixer_ctl;
    uint32_t data;

    snprintf(name, sizeof(name), "%sDSP1 Protection cd %s", amp->prefix, ctl);
    mixer_ctl = mixer_get_ctl_by_name(mixer, name);
    if (!mixer_ctl || mixer_ctl_get_array(mixer_ctl, &data, sizeof(data))) return false;

    *value = be32toh(data);
    return true;
}

static bool has_cal_ctl(struct mixer* mixer, const cspl_amp_t* amp) {
    return get_amp_ctl(mixer, amp, "DSP Set CAL_Z") != NULL;
}

static bool cal_applied(struct mixer* mixer, const cspl_amp_t* amp) {
    uint32_t selected;

    return get_cspl_ctl(mixer, amp, "CAL_R_SELECTED", &selected) && selected == amp->cal_r;
}

static bool dsp_booted(struct mixer* mixer, const cspl_amp_t* amp) {
    struct mixer_ctl* mixer_ctl = get_amp_ctl(mixer, amp, "DSP Booted");

    return mixer_ctl && mixer_ctl_get_value(mixer_ctl, 0) > 0;
}

static struct mixer* wait_for_cal_mixer(void) {
    for (int i = 0; i < CSPL_RETRIES; i++) {
        struct mixer* mixer = mixer_open(SND_CARD);

        if (mixer) {
            for (size_t j = 0; j < NUM_CSPL_AMPS; j++) {
                if (cspl_amps[j].calibrated && has_cal_ctl(mixer, &cspl_amps[j])) return mixer;
            }
            mixer_close(mixer);
        }
        usleep(CSPL_RETRY_DELAY_US);
    }
    return NULL;
}

/*
 * wm_adsp caches CAL_Z and writes CAL_R, CAL_STATUS and CAL_CHECKSUM into
 * CSPL when the DSP boots, so reboot it if the firmware is already running.
 */
static bool write_cal(struct mixer* mixer, cspl_amp_t* amp) {
    if (!set_amp_ctl(mixer, amp, "DSP Set CAL_Z", amp->cal_r)) return false;

    if (!dsp_booted(mixer, amp)) {
        ALOGI("%s: %scalibration queued for next DSP boot (ReDC %u)", __func__, amp->prefix,
              amp->cal_r);
        return true;
    }

    if (!set_amp_ctl(mixer, amp, "DSP Booted", 0) ||
        !set_amp_ctl(mixer, amp, "DSP1 Preload Switch", 0) ||
        !set_amp_ctl(mixer, amp, "DSP1 Preload Switch", 1)) {
        return false;
    }

    amp->pending = true;
    return true;
}

static void* cspl_calibrate_thread(UNUSED void* arg) {
    struct mixer* mixer;
    size_t pending = 0;

    mixer = wait_for_cal_mixer();
    if (!mixer) {
        ALOGE("%s: Calibration controls not found", __func__);
        return NULL;
    }

    for (size_t i = 0; i < NUM_CSPL_AMPS; i++) {
        cspl_amp_t* amp = &cspl_amps[i];

        if (!amp->calibrated) continue;

        if (!has_cal_ctl(mixer, amp)) {
            ALOGI("%s: %sno calibration controls, skipping", __func__, amp->prefix);
        } else if (cal_applied(mixer, amp)) {
            ALOGI("%s: %scalibration already applied (ReDC %u)", __func__, amp->prefix,
                  amp->cal_r);
        } else if (write_cal(mixer, amp) && amp->pending) {
            pending++;
        }
    }
    mixer_close(mixer);

    /* The CSPL controls get re-registered on DSP boot, so reopen the mixer on every check */
    for (int i = 0; i < CSPL_RETRIES && pending; i++) {
        usleep(CSPL_RETRY_DELAY_US);
        mixer = mixer_open(SND_CARD);
        if (!mixer) continue;

        for (size_t j = 0; j < NUM_CSPL_AMPS; j++) {
            cspl_amp_t* amp = &cspl_amps[j];

            if (!amp->pending || !dsp_booted(mixer, amp) || !cal_applied(mixer, amp)) continue;

            ALOGI("%s: %sapplied speaker calibration: ReDC %u", __func__, amp->prefix,
                  amp->cal_r);
            amp->pending = false;
            pending--;
        }
        mixer_close(mixer);
    }

    for (size_t i = 0; i < NUM_CSPL_AMPS; i++) {
        if (cspl_amps[i].pending) {
            ALOGE("%s: %sCSPL did not accept calibration (ReDC %u)", __func__,
                  cspl_amps[i].prefix, cspl_amps[i].cal_r);
        }
    }

    return NULL;
}

static int amp_calibrate(UNUSED amplifier_device_t* device, UNUSED void* adev) {
    bool calibrated = false;
    pthread_attr_t attr;
    pthread_t thread;
    int rc;

    for (size_t i = 0; i < NUM_CSPL_AMPS; i++) {
        cspl_amp_t* amp = &cspl_amps[i];

        amp->calibrated = read_cal_value(amp->cal_r_path, &amp->cal_r);
        calibrated |= amp->calibrated;
    }
    if (!calibrated) {
        ALOGI("%s: Speakers not calibrated, keeping firmware defaults", __func__);
        return 0;
    }

    /* Rebooting the amp DSPs takes a while, don't hold up the audio HAL */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    rc = pthread_create(&thread, &attr, cspl_calibrate_thread, NULL);
    pthread_attr_destroy(&attr);
    if (rc) {
        ALOGE("%s: Failed to create calibration thread: %s", __func__, strerror(rc));
        return -rc;
    }

    return 0;
}

static int amp_dev_close(hw_device_t* device) {
    free(device);

    return 0;
}

static int amp_module_open(const hw_module_t* module, const char* name, hw_device_t** device) {
    amplifier_device_t* amp_dev;

    if (strcmp(name, AMPLIFIER_HARDWARE_INTERFACE)) {
        ALOGE("%s: %s does not match amplifier hardware interface name", __func__, name);
        return -ENODEV;
    }

    amp_dev = calloc(1, sizeof(amplifier_device_t));
    if (!amp_dev) {
        ALOGE("%s: Unable to allocate memory for amplifier device", __func__);
        return -ENOMEM;
    }

    amp_dev->common.tag = HARDWARE_DEVICE_TAG;
    amp_dev->common.module = (hw_module_t*)module;
    amp_dev->common.version = HARDWARE_DEVICE_API_VERSION(1, 0);
    amp_dev->common.close = amp_dev_close;

    amp_dev->calibrate = amp_calibrate;

    *device = (hw_device_t*)amp_dev;

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
        .open = amp_module_open,
};

/* clang-format off */
amplifier_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AMPLIFIER_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AMPLIFIER_HARDWARE_MODULE_ID,
        .name = "CS35L41 audio amplifier HAL",
        .author = "The LineageOS Project",
        .methods = &hal_module_methods,
    },
};
