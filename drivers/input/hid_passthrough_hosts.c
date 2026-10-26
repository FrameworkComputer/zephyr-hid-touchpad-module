/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Per-host copies of the touchpad's host-owned feature reports.
 *
 * There is one pad but several hosts: USB stays enumerated while BLE is
 * selected, and every bonded BLE profile stays connected while another one
 * is. Each host's touchpad driver writes the pad's mode reports (PTP Input
 * Mode, Surface/Button Switch, Latency Mode) when it sees fit -- Linux and
 * Windows on connect and around suspend, macOS never. Forwarded straight to
 * the pad, the last writer's mode applied to whichever host was selected: a
 * Mac selected after a Linux or Windows host got PTP frames it has no driver
 * for, so its touchpad was dead, and a Linux host suspending in the
 * background could switch the surface off under the host in use.
 *
 * Instead every host has its own copy of the reports in the touchpad node's
 * host-state-report-ids. A host's SETs and GETs go to its copy; only the
 * selected host's copy reaches the pad, and it is written again whenever
 * the selection changes. A host that never wrote a report gets the pad's
 * power-on value (host-state-report-defaults), which for a PTP pad is mouse
 * mode: what a host without a PTP driver needs.
 *
 * No host re-sends its reports on a switch, since none of them saw a
 * reconnect, so the copies have to last as long as the host's own idea of
 * the pad's state:
 * - USB: until the bus is deconfigured (unplug, host reboot). The host
 *   re-enumerates the pad after that and writes them again.
 * - BLE: until the profile's bond changes. The reports in
 *   host-state-saved-report-ids (Input Mode) are also saved to settings,
 *   once per change, and survive keyboard resets, in case a host doesn't
 *   write them again when it reconnects. The others go back to the
 *   defaults: hosts rewrite them around every suspend and open, so saving
 *   them would cost flash writes every day and could keep a stale "surface
 *   off" from a host that is gone.
 *
 * The pad side is hid_touchpad_retain_feature(), which writes only what the
 * pad doesn't hold yet, leaves a sleeping pad alone and writes the values
 * again after every pad power-on or reset.
 *
 * While a test tool has the pad (tp_hosts_hold), every host's reports go
 * straight to the pad again, as without this file.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/endpoint_changed.h>

#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_USB)
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#endif

#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zmk/ble.h>
#include <zmk/events/ble_active_profile_changed.h>
#define TP_HOSTS_BLE_PROFILES ZMK_BLE_PROFILE_COUNT
#else
#define TP_HOSTS_BLE_PROFILES 0
#endif

#include "hid_touchpad.h"
#include "hid_passthrough_hosts.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hid_passthrough_hosts, CONFIG_HID_PASSTHROUGH_HOSTS_LOG_LEVEL);

#define TP_NODE DT_NODELABEL(touchpad)

BUILD_ASSERT(DT_NODE_HAS_PROP(TP_NODE, host_state_report_defaults),
             "host-state-report-ids needs host-state-report-defaults");
BUILD_ASSERT(DT_PROP_LEN(TP_NODE, host_state_report_ids) ==
                 DT_PROP_LEN(TP_NODE, host_state_report_defaults),
             "host-state-report-defaults must be parallel to host-state-report-ids");
BUILD_ASSERT(DT_PROP_LEN(TP_NODE, host_state_report_ids) <= HID_TOUCHPAD_MAX_RETAINED,
             "more host-state reports than the driver can retain");

static const uint8_t state_ids[] = DT_PROP(TP_NODE, host_state_report_ids);
static const uint8_t state_defaults[] = DT_PROP(TP_NODE, host_state_report_defaults);
#define TP_STATES ARRAY_SIZE(state_ids)

#define TP_HOSTS (1 + TP_HOSTS_BLE_PROFILES)

/* Also the settings value of a BLE host, so all uint8_t: no padding, and a
 * size change (another descriptor) shows up as a length mismatch on load. */
struct tp_host {
#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)
    /* Bond the values belong to; BT_ADDR_LE_ANY until the profile has one. */
    bt_addr_le_t peer;
#endif
    uint8_t val[TP_STATES];
};

static struct tp_host hosts[TP_HOSTS];
/* Held across the pad writes, so a host's SET can't interleave with a
 * switch rewriting the pad. Never taken by the driver. */
static K_MUTEX_DEFINE(hosts_lock);
/* NULL until init found the pad and the reports usable; everything then
 * forwards to the pad as before. */
static const struct device *tp_dev;
/* tp_hosts_hold(): forward everything to the pad as before. Under
 * hosts_lock. */
static bool held;

static int state_index(uint8_t id) {
    for (size_t i = 0; i < TP_STATES; i++) {
        if (state_ids[i] == id) {
            return i;
        }
    }
    return -ENOENT;
}

static int selected_host(void) {
    struct zmk_endpoint_instance ep = zmk_endpoint_get_selected();

    switch (ep.transport) {
    case ZMK_TRANSPORT_USB:
        return TP_HOST_USB;
#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)
    case ZMK_TRANSPORT_BLE:
        return TP_HOST_BLE(ep.ble.profile_index);
#endif
    default:
        return -ENODEV;
    }
}

static void apply_work_cb(struct k_work *work);
static K_WORK_DEFINE(apply_work, apply_work_cb);
/* A pad write that failed (pad claimed by an update check, I2C error) is
 * tried again RETRY_DELAY later, up to RETRY_MAX times in a row; any event
 * that applies the copy starts the count again. */
#define RETRY_DELAY K_SECONDS(1)
#define RETRY_MAX 5
static K_WORK_DELAYABLE_DEFINE(retry_work, apply_work_cb);
static atomic_t retries;

static void retry_later(void) {
    if (atomic_inc(&retries) < RETRY_MAX) {
        k_work_schedule(&retry_work, RETRY_DELAY);
    }
}

static void set_defaults(int host) { memcpy(hosts[host].val, state_defaults, TP_STATES); }

#if IS_ENABLED(CONFIG_SETTINGS) && IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)

static bool is_saved_report(uint8_t id) {
#if DT_NODE_HAS_PROP(TP_NODE, host_state_saved_report_ids)
    static const uint8_t saved_ids[] = DT_PROP(TP_NODE, host_state_saved_report_ids);

    for (size_t i = 0; i < ARRAY_SIZE(saved_ids); i++) {
        if (saved_ids[i] == id) {
            return true;
        }
    }
#endif
    return false;
}

/* What settings keep of a BLE host: its bond and its saved reports, with
 * the rest at the defaults. Caller holds hosts_lock. */
static void saved_view(int host, struct tp_host *out) {
    bt_addr_le_copy(&out->peer, &hosts[host].peer);
    for (size_t i = 0; i < TP_STATES; i++) {
        out->val[i] = is_saved_report(state_ids[i]) ? hosts[host].val[i] : state_defaults[i];
    }
}

/* What settings hold for each BLE host, so only a change is written. Only
 * touched by init, settings load and save_work_cb. */
static struct tp_host stored[TP_HOSTS];

static void save_work_cb(struct k_work *work) {
    for (int host = TP_HOST_BLE(0); host < TP_HOSTS; host++) {
        struct tp_host view;

        k_mutex_lock(&hosts_lock, K_FOREVER);
        saved_view(host, &view);
        k_mutex_unlock(&hosts_lock);

        if (memcmp(&view, &stored[host], sizeof(view)) == 0) {
            continue;
        }
        char name[24];
        snprintf(name, sizeof(name), "tp_host/%d", host - TP_HOST_BLE(0));
        int err = settings_save_one(name, &view, sizeof(view));
        if (err) {
            /* Tried again on the host's next change. */
            LOG_ERR("saving %s failed: %d", name, err);
            continue;
        }
        stored[host] = view;
    }
}

static K_WORK_DEFINE(save_work, save_work_cb);

/* A BLE host's bond or values changed; save it if settings keep the change.
 * Not debounced: what is saved changes about once per bond. */
static void schedule_save(int host) {
    if (host != TP_HOST_USB) {
        k_work_submit(&save_work);
    }
}

static int tp_hosts_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                 void *cb_arg) {
    char *end;
    long profile = strtol(name, &end, 10);
    if (end == name || *end != '\0' || profile < 0 || profile >= TP_HOSTS_BLE_PROFILES) {
        return -ENOENT;
    }
    if (len != sizeof(struct tp_host)) {
        /* Saved for another set of host-state reports; start from defaults. */
        LOG_WRN("ignoring tp_host/%s: %u bytes, expected %u", name, (unsigned int)len,
                (unsigned int)sizeof(struct tp_host));
        return 0;
    }

    int host = TP_HOST_BLE(profile);
    struct tp_host loaded;
    int err = read_cb(cb_arg, &loaded, sizeof(loaded));
    if (err < 0) {
        return err;
    }

    k_mutex_lock(&hosts_lock, K_FOREVER);
    stored[host] = loaded;
    hosts[host] = loaded;
    /* Whatever an older image saved beyond the saved reports. */
    for (size_t i = 0; i < TP_STATES; i++) {
        if (!is_saved_report(state_ids[i])) {
            hosts[host].val[i] = state_defaults[i];
        }
    }
    k_mutex_unlock(&hosts_lock);
    return 0;
}

static int tp_hosts_settings_commit(void) {
    /* The selected host's saved copy, not the defaults, belongs on the pad. */
    k_work_submit(&apply_work);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(tp_host, "tp_host", NULL, tp_hosts_settings_set,
                               tp_hosts_settings_commit, NULL);

static void init_stored(void) {
    for (int host = TP_HOST_BLE(0); host < TP_HOSTS; host++) {
        saved_view(host, &stored[host]);
    }
}

#else

static inline void schedule_save(int host) {}
static inline void init_stored(void) {}

#endif

/* A BLE host's copy belongs to the bond it was made with. Once the profile
 * is cleared or bonded to another host, start that host from the defaults:
 * a Mac paired on a profile a Linux host had must not inherit PTP mode.
 * Copies made before the profile had an address (writes during pairing)
 * are adopted by the bond that pairing produced. */
static void check_bond(int host) {
#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)
    if (host == TP_HOST_USB) {
        return;
    }
    struct tp_host *h = &hosts[host];
    const bt_addr_le_t *peer = zmk_ble_profile_address(host - TP_HOST_BLE(0));

    if (bt_addr_le_cmp(&h->peer, peer) == 0) {
        return;
    }
    if (bt_addr_le_cmp(&h->peer, BT_ADDR_LE_ANY) != 0) {
        LOG_INF("profile %d bond changed, its pad state back to defaults",
                host - TP_HOST_BLE(0));
        set_defaults(host);
    }
    bt_addr_le_copy(&h->peer, peer);
    schedule_save(host);
#endif
}

static bool host_valid(int host) { return host >= 0 && host < TP_HOSTS; }

int tp_hosts_from_conn(struct bt_conn *conn) {
#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        profile = zmk_ble_active_profile_index();
    }
    return TP_HOST_BLE(profile);
#else
    return -ENODEV;
#endif
}

int tp_hosts_set_feature(int host, uint8_t id, const uint8_t *data, uint16_t len) {
    int i = state_index(id);
    if (tp_dev == NULL || i < 0) {
        return -ENOENT;
    }
    if (!host_valid(host) || len < 1) {
        return -EINVAL;
    }

    k_mutex_lock(&hosts_lock, K_FOREVER);
    if (held) {
        k_mutex_unlock(&hosts_lock);
        return -ENOENT;
    }
    check_bond(host);
    if (hosts[host].val[i] != data[0]) {
        LOG_DBG("host %d: feature %u = 0x%02x", host, id, data[0]);
        hosts[host].val[i] = data[0];
        schedule_save(host);
    }
    int err = 0;
    if (host == selected_host()) {
        err = hid_touchpad_retain_feature(tp_dev, id, data[0]);
    }
    k_mutex_unlock(&hosts_lock);
    if (err) {
        /* The host's SET still succeeded: its copy has the value, and a
         * Windows host that saw an error might give up on PTP. */
        LOG_WRN("host %d: writing feature %u failed: %d", host, id, err);
        retry_later();
    }
    return 0;
}

void tp_hosts_hold(bool hold) {
    if (tp_dev == NULL) {
        return;
    }
    k_mutex_lock(&hosts_lock, K_FOREVER);
    held = hold;
    int err = hid_touchpad_hold_retained(tp_dev, hold);
    k_mutex_unlock(&hosts_lock);
    LOG_INF("host state %s", hold ? "held: hosts write the pad directly" : "resumed");
    if (err) {
        LOG_WRN("writing host state on resume failed: %d", err);
    }
}

int tp_hosts_get_feature(int host, uint8_t id, uint8_t *buf, uint16_t buf_len,
                         uint16_t *out_len) {
    int i = state_index(id);
    if (tp_dev == NULL || i < 0) {
        return -ENOENT;
    }
    if (!host_valid(host) || buf_len < 1) {
        return -EINVAL;
    }

    k_mutex_lock(&hosts_lock, K_FOREVER);
    if (held) {
        k_mutex_unlock(&hosts_lock);
        return -ENOENT;
    }
    check_bond(host);
    buf[0] = hosts[host].val[i];
    k_mutex_unlock(&hosts_lock);
    *out_len = 1;
    return 0;
}

/* Put the selected host's copy on the pad. The driver writes only what the
 * pad doesn't hold yet and leaves a sleeping pad alone, so this runs on
 * every event that might change the selection or the selected bond. While
 * held, the driver only records the copy and writes it on release. */
static void apply_work_cb(struct k_work *work) {
    if (tp_dev == NULL) {
        return;
    }

    bool failed = false;
    k_mutex_lock(&hosts_lock, K_FOREVER);
    int host = selected_host();
    if (host_valid(host)) {
        check_bond(host);
        LOG_DBG("pad state from host %d", host);
        for (size_t i = 0; i < TP_STATES; i++) {
            int err = hid_touchpad_retain_feature(tp_dev, state_ids[i], hosts[host].val[i]);
            if (err) {
                /* Retained anyway: written again by the retry, the next
                 * switch or the next pad power-on. */
                LOG_WRN("host %d: writing feature %u failed: %d", host, state_ids[i], err);
                failed = true;
            }
        }
    }
    k_mutex_unlock(&hosts_lock);
    if (failed) {
        retry_later();
    } else {
        atomic_set(&retries, 0);
    }
}

void tp_hosts_usb_reset(void) {
    k_mutex_lock(&hosts_lock, K_FOREVER);
    set_defaults(TP_HOST_USB);
    k_mutex_unlock(&hosts_lock);
    k_work_submit(&apply_work);
}

static int tp_hosts_listener(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_USB)
    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb != NULL && usb->conn_state != ZMK_USB_CONN_HID) {
        /* Deconfigured: whatever enumerates us next writes its own. The
         * USB backend also reports this itself (tp_hosts_usb_reset): this
         * event can miss a reset that was quickly followed by a new
         * configuration. */
        tp_hosts_usb_reset();
    }
#endif
    atomic_set(&retries, 0);
    /* Pad I2C off the raiser's thread. */
    k_work_submit(&apply_work);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(hid_passthrough_hosts, tp_hosts_listener);
ZMK_SUBSCRIPTION(hid_passthrough_hosts, zmk_endpoint_changed);
#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_USB)
ZMK_SUBSCRIPTION(hid_passthrough_hosts, zmk_usb_conn_state_changed);
#endif
#if IS_ENABLED(CONFIG_HID_PASSTHROUGH_BLE)
/* Also raised when the selected profile is cleared or finishes pairing. */
ZMK_SUBSCRIPTION(hid_passthrough_hosts, zmk_ble_active_profile_changed);
#endif

static int tp_hosts_init(void) {
    /* Before settings_load(), which overwrites saved BLE hosts. */
    for (int host = 0; host < TP_HOSTS; host++) {
        set_defaults(host);
    }
    init_stored();

    const struct device *dev = DEVICE_DT_GET(TP_NODE);
    if (!device_is_ready(dev)) {
        LOG_WRN("Touchpad device not ready; host state off");
        return 0;
    }
    for (size_t i = 0; i < TP_STATES; i++) {
        if (hid_touchpad_feature_size(state_ids[i]) != 1) {
            LOG_ERR("host-state report %u is not a 1-byte feature report; host state off",
                    state_ids[i]);
            return 0;
        }
    }
    tp_dev = dev;

    /* Endpoint events and the settings commit apply it again later; this
     * covers a build with neither firing before the first host connects. */
    k_work_submit(&apply_work);
    return 0;
}

SYS_INIT(tp_hosts_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
