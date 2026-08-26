/*
  Simple DirectMedia Layer
  Copyright (C) 1997-2026 Sam Lantinga <slouken@libsdl.org>

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/
#include "SDL_internal.h"

#ifdef SDL_JOYSTICK_HIDAPI

#include "../SDL_sysjoystick.h"
#include "SDL_hidapijoystick_c.h"

#ifdef SDL_JOYSTICK_HIDAPI_PINSCAPE

/* Pinscape Controller, a virtual pinball cabinet I/O board.
 *
 * The reason this device needs a driver of its own is not its report format,
 * which is an ordinary joystick report, but the meaning of two of its axes.
 * X and Y carry accelerometer readings for cabinet nudging, and the physical
 * acceleration a full-scale reading stands for is a firmware setting: the same
 * board, with the same accelerometer chip, reports +/-1 g or +/-2 g or more
 * depending on how it was configured. Nothing in the HID descriptor says which,
 * so an application receives a number with no unit and has to ask the user what
 * it means.
 *
 * The board does know, and will say so over its own HID protocol. This driver
 * asks at startup and publishes the answer as a joystick property, so that
 * applications can convert axis readings to m/s^2 without involving the user.
 *
 * Protocol reference: mjrgh/Pinscape_Controller, USBProtocol.h.
 */

#define PINSCAPE_VENDOR_ID  0x1209
#define PINSCAPE_PRODUCT_ID 0xEAEA

/* Extended message prefix, and the subtypes used here. */
#define PINSCAPE_CMD_EXTENDED   65
#define PINSCAPE_SUB_QUERY_VAR   9
#define PINSCAPE_SUB_BUILD_INFO 10

/* Configuration variable holding the accelerometer settings. */
#define PINSCAPE_VAR_ACCELEROMETER 4

/* Reply headers, little-endian in the first two bytes of a report. */
#define PINSCAPE_REPLY_CONFIG 0x9800
#define PINSCAPE_REPLY_BUILD  0xA000

/* The board's reports are 8 bytes for replies, 22 for joystick state. */
#define PINSCAPE_REPORT_SIZE 32

/* Axis full scale, from the HID descriptor: logical -4096..+4096. */
#define PINSCAPE_AXIS_FULL_SCALE 4096

/* The board streams joystick reports continuously, so a reply to a query
 * arrives among them rather than first. */
#define PINSCAPE_REPLY_ATTEMPTS 64

typedef struct
{
    SDL_JoystickID joystick;
    bool have_accel_range;
    float accel_full_scale; /* m/s^2 at a full-scale axis reading */
    Uint16 firmware_version;
    Uint8 last_state[PINSCAPE_REPORT_SIZE];
} SDL_DriverPinscape_Context;

static void HIDAPI_DriverPinscape_RegisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_AddHintCallback(SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE, callback, userdata);
}

static void HIDAPI_DriverPinscape_UnregisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_RemoveHintCallback(SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE, callback, userdata);
}

static bool HIDAPI_DriverPinscape_IsEnabled(void)
{
    return SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE,
                              SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI, SDL_HIDAPI_DEFAULT));
}

static bool HIDAPI_DriverPinscape_IsSupportedDevice(SDL_HIDAPI_Device *device, const char *name, SDL_GamepadType type, Uint16 vendor_id, Uint16 product_id, Uint16 version, int interface_number, int interface_class, int interface_subclass, int interface_protocol)
{
    if (vendor_id != PINSCAPE_VENDOR_ID || product_id != PINSCAPE_PRODUCT_ID) {
        return false;
    }

    /* Both Pinscape generations answer on these ids and share nothing else.
     * The Pico speaks a different protocol entirely, and says so in its
     * product string. */
    if (name && SDL_strcasestr(name, "pico")) {
        return false;
    }
    return true;
}

/* Read one reply of the given kind, ignoring the joystick reports in between.
 *
 * Returns the number of bytes read, or -1. The board answers within a few
 * milliseconds when it answers at all; a variable it does not recognise simply
 * produces nothing, which is why this gives up rather than blocking.
 */
static int ReadReply(SDL_HIDAPI_Device *device, Uint16 header, Uint8 *data, size_t size)
{
    int attempt;

    for (attempt = 0; attempt < PINSCAPE_REPLY_ATTEMPTS; ++attempt) {
        int size_read = SDL_hid_read_timeout(device->dev, data, size, 10);
        if (size_read < 0) {
            return -1;
        }
        if (size_read >= 8 && (Uint16)(data[0] | (data[1] << 8)) == header) {
            return size_read;
        }
    }
    return -1;
}

/* Ask the board for its accelerometer configuration.
 *
 * Byte 3 of the reply is the orientation, byte 4 the dynamic range, byte 5 the
 * auto-centring time. The range codes are 0 for +/-1 g, then 1, 2 and 3 for
 * 2 g, 4 g and 8 g. The 1 g setting is not a hardware mode: the chip runs at
 * 2 g and the firmware halves the reported scale, which is exactly why this
 * cannot be guessed from the accelerometer part number.
 *
 * Only the range is of any use here. The orientation says how the board is
 * mounted in the cabinet, and the firmware has already applied it by the time
 * anything is reported, so the axis values arrive in the cabinet's frame. It
 * changes nothing a host can observe, whereas the range changes what the
 * numbers mean.
 */
static bool QueryAccelRange(SDL_HIDAPI_Device *device, float *full_scale)
{
    Uint8 request[9];
    Uint8 reply[PINSCAPE_REPORT_SIZE];
    static const float ranges_in_g[4] = { 1.0f, 2.0f, 4.0f, 8.0f };

    SDL_zeroa(request);
    request[0] = 0x00; /* report id: the board uses unnumbered reports */
    request[1] = PINSCAPE_CMD_EXTENDED;
    request[2] = PINSCAPE_SUB_QUERY_VAR;
    request[3] = PINSCAPE_VAR_ACCELEROMETER;
    request[4] = 0x00; /* array index, unused for a scalar variable */

    if (SDL_hid_write(device->dev, request, sizeof(request)) < 0) {
        return false;
    }
    if (ReadReply(device, PINSCAPE_REPLY_CONFIG, reply, sizeof(reply)) < 0) {
        return false;
    }
    if (reply[2] != PINSCAPE_VAR_ACCELEROMETER) {
        return false;
    }

    *full_scale = ranges_in_g[reply[4] & 0x03] * SDL_STANDARD_GRAVITY;
    return true;
}

/* Ask the board when it was built.
 *
 * The reply carries the build date as a decimal YYYYMMDD and the time as
 * HHMMSS, both 32-bit little-endian. This firmware has no version number of
 * its own — the official builds are named by date — so the date is what a
 * caller compares against the latest release to know whether a board is up to
 * date.
 *
 * SDL reports a firmware version in 16 bits and a date needs 25, so it is
 * packed: year since 2000 in the top 7 bits, month in the next 4, day in the
 * low 5. That fits exactly, and the date can be read back out of it, which a
 * decimal truncation would not allow.
 */
static bool QueryBuildVersion(SDL_HIDAPI_Device *device, Uint16 *version)
{
    Uint8 request[9];
    Uint8 reply[PINSCAPE_REPORT_SIZE];
    Uint32 date;

    SDL_zeroa(request);
    request[1] = PINSCAPE_CMD_EXTENDED;
    request[2] = PINSCAPE_SUB_BUILD_INFO;

    if (SDL_hid_write(device->dev, request, sizeof(request)) < 0) {
        return false;
    }
    if (ReadReply(device, PINSCAPE_REPLY_BUILD, reply, sizeof(reply)) < 0) {
        return false;
    }

    date = (Uint32)reply[2] | ((Uint32)reply[3] << 8) | ((Uint32)reply[4] << 16) | ((Uint32)reply[5] << 24);
    {
        const Uint32 year = date / 10000;
        const Uint32 month = (date / 100) % 100;
        const Uint32 day = date % 100;

        if (year < 2000 || year > 2127 || month < 1 || month > 12 || day < 1 || day > 31) {
            return false;
        }
        *version = (Uint16)(((year - 2000) << 9) | (month << 5) | day);
    }
    return true;
}

static bool HIDAPI_DriverPinscape_InitDevice(SDL_HIDAPI_Device *device)
{
    SDL_DriverPinscape_Context *ctx;

    ctx = (SDL_DriverPinscape_Context *)SDL_calloc(1, sizeof(*ctx));
    if (!ctx) {
        return false;
    }
    device->context = ctx;

    /* Both queries are best-effort: an older build may not answer the second,
     * and a board is still perfectly usable without either. */
    ctx->have_accel_range = QueryAccelRange(device, &ctx->accel_full_scale);
    QueryBuildVersion(device, &ctx->firmware_version);

    return HIDAPI_JoystickConnected(device, NULL);
}

static int HIDAPI_DriverPinscape_GetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id)
{
    return -1;
}

static void HIDAPI_DriverPinscape_SetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id, int player_index)
{
}

static bool HIDAPI_DriverPinscape_OpenJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    SDL_DriverPinscape_Context *ctx = (SDL_DriverPinscape_Context *)device->context;

    SDL_AssertJoysticksLocked();

    ctx->joystick = joystick->instance_id;
    SDL_zeroa(ctx->last_state);

    /* The report descriptor declares seven axes and thirty-two buttons,
     * whatever the cabinet actually has wired: unused buttons simply never
     * change state. */
    joystick->naxes = 7;
    joystick->nbuttons = 32;
    joystick->nhats = 0;

    joystick->firmware_version = ctx->firmware_version;

    if (ctx->have_accel_range) {
        SDL_PropertiesID props = SDL_GetJoystickProperties(joystick);
        SDL_SetFloatProperty(props, SDL_PROP_JOYSTICK_ACCEL_FULL_SCALE_FLOAT, ctx->accel_full_scale);
    }
    return true;
}

static bool HIDAPI_DriverPinscape_RumbleJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 low_frequency_rumble, Uint16 high_frequency_rumble)
{
    return SDL_Unsupported();
}

static bool HIDAPI_DriverPinscape_RumbleJoystickTriggers(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 left_rumble, Uint16 right_rumble)
{
    return SDL_Unsupported();
}

static Uint32 HIDAPI_DriverPinscape_GetJoystickCapabilities(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    return 0;
}

static bool HIDAPI_DriverPinscape_SetJoystickLED(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint8 red, Uint8 green, Uint8 blue)
{
    return SDL_Unsupported();
}

static bool HIDAPI_DriverPinscape_SendJoystickEffect(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, const void *data, int size)
{
    return SDL_Unsupported();
}

static bool HIDAPI_DriverPinscape_SetJoystickSensorsEnabled(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, bool enabled)
{
    return SDL_Unsupported();
}

/* Decode one joystick report.
 *
 * Layout, from USBProtocol.h: four status bytes, four bytes of buttons one bit
 * each, then seven 16-bit axes, all little-endian. The status bytes carry
 * plunger and night-mode flags that mean nothing to a joystick API, so they
 * are read past.
 */
static void HandleStatePacket(SDL_Joystick *joystick, SDL_DriverPinscape_Context *ctx, Uint8 *data, int size)
{
    Uint64 timestamp = SDL_GetTicksNS();
    int axis, button;

    if (size < 22) {
        return;
    }

    for (button = 0; button < 32; ++button) {
        const int byte = 4 + (button / 8);
        const Uint8 mask = (Uint8)(1 << (button % 8));
        if ((data[byte] & mask) != (ctx->last_state[byte] & mask)) {
            SDL_SendJoystickButton(timestamp, joystick, (Uint8)button,
                                   ((data[byte] & mask) != 0));
        }
    }

    for (axis = 0; axis < 7; ++axis) {
        const int at = 8 + axis * 2;
        Sint16 value = (Sint16)(data[at] | (data[at + 1] << 8));

        /* The axes run -4096..+4096, so they are scaled to the range SDL
         * reports. The property published at open time says what a full-scale
         * reading means in m/s^2 for the two nudge axes. */
        Sint32 scaled = ((Sint32)value * SDL_JOYSTICK_AXIS_MAX) / PINSCAPE_AXIS_FULL_SCALE;
        if (scaled > SDL_JOYSTICK_AXIS_MAX) {
            scaled = SDL_JOYSTICK_AXIS_MAX;
        } else if (scaled < SDL_JOYSTICK_AXIS_MIN) {
            scaled = SDL_JOYSTICK_AXIS_MIN;
        }
        SDL_SendJoystickAxis(timestamp, joystick, (Uint8)axis, (Sint16)scaled);
    }

    SDL_memcpy(ctx->last_state, data, SDL_min((size_t)size, sizeof(ctx->last_state)));
}

static bool HIDAPI_DriverPinscape_UpdateDevice(SDL_HIDAPI_Device *device)
{
    SDL_DriverPinscape_Context *ctx = (SDL_DriverPinscape_Context *)device->context;
    SDL_Joystick *joystick = NULL;
    Uint8 data[PINSCAPE_REPORT_SIZE];
    int size = 0;

    if (device->num_joysticks > 0) {
        joystick = SDL_GetJoystickFromID(ctx->joystick);
    } else {
        return false;
    }

    while ((size = SDL_hid_read_timeout(device->dev, data, sizeof(data), 0)) > 0) {
        if (!joystick) {
            continue;
        }
        /* Replies to our own queries share the pipe with state reports; they
         * are told apart by their header, which no joystick report uses. */
        const Uint16 header = (Uint16)(data[0] | (data[1] << 8));
        if (header == PINSCAPE_REPLY_CONFIG || header == PINSCAPE_REPLY_BUILD ||
            (header & 0xF000) == 0x9000) {
            continue;
        }
        HandleStatePacket(joystick, ctx, data, size);
    }

    if (size < 0) {
        HIDAPI_JoystickDisconnected(device, ctx->joystick);
    }
    return (size >= 0);
}

static void HIDAPI_DriverPinscape_CloseJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    SDL_DriverPinscape_Context *ctx = (SDL_DriverPinscape_Context *)device->context;

    ctx->joystick = 0;
}

static void HIDAPI_DriverPinscape_FreeDevice(SDL_HIDAPI_Device *device)
{
}

SDL_HIDAPI_DeviceDriver SDL_HIDAPI_DriverPinscape = {
    SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE,
    true,
    HIDAPI_DriverPinscape_RegisterHints,
    HIDAPI_DriverPinscape_UnregisterHints,
    HIDAPI_DriverPinscape_IsEnabled,
    HIDAPI_DriverPinscape_IsSupportedDevice,
    HIDAPI_DriverPinscape_InitDevice,
    HIDAPI_DriverPinscape_GetDevicePlayerIndex,
    HIDAPI_DriverPinscape_SetDevicePlayerIndex,
    HIDAPI_DriverPinscape_UpdateDevice,
    HIDAPI_DriverPinscape_OpenJoystick,
    HIDAPI_DriverPinscape_RumbleJoystick,
    HIDAPI_DriverPinscape_RumbleJoystickTriggers,
    HIDAPI_DriverPinscape_GetJoystickCapabilities,
    HIDAPI_DriverPinscape_SetJoystickLED,
    HIDAPI_DriverPinscape_SendJoystickEffect,
    HIDAPI_DriverPinscape_SetJoystickSensorsEnabled,
    HIDAPI_DriverPinscape_CloseJoystick,
    HIDAPI_DriverPinscape_FreeDevice,
};

#endif // SDL_JOYSTICK_HIDAPI_PINSCAPE

#endif // SDL_JOYSTICK_HIDAPI
