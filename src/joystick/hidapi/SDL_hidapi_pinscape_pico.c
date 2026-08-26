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

#ifdef SDL_JOYSTICK_HIDAPI_PINSCAPE_PICO

/* Pinscape Pico, the second generation of the Pinscape virtual pinball I/O
 * board. It shares a name and a purpose with the original, and nothing else:
 * different microcontroller, different firmware, different protocol. Hence a
 * driver of its own rather than a branch in the first one.
 *
 * The reason it needs a driver at all is the same as for the original. Some of
 * its axes carry accelerometer readings for cabinet nudging, and the physical
 * acceleration a full-scale reading stands for is a configuration setting: the
 * same board, with the same chip, reports +/-1 g through +/-16 g depending on
 * how it was set up. Nothing in the HID descriptor says which, so an
 * application receives a number with no unit.
 *
 * The board's mounting orientation is not part of this. It is described to the
 * firmware as a rotation from the physical sensor's axes to the cabinet's, and
 * applied there, so readings arrive in the cabinet's frame whichever way the
 * board is screwed down. The range is the only part a host cannot infer.
 *
 * Note that which axes carry the nudge is up to the user here -- the board lets
 * every axis be assigned a source, and cabinets commonly move the accelerometer
 * off X/Y to keep non-pinball games from seeing it. That assignment stays the
 * application's business, as it is today. What cannot be discovered by asking
 * the user, and is published here, is the scale.
 *
 * Where the board keeps the answer is the awkward part: the nudge range lives
 * in the identification report of the Feedback Controller, which is a separate
 * HID interface from the one carrying the axes. So this opens a sibling
 * interface to ask. That interface is filtered out of the joystick enumeration
 * before drivers are consulted -- it is not on the Generic Desktop page -- so
 * it will never be claimed as a joystick in its own right.
 *
 * Protocol reference: mjrgh/PinscapePico, USBProtocol/FeedbackControllerProtocol.h.
 */

#define PINSCAPE_PICO_VENDOR_ID  0x1209
#define PINSCAPE_PICO_PRODUCT_ID 0xEAEB

/* Feedback Controller interface: HID report id, and the fixed report size the
 * protocol uses in both directions, excluding that id byte. */
#define PINSCAPE_PICO_FEEDBACK_REPORT_ID 0x04
#define PINSCAPE_PICO_FEEDBACK_SIZE      63

/* Request and reply type codes, in the first payload byte. */
#define PINSCAPE_PICO_REQ_QUERY_ID 0x01
#define PINSCAPE_PICO_RPT_ID       0x01

/* Offset of NudgeRange within the identification report payload, after
 * UnitNumber, UnitName[32], ProtocolVer, HardwareID[8], NumPorts, PlungerType
 * and LedWizUnitMask. Added in firmware 1.0.3; older builds leave it zero,
 * which the protocol defines as "not available". */
#define PINSCAPE_PICO_ID_NUDGE_RANGE_OFFSET 50

/* Above +/-16 g is reserved, for ranges that are not whole multiples of 1 g. */
#define PINSCAPE_PICO_MAX_NUDGE_RANGE 0x10

/* Gamepad report: 32 buttons as a bitmask, eight 16-bit axes, one hat. */
#define PINSCAPE_PICO_NUM_BUTTONS 32
#define PINSCAPE_PICO_NUM_AXES    8
#define PINSCAPE_PICO_REPORT_SIZE 64

/* The board answers a query promptly when it answers at all, but its input
 * pipe also carries unsolicited reports, so a reply may not be the first thing
 * read back. */
#define PINSCAPE_PICO_REPLY_ATTEMPTS 32

typedef struct
{
    SDL_JoystickID joystick;
    bool have_accel_range;
    float accel_full_scale; /* m/s^2 at a full-scale axis reading */
    Uint8 last_state[PINSCAPE_PICO_REPORT_SIZE];
} SDL_DriverPinscapePico_Context;

static void HIDAPI_DriverPinscapePico_RegisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_AddHintCallback(SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE_PICO, callback, userdata);
}

static void HIDAPI_DriverPinscapePico_UnregisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_RemoveHintCallback(SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE_PICO, callback, userdata);
}

static bool HIDAPI_DriverPinscapePico_IsEnabled(void)
{
    return SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE_PICO,
                              SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI, SDL_HIDAPI_DEFAULT));
}

static bool HIDAPI_DriverPinscapePico_IsSupportedDevice(SDL_HIDAPI_Device *device, const char *name, SDL_GamepadType type, Uint16 vendor_id, Uint16 product_id, Uint16 version, int interface_number, int interface_class, int interface_subclass, int interface_protocol)
{
    if (vendor_id == PINSCAPE_PICO_VENDOR_ID && product_id == PINSCAPE_PICO_PRODUCT_ID) {
        return true;
    }

    /* The ids are settable on the board, so a reconfigured one is recognised by
     * the product string instead. The original generation is deliberately not
     * matched here: it has its own driver, and speaks nothing in common. */
    if (name && SDL_strcasestr(name, "pinscape") && SDL_strcasestr(name, "pico")) {
        return true;
    }
    return false;
}

/* Send a Feedback Controller request and read back the reply of the given type.
 *
 * Returns true if a reply arrived, with its payload -- everything after the
 * report id -- copied into payload.
 */
static bool QueryFeedbackController(SDL_hid_device *dev, Uint8 request_type, Uint8 reply_type, Uint8 *payload)
{
    Uint8 request[1 + PINSCAPE_PICO_FEEDBACK_SIZE];
    Uint8 reply[1 + PINSCAPE_PICO_FEEDBACK_SIZE];
    int attempt;

    SDL_zeroa(request);
    request[0] = PINSCAPE_PICO_FEEDBACK_REPORT_ID;
    request[1] = request_type;

    if (SDL_hid_write(dev, request, sizeof(request)) < 0) {
        return false;
    }

    for (attempt = 0; attempt < PINSCAPE_PICO_REPLY_ATTEMPTS; ++attempt) {
        int size_read = SDL_hid_read_timeout(dev, reply, sizeof(reply), 10);
        if (size_read < 0) {
            return false;
        }
        if (size_read > 1 && reply[0] == PINSCAPE_PICO_FEEDBACK_REPORT_ID && reply[1] == reply_type) {
            SDL_memcpy(payload, &reply[1], PINSCAPE_PICO_FEEDBACK_SIZE);
            return true;
        }
    }
    return false;
}

/* Ask the board what its accelerometer is set to, over the Feedback Controller
 * interface.
 *
 * The candidate interfaces are narrowed by usage where the platform reports it
 * -- the axes are on the Generic Desktop page and the pinball device interface
 * on Game Controls, neither of which is this one -- and then confirmed by
 * asking: an interface that answers a query for the identification report with
 * an identification report is the right one. That handshake is what makes this
 * work when the usage is unknown, and it is also why the board's own
 * configurable usage codes do not matter here.
 */
static bool QueryAccelRange(SDL_HIDAPI_Device *device, float *full_scale)
{
    struct SDL_hid_device_info *devs, *info;
    bool found = false;

    devs = SDL_hid_enumerate(device->vendor_id, device->product_id);
    for (info = devs; info && !found; info = info->next) {
        SDL_hid_device *dev;
        Uint8 payload[PINSCAPE_PICO_FEEDBACK_SIZE];
        Uint8 range;

        if (!info->path || (device->path && SDL_strcmp(info->path, device->path) == 0)) {
            continue;
        }
        if (info->usage_page == 0x0001 || info->usage_page == 0x0005) {
            continue;
        }

        dev = SDL_hid_open_path(info->path);
        if (!dev) {
            continue;
        }
        if (QueryFeedbackController(dev, PINSCAPE_PICO_REQ_QUERY_ID, PINSCAPE_PICO_RPT_ID, payload)) {
            range = payload[PINSCAPE_PICO_ID_NUDGE_RANGE_OFFSET];
            if (range > 0 && range <= PINSCAPE_PICO_MAX_NUDGE_RANGE) {
                *full_scale = (float)range * SDL_STANDARD_GRAVITY;
                found = true;
            } else {
                /* The interface answered, so there is nothing more to look for:
                 * either no nudge device is configured, or the firmware predates
                 * the field. Either way the scale is unknown, not elsewhere. */
                break;
            }
        }
        SDL_hid_close(dev);
    }
    SDL_hid_free_enumeration(devs);

    return found;
}

static bool HIDAPI_DriverPinscapePico_InitDevice(SDL_HIDAPI_Device *device)
{
    SDL_DriverPinscapePico_Context *ctx;

    ctx = (SDL_DriverPinscapePico_Context *)SDL_calloc(1, sizeof(*ctx));
    if (!ctx) {
        return false;
    }
    device->context = ctx;

    /* Best-effort: a board with no nudge device, or firmware older than the
     * field, simply has no scale to publish, and is perfectly usable without. */
    ctx->have_accel_range = QueryAccelRange(device, &ctx->accel_full_scale);

    return HIDAPI_JoystickConnected(device, NULL);
}

static int HIDAPI_DriverPinscapePico_GetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id)
{
    return -1;
}

static void HIDAPI_DriverPinscapePico_SetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id, int player_index)
{
}

static bool HIDAPI_DriverPinscapePico_OpenJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    SDL_DriverPinscapePico_Context *ctx = (SDL_DriverPinscapePico_Context *)device->context;

    SDL_AssertJoysticksLocked();

    ctx->joystick = joystick->instance_id;
    SDL_zeroa(ctx->last_state);

    /* The report descriptor declares the full set whatever the cabinet has
     * wired; unused buttons simply never change state, and an axis with no
     * source assigned reads zero. */
    joystick->naxes = PINSCAPE_PICO_NUM_AXES;
    joystick->nbuttons = PINSCAPE_PICO_NUM_BUTTONS;
    joystick->nhats = 1;

    if (ctx->have_accel_range) {
        SDL_PropertiesID props = SDL_GetJoystickProperties(joystick);
        SDL_SetFloatProperty(props, SDL_PROP_JOYSTICK_ACCEL_FULL_SCALE_FLOAT, ctx->accel_full_scale);
    }
    return true;
}

static bool HIDAPI_DriverPinscapePico_RumbleJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 low_frequency_rumble, Uint16 high_frequency_rumble)
{
    return SDL_Unsupported();
}

static bool HIDAPI_DriverPinscapePico_RumbleJoystickTriggers(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 left_rumble, Uint16 right_rumble)
{
    return SDL_Unsupported();
}

static Uint32 HIDAPI_DriverPinscapePico_GetJoystickCapabilities(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    return 0;
}

static bool HIDAPI_DriverPinscapePico_SetJoystickLED(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint8 red, Uint8 green, Uint8 blue)
{
    return SDL_Unsupported();
}

static bool HIDAPI_DriverPinscapePico_SendJoystickEffect(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, const void *data, int size)
{
    return SDL_Unsupported();
}

static bool HIDAPI_DriverPinscapePico_SetJoystickSensorsEnabled(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, bool enabled)
{
    return SDL_Unsupported();
}

/* Decode one gamepad report.
 *
 * Layout, from Gamepad.cpp: the report id, four bytes of buttons one bit each,
 * eight 16-bit axes in descriptor order (X, Y, Z, RX, RY, RZ and two sliders),
 * then the hat. All little-endian, all signed -32767..+32767, which is SDL's
 * range already, so the axes pass through untouched.
 */
static void HandleStatePacket(SDL_Joystick *joystick, SDL_DriverPinscapePico_Context *ctx, Uint8 *data, int size)
{
    Uint64 timestamp = SDL_GetTicksNS();
    int axis, button;

    /* report id + buttons + axes + hat */
    if (size < 1 + 4 + PINSCAPE_PICO_NUM_AXES * 2 + 1) {
        return;
    }

    for (button = 0; button < PINSCAPE_PICO_NUM_BUTTONS; ++button) {
        const int byte = 1 + (button / 8);
        const Uint8 mask = (Uint8)(1 << (button % 8));
        if ((data[byte] & mask) != (ctx->last_state[byte] & mask)) {
            SDL_SendJoystickButton(timestamp, joystick, (Uint8)button,
                                   ((data[byte] & mask) != 0));
        }
    }

    for (axis = 0; axis < PINSCAPE_PICO_NUM_AXES; ++axis) {
        const int at = 5 + axis * 2;
        const Sint16 value = (Sint16)(data[at] | (data[at + 1] << 8));
        SDL_SendJoystickAxis(timestamp, joystick, (Uint8)axis, value);
    }

    {
        /* The hat is reported as an angle in eighths of a turn, 1 at north and
         * increasing clockwise, with anything outside 1..8 meaning centred. */
        static const Uint8 hat_positions[8] = {
            SDL_HAT_UP, SDL_HAT_RIGHTUP, SDL_HAT_RIGHT, SDL_HAT_RIGHTDOWN,
            SDL_HAT_DOWN, SDL_HAT_LEFTDOWN, SDL_HAT_LEFT, SDL_HAT_LEFTUP
        };
        const Uint8 angle = (Uint8)(data[1 + 4 + PINSCAPE_PICO_NUM_AXES * 2] & 0x0F);
        const Uint8 hat = (angle >= 1 && angle <= 8) ? hat_positions[angle - 1] : SDL_HAT_CENTERED;
        SDL_SendJoystickHat(timestamp, joystick, 0, hat);
    }

    SDL_memcpy(ctx->last_state, data, SDL_min((size_t)size, sizeof(ctx->last_state)));
}

static bool HIDAPI_DriverPinscapePico_UpdateDevice(SDL_HIDAPI_Device *device)
{
    SDL_DriverPinscapePico_Context *ctx = (SDL_DriverPinscapePico_Context *)device->context;
    SDL_Joystick *joystick = NULL;
    Uint8 data[PINSCAPE_PICO_REPORT_SIZE];
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
        HandleStatePacket(joystick, ctx, data, size);
    }

    if (size < 0) {
        HIDAPI_JoystickDisconnected(device, ctx->joystick);
    }
    return (size >= 0);
}

static void HIDAPI_DriverPinscapePico_CloseJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    SDL_DriverPinscapePico_Context *ctx = (SDL_DriverPinscapePico_Context *)device->context;

    ctx->joystick = 0;
}

static void HIDAPI_DriverPinscapePico_FreeDevice(SDL_HIDAPI_Device *device)
{
}

SDL_HIDAPI_DeviceDriver SDL_HIDAPI_DriverPinscapePico = {
    SDL_HINT_JOYSTICK_HIDAPI_PINSCAPE_PICO,
    true,
    HIDAPI_DriverPinscapePico_RegisterHints,
    HIDAPI_DriverPinscapePico_UnregisterHints,
    HIDAPI_DriverPinscapePico_IsEnabled,
    HIDAPI_DriverPinscapePico_IsSupportedDevice,
    HIDAPI_DriverPinscapePico_InitDevice,
    HIDAPI_DriverPinscapePico_GetDevicePlayerIndex,
    HIDAPI_DriverPinscapePico_SetDevicePlayerIndex,
    HIDAPI_DriverPinscapePico_UpdateDevice,
    HIDAPI_DriverPinscapePico_OpenJoystick,
    HIDAPI_DriverPinscapePico_RumbleJoystick,
    HIDAPI_DriverPinscapePico_RumbleJoystickTriggers,
    HIDAPI_DriverPinscapePico_GetJoystickCapabilities,
    HIDAPI_DriverPinscapePico_SetJoystickLED,
    HIDAPI_DriverPinscapePico_SendJoystickEffect,
    HIDAPI_DriverPinscapePico_SetJoystickSensorsEnabled,
    HIDAPI_DriverPinscapePico_CloseJoystick,
    HIDAPI_DriverPinscapePico_FreeDevice,
};

#endif // SDL_JOYSTICK_HIDAPI_PINSCAPE_PICO

#endif // SDL_JOYSTICK_HIDAPI
