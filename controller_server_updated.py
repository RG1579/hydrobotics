"""
Controller -> 6x1 velocity vector -> Ethernet (TCP server)

This version maps controls in a way that matches typical ROV pilot feedback:

- Joystick 1 (left stick):  left/right + forward/back   -> linear X/Y
- Joystick 2 (right stick): pitch + yaw                 -> angular pitch/yaw
- Separate control:          up/down                     -> linear Z
- Separate control:          roll                        -> angular roll

Vector order is kept as:
    [X, Y, Z, RX, RY, RZ]
where:
    X, Y, Z   = linear velocities (body frame) in [-1, 1]
    RX, RY, RZ = angular velocities (roll, pitch, yaw) in [-1, 1]

Packet format (per message):
    [uint64 length][6 x float32 values]
"""

import socket
import struct
import sys

import pygame


# --------------------- configuration ---------------------

HOST = "0.0.0.0"   # listen on all interfaces
PORT = 12345       # change if needed

UPDATE_RATE_HZ = 50          # how often to send commands
DEADZONE = 0.05              # ignore very small stick movements

# ---- Axis/button mapping (EDIT PER CONTROLLER) ----
#
# Pygame axis indices vary by controller/OS. Use the startup printout
# to see how many axes/buttons you have, then adjust these indices.
#
# Common-ish defaults:
#   Left stick:   LX=0, LY=1
#   Right stick:  RX=2, RY=3
#   Triggers:     LT=4, RT=5   (sometimes a *single* trigger axis exists instead)
#
AXIS = {
    "LX": 0,   # left stick horizontal  (strafe left/right)  -> X
    "LY": 1,   # left stick vertical    (forward/back)       -> Y

    "RX": 2,   # right stick horizontal (yaw)                -> RZ
    "RY": 3,   # right stick vertical   (pitch)              -> RY

    # Up/down options:
    # - If separate trigger axes: set LT_AXIS and RT_AXIS and keep TRIGGERS_COMBINED=False
    # - If single combined trigger axis: set COMBINED_TRIGGER_AXIS and set TRIGGERS_COMBINED=True
    "LT_AXIS": 4,
    "RT_AXIS": 5,
    "COMBINED_TRIGGER_AXIS": 4,
}

TRIGGERS_COMBINED = False
# If TRIGGERS_COMBINED=True, choose whether pushing "down" should be positive or negative.
COMBINED_TRIGGER_INVERT = False

# Roll options:
# - If axis (e.g., d-pad as hat mapped to an axis): set ROLL_AXIS and set ROLL_MODE="axis"
# - If buttons (recommended): set BUTTON_ROLL_LEFT/RIGHT and set ROLL_MODE="buttons"
ROLL_MODE = "buttons"  # "buttons" or "axis"
ROLL_AXIS = 5

BUTTON_ROLL_LEFT = 9   # e.g. LB
BUTTON_ROLL_RIGHT = 10  # e.g. RB

ROLL_BUTTON_VALUE = 1.0  # output when pressed

# Inversions (set True if your axes feel backwards)
INVERT = {
    "X": False,
    "Y": True,    # set True to make forward positive
    "Z": False,
    "ROLL": False,
    "PITCH": False,  # set True to make stick-up = pitch-up
    "YAW": False,
}

# Network packing: 6 float32 values
VECTOR_STRUCT = struct.Struct("!6f")   # network byte order, 6 floats
LENGTH_STRUCT = struct.Struct("!Q")    # uint64 length prefix

# ---------------------------------------------------------


def init_joystick() -> pygame.joystick.Joystick:
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() < 1:
        print("No joystick/gamepad detected. Please plug one in.", file=sys.stderr)
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()

    print(f"Using joystick: {js.get_name()}")
    print(f"  Axes:    {js.get_numaxes()}")
    print(f"  Buttons: {js.get_numbuttons()}")
    print(f"  Hats:    {js.get_numhats()}")
    print("Tip: move one control at a time and watch the debug print (enable below) to identify indices.")

    return js


def apply_deadzone(value: float, dz: float) -> float:
    """Zero small values to remove noise."""
    return 0.0 if abs(value) < dz else value


def get_axis_safe(js: pygame.joystick.Joystick, index: int) -> float:
    if 0 <= index < js.get_numaxes():
        return js.get_axis(index)
    return 0.0


def get_button_safe(js: pygame.joystick.Joystick, index: int) -> int:
    if 0 <= index < js.get_numbuttons():
        return js.get_button(index)
    return 0


def inv(value: float, flag: bool) -> float:
    return -value if flag else value


def read_up_down(js: pygame.joystick.Joystick) -> float:
    """
    Returns Z in [-1, 1].
    """
    if TRIGGERS_COMBINED:
        z = get_axis_safe(js, AXIS["COMBINED_TRIGGER_AXIS"])
        z = inv(z, COMBINED_TRIGGER_INVERT)
        return apply_deadzone(z, DEADZONE)

    lt = get_axis_safe(js, AXIS["LT_AXIS"])
    rt = get_axis_safe(js, AXIS["RT_AXIS"])

    # Some drivers give LT/RT in [-1,1] where released is -1, pressed is +1.
    # Others give [0,1], put into [-1,1].
    z = (rt - lt) * 0.5
    z = max(-1.0, min(1.0, z))
    return apply_deadzone(z, DEADZONE)


def read_roll(js: pygame.joystick.Joystick) -> float:
    """Returns roll command RX in [-1, 1]."""
    if ROLL_MODE == "axis":
        r = get_axis_safe(js, ROLL_AXIS)
        r = apply_deadzone(r, DEADZONE)
        return inv(r, INVERT["ROLL"])

    left = get_button_safe(js, BUTTON_ROLL_LEFT)
    right = get_button_safe(js, BUTTON_ROLL_RIGHT)
    r = (right - left) * ROLL_BUTTON_VALUE
    return inv(float(r), INVERT["ROLL"])


def read_velocity_vector(js: pygame.joystick.Joystick) -> list[float]:
    """
    Read current joystick state and map to [X, Y, Z, RX, RY, RZ] in [-1, 1].
    """
    pygame.event.pump()   # process internal events so axes update

    # Linear:
    x = apply_deadzone(get_axis_safe(js, AXIS["LX"]), DEADZONE)
    y = apply_deadzone(get_axis_safe(js, AXIS["LY"]), DEADZONE)
    z = read_up_down(js)

    x = inv(x, INVERT["X"])
    y = inv(y, INVERT["Y"])
    z = inv(z, INVERT["Z"])

    # Angular:
    yaw = apply_deadzone(get_axis_safe(js, AXIS["RX"]), DEADZONE)   # right stick X
    pitch = apply_deadzone(get_axis_safe(js, AXIS["RY"]), DEADZONE) # right stick Y
    roll = read_roll(js)

    pitch = inv(pitch, INVERT["PITCH"])
    yaw = inv(yaw, INVERT["YAW"])

    # Vector order: [X, Y, Z, RX, RY, RZ] = [strafe, forward, up, roll, pitch, yaw]
    return [x, y, z, roll, pitch, yaw]


def start_server(host: str, port: int) -> socket.socket:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # quick restart after closing
    srv.bind((host, port))
    srv.listen(1)
    print(f"Controller server listening on {host}:{port}")
    return srv


def send_vector(conn: socket.socket, vector: list[float]) -> None:
    payload = VECTOR_STRUCT.pack(*vector)
    header = LENGTH_STRUCT.pack(len(payload))
    conn.sendall(header + payload)


# --------------------------- main ---------------------------

def main() -> None:
    joystick = init_joystick()
    server_socket = start_server(HOST, PORT)

    print("Waiting for client to connect...")
    conn, addr = server_socket.accept()
    print(f"Client connected from {addr}")

    clock = pygame.time.Clock()

    # Set True temporarily to identify axis indices:
    DEBUG_PRINT_AXES = False
    DEBUG_PRINT_HZ = 1
    _dbg_counter = 0

    try:
        while True:
            vector = read_velocity_vector(joystick)

            if DEBUG_PRINT_AXES:
                _dbg_counter += 1
                if _dbg_counter >= (UPDATE_RATE_HZ // DEBUG_PRINT_HZ):
                    _dbg_counter = 0
                    axes = [f"{i}:{joystick.get_axis(i):+0.3f}" for i in range(joystick.get_numaxes())]
                    buttons = [f"{i}:{joystick.get_button(i)}" for i in range(joystick.get_numbuttons())]
                    print("axes:", " ".join(axes))
                    print("btns:", " ".join(buttons))
                    print("vec: ", " ".join(f"{v:+0.3f}" for v in vector))
                    print("-" * 60)

            try:
                send_vector(conn, vector)
            except (BrokenPipeError, ConnectionResetError):
                print("Client disconnected.")
                break

            clock.tick(UPDATE_RATE_HZ)

    except KeyboardInterrupt:
        print("Shutting down (Ctrl+C).")

    finally:
        conn.close()
        server_socket.close()
        pygame.quit()


if __name__ == "__main__":
    main()
