import time

from sensor_msgs.msg import Joy


class JoyTeleop:
    def __init__(self, node):
        self.node = node
        node.declare_parameter("joy_axis_vx", 1)    # stick kiri vertikal
        node.declare_parameter("joy_axis_vy", 0)    # stick kiri horizontal
        node.declare_parameter("joy_axis_vz", 4)    # stick kanan vertikal
        node.declare_parameter("joy_axis_yaw", 3)   # stick kanan horizontal
        node.declare_parameter("joy_btn_ibvs", 0)     # Cross
        node.declare_parameter("joy_btn_hold", 1)     # Circle
        node.declare_parameter("joy_btn_land", 3)     # Square
        node.declare_parameter("joy_btn_takeoff", 2)  # Triangle
        node.declare_parameter("joy_deadzone", 0.15)
        node.declare_parameter("takeoff_hold_s", 1.0)

        g = lambda n: node.get_parameter(n).value
        self.ax_vx, self.ax_vy = g("joy_axis_vx"), g("joy_axis_vy")
        self.ax_vz, self.ax_yaw = g("joy_axis_vz"), g("joy_axis_yaw")
        self.b_ibvs, self.b_hold = g("joy_btn_ibvs"), g("joy_btn_hold")
        self.b_land, self.b_takeoff = g("joy_btn_land"), g("joy_btn_takeoff")
        self.deadzone = g("joy_deadzone")
        self.takeoff_hold_s = g("takeoff_hold_s")

        self.sticks = [0.0, 0.0, 0.0, 0.0]
        self.last_msg_time = None

        self._prev = {"ibvs": False, "hold": False, "land": False}
        self._events = []
        self._takeoff_down = False
        self._takeoff_since = None
        self._takeoff_fired = False

    @staticmethod
    def _axis(msg, idx):
        return float(msg.axes[idx]) if 0 <= idx < len(msg.axes) else 0.0

    @staticmethod
    def _btn(msg, idx):
        return 0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1

    def _shape(self, v):
        a = abs(v)
        if a < self.deadzone:
            return 0.0
        s = (a - self.deadzone) / (1.0 - self.deadzone)
        return s if v > 0 else -s

    def on_joy(self, msg: Joy):
        self.last_msg_time = time.time()

        self.sticks = [
            self._shape(self._axis(msg, self.ax_vx)),
            -self._shape(self._axis(msg, self.ax_vy)),
            -self._shape(self._axis(msg, self.ax_vz)),
            -self._shape(self._axis(msg, self.ax_yaw)),
        ]

        cur = {
            "ibvs": self._btn(msg, self.b_ibvs),
            "hold": self._btn(msg, self.b_hold),
            "land": self._btn(msg, self.b_land),
        }
        for name, pressed in cur.items():
            if pressed and not self._prev[name]:  # rising edge
                self._events.append(name)
        self._prev = cur
        self._takeoff_down = self._btn(msg, self.b_takeoff)

    def poll(self, now):
        if self._takeoff_down:
            if self._takeoff_since is None:
                self._takeoff_since = now
                self._takeoff_fired = False
            elif not self._takeoff_fired and now - self._takeoff_since >= self.takeoff_hold_s:
                self._events.append("takeoff")
                self._takeoff_fired = True
        else:
            self._takeoff_since = None
            self._takeoff_fired = False

        ev, self._events = self._events, []
        return ev

    def age(self, now):
        return float("inf") if self.last_msg_time is None else now - self.last_msg_time

    def magnitude(self):
        return max(abs(s) for s in self.sticks)