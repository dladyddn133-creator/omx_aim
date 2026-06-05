#!/usr/bin/env python3
"""OMX-Follower keyboard teleop (config.yaml 적용).

변경점:
- arm_step, gripper_step, command_interval, PID gain 등을 config.yaml 에서 로드
- argparse 는 --config, --port 같은 모드 전환용만 유지
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty

from omx.hardware import (
    build_bus,
    get_dxl_symbols,
    ARM_MOTORS,
    GRIPPER_MOTOR,
)
from omx.config import load_config, Config

EXPECTED_MOTOR_IDS = {11, 12, 13, 14, 15, 16}

KEY_BINDINGS = {
    "1": ("shoulder_pan", +1),
    "q": ("shoulder_pan", -1),
    "2": ("shoulder_lift", +1),
    "w": ("shoulder_lift", -1),
    "3": ("elbow_flex", +1),
    "e": ("elbow_flex", -1),
    "4": ("wrist_flex", +1),
    "r": ("wrist_flex", -1),
    "5": ("wrist_roll", +1),
    "t": ("wrist_roll", -1),
    "o": ("gripper", +1),
    "p": ("gripper", -1),
}


class KeyboardTeleop:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.bus = build_bus(cfg.motor.port)
        self.arm_step = cfg.keyboard.arm_step
        self.gripper_step = cfg.keyboard.gripper_step
        self.command_interval = cfg.keyboard.command_interval
        self.positions: dict[str, int] = {}
        self.min_limits: dict[str, int] = {}
        self.max_limits: dict[str, int] = {}
        self.last_command_time = 0.0

    def connect_and_configure(self) -> None:
        s = get_dxl_symbols()
        DriveMode = s["DriveMode"]
        OperatingMode = s["OperatingMode"]

        self.bus.connect()
        with self.bus.torque_disabled():
            self.bus.configure_motors(return_delay_time=0)

            for motor in ARM_MOTORS:
                self.bus.write(
                    "Operating_Mode",
                    motor,
                    OperatingMode.EXTENDED_POSITION.value,
                    normalize=False,
                )

            self.bus.write(
                "Operating_Mode",
                GRIPPER_MOTOR,
                OperatingMode.CURRENT_POSITION.value,
                normalize=False,
            )

            self.bus.write("Drive_Mode", GRIPPER_MOTOR,
                           DriveMode.NON_INVERTED.value, normalize=False)

            self.bus.write("Position_P_Gain", "elbow_flex",
                           self.cfg.motor.elbow_p_gain, normalize=False)
            self.bus.write("Position_I_Gain", "elbow_flex",
                           self.cfg.motor.elbow_i_gain, normalize=False)
            self.bus.write("Position_D_Gain", "elbow_flex",
                           self.cfg.motor.elbow_d_gain, normalize=False)

        self.bus.enable_torque(num_retry=3)
        self.positions = self.bus.sync_read("Present_Position", normalize=False)
        self.min_limits = self.bus.sync_read("Min_Position_Limit", normalize=False)
        self.max_limits = self.bus.sync_read("Max_Position_Limit", normalize=False)

    def disconnect(self, disable_torque: bool) -> None:
        if self.bus.is_connected:
            self.bus.disconnect(disable_torque=disable_torque)

    def _get_key(self, timeout: float = 0.01) -> str | None:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _clamp(self, motor: str, value: int) -> int:
        min_limit = self.min_limits[motor]
        max_limit = self.max_limits[motor]
        return max(min(value, max_limit), min_limit)

    def _send_command(self, motor: str, direction: int) -> None:
        step = self.gripper_step if motor == GRIPPER_MOTOR else self.arm_step
        target = self.positions[motor] + direction * step
        target = self._clamp(motor, target)

        self.bus.write("Goal_Position", motor, target, normalize=False)
        self.positions[motor] = target
        print(f"\r{motor:<14} -> {target:<6}  ", end="", flush=True)

    def run(self) -> None:
        print("Ready. Keyboard teleop started.")
        print("")
        print("Joint Control")
        print("1 / q - Joint 1")
        print("2 / w - Joint 2")
        print("3 / e - Joint 3")
        print("4 / r - Joint 4")
        print("5 / t - Joint 5")
        print("Gripper Control")
        print("o - Open gripper")
        print("p - Close gripper")
        print("ESC - Exit")

        while True:
            key = self._get_key()
            if key is None:
                continue

            if key == "\x1b":
                print("\nExit requested.")
                break

            if key not in KEY_BINDINGS:
                continue

            now = time.time()
            if now - self.last_command_time < self.command_interval:
                continue

            motor, direction = KEY_BINDINGS[key]
            self._send_command(motor, direction)
            self.last_command_time = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OMX-Follower keyboard teleop.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--config", default=None,
                        help="config.yaml 경로 (default: ./config.yaml)")
    parser.add_argument("--keep-torque-on-exit", action="store_true",
                        help="종료 시 토크 끄지 않음 (default: 토크 OFF)")
    return parser.parse_args()


def print_port_troubleshooting(selected_port: str) -> None:
    print("\nPort hint:")
    print(f"- Current port: {selected_port}")
    print("- Check follower port: ls -l /dev/omx_follower")


def main() -> int:
    args = parse_args()
    try:
        cfg = load_config(args.config)
    except (FileNotFoundError, ValueError) as e:
        print(f"Config 로드 실패: {e}", file=sys.stderr)
        return 1

    print(f"Using port: {cfg.motor.port}")
    teleop = KeyboardTeleop(cfg)

    try:
        teleop.connect_and_configure()
        teleop.run()
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting.")
        return 0
    except Exception as exc:
        print(f"\nError: {exc}", file=sys.stderr)
        error_text = str(exc)
        if (
            "motor check failed" in error_text
            or "Could not connect on port" in error_text
            or "Failed to open port" in error_text
        ):
            if "motor check failed" in error_text:
                print(
                    f"\nDetected no follower response (expected IDs: {sorted(EXPECTED_MOTOR_IDS)}).",
                    file=sys.stderr,
                )
            print_port_troubleshooting(cfg.motor.port)
        return 1
    finally:
        teleop.disconnect(disable_torque=not args.keep_torque_on_exit)


if __name__ == "__main__":
    raise SystemExit(main())