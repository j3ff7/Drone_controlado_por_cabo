#!/usr/bin/env python3
import argparse
import csv
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

TYPE_MASK_POSITION_YAW = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def px4_custom_mode(main_mode):
    return main_mode << 16


def norm2(x, y):
    return math.sqrt(x * x + y * y)


def norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def finite_or_none(value):
    return value if value is not None and math.isfinite(value) else None


class OffboardMission:
    def __init__(self, args):
        self.args = args
        self.master = mavutil.mavlink_connection(
            args.connection,
            source_system=args.source_system,
            source_component=args.source_component,
        )
        self.target_system = None
        self.target_component = None
        self.local_position = None
        self.attitude = None
        self.heartbeat = None
        self.extended_state = None
        self.rows = []
        self.start_wall = None
        self.mission_failed = False
        self.failure_reason = ''

    def wait_ready(self):
        heartbeat = self.master.wait_heartbeat(timeout=self.args.heartbeat_timeout)
        if heartbeat is None:
            raise RuntimeError('heartbeat timeout')
        self.heartbeat = heartbeat
        self.target_system = self.master.target_system
        self.target_component = self.master.target_component
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
        deadline = time.monotonic() + self.args.position_timeout
        while time.monotonic() < deadline:
            self.drain_messages()
            if self.local_position is not None and self.attitude is not None:
                return
            time.sleep(0.05)
        raise RuntimeError('local position or attitude timeout')

    def request_message_interval(self, message_id, hz):
        interval_us = int(1_000_000 / hz)
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def drain_messages(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == 'BAD_DATA':
                continue
            if msg_type == 'LOCAL_POSITION_NED':
                self.local_position = msg
            elif msg_type == 'ATTITUDE':
                self.attitude = msg
            elif msg_type == 'HEARTBEAT':
                self.heartbeat = msg
            elif msg_type == 'EXTENDED_SYS_STATE':
                self.extended_state = msg

    def command_long(self, command, *params):
        padded = list(params)[:7] + [0.0] * (7 - len(params))
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,
            *padded,
        )

    def arm(self):
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)

    def land(self):
        self.command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, 0, 0, 0)

    def set_offboard(self):
        self.master.mav.set_mode_send(
            self.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            px4_custom_mode(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
        )

    def send_setpoint(self, x, y, z, yaw):
        self.master.mav.set_position_target_local_ned_send(
            int((time.monotonic() - self.start_wall) * 1000),
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            TYPE_MASK_POSITION_YAW,
            x,
            y,
            z,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw,
            0.0,
        )

    def record(self, phase, ref):
        lp = self.local_position
        att = self.attitude
        hb = self.heartbeat
        row = {
            't_epoch': time.time(),
            't_wall': time.monotonic() - self.start_wall,
            'phase': phase,
            'x_ref': ref[0],
            'y_ref': ref[1],
            'z_ref': ref[2],
            'yaw_ref': ref[3],
            'x': finite_or_none(getattr(lp, 'x', None)),
            'y': finite_or_none(getattr(lp, 'y', None)),
            'z': finite_or_none(getattr(lp, 'z', None)),
            'vx': finite_or_none(getattr(lp, 'vx', None)),
            'vy': finite_or_none(getattr(lp, 'vy', None)),
            'vz': finite_or_none(getattr(lp, 'vz', None)),
            'roll_deg': math.degrees(att.roll) if att else None,
            'pitch_deg': math.degrees(att.pitch) if att else None,
            'yaw_deg': math.degrees(att.yaw) if att else None,
            'base_mode': getattr(hb, 'base_mode', None),
            'custom_mode': getattr(hb, 'custom_mode', None),
            'system_status': getattr(hb, 'system_status', None),
        }
        if lp:
            row['ex'] = row['x_ref'] - lp.x
            row['ey'] = row['y_ref'] - lp.y
            row['ez'] = row['z_ref'] - lp.z
            row['e_xy'] = norm2(row['ex'], row['ey'])
            row['e_3d'] = norm3(row['ex'], row['ey'], row['ez'])
        else:
            row['ex'] = row['ey'] = row['ez'] = None
            row['e_xy'] = row['e_3d'] = None
        self.rows.append(row)

    def run_phase(self, name, ref, duration):
        period = 1.0 / self.args.rate
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            before = time.monotonic()
            self.drain_messages()
            self.send_setpoint(*ref)
            self.record(name, ref)
            if self.heartbeat and self.heartbeat.system_status == mavutil.mavlink.MAV_STATE_CRITICAL:
                self.mission_failed = True
                self.failure_reason = 'heartbeat system_status CRITICAL'
            elapsed = time.monotonic() - before
            time.sleep(max(0.0, period - elapsed))

    def run(self):
        self.start_wall = time.monotonic()
        self.wait_ready()
        initial = self.local_position
        yaw = self.attitude.yaw if self.attitude else 0.0
        home = (initial.x, initial.y, -abs(self.args.altitude), yaw)
        shifted = (initial.x + self.args.dx, initial.y + self.args.dy, -abs(self.args.altitude), yaw)

        self.run_phase('prestream', home, self.args.prestream)
        self.set_offboard()
        self.run_phase('offboard_settle', home, self.args.offboard_settle)
        self.arm()
        self.run_phase('climb_hover', home, self.args.takeoff_hover)
        self.run_phase('translate_out', shifted, self.args.move_hold)
        self.run_phase('return_home', home, self.args.return_hold)
        self.land()
        self.run_phase('land_stream', home, self.args.land_stream)
        return self.summary()

    def summary(self):
        tracking_rows = [row for row in self.rows if row['phase'] in ('translate_out', 'return_home')]
        hover_rows = [row for row in self.rows if row['phase'] == 'climb_hover']
        all_rows = self.rows

        def max_abs(field, rows=all_rows):
            values = [abs(row[field]) for row in rows if row.get(field) is not None]
            return max(values) if values else None

        def rms(field, rows=all_rows):
            values = [row[field] for row in rows if row.get(field) is not None]
            if not values:
                return None
            return math.sqrt(sum(value * value for value in values) / len(values))

        final = self.rows[-1] if self.rows else {}
        return {
            'connection': self.args.connection,
            'rate_hz': self.args.rate,
            'samples': len(self.rows),
            'failed': self.mission_failed,
            'failure_reason': self.failure_reason,
            'dx_m': self.args.dx,
            'dy_m': self.args.dy,
            'altitude_m': self.args.altitude,
            'final_phase': final.get('phase'),
            'final_position': {
                'x': final.get('x'),
                'y': final.get('y'),
                'z': final.get('z'),
            },
            'rms_xy_tracking_m': rms('e_xy', tracking_rows),
            'rms_z_hover_m': rms('ez', hover_rows),
            'max_xy_error_m': max_abs('e_xy'),
            'max_3d_error_m': max_abs('e_3d'),
            'roll_max_abs_deg': max_abs('roll_deg'),
            'pitch_max_abs_deg': max_abs('pitch_deg'),
            'last_system_status': final.get('system_status'),
            'last_custom_mode': final.get('custom_mode'),
        }

    def write_outputs(self, summary):
        out_dir = Path(self.args.output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / 'px4_offboard_horizontal_mission.csv'
        json_path = out_dir / 'px4_offboard_horizontal_mission.json'
        fields = list(self.rows[0].keys()) if self.rows else []
        with csv_path.open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        json_path.write_text(json.dumps(summary, indent=2))
        return csv_path, json_path


def main():
    parser = argparse.ArgumentParser(description='Run a small PX4 MAVLink OFFBOARD horizontal mission.')
    parser.add_argument('--connection', default='udp:127.0.0.1:14550')
    parser.add_argument('--source-system', type=int, default=245)
    parser.add_argument('--source-component', type=int, default=190)
    parser.add_argument('--heartbeat-timeout', type=float, default=30.0)
    parser.add_argument('--position-timeout', type=float, default=30.0)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--prestream', type=float, default=2.0)
    parser.add_argument('--offboard-settle', type=float, default=1.0)
    parser.add_argument('--takeoff-hover', type=float, default=12.0)
    parser.add_argument('--move-hold', type=float, default=10.0)
    parser.add_argument('--return-hold', type=float, default=10.0)
    parser.add_argument('--land-stream', type=float, default=8.0)
    parser.add_argument('--altitude', type=float, default=2.0)
    parser.add_argument('--dx', type=float, default=0.5)
    parser.add_argument('--dy', type=float, default=0.0)
    parser.add_argument('--output-dir', default='results/offboard_horizontal')
    args = parser.parse_args()

    mission = OffboardMission(args)
    summary = mission.run()
    csv_path, json_path = mission.write_outputs(summary)
    print(json.dumps(summary, indent=2))
    print(f'csv={csv_path}')
    print(f'json={json_path}')
    if summary['failed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
