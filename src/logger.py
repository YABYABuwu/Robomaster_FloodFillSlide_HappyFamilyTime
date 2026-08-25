# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the LICENSE.txt file or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import csv
import os
import statistics
import threading
import time
from collections import deque


class Logger:
    """Subscribe to robot telemetry, keep the latest values, and log CSV files."""

    ADC_MAX = 1023
    ADC_REFERENCE_V = 3.3
    VALID_MIN_CM = 4.0
    VALID_MAX_CM = 30.0
    REGRESSION_SLOPE = 12.1967
    REGRESSION_INTERCEPT = 0.0705
    DISTANCE_OFFSET_CM = 0.42
    VALID_FREQUENCIES = (1, 5, 10, 20, 50)

    def __init__(
        self,
        ep_robot,
        base_data_dir="data/raw",
        sharp_sensors=None,
        sharp_filter_size=5,
    ):
        """
        Create a log directory and configure the Sharp sensor channels.

        ``sharp_sensors`` maps a sensor name to ``(board, port)``. Example::

            {
                "left_front": (1, 1),
                "left_rear": (1, 2),
                "right_front": (3, 1),
                "right_rear": (3, 2),
            }

        All Sharp sensors share one Sensor Adapter subscription.
        """
        if sharp_filter_size <= 0 or sharp_filter_size % 2 == 0:
            raise ValueError("sharp_filter_size must be a positive odd number")

        self.base_data_dir = base_data_dir
        self.run_dir = self._create_run_directory()
        self.chassis = ep_robot.chassis
        self.tof_sensor = ep_robot.sensor
        self.sensor_adaptor = ep_robot.sensor_adaptor
        self.start_time = time.time()

        # DDS callbacks can run concurrently in the RoboMaster SDK thread pool.
        self._data_lock = threading.Lock()
        self._file_lock = threading.Lock()

        self._latest = {
            "attitude": None,
            "position": None,
            "imu": None,
            "esc": None,
            "status": None,
            "tof": None,
        }
        self._ready = {name: threading.Event() for name in self._latest}

        self._sharp_channels = {}
        self._sharp_buffers = {}
        self._sharp_latest = {}
        self._sharp_ready = {}
        self._sharp_subscribed = False

        for name, location in (sharp_sensors or {}).items():
            try:
                board, port = location
            except (TypeError, ValueError):
                raise ValueError(
                    "each sharp sensor location must be a (board, port) pair"
                )

            if board not in range(1, 7):
                raise ValueError("Sharp sensor board must be in the range 1..6")
            if port not in (1, 2):
                raise ValueError("Sharp sensor port must be 1 or 2")

            channel_index = (board - 1) * 2 + (port - 1)
            if any(
                channel["index"] == channel_index
                for channel in self._sharp_channels.values()
            ):
                raise ValueError(
                    "Sharp sensors cannot use the same board and port twice"
                )

            self._sharp_channels[name] = {
                "board": board,
                "port": port,
                "index": channel_index,
            }
            self._sharp_buffers[name] = deque(maxlen=sharp_filter_size)
            self._sharp_latest[name] = None
            self._sharp_ready[name] = threading.Event()

    def _create_run_directory(self):
        """Create the next run directory, for example data/raw/run3."""
        os.makedirs(self.base_data_dir, exist_ok=True)

        existing_runs = [
            name
            for name in os.listdir(self.base_data_dir)
            if name.startswith("run")
        ]
        run_numbers = [
            int(name.replace("run", ""))
            for name in existing_runs
            if name.replace("run", "").isdigit()
        ]

        next_run_number = max(run_numbers) + 1 if run_numbers else 1
        run_dir = os.path.join(
            self.base_data_dir,
            "run{0}".format(next_run_number),
        )
        os.makedirs(run_dir, exist_ok=True)
        return run_dir

    def _write_csv(self, filename, header, row):
        """Write one CSV row without allowing callbacks to interleave writes."""
        self._write_csv_rows(filename, header, [row])

    def _write_csv_rows(self, filename, header, rows):
        """Write a batch with one open call, so a callback stays short."""
        log_path = os.path.join(self.run_dir, filename)

        with self._file_lock:
            file_exists = os.path.exists(log_path)
            with open(log_path, mode="a", newline="") as file:
                writer = csv.writer(file)
                if not file_exists:
                    writer.writerow(header)
                writer.writerows(rows)

    def _record_telemetry_sample(self, name, sub_info, filename, header):
        timestamp = time.time()
        values = tuple(sub_info)

        with self._data_lock:
            self._latest[name] = values
        self._ready[name].set()

        self._write_csv(
            filename,
            ["timestamp", "relative_time"] + list(header),
            [
                round(timestamp, 3),
                round(timestamp - self.start_time, 3),
            ]
            + list(values),
        )

    def sub_attitude_info_handler(self, sub_info):
        self._record_telemetry_sample(
            "attitude",
            sub_info,
            "log_attitude.csv",
            ["yaw", "pitch", "roll"],
        )

    def sub_position_info_handler(self, sub_info):
        self._record_telemetry_sample(
            "position",
            sub_info,
            "log_position.csv",
            ["pos_x", "pos_y", "pos_z"],
        )

    def sub_imu_info_handler(self, sub_info):
        self._record_telemetry_sample(
            "imu",
            sub_info,
            "log_imu.csv",
            ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"],
        )

    def sub_esc_info_handler(self, sub_info):
        self._record_telemetry_sample(
            "esc",
            sub_info,
            "log_esc.csv",
            ["esc_speed", "esc_angle", "esc_timestamp", "esc_state"],
        )

    def sub_status_info_handler(self, sub_info):
        self._record_telemetry_sample(
            "status",
            sub_info,
            "log_status.csv",
            [
                "status_static_flag",
                "status_up_hill",
                "status_down_hill",
                "status_on_slope",
                "status_pick_up",
                "status_slip_flag",
                "status_impact_x",
                "status_impact_y",
                "status_impact_z",
                "status_roll_over",
                "status_hill_static",
            ],
        )

    def sub_distance_info_handler(self, sub_info):
        """Store and log the four ToF distances supplied by the SDK in mm."""
        self._record_telemetry_sample(
            "tof",
            sub_info,
            "log_tof.csv",
            ["tof_1_mm", "tof_2_mm", "tof_3_mm", "tof_4_mm"],
        )

    def sub_sharp_info_handler(self, adapter_info):
        """Filter and store every configured Sharp sensor ADC sample."""
        if not self._sharp_channels:
            return

        try:
            _, adc_data = adapter_info
            # AdapterSubject exposes a mutable list. Copy it before the next
            # SDK decode can update the same object from another thread.
            adc_snapshot = tuple(adc_data)
        except (TypeError, ValueError):
            return

        timestamp = time.time()
        log_rows = []

        with self._data_lock:
            for name, channel in self._sharp_channels.items():
                index = channel["index"]
                if index >= len(adc_snapshot):
                    continue

                raw_adc = adc_snapshot[index]
                if not isinstance(raw_adc, (int, float)):
                    continue
                if not 0 <= raw_adc <= self.ADC_MAX:
                    continue

                samples = self._sharp_buffers[name]
                samples.append(raw_adc)
                filtered_adc = statistics.median(samples)
                voltage = self.adc_to_voltage(filtered_adc)
                distance_cm = self.voltage_to_distance_cm(voltage)

                sample = {
                    "timestamp": timestamp,
                    "board": channel["board"],
                    "port": channel["port"],
                    "raw_adc": raw_adc,
                    "filtered_adc": filtered_adc,
                    "distance_cm": distance_cm,
                }
                self._sharp_latest[name] = sample
                self._sharp_ready[name].set()
                log_rows.append((name, sample.copy()))

        # Do not hold the data lock while doing file I/O.
        if log_rows:
            self._write_csv_rows(
                "log_sharp.csv",
                [
                    "timestamp",
                    "relative_time",
                    "sensor",
                    "board",
                    "port",
                    "raw_adc",
                    "filtered_adc",
                    "distance_cm",
                ],
                [
                    [
                        round(timestamp, 3),
                        round(timestamp - self.start_time, 3),
                        name,
                        sample["board"],
                        sample["port"],
                        sample["raw_adc"],
                        sample["filtered_adc"],
                        sample["distance_cm"],
                    ]
                    for name, sample in log_rows
                ],
            )

    def adc_to_voltage(self, adc_value):
        """Convert a Sensor Adapter 10-bit ADC reading to volts."""
        if not 0 <= adc_value <= self.ADC_MAX:
            raise ValueError("ADC value must be in the range 0..1023")
        return adc_value * self.ADC_REFERENCE_V / self.ADC_MAX

    def voltage_to_distance_cm(self, voltage):
        """Convert voltage to a Sharp distance inside the valid 4-30 cm range."""
        if voltage <= self.REGRESSION_INTERCEPT:
            return None

        distance_cm = (
            self.REGRESSION_SLOPE
            / (voltage - self.REGRESSION_INTERCEPT)
            + self.DISTANCE_OFFSET_CM
        )
        if not self.VALID_MIN_CM <= distance_cm <= self.VALID_MAX_CM:
            return None
        return distance_cm

    def get_latest(self, name):
        """Return the latest tuple for a chassis telemetry subject."""
        if name not in self._latest:
            raise KeyError("unknown telemetry subject: {0}".format(name))
        with self._data_lock:
            return self._latest[name]

    def get_attitude(self):
        return self.get_latest("attitude")

    def get_position(self):
        return self.get_latest("position")

    def get_imu(self):
        return self.get_latest("imu")

    def get_esc(self):
        return self.get_latest("esc")

    def get_status(self):
        return self.get_latest("status")

    def get_tof(self, sensor_id=None):
        """Return all ToF values, or one 1-based sensor value, in millimetres."""
        if sensor_id is not None and sensor_id not in (1, 2, 3, 4):
            raise ValueError("sensor_id must be in the range 1..4")
        distances = self.get_latest("tof")
        if sensor_id is None or distances is None:
            return distances
        return distances[sensor_id - 1]

    def wait_until_ready(self, name, timeout=3.0):
        """Wait until a chassis telemetry subject has delivered one sample."""
        if name not in self._ready:
            raise KeyError("unknown telemetry subject: {0}".format(name))
        return self._ready[name].wait(timeout)

    def get_sharp_sample(self, name):
        """Return a copy of the latest filtered Sharp sample."""
        if name not in self._sharp_latest:
            raise KeyError("unknown Sharp sensor: {0}".format(name))
        with self._data_lock:
            sample = self._sharp_latest[name]
            return None if sample is None else sample.copy()

    def get_sharp_distance(self, name):
        sample = self.get_sharp_sample(name)
        return None if sample is None else sample["distance_cm"]

    def wait_for_sharp(self, name, timeout=3.0):
        if name not in self._sharp_ready:
            raise KeyError("unknown Sharp sensor: {0}".format(name))
        return self._sharp_ready[name].wait(timeout)

    @classmethod
    def _validate_frequency(cls, frequency):
        if frequency not in cls.VALID_FREQUENCIES:
            raise ValueError(
                "frequency must be one of {0}".format(cls.VALID_FREQUENCIES)
            )

    def start_attitude_log(self, feq=5):
        self._validate_frequency(feq)
        return self.chassis.sub_attitude(
            freq=feq,
            callback=self.sub_attitude_info_handler,
        )

    def start_position_log(self, feq=5):
        self._validate_frequency(feq)
        return self.chassis.sub_position(
            freq=feq,
            callback=self.sub_position_info_handler,
        )

    def start_imu_log(self, feq=5):
        self._validate_frequency(feq)
        return self.chassis.sub_imu(
            freq=feq,
            callback=self.sub_imu_info_handler,
        )

    def start_esc_log(self, feq=5):
        self._validate_frequency(feq)
        return self.chassis.sub_esc(
            freq=feq,
            callback=self.sub_esc_info_handler,
        )

    def start_status_log(self, feq=5):
        self._validate_frequency(feq)
        return self.chassis.sub_status(
            freq=feq,
            callback=self.sub_status_info_handler,
        )

    def start_tof_log(self, feq=20):
        self._validate_frequency(feq)
        return self.tof_sensor.sub_distance(
            freq=feq,
            callback=self.sub_distance_info_handler,
        )

    def start_sharp_log(self, feq=20):
        self._validate_frequency(feq)
        if not self._sharp_channels:
            return False
        if self._sharp_subscribed:
            return True

        result = self.sensor_adaptor.sub_adapter(
            freq=feq,
            callback=self.sub_sharp_info_handler,
        )
        self._sharp_subscribed = bool(result)
        return result

    def start_all(
        self,
        feq_att=5,
        feq_pos=5,
        feq_imu=5,
        feq_esc=5,
        feq_status=5,
        feq_sharp=20,
        feq_tof=20,
    ):
        results = {
            "attitude": self.start_attitude_log(feq_att),
            "position": self.start_position_log(feq_pos),
            "imu": self.start_imu_log(feq_imu),
            "esc": self.start_esc_log(feq_esc),
            "status": self.start_status_log(feq_status),
            "tof": self.start_tof_log(feq_tof),
        }
        if self._sharp_channels:
            results["sharp"] = self.start_sharp_log(feq_sharp)
        return results

    def stop_attitude_log(self):
        return self.chassis.unsub_attitude()

    def stop_position_log(self):
        return self.chassis.unsub_position()

    def stop_imu_log(self):
        return self.chassis.unsub_imu()

    def stop_esc_log(self):
        return self.chassis.unsub_esc()

    def stop_status_log(self):
        return self.chassis.unsub_status()

    def stop_tof_log(self):
        return self.tof_sensor.unsub_distance()

    def stop_sharp_log(self):
        if not self._sharp_subscribed:
            return True
        result = self.sensor_adaptor.unsub_adapter()
        self._sharp_subscribed = False
        return result

    def stop_all(self):
        results = {
            "attitude": self.stop_attitude_log(),
            "position": self.stop_position_log(),
            "imu": self.stop_imu_log(),
            "esc": self.stop_esc_log(),
            "status": self.stop_status_log(),
            "tof": self.stop_tof_log(),
        }
        if self._sharp_subscribed:
            results["sharp"] = self.stop_sharp_log()
        return results
