"""Shared AirSim-only return-home tuning and ground-truth quality measurements."""

import math


SIM_RESCUE_CONFIG = (
    "feature GPS",
    "set gps_provider = VIRTUAL",
    "set trust_mag = ON",
    "set gps_rescue_min_start_dist = 15",
    "set gps_rescue_alt_mode = FIXED_ALT",
    "set gps_rescue_return_alt = 8",
    "set gps_rescue_initial_climb = 3",
    "set gps_rescue_ascend_rate = 200",
    "set gps_rescue_descend_rate = 200",
    "set gps_rescue_ground_speed = 300",
    "set gps_rescue_landing_rate = 50",
    "set gps_rescue_descent_dist = 8",
    "set gps_rescue_min_sats = 8",
    "set gps_rescue_allow_arming_without_fix = OFF",
    # The delayed GPS/carrot loop needs less position drive, not lower rate PID gains.
    "set ap_position_p = 6",
    "set ap_velocity_drag_coeff = 0",
    "set ap_yaw_mode = BEARING",
    "set ap_max_angle = 25",
    "set ap_nav_accel = 100",
)


class ReturnFlightMetrics:
    """Observe cruise separately from takeover/braking and the home descent."""

    def __init__(self, cruise_speed_m_s=3.0):
        self.cruise_speed = cruise_speed_m_s
        self.initial_distance = None
        self.peak_speed = 0.0
        self.peak_tilt = 0.0
        self.cruise_samples = 0
        self.speed_sum = 0.0
        self.speed_sq_sum = 0.0
        self.cruise_peak_speed = 0.0
        self.tilt_sq_sum = 0.0
        self.cruise_peak_tilt = 0.0
        self.reverse_samples = 0

    def update(self, elapsed_s, home_offset_xy, velocity, rpy):
        distance = math.hypot(*home_offset_xy)
        if self.initial_distance is None:
            self.initial_distance = distance
        speed = math.hypot(*velocity[:2])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0,
            math.cos(math.radians(rpy[0])) * math.cos(math.radians(rpy[1]))))))
        self.peak_speed = max(self.peak_speed, speed)
        self.peak_tilt = max(self.peak_tilt, tilt)
        # Do not score the entry climb/turn or the final home brake as cruise.
        # No speed-based exclusion: stops and reversals must remain visible.
        if elapsed_s < 5.0 or distance <= 10.0 or self.initial_distance - distance < 2.0:
            return
        self.cruise_samples += 1
        self.speed_sum += speed
        self.speed_sq_sum += speed * speed
        self.cruise_peak_speed = max(self.cruise_peak_speed, speed)
        self.tilt_sq_sum += tilt * tilt
        self.cruise_peak_tilt = max(self.cruise_peak_tilt, tilt)
        closing_speed = -sum(p * v for p, v in zip(home_offset_xy, velocity)) / distance
        self.reverse_samples += closing_speed < -0.5

    def summary(self):
        result = {"peak_horizontal_speed_m_s": self.peak_speed,
                  "peak_tilt_deg": self.peak_tilt, "cruise_samples": self.cruise_samples}
        if not self.cruise_samples:
            return result
        mean = self.speed_sum / self.cruise_samples
        mean_sq = self.speed_sq_sum / self.cruise_samples
        result.update(
            cruise_speed_mean_m_s=mean,
            cruise_speed_std_m_s=math.sqrt(max(0.0, mean_sq - mean * mean)),
            cruise_speed_error_rms_m_s=math.sqrt(max(0.0,
                mean_sq - 2 * self.cruise_speed * mean + self.cruise_speed ** 2)),
            cruise_peak_horizontal_speed_m_s=self.cruise_peak_speed,
            cruise_tilt_rms_deg=math.sqrt(self.tilt_sq_sum / self.cruise_samples),
            cruise_peak_tilt_deg=self.cruise_peak_tilt,
            cruise_reverse_fraction=self.reverse_samples / self.cruise_samples)
        return result

    def require_stable_cruise(self):
        metrics = self.summary()
        if self.cruise_samples < 20:
            raise AssertionError(f"too few Rescue cruise samples to assess stability: {metrics}")
        if metrics["cruise_speed_error_rms_m_s"] > 1.5 \
                or metrics["cruise_speed_std_m_s"] > 1.0 \
                or self.cruise_peak_speed > 4.5 or self.cruise_peak_tilt > 20.0 \
                or metrics["cruise_reverse_fraction"] > 0.05:
            raise AssertionError(f"unstable Rescue cruise: {metrics}")
