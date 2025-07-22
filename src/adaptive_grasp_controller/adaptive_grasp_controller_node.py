#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
from collections import deque
import time

class AdaptiveGraspController:
    def __init__(self):
        rospy.init_node('adaptive_grasp_controller_node')

        # Parameters
        self.continuous_delta_threshold = 0.5
        self.abrupt_slope_threshold = 1.0
        self.mu = 1.2
        self.pressure_max = 26.0
        self.pid_pressure = 0.0
        self.delta_p_multiplier = 10.0

        # PID parameters
        self.kp = 0.5
        self.ki = 0.05
        self.kd = 0.2
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

        # Rolling window buffers
        self.window_size = 20
        self.fx_window_s1 = deque(maxlen=self.window_size)
        self.fy_window_s1 = deque(maxlen=self.window_size)
        self.fx_window_s2 = deque(maxlen=self.window_size)
        self.fy_window_s2 = deque(maxlen=self.window_size)

        # Force values
        self.fx_s1 = 0.0
        self.fy_s1 = 0.0
        self.fz_s1 = 0.0
        self.fx_s2 = 0.0
        self.fy_s2 = 0.0
        self.fz_s2 = 0.0

        # State
        self.active = False
        self.ref_force_xy_s1 = 0.0
        self.ref_force_xy_s2 = 0.0
        self.last_time = time.time()

        # Subscribers
        rospy.Subscriber('/start_adaptive_grasp', Bool, self.start_callback)
        rospy.Subscriber('/adaptive_grasp/pid_kp', Float32, self.kp_callback)
        rospy.Subscriber('/adaptive_grasp/pid_ki', Float32, self.ki_callback)
        rospy.Subscriber('/adaptive_grasp/pid_kd', Float32, self.kd_callback)
        rospy.Subscriber('/prediction_fx_S1', Float32, self.fx_s1_callback)
        rospy.Subscriber('/prediction_fy_S1', Float32, self.fy_s1_callback)
        rospy.Subscriber('/prediction_fz_S1', Float32, self.fz_s1_callback)
        rospy.Subscriber('/prediction_fx_S2', Float32, self.fx_s2_callback)
        rospy.Subscriber('/prediction_fy_S2', Float32, self.fy_s2_callback)
        rospy.Subscriber('/prediction_fz_S2', Float32, self.fz_s2_callback)

        # Publishers
        self.pid_pressure_pub = rospy.Publisher('/adaptive_pressure_delta', Float32, queue_size=10)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    def start_callback(self, msg):
        if msg.data:
            self.active = True
            self.pid_pressure = 0.0
            self.reset_reference()

            self.publish_diagnostics(True, "Adaptive grasp started with new reference forces.")

    def reset_reference(self):
        self.fx_window_s1.clear()
        self.fy_window_s1.clear()
        self.fx_window_s2.clear()
        self.fy_window_s2.clear()
        self.prev_error = 0.0
        self.integral_error = 0.0

    def update(self):
        if not self.active:
            return

        # Update windows
        self.fx_window_s1.append(self.fx_s1)
        self.fy_window_s1.append(self.fy_s1)
        self.fx_window_s2.append(self.fx_s2)
        self.fy_window_s2.append(self.fy_s2)

        # Only proceed if enough data
        if len(self.fx_window_s1) < self.window_size:
            return

        # Compute averages
        avg_fx_fy_s1 = np.hypot(np.mean(self.fx_window_s1), np.mean(self.fy_window_s1))
        avg_fx_fy_s2 = np.hypot(np.mean(self.fx_window_s2), np.mean(self.fy_window_s2))

        delta1 = abs(avg_fx_fy_s1 - self.ref_force_xy_s1)
        delta2 = abs(avg_fx_fy_s2 - self.ref_force_xy_s2)

        current_time = time.time()
        dt = current_time - self.last_time

        # Compute slopes
        slope1 = abs(avg_fx_fy_s1 - self.ref_force_xy_s1) / dt if dt > 0 else 0.0
        slope2 = abs(avg_fx_fy_s2 - self.ref_force_xy_s2) / dt if dt > 0 else 0.0

        slip_detected = (
            delta1 > self.continuous_delta_threshold or slope1 > self.abrupt_slope_threshold or
            delta2 > self.continuous_delta_threshold or slope2 > self.abrupt_slope_threshold
        )

        if slip_detected:
            avg_fx_fy = (avg_fx_fy_s1 + avg_fx_fy_s2) / 2.0
            desired_z = avg_fx_fy / (2.0 * self.mu)
            current_z = (self.fz_s1 + self.fz_s2) / 2.0
            error = desired_z - current_z

            self.integral_error += error * dt
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

            pid_output = self.kp * error + self.ki * self.integral_error + self.kd * derivative
            self.pid_pressure += pid_output

            self.pid_pressure = max(0.0, min(self.pid_pressure, self.pressure_max))
            self.pid_pressure_pub.publish(Float32(self.pid_pressure))

            # Reset windows and update reference
            self.reset_reference()
            self.ref_force_xy_s1 = avg_fx_fy_s1
            self.ref_force_xy_s2 = avg_fx_fy_s2
            self.prev_error = error
            self.last_time = current_time

            self.publish_diagnostics(True, f"Slip detected, ΔP={pid_output:.2f}, P={self.pid_pressure:.2f}")

    # --- PID param callbacks ---
    def kp_callback(self, msg): self.kp = msg.data
    def ki_callback(self, msg): self.ki = msg.data
    def kd_callback(self, msg): self.kd = msg.data

    # --- Force updates ---
    def fx_s1_callback(self, msg): self.fx_s1 = msg.data; self.update()
    def fy_s1_callback(self, msg): self.fy_s1 = msg.data; self.update()
    def fz_s1_callback(self, msg): self.fz_s1 = msg.data
    def fx_s2_callback(self, msg): self.fx_s2 = msg.data; self.update()
    def fy_s2_callback(self, msg): self.fy_s2 = msg.data; self.update()
    def fz_s2_callback(self, msg): self.fz_s2 = msg.data

    def publish_diagnostics(self, status, message):
        diag = DiagnosticArray()
        diag.header.stamp = rospy.Time.now()
        stat = DiagnosticStatus()
        stat.name = "AdaptiveGraspController"
        stat.level = DiagnosticStatus.OK if status else DiagnosticStatus.WARN
        stat.message = message
        stat.hardware_id = "adaptive_grasp"
        stat.values = [
            KeyValue("Active", str(self.active)),
            KeyValue("PID_Pressure", f"{self.pid_pressure:.2f}"),
            KeyValue("Fz_S1", f"{self.fz_s1:.2f}"),
            KeyValue("Fz_S2", f"{self.fz_s2:.2f}")
        ]
        diag.status.append(stat)
        self.diag_pub.publish(diag)

if __name__ == '__main__':
    try:
        node = AdaptiveGraspController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
