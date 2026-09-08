#pragma once

class filter {
public:
  float roll; // euler for printing
  float pitch;
  float yaw;
  // quaternion state can be found using another function
  void update_imu(float gx, float gy, float gz, float ax, float ay, float az, float delta_time_s);
  void computeangles();

  float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // the initial position is
private:
  //
  float integral_err_x = 0.0f, integral_err_y = 0.0f, integral_err_z = 0.0f;
  float ki = 0.01f;
  float kp = 10.0f;
  float two_ki = 2 * ki;
  float two_kp = 2 * kp;
  // updates the current pos estimate given gyro measurments in rad/s. raw
};
