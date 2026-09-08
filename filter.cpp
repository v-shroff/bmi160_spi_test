#include "filter.h"
#include <cmath>

void filter::update_imu(float gx, float gy, float gz, float ax, float ay, float az, float delta_time_s) {

  float qa, qb, qc;
  float half_gx;
  float half_gy;
  float half_gz;

  // solve for half of current steps error
  float half_ex;
  float half_ey;
  float half_ez;
  float acc_normalize;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) // else u get division by zero
  {
    //
    acc_normalize = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= acc_normalize;
    ay *= acc_normalize;
    az *= acc_normalize;
    // based on our current transform, where should gravity be ?
    half_gx = q1 * q3 - q0 * q2;
    half_gy = q0 * q1 + q2 * q3;
    half_gz = q0 * q0 - 0.5f + q3 * q3;

    // solve for half of current steps error
    half_ex = (ay * half_gz - az * half_gy);
    half_ey = (az * half_gx - ax * half_gz);
    half_ez = (ax * half_gy - ay * half_gx);

    if (two_ki > 0.0f) {
      integral_err_x += two_ki * half_ex * delta_time_s;
      integral_err_y += two_ki * half_ey * delta_time_s;
      integral_err_z += two_ki * half_ez * delta_time_s;
      gx += integral_err_x; // apply integral feedback
      gy += integral_err_y;
      gz += integral_err_z;
    } else {
      integral_err_x = 0.0f;
      integral_err_y = 0.0f;
      integral_err_z = 0.0f;
    }
    gx += two_kp * half_ex;
    gy += two_kp * half_ey;
    gz += two_kp * half_ez;
  }
  // integrate rate of change of quaternion
  gx *= (0.5f * delta_time_s); // pre-multiply common factors
  gy *= (0.5f * delta_time_s);
  gz *= (0.5f * delta_time_s);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // normalise quaternion
  acc_normalize = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= acc_normalize;
  q1 *= acc_normalize;
  q2 *= acc_normalize;
  q3 *= acc_normalize;
}

void filter::computeangles() {
  roll = 57.29578f * atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
  pitch = 57.29578f * asinf(-2.0f * (q1 * q3 - q0 * q2));
  yaw = 57.29578f * atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
}
