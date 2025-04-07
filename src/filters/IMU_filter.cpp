#include "IMU_filter.hpp"

void IMU_filter::init_EKF_6axis(IMU_data data) {
  // Set a initial value for the quaternion
  float recipNorm =
      1.0f / sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y +
                  data.accel_Z * data.accel_Z);
  data.accel_X = data.accel_X * recipNorm;
  data.accel_Y = data.accel_Y * recipNorm;
  data.accel_Z = data.accel_Z * recipNorm; // This is the unit gravity
  float axis_norm =
      sqrt(data.accel_Y * data.accel_Y + data.accel_X * data.accel_X);
  float sin_halfangle = sinf(acosf(data.accel_Z) / 2.0f);
  if (axis_norm > 1e-6) {
    x[0] = cosf(data.accel_Z / 2.0f);
    x[1] = data.accel_Y * sin_halfangle / axis_norm;
    x[2] = -data.accel_X * sin_halfangle / axis_norm;
    x[3] = 1 - x[0] * x[0] - x[1] * x[1] - x[2] * x[2];
  } else {
    x[0] = 1;
    x[1] = 0;
    x[2] = 0;
    x[3] = 0;
  }
  // After test. within 0.5 second pitch and roll will get to the right value
  // when IMU facing up;

  // Since the weight of Q and R for all element should be the same
  Q = 1;
  R = 1000;
  P[0] = {1000, 0, 0, 0};
  P[1] = {0, 1000, 0, 0};
  P[2] = {0, 0, 1000, 0};
  P[3] = {0, 0, 0, 1000};
}

int IMU_filter::step_EKF_6axis(IMU_data data) {
  // For unit Quaternions, The seperation of garvity is what q_i, q_j, q_k
  // should be
  dt = timer.delta();
  float gravity_now =
      sqrt(data.accel_X * data.accel_X + data.accel_Y * data.accel_Y +
           data.accel_Z * data.accel_Z);
  float recipNorm = 1.0f / gravity_now;
  float unit_accel_X = data.accel_X * recipNorm;
  float unit_accel_Y = data.accel_Y * recipNorm;
  float unit_accel_Z = data.accel_Z * recipNorm; // This is the unit gravity

  // Helping numbers for F
  float helpgx = (data.gyro_X * dt) * 0.5f;
  float helpgy = (data.gyro_Y * dt) * 0.5f;
  float helpgz = (data.gyro_Z * dt) * 0.5f;
  // Predict for x
  float x0 = x[0];
  float x1 = x[1];
  float x2 = x[2];
  float x3 = x[3];

  x[0] = x0 - x1 * helpgx - x2 * helpgy - x3 * helpgz;
  x[1] = x0 * helpgx + x1 + x2 * helpgz - x3 * helpgy;
  x[2] = x0 * helpgy - x1 * helpgz + x2 + x3 * helpgx;
  x[3] = x0 * helpgz + x1 * helpgy - x2 * helpgx + x3;

  recipNorm =
      1.0f / sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
  x[0] *= recipNorm;
  x[1] *= recipNorm;
  x[2] *= recipNorm;
  x[3] *= recipNorm;

  // unit x
  // float invqua = 1/sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
  // x[0] = x[0] * invqua;
  // x[1] = x[1] * invqua;
  // x[2] = x[2] * invqua;
  // x[3] = x[3] * invqua;
  // predict for P
  float F[4][4] = {{1, -helpgx, -helpgy, -helpgz},
                   {helpgx, 1, helpgz, -helpgy},
                   {helpgy, -helpgz, 1, helpgx},
                   {helpgz, helpgy, -helpgx, 1}};
  // [        1, -(dt*gx)/2, -(dt*gy)/2, -(dt*gz)/2]
  // [(dt*gx)/2,          1,  (dt*gz)/2, -(dt*gy)/2]
  // [(dt*gy)/2, -(dt*gz)/2,          1,  (dt*gx)/2]
  // [(dt*gz)/2,  (dt*gy)/2, -(dt*gx)/2,          1] Jacobian function F state
  // transition model

  // float F_transpose[4][4] = {0};
  float FxP[4][4] = {0};
  // float PxF_transpose[4][4] = {0};

  // Calculate FxPxF_transpose
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        FxP[i][j] += F[i][k] * P[k][j];
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        P[i][j] += FxP[i][k] * F[j][k]; // P = F*P*F^T + Q
        if (i == j) {
          P[i][j] += Q * dt;
        }
      }
    }
  }
  // // Updata
  float H[3][4] = {{-2 * x[2], 2 * x[3], -2 * x[0], 2 * x[1]},
                   {2 * x[1], 2 * x[0], 2 * x[3], 2 * x[2]},
                   {2 * x[0], -2 * x[1], -2 * x[2], 2 * x[3]}};
  // [-2*q2,  2*q3, -2*q0, 2*q1]
  // [ 2*q1,  2*q0,  2*q3, 2*q2]
  // [ 2*q0, -2*q1, -2*q2, 2*q3] Jacobian function H observation model

  float PH_transpose[4][3] = {0};
  float HPH_transpose[3][3] = {0};
  // Calculate HPH_transpose
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 4; k++) {
        PH_transpose[i][j] += P[i][k] * H[j][k];
      }
    }
  }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 4; k++) {
        HPH_transpose[i][j] += H[i][k] * PH_transpose[k][j];
        if (i == j) {
          HPH_transpose[i][j] += R;
        }
      }
    }
  }
  float HPH_transpose_inv[3][3] = {0};
  inverse3x3(HPH_transpose, HPH_transpose_inv);
  float K[4][3] = {0};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        K[i][j] += PH_transpose[i][k] * HPH_transpose_inv[k][j];
      }
    }
  }
  // caulculate the error between the predicted and the measured
  float g[3] = {unit_accel_X - 2 * (x[1] * x[3] - x[0] * x[2]),
                unit_accel_Y - 2 * (x[2] * x[3] + x[0] * x[1]),
                unit_accel_Z - (1 - 2 * (x[1] * x[1] + x[2] * x[2]))};

  if (gravity_now > 9.7 && gravity_now < 9.9) {
    // Updata x
    for (int i = 1; i < 3; i++) {
      x[i] += K[i][0] * g[0] + K[i][1] * g[1] + K[i][2] * g[2];
    }
    recipNorm =
        1.0f / sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);
    x[0] *= recipNorm;
    x[1] *= recipNorm;
    x[2] *= recipNorm;
    x[3] *= recipNorm;
    // Update P
    float KH[4][4] = {0};
    float I[4][4] = {0};
    for (int i = 0; i < 4; i++) {
      I[i][i] = 1.0f;
    }

    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 3; k++) {
          KH[i][j] += K[i][k] * H[k][j];
        }
      }
    }
    float temp2[4][4] = {0};
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 4; k++) {
          temp2[i][j] += (I[i][k] - KH[i][k]) * P[k][j];
        }
      }
    }
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        P[i][j] = temp2[i][j];
      }
    }
  }
  float temp1 = filtered_data.pitch;
  float temp2 = filtered_data.roll;
  float temp3 = filtered_data.yaw;
  filtered_data = data; // Reset all filtered data to the raw data
  filtered_data.pitch = (atan2f(2.0f * (x[0] * x[1] + x[2] * x[3]),
                                1.0f - 2.0f * (x[1] * x[1] + x[2] * x[2])));
  if (abs(filtered_data.pitch) > M_PI_2) {
    filtered_data.roll = -(asinf(2.0f * (x[0] * x[2] - x[1] * x[3])));
    if (filtered_data.roll > 0) {
      filtered_data.roll -= M_PI;
    } else {
      filtered_data.roll += M_PI;
    }
  } else {
    filtered_data.roll = asinf(2.0f * (x[0] * x[2] - x[1] * x[3]));
  }
  filtered_data.yaw = atan2f(2.0f * (x[0] * x[3] + x[1] * x[2]),
                             1.0f - 2.0f * (x[2] * x[2] + x[3] * x[3]));
  filtered_data.gyro_yaw = (filtered_data.yaw - temp3) / dt;
  filtered_data.gyro_roll = (filtered_data.roll - temp2) / dt;
  filtered_data.gyro_pitch = (filtered_data.pitch - temp1) / dt;
  // Convert to the original
  filtered_data.accel_world_X =
      filtered_data.accel_X * (1 - 2 * (x[2] * x[2] + x[3] * x[3])) +
      filtered_data.accel_Y * (2 * (x[1] * x[2] - x[3] * x[0])) +
      filtered_data.accel_Z * (2 * (x[1] * x[3] + x[0] * x[2]));
  filtered_data.accel_world_Y =
      filtered_data.accel_X * (2 * (x[1] * x[2] + x[3] * x[0])) +
      filtered_data.accel_Y * (1 - 2 * (x[1] * x[1] + x[3] * x[3])) +
      filtered_data.accel_Z * (2 * (x[2] * x[3] - x[1] * x[0]));
  filtered_data.accel_world_Z =
      filtered_data.accel_X * (2 * (x[1] * x[3] - x[2] * x[0])) +
      filtered_data.accel_Y * (2 * (x[2] * x[3] + x[1] * x[0])) +
      filtered_data.accel_Z * (1 - 2 * (x[1] * x[1] + x[2] * x[2])) -
      SENSORS_GRAVITY_EARTH;
  return 0;
}

IMU_data *IMU_filter::get_filter_data() { return &filtered_data; }

void IMU_filter::inverse3x3(float mat[3][3], float inv[3][3]) {
  float det = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) -
              mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
              mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

  if (det == 0) {
    // Matrix is not invertible
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        inv[i][j] = mat[i][j];

    return;
  }

  float invDet = 1.0f / det;

  inv[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) * invDet;
  inv[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) * invDet;
  inv[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * invDet;
  inv[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) * invDet;
  inv[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * invDet;
  inv[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) * invDet;
  inv[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) * invDet;
  inv[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) * invDet;
  inv[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) * invDet;
}
