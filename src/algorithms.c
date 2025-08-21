
#include <math.h>

#include "gci_sensors/algorithms.h"

// Same as mean sea level (MSL) altitude
// Altitude from pressure:
// https://www.mide.com/air-pressure-at-altitude-calculator
//
// Returns meters above sealevel
float pressure_altitude(const float p) {
  // const float Tb = 15; // temperature at sea level [C] - doesn't work
  // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... diff exp for pow??
  const float Tb = 288.15f;           // temperature at sea level [K]
  const float Lb = -0.0065f;          // lapse rate [K/m]
  const float R  = 8.31446261815324f; // universal gas const [Nm/(mol K)]
  const float M  = 0.0289644f;        // molar mass of Earth's air [kg/mol]
  const float g0 = 9.80665f;          // gravitational const [m/s^2]
  const float Pb = 101325.0f;         // pressure at sea level [Pa]

  const float exp    = -R * Lb / (g0 * M);
  const float scale  = Tb / Lb;
  const float inv_Pb = 1.0f / Pb;

  return scale * (powf(p * inv_Pb, exp) - 1.0f);
}

int vec_calibrate(float *cal, vec3f_t *v) {
  if ((cal == NULL) || (v == NULL)) return -1;

  vec3f_t m = *v;
  v->x = cal[0] * m.x + cal[1] * m.y + cal[2] * m.z - cal[3];
  v->y = cal[4] * m.x + cal[5] * m.y + cal[6] * m.z - cal[7];
  v->z = cal[8] * m.x + cal[9] * m.y + cal[10] * m.z - cal[11];

  return 0;
}

int imu_calibrate(float *acal, float *gcal, imuf_t *imu) {
  if ((acal == NULL) || (gcal == NULL) || (imu == NULL)) return -1;

  // vec3f_t m = imu->a;
  // accel = A * accel_meas - bias
  // imu->a.x = acal[0] * m.x + acal[1] * m.y + acal[2] * m.z - acal[3];
  // imu->a.y = acal[4] * m.x + acal[5] * m.y + acal[6] * m.z - acal[7];
  // imu->a.z = acal[8] * m.x + acal[9] * m.y + acal[10] * m.z - acal[11];
  if (vec_calibrate(acal, &(imu->a)) < 0) return -1;

  // m = imu->g;
  // // gyro = A * gyro_meas - bias
  // imu->g.x = gcal[0] * m.x + gcal[1] * m.y + gcal[2] * m.z - gcal[3];
  // imu->g.y = gcal[4] * m.x + gcal[5] * m.y + gcal[6] * m.z - gcal[7];
  // imu->g.z = gcal[8] * m.x + gcal[9] * m.y + gcal[10] * m.z - gcal[11];
  if (vec_calibrate(gcal, &(imu->g)) < 0) return -1;

  return 0;
}

// void set_cal_data(float *dst, float src[12]) {
//   // if (hw == NULL) return GCIS_ERROR_IO_NULL;
//   const uint32_t size = 12 * sizeof(float);
//   memcpy(dst, src, size);
// }