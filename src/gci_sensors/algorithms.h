////////////////////////////////////////////////
//  The MIT License (MIT)
//  Copyright (c) 2022 Kevin Walchko
//  see LICENSE for full details
////////////////////////////////////////////////
#pragma once

#if defined __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "gci_sensors/typedefs.h"

float pressure_altitude(const float pressure_pa);

int vec_calibrate(float *cal, vec3f_t *v);
int imu_calibrate(float *acal, float *gcal, imuf_t *imu);

#if defined __cplusplus
}
#endif