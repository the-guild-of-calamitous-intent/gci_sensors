
#include <math.h>

// Same as mean sea level (MSL) altitude
// Altitude from pressure:
// https://www.mide.com/air-pressure-at-altitude-calculator
//
// Returns meters above sealevel
float pressure_altitude(const float p) {
  // constexpr float Tb = 15; // temperature at sea level [C] - doesn't work
  // constexpr float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... diff exp for pow??
  constexpr float Tb = 288.15f;           // temperature at sea level [K]
  constexpr float Lb = -0.0065f;          // lapse rate [K/m]
  constexpr float R  = 8.31446261815324f; // universal gas const [Nm/(mol K)]
  constexpr float M  = 0.0289644f;        // molar mass of Earth's air [kg/mol]
  constexpr float g0 = 9.80665f;          // gravitational const [m/s^2]
  constexpr float Pb = 101325.0f;         // pressure at sea level [Pa]

  constexpr float exp    = -R * Lb / (g0 * M);
  constexpr float scale  = Tb / Lb;
  constexpr float inv_Pb = 1.0f / Pb;

  return scale * (powf(p * inv_Pb, exp) - 1.0f);
}