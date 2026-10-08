#include "utils/pwm_to_axis.hpp" // nuestra función de traducción
#include <algorithm>

namespace asv_utils
{
// Convierte un valor de PWM en µs a un eje normalizado [-1.0, 1.0],
// asumiendo mapeo lineal simétrico alrededor de rc_mid.
// rc_min/rc_mid/rc_max deben coincidir con los parámetros reales
// RCx_MIN / RCx_TRIM / RCx_MAX cargados en la Pixhawk (verificar en
// Mission Planner, no asumir 1000/1500/2000 a ciegas).
float pwm_to_axis(int pwm_us)
{
  int rc_min = 1000;
  int rc_mid = 1500;
  int rc_max = 2000;
  pwm_us = std::clamp(pwm_us, rc_min, rc_max);
  if (pwm_us >= rc_mid) {
    return static_cast<float>(pwm_us - rc_mid) / static_cast<float>(rc_max - rc_mid);
  } else {
    return static_cast<float>(pwm_us - rc_mid) / static_cast<float>(rc_mid - rc_min);
  }
}
}  // namespace asv_utils
