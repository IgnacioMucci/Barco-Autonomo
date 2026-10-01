#ifndef PWM_TO_AXIS_HPP
#define PWM_TO_AXIS_HPP

namespace asv_utils {
    // Declaración de la función que traduce microsegundos a ejes normalizados (-1.0 a 1.0)
    float pwm_to_axis(int pwm_us);
}

#endif // PWM_TO_AXIS_HPP