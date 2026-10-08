#ifndef GEO_UTILS_HPP
#define GEO_UTILS_HPP

#include <tuple>

/*  Recibe dos coordenadas GPS (la ubicación actual del barco y el destino) y 
    devuelve la distancia exacta en metros y el ángulo de rumbo en radianes.    */

namespace asv_utils {
    // Retorna una tupla con {distancia_metros, rumbo_radianes}
    std::tuple<double, double> calcular_distancia_rumbo(
        double lat_actual, double lon_actual, 
        double lat_objetivo, double lon_objetivo);
}

#endif // GEO_UTILS_HPP