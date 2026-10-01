#include "control_asv/utils/geo_utils.hpp"
#include <cmath>

namespace asv_utils {
    std::tuple<double, double> calcular_distancia_rumbo(double lat_actual, double lon_actual, double lat_objetivo, double lon_objetivo){
        double lat_rad = lat_actual * M_PI / 180.0;
        double d_lat = lat_objetivo - lat_actual;
        double d_lon = lon_objetivo - lon_actual;
        
        double metros_lat = d_lat * 111320.0;  
        double metros_lon = d_lon * 40075000.0 * std::cos(lat_rad) / 360.0; //teniendo en cuenta la curvatura de la tierra
        
        double distancia = std::sqrt(metros_lat*metros_lat + metros_lon*metros_lon);
        double rumbo = std::atan2(metros_lon, metros_lat); 
        
        return {distancia, rumbo};
    }
}