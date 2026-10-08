# Establece la versión mínima requerida de CMake para compilar el proyecto
cmake_minimum_required(VERSION 3.8)

# Define el nombre del paquete ROS 2
project(control_asv)

# Verifica si el compilador es GNU C++ o Clang para añadir opciones de compilación estrictas
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Activa las advertencias (warnings) para mantener un código limpio y seguro
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# BÚSQUEDA DE DEPENDENCIAS
find_package(ament_cmake REQUIRED)      # ament_cmake es el sistema de construcción (build system) estándar de ROS 2
find_package(rclcpp REQUIRED)           # Librerías core de ROS 2 (C++)

# Paquetes de tipos de mensajes estándar requeridos por los nodos
find_package(std_msgs REQUIRED)          # Para mensajes básicos (ej. arreglos de enteros para PWM)
find_package(geometry_msgs REQUIRED)     # Necesario para nodo_cmd_vel (Twist / velocidades)
find_package(geographic_msgs REQUIRED)   # Para nodo_gps_waypoint (Latitud/Longitud)
find_package(sensor_msgs REQUIRED)       # Para nodo_ap_dds_joy (sensor_msgs::msg::Joy)
find_package(mavros_msgs REQUIRED)       # para nodo_mavros_override (mavros_msgs::msg::OverrideRCIn)
find_package(ardupilot_msgs REQUIRED)    # para mensajes de ardupilot como estado, posicion, etc.
find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Para las cabeceras  
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include)

# CONFIGURACIÓN DE TESTING 
if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files

  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files

  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

# DECLARACIÓN DE EJECUTABLES

# Nodo Principal - Crea el ejecutable llamando al archivo fuente
add_executable(nodo_principal src/nodo_principal.cpp)
# Vincula el ejecutable con las librerías que utiliza
ament_target_dependencies(nodo_principal rclcpp mavros_msgs)

# Nodo MAVROS (PWM crudo)
add_executable(nodo_mavros src/nodo_mavros.cpp)
ament_target_dependencies(nodo_mavros rclcpp mavros_msgs)

# Nodo AP DDS (traduce PWM -> eje normalizado antes de publicar)
add_executable(nodo_ap_dds_joy src/nodo_ap_dds_joy.cpp utils/pwm_to_axis.cpp)
ament_target_dependencies(nodo_ap_dds_joy rclcpp sensor_msgs)

# Nodo de Velocidades (cmd_vel) 
add_executable(nodo_cmd_vel src/nodo_cmd_vel.cpp)
ament_target_dependencies(nodo_cmd_vel rclcpp geometry_msgs ardupilot_msgs)

# Nodo gestor de mision de destino. Agarra los paquetes de mission planner y 
# le manda un destino (latitud y longitud) a la pixhawk
add_executable(nodo_gestor_mision_destino src/nodo_gestor_mision_destino.cpp)
ament_target_dependencies(nodo_gestor_mision_destino rclcpp geographic_msgs ardupilot_msgs)

# Nodo de Telemetría
add_executable(nodo_telemetria src/nodo_telemetria.cpp)
ament_target_dependencies(nodo_telemetria rclcpp mavros_msgs sensor_msgs geometry_msgs)

# Nodo de Traducción de MavLink a ros2
add_executable(nodo_gestor_mision_dist_rumbo src/nodo_gestor_mision_dist_rumbo.cpp)
ament_target_dependencies(nodo_gestor_mision_dist_rumbo rclcpp geometry_msgs mavros_msgs)

# Nodo de gestor de mision, agarra los paquetes de mission planner y los resuelve
add_executable(nodo_gestor_mision_pwm src/nodo_gestor_mision_pwm.cpp)
ament_target_dependencies(nodo_gestor_mision_pwm rclcpp mavros_msgs)

# Nodo de control de propulsión, una vez obtenida la ruta, la compu companion arma instrucciones para la placa de navegación
# por ahora se controla con velocidades y angulos, pero si se quiere controlar con pwm entonces se debe usar mavros_msgs, y no geometry_msgs
add_executable(nodo_control_propulsion src/nodo_control_propulsion.cpp utils/geo_utils.cpp)
ament_target_dependencies(nodo_control_propulsion rclcpp geographic_msgs sensor_msgs geometry_msgs)

# Nodo de prueba de comunicación entre la Pixhawk y la compu companion.
add_executable(nodo_prueba_comunicacion src/nodo_prueba_comunicacion.cpp)
ament_target_dependencies(nodo_prueba_comunicacion rclcpp builtin_interfaces sensor_msgs std_srvs)

# REGLAS DE INSTALACIÓN
# Le dice a CMake dónde colocar los binarios compilados para que "ros2 run" los encuentre
install(TARGETS
  nodo_principal
  nodo_mavros
  nodo_ap_dds_joy
  nodo_cmd_vel
  nodo_gestor_mision_destino
  nodo_telemetria
  nodo_gestor_mision_dist_rumbo
  nodo_gestor_mision_pwm
  nodo_control_propulsion
  nodo_prueba_comunicacion
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALACIÓN DEL ARCHIVO LAUNCH
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Exporta la configuración del paquete para ament
ament_package()