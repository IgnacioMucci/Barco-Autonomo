# Importaciones básicas del framework de ROS 2 para manejar descripciones de lanzamiento
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# funcion principal de ros2 para estructurar y retornar acciones y nodos que se van a ejecutar
def generate_launch_description():

    # configuramos argumentos de conexión con la pixhawk (de mavros)
    # Definimos la ruta del puerto de la placa (fcu_url) por USB/Serial con su baudrate,
    # permitiendo cambiarlo fácilmente sin modificar código duro en los ejecutables.
    fcu_url = LaunchConfiguration('fcu_url', default='/dev/ttyACM0:57600')

    # Redirigimos la telemetría hacia Mission Planner en Windows
    # 'host.docker.internal' permite a Docker comunicarse con la IP de mi compu Windows
    # 14550 es el puerto UDP estándar que escucha Mission Planner
    #gcs_url = LaunchConfiguration('gcs_url', default='udp://@host.docker.internal:14550')
    gcs_url = LaunchConfiguration('gcs_url', default='') #por ahora dejamos este para que no rompa nada 

    tgt_system = LaunchConfiguration('tgt_system', default='1')
    tgt_component = LaunchConfiguration('tgt_component', default='1')

    # Lanzamiento, nodos a ejecutar:
    return LaunchDescription([
        # primero lanzamos la capa de comunicación MAVROS con la pixhawk
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'target_system_id': tgt_system,
                'target_component_id': tgt_component
            }],
            output='screen'
        ),

        # Nodo de telemetría para recuperar vars de estado de la placa
        Node(
            package='control_asv',
            executable='nodo_telemetria',
            name='telemetria_node',
            output='screen'
        ),

        # desp nodo personalizado para mandar señales PWM crudas en microsegundos 
        # a través del tópico /mavros/rc/override
        Node(
            package='control_asv',
            executable='nodo_mavros',
            name='mavros_override_node',
            output='screen'  # Redirige los logs y RCLCPP_INFO directamente a la terminal
        ),

        # nodo secundario para el control de velocidad y rumbo intermedio (cmd_vel)
        Node(
            package='control_asv',
            executable='nodo_cmd_vel',
            name='cmd_vel_node',
            output='screen'
        ),

        # nodo de alto nivel para gestión de misiones y waypoints basados en GPS
        Node(
            package='control_asv',
            executable='nodo_gps_waypoint',
            name='gps_node',
            output='screen'
        ),

        # nodo gestor de misión de bajo nivel (porque no le deja libertad a la placa)
        # recibe paquetes mavlink de mission planner y los traduce a instrucciones pwm para la placa de navegación
        Node(
            package='control_asv',
            executable='nodo_gestor_mision_pwm',
            name='gestor_mission_pwm_node',
            output='screen'
        ),

        # nodo gestor de mision de alto nivel (le deja decidir a la placa cómo cumplir el objetivo propuesto)
        # 
        Node(
            package='control_asv',
            executable='nodo_gestor_mision',
            name='gestor_mission_node',
            output='screen'
        ),

        # nodo control de propulsión, son los comandos que le manda la computadora companion a la placa de navegación
        # por ahora serán comandos de velocidad y ángulos, pero también se podrían agregar pwm
        Node(
            package='control_asv',
            executable='nodo_control_propulsion',
            name='propulsion_control_node',
            output='screen'
        ),
        
        # acá seguiremos agregando los nodos q vayamos haciendo y queramos q arranquen al toque
    ])
