from pymavlink import mavutil

# Cambiá esto por el puerto real de la Pixhawk.
# Windows: "COM5"
# Ubuntu/WSL: "/dev/ttyACM0"
PORT = "COM10"
BAUD = 115200

CANAL = 9       # AUX1 en una configuración estándar
PWM_US = 1500   # Ancho del pulso solicitado

# Abrir conexión MAVLink directa.
link = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=255,
    source_component=190
)

try:
    print("Esperando HEARTBEAT de la Pixhawk...")
    link.wait_heartbeat(timeout=30)

    print(
        f"Conectado: sistema={link.target_system}, "
        f"componente={link.target_component}"
    )

    # Enviar directamente COMMAND_LONG.
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,          # confirmation
        float(CANAL),
        float(PWM_US),
        0.0, 0.0, 0.0, 0.0, 0.0
    )

    print(f"Comando enviado: canal {CANAL}, PWM {PWM_US} us")
    print("Esperando respuesta COMMAND_ACK...")

    while True:
        respuesta = link.recv_match(
            type="COMMAND_ACK",
            blocking=True,
            timeout=5
        )

        if respuesta is None:
            print("No llegó COMMAND_ACK dentro del tiempo esperado.")
            break

        if respuesta.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            if respuesta.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("ArduPilot aceptó el comando.")
            else:
                print(
                    f"Comando rechazado o no completado. "
                    f"Resultado MAVLink: {respuesta.result}"
                )
            break

finally:
    link.close()