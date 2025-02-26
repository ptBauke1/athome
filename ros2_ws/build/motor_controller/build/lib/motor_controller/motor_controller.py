import rclpy
from rclpy.node import Node
import serial
import sys
import termios
import tty

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial')
        
        # Configuração da porta serial
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.get_logger().info('Serial conectado em /dev/ttyACM0    ')
        
        self.get_logger().info("Use WASD para mover e QE para girar. Pressione 'ESC' para sair.")
        self.run()
    
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        vel_x = 0.0
        vel_y = 0.0
        rot_z = 0.0
        step = 0.1
        rotation_step = 0.1
        
        while rclpy.ok():
            key = self.get_key()
            
            if key == 'w':
                vel_x += step
            elif key == 's':
                vel_x -= step
            elif key == 'a':
                vel_y += step
            elif key == 'd':
                vel_y -= step
            elif key == 'q':
                rot_z += rotation_step
            elif key == 'e':
                rot_z -= rotation_step
            elif key == '\x1b':  # ESC para sair
                break
            
            # Criar string de comando
            command = f'x:{vel_x:.2f},y:{vel_y:.2f},z:{rot_z:.2f}\n'
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().info(f'Enviado: {command.strip()}')
        
        self.serial_port.close()
        self.get_logger().info('Serial desconectado.')
        

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
