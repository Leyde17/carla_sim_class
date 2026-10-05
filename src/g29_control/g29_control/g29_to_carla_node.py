import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from carla_msgs.msg import CarlaEgoVehicleControl

class G29ToCarlaNode(Node):
    def __init__(self):
        super().__init__('g29_to_carla_node')
        
        # Axis mappings based on your jstest verification
        self.steer_axis = 0
        self.throttle_axis = 2
        self.brake_axis = 3
        
        self.reverse_button = 4 
        self.reverse_engaged = False
        self.last_reverse_button_state = 0
        
        # Hardware Initialization Flags
        self.throttle_ready = False
        self.brake_ready = False
        
        # Deadzones
        self.steer_deadzone = 0.00
        
        self.declare_parameter('role_name', 'ego_vehicle')
        role_name = self.get_parameter('role_name').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(
            CarlaEgoVehicleControl,
            f'/carla/{role_name}/vehicle_control_cmd',
            10)
            
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        control_msg = CarlaEgoVehicleControl()
        
        # 1. Steering with Deadzone
        raw_steer = float(-msg.axes[self.steer_axis])
        if abs(raw_steer) < self.steer_deadzone:
            control_msg.steer = 0.0
        else:
            control_msg.steer = raw_steer

        # 2. Throttle Mapping (Inverted Hardware)
        # Waiting for initial hardware ping to avoid 50% throttle at launch
        if not self.throttle_ready:
            if abs(msg.axes[self.throttle_axis]) > 0.9:
                self.throttle_ready = True
            control_msg.throttle = 0.0
        else:
            # Map from Rest (-1.0) to Pressed (1.0) -> CARLA (0.0 to 1.0)
            val = (msg.axes[self.throttle_axis] + 1.0) / 2.0
            control_msg.throttle = max(0.0, min(1.0, float(val)))

        # 3. Brake Mapping (Inverted Hardware)
        if not self.brake_ready:
            if abs(msg.axes[self.brake_axis]) > 0.9:
                self.brake_ready = True
            control_msg.brake = 0.0
        else:
            # Map from Rest (-1.0) to Pressed (1.0) -> CARLA (0.0 to 1.0)
            val = (msg.axes[self.brake_axis] + 1.0) / 2.0
            control_msg.brake = max(0.0, min(1.0, float(val)))
        
        # 4. Reverse Toggle
        current_reverse_state = msg.buttons[self.reverse_button]
        if current_reverse_state == 1 and self.last_reverse_button_state == 0:
            self.reverse_engaged = not self.reverse_engaged
        self.last_reverse_button_state = current_reverse_state
        
        control_msg.reverse = self.reverse_engaged
        
        self.publisher_.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    node = G29ToCarlaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()