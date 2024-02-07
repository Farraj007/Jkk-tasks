import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pacmod3_msgs.msg import SteeringCmd
from pacmod3_msgs.msg import VehicleSpeedRpt, SteeringAuxRpt, SteeringCmd
import pygame
import math
import inspect

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.screen_width = 600
        self.screen_height = 400

        self.init_pygame()
        self.init_subscriptions()

        self.last_driving_status = None
        self.last_reference_speed = None
        self.last_current_speed = None
        self.last_steering_angle = None
        self.last_current_steering_angle=None

        self.knob_center=None
        
    def init_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode([self.screen_width, self.screen_height])
        pygame.display.set_caption("ROS2 Data Visualization")
        self.font = pygame.font.SysFont(None, 36)
        self.steering_wheel_image = pygame.image.load('task/task/steering_wheel.svg').convert_alpha()
        self.steering_wheel_image2 = pygame.image.load('task/task/steering_wheel2.svg').convert_alpha()
        self.steering_wheel_image.set_alpha(180)
        
    def init_subscriptions(self):
        self.subscription = self.create_subscription(Bool, '/lexus3/pacmod/enabled', self.listener_callback, 10)
        self.subscription1 = self.create_subscription(Twist, '/lexus3/cmd_vel', self.listener_callback1, 10)
        self.subscription2 = self.create_subscription(VehicleSpeedRpt, '/lexus3/pacmod/vehicle_speed_rpt', self.listener_callback2, 10)
        self.subscription3 = self.create_subscription(SteeringCmd, '/lexus3/pacmod/steering_aux_rpt', self.listener_callback3, 10)
        self.subscription4 = self.create_subscription(SteeringAuxRpt, '/lexus3/pacmod/steering_cmd', self.listener_callback4, 10)

    def update_display(self):
        self.screen.fill((0, 0, 0))

        if self.last_driving_status is not None:
            text = self.font.render(self.last_driving_status, True, (0, 255, 0) if self.last_driving_status.startswith('You') else (255, 0, 0))
            self.screen.blit(text, (200, 20))

        if self.last_reference_speed is not None:
            text = self.font.render('Reference Speed: {:.2f}Km/h'.format(self.last_reference_speed), True, (0, 0, 255))
            self.screen.blit(text, (150, 70))

        if self.last_current_speed is not None:
            text = self.font.render('Current Speed: {:.2f}Km/h'.format(self.last_current_speed), True, (255, 0, 0))
            self.screen.blit(text, (150, 50))
            
        
        #printing a photo
        if self.last_current_steering_angle is not None:
            rotated_steering_wheel = pygame.transform.rotate(self.steering_wheel_image, self.last_current_steering_angle)
            self.screen.blit(rotated_steering_wheel, ((400 - rotated_steering_wheel.get_width()) // 2, (500 - rotated_steering_wheel.get_height()) // 2))

            text = self.font.render('Current angle: {:.2f}°'.format(self.last_current_steering_angle), True, (255, 0, 0))
            self.screen.blit(text, (325, 200))
            
        if self.last_steering_angle is not None:
            rotated_steering_wheel = pygame.transform.rotate(self.steering_wheel_image2, self.last_steering_angle)
            self.screen.blit(rotated_steering_wheel, ((400 - rotated_steering_wheel.get_width()) // 2, (500 - rotated_steering_wheel.get_height()) // 2))

            text = self.font.render('Ref angle: {:.2f}°'.format(self.last_steering_angle), True, (0, 0, 255))
            self.screen.blit(text, (325, 220))

        pygame.display.flip()
        #drawing
        # if self.last_current_steering_angle is not None:
        #     self.draw_steering_wheel((self.screen_width // 2, self.screen_height // 2), self.last_current_steering_angle)
            
        # if self.last_steering_angle is not None:
        #     self.draw_ref_knob((self.screen_width // 2, self.screen_height // 2),self.last_steering_angle)

        # pygame.display.flip()  

    # def draw_steering_wheel(self, position, angle):
    
    #     wheel_radius = 100
    #     knob_radius = 20
        
    #     pygame.draw.circle(self.screen, (255, 255, 255), position, wheel_radius, 2)
    #     knob_center = (position[0] + int(math.cos(math.radians(angle)) * wheel_radius),
    #                    position[1] - int(math.sin(math.radians(angle)) * wheel_radius))
    #     pygame.draw.circle(self.screen, (255, 0, 0), knob_center, knob_radius)
    #     text = self.font.render('Current angle: {:.2f}°'.format(angle), True, (255,0,0))
    #     self.screen.blit(text, (325, 300))

    # def draw_ref_knob(self,position,angle):
    #     wheel_radius = 100
    #     knob_radius = 20

    #     knob_center = (position[0] + int(math.cos(math.radians(angle)) * wheel_radius),
    #                    position[1] - int(math.sin(math.radians(angle)) * wheel_radius))
    #     pygame.draw.circle(self.screen, (0, 0, 255), knob_center, knob_radius)
    #     text = self.font.render('Ref angle: {:.2f}°'.format(angle), True, (0,0,255))
    #     self.screen.blit(text, (325, 320))

    def listener_callback(self, msg):
        self.last_driving_status = 'You Are Driving' if msg.data else 'In Car Driver'
        self.update_display()

    def listener_callback1(self, msg):
        self.last_reference_speed = msg.linear.x
        self.update_display()

    def listener_callback2(self, msg):
        self.last_current_speed = msg.vehicle_speed
        self.update_display()

    def listener_callback3(self, msg):
        self.last_current_steering_angle = math.degrees(msg.rotation_rate)
        self.update_display()

    def listener_callback4(self, msg):
        self.last_steering_angle = math.degrees(msg.rotation_rate)
        self.update_display()
        pass

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
