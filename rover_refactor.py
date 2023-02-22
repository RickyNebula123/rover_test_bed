import keyboard
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class motor:
    def __init__(self, duty_cycle, bwd, fwd):
        self.duty_cycle = duty_cycle        # Duty cycle pin

        # Direction pins
        self.fwd = fwd                      
        self.bwd = bwd

        # Motor speed
        self.speed = 0

        self.initialize_connections()
        self.start_PWM()

    def initialize_connections(self):
        GPIO.setup(self.duty_cycle, GPIO.OUT)
        GPIO.setup(self.fwd, GPIO.OUT)   # Register motor connections with RPi
        GPIO.setup(self.bwd, GPIO.OUT)

    def start_PWM(self):
        self.motor_control = GPIO.PWM(self.duty_cycle, 1000) # Create PWM instance to drive motor.
        self.motor_control.start(0)

    def fwd_(self):
        GPIO.output(self.bwd, GPIO.LOW)
        GPIO.output(self.fwd, GPIO.HIGH)

    def bwd_(self):
        GPIO.output(self.fwd, GPIO.LOW)
        GPIO.output(self.bwd, GPIO.HIGH)

    def stop(self):
        self.motor_control.stop()
        self.motor_control.start(0)
        self.speed = 0

    def update_motor_speed(self, speed):
        self.motor_control.ChangeDutyCycle(speed)

class rover:
    MAX_SPEED = 90
    MIN_SPEED = 0
    SPEED_INCREMENT = 10

    def __init__(self):
        self.speed     = 0
        self.direction = 0                      # 0 equals forward | 1 equals backwards
        self.left_wheels_speed = self.speed
        self.right_wheels_speed = self.speed

        self.initialize_motors()

    def initialize_motors(self):       
        # Motor speed pins; Enable pins
        speed_pin_FR = 18
        speed_pin_FL = 12
        speed_pin_BR = 13
        speed_pin_BL = 19

        # Motor direction pins: (For duty cycle control)
        right_motor_dir_pin1F = 23 # Front
        right_motor_dir_pin2F = 24 # Front
        left_motor_dir_pin1F  = 25 # Front
        left_motor_dir_pin2F  = 8  # Front

        right_motor_dir_pin1B = 22 # Back
        right_motor_dir_pin2B = 27  # Back
        left_motor_dir_pin1B  = 5  # Back
        left_motor_dir_pin2B  = 6  # Back

        # Create motor instances -> These need to be verified.
        self.front_right_motor = motor(speed_pin_FR, right_motor_dir_pin1F, right_motor_dir_pin2F)
        self.front_left_motor  = motor(speed_pin_FL, left_motor_dir_pin1F, left_motor_dir_pin2F)
        self.back_right_motor  = motor(speed_pin_BR, right_motor_dir_pin1B, right_motor_dir_pin2B)
        self.back_left_motor   = motor(speed_pin_BL, left_motor_dir_pin1B, left_motor_dir_pin2B)
        self.motors = [self.front_right_motor, self.front_left_motor, self.back_right_motor, self.back_left_motor]

    def speed_up(self, direction_handler):                                  # direction is function object
        adjustment = rover.SPEED_INCREMENT
        if(self.speed + rover.SPEED_INCREMENT >= rover.MAX_SPEED): 
            adjustment = 0
        
        self.speed += adjustment 
        self.left_wheels_speed += adjustment 
        self.right_wheels_speed += adjustment 

        print(f"l: {self.left_wheels_speed} r: {self.right_wheels_speed} s: {self.speed}")
        direction_handler(self.speed)
        
    def slow_down(self, direction_handler):                                 # dir is a function object.
        adjustment = rover.SPEED_INCREMENT
        if(self.speed - rover.SPEED_INCREMENT <= rover.MIN_SPEED):
            self.direction = not self.direction     # Update direction of the rover
            adjustment = 0
        
        self.speed -= adjustment 
        self.left_wheels_speed -= adjustment 
        self.right_wheels_speed -= adjustment 

        print(f"l: {self.left_wheels_speed} r: {self.right_wheels_speed} s: {self.speed}")

        direction_handler(self.speed)

    def move_fwd(self, speed):
        for motor in self.motors:
            motor.fwd_()
        for motor in self.motors:
            motor.update_motor_speed(speed)

    def move_bwd(self, speed):
        for motor in self.motors:
            motor.bwd_()
        for motor in self.motors:
            motor.update_motor_speed(speed)

    def turn_left_wheels(self, speed):
        self.front_left_motor.update_motor_speed(speed)
        self.back_left_motor.update_motor_speed(speed)

    def turn_left(self):
        if self.left_wheels_speed - rover.SPEED_INCREMENT < rover.MIN_SPEED: 
            self.left_wheels_speed = rover.MIN_SPEED
        else:
            self.left_wheels_speed -= rover.SPEED_INCREMENT

        print(f"Left wheel speed: {self.left_wheels_speed}")
        self.turn_left_wheels(self.left_wheels_speed)

    def turn_right_wheels(self, speed):
        self.front_right_motor.update_motor_speed(speed)
        self.back_right_motor.update_motor_speed(speed)

    def turn_right(self):
        if self.right_wheels_speed - rover.SPEED_INCREMENT < rover.MIN_SPEED: 
            self.right_wheels_speed = rover.MIN_SPEED
        else:
            self.right_wheels_speed -= rover.SPEED_INCREMENT

        print(f"Right wheel speed: {self.right_wheels_speed}")
        self.turn_right_wheels(self.right_wheels_speed)
    
    def straighten(self):
        max_speed = max(self.left_wheels_speed, self.right_wheels_speed)
        self.left_wheels_speed = max_speed
        self.right_wheels_speed = max_speed
        
        self.turn_left_wheels(self.left_wheels_speed)
        self.turn_right_wheels(self.right_wheels_speed)

        print(f'LW speed: {self.left_wheels_speed}\nRW speed: {self.right_wheels_speed}')

    def stop_motors(self):
        for motor in self.motors:
            motor.stop()

    def turn_on(self):
        print("Robot on...")
        while 1:
            event = keyboard.read_event()
            if event.event_type == keyboard.KEY_DOWN:
                key = event.name

                if key == 'q':
                    print("Turning off engine.")
                    break

                if key == 'up':
                    print("Accelerate\n")
                    if not self.direction:          # If moving fwd, up = speed up ; if moving bwd, down = slow down
                        self.speed_up(self.move_fwd)
                    else:
                        self.slow_down(self.move_bwd)
                if key == 'down':                   # If moving fwd, down = slow down ; if moving bwd, up = speed up
                    print("Slowing down\n")
                    if not self.direction:
                        self.slow_down(self.move_fwd)
                    else:
                        self.speed_up(self.move_bwd)

                if key == 'left':
                    print("Turning left\n")
                    self.turn_left()
                if key == 'right':
                    print("Turning right\n")
                    self.turn_right()
                if key == 's':
                    print("Stopping.\n")
                    self.speed = rover.MIN_SPEED
                    self.stop_motors()
                if key == 'i':
                    self.straighten()

robot = rover()
robot.turn_on()
# while 1:
#     event = keyboard.read_event()
#     if event.event_type == keyboard.KEY_DOWN:
#         key = event.name
#         print(f"Pressed: {key}")
