import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Initialize PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Servo motor frequency (50Hz)

# Set the minimum and maximum pulse lengths for servo motors
SERVO_MIN = 150  # Pulse length for 0 degrees
SERVO_MAX = 600  # Pulse length for 180 degrees
SERVO_MID = (SERVO_MIN + SERVO_MAX) // 2  # Pulse length for 90 degrees

# Default angles for each motor
default_angles = [90, 70, 30, 150, 90, 60]  # Initial positions
current_angles = default_angles.copy()  # Current positions for tracking

# Action recording
recorded_steps = []  # Stores individual steps for manual recording  # modify
recorded_actions = []  # Stores sequences of actions for pick-and-place


def angle_to_pwm(angle):
    """Convert angle to PWM value."""
    return int(SERVO_MIN + (angle / 180.0) * (SERVO_MAX - SERVO_MIN))
def move_servo(channel, angle):
    """Move the servo motor on the specified channel to the given angle."""
    pwm_value = angle_to_pwm(angle)
    duty_cycle = int((pwm_value / 4096.0) * 65535)
    pca.channels[channel].duty_cycle = duty_cycle
    current_angles[channel] = angle
    print(f"Channel {channel} moved to angle {angle} (PWM: {pwm_value}, Duty Cycle: {duty_cycle})")

def smooth_move_servo(channel, target_angle, step=1, delay=0.02):
    """Smoothly move the servo motor to the target angle."""
    current_angle = current_angles[channel]
    if target_angle > current_angle:
        for angle in range(current_angle, target_angle + 1, step):
            move_servo(channel, angle)
            time.sleep(delay)
    elif target_angle < current_angle:
        for angle in range(current_angle, target_angle - 1, -step):
            move_servo(channel, angle)
            time.sleep(delay)
    current_angles[channel] = target_angle

def reset_servos():
    """Reset all servos to their default positions."""
    for channel in range(6):
        smooth_move_servo(channel, default_angles[channel])
        
def record_step():  # modify
    """Record a single step of servo movement."""
    motor = int(input("Select motor (0-5): "))
    if 0 <= motor <= 5:
        angle = int(input("Enter angle (0-180): "))
        if 0 <= angle <= 180:
            smooth_move_servo(motor, angle)
            step = current_angles.copy()
            recorded_steps.append(step)  # Store individual step
            print(f"Step recorded: {step}")
        else:
            print("Invalid angle. Use 0-180.")
    else:
        print("Invalid motor selection. Use 0-5.")

def save_steps():  # modify
    """Save all recorded steps into an action sequence."""
    if recorded_steps:
        recorded_actions.append(recorded_steps.copy())
        print(f"Action saved with {len(recorded_steps)} steps.")
        recorded_steps.clear()  # Clear steps after saving
    else:
        print("No steps to save.")

def playback_actions():
    """Play back recorded actions in a loop until 'q' is pressed."""
    while True:  # modify
        for action in recorded_actions:
            for step in action:
                for channel, angle in enumerate(step):
                    smooth_move_servo(channel, angle)
                time.sleep(1)  # Pause between steps
            reset_servos()  # Return to default positions after each action
            time.sleep(1)  # Pause before next action
        user_input = input("Press 'q' to stop playback or Enter to continue: ").lower()  # modify
        if user_input == 'q':
            print("Stopping playback.")  # modify
            break
def main():
    print("Servo Control: Enter a specific angle or use the following commands:")
    print("- s: Start manual control for recording steps")  # modify
    print("- t: Record a step")  # modify
    print("- d: Save all recorded steps into an action")  # modify
    print("- p: Play back recorded actions")  # modify
    print("- q: Reset servos to default positions")
    print("- qq: Quit the program")

    while True:
        user_input = input("Enter your choice or angle: ")

        if user_input.isdigit():
            angle = int(user_input)
            if 0 <= angle <= 180:
                selected_motor = int(input("Select motor (0-5): "))
                if 0 <= selected_motor <= 5:
                    smooth_move_servo(selected_motor, angle)
                else:
                    print("Invalid motor selection. Use 0-5.")
            else:
                print("Invalid angle. Use 0-180.")

        elif user_input.lower() == "s":  # modify
            print("Manual control mode. Use 't' to record steps and 'd' to save.")

        elif user_input.lower() == "t":  # modify
            record_step()

        elif user_input.lower() == "d":  # modify
            save_steps()

        elif user_input.lower() == "p":
            print("Playing back recorded actions...")  # modify
            playback_actions()

        elif user_input.lower() == "q":
            print("Resetting servos to default positions.")
            reset_servos()

        elif user_input.lower() == "qq":
            print("Exiting program.")
            reset_servos()
            break

        else:
            print("Invalid input. Try again.")

if __name__ == "__main__":
    try:
        reset_servos()  # Initialize all servos to default positions
        main()
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        reset_servos()  # Reset servos before exiting
        for channel in range(16):
            pca.channels[channel].duty_cycle = 0
        print("Program terminated.")
