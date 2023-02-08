# coding:utf-8
from PCA9685 import PCA9685
import time


class Servo:
    def __init__(self):
        self.pwm_40 = PCA9685(0x40, debug=True)
        self.pwm_41 = PCA9685(0x41, debug=True)
        # Set the cycle frequency of PWM  
        self.pwm_40.set_pwm_freq(50)
        time.sleep(0.01)
        self.pwm_41.set_pwm_freq(50)
        time.sleep(0.01)

    @staticmethod
    def map_num(value, from_low, from_high, to_low, to_high):
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

    # Convert the input angle to the value of pca9685
    def set_angle(self, channel, angle):
        if channel < 16:
            date = self.map_num(self.map_num(angle, 0, 180, 500, 2500), 0, 20000, 0,
                                4095)  # 0-180 map to 500-2500us ,then map to duty 0-4095
            self.pwm_41.set_pwm(channel, 0, int(date))
        elif 16 <= channel < 32:
            channel -= 16
            date = self.map_num(self.map_num(angle, 0, 180, 500, 2500), 0, 20000, 0, 4095)  #
            self.pwm_40.set_pwm(channel, 0, int(date))
        # time.sleep(0.0001)

    def relax(self):
        for i in range(8):
            self.pwm_41.set_pwm(i + 8, 4096, 4096)
            self.pwm_40.set_pwm(i, 4096, 4096)
            self.pwm_40.set_pwm(i + 8, 4096, 4096)


def servo_installation_position():
    servo = Servo()
    for i in range(32):
        # right tibia
        if i == 10 or i == 13 or i == 31:
            servo.set_angle(i, 0)
        # left tibia
        elif i == 18 or i == 21 or i == 27:
            servo.set_angle(i, 180)
        # right & left femur and coxa
        else:
            servo.set_angle(i, 90)
    time.sleep(3)


# Main program logic follows:
if __name__ == '__main__':
    print("Now servos will rotate to certain angles.")
    print("Please keep the program running when installing the servos.")
    print("After that, you can press ctrl-C to end the program.")
    while True:
        try:
            servo_installation_position()
        except KeyboardInterrupt:
            print("\nEnd of program")
            break
