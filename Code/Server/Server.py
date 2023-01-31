# -*- coding: utf-8 -*-
import time
import fcntl
import socket
import struct
import logging
from threading import Thread
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from picamera2.encoders import Quality
from Led import *
from Servo import *
from Thread import *
from Buzzer import *
from Control import *
from ADC import *
from Ultrasonic import *
from Command import COMMAND as cmd
from Streaming import StreamingOutput



class Server:
    def __init__(self):
        self.debug_mode = False
        self.tcp_flag = False
        self.led = Led()
        self.adc = ADC()
        self.servo = Servo()
        self.buzzer = Buzzer()
        self.control = Control()
        self.sonic = Ultrasonic()
        self.thread_led = None
        self.thread_sonic = None

        self.control.Thread_conditiona.start()

    @staticmethod
    def get_interface_ip():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(),
                                            0x8915,
                                            struct.pack('256s', b'wlan0'[:15])
                                            )[20:24])

    @staticmethod
    def send_data(connect, data):
        try:
            connect.send(data.encode('utf-8'))
        except Exception as e:
            logging.debug(f"Data send failed with the error: {e}")

    def turn_on_server(self):
        # ip address of the server
        host = self.get_interface_ip()
        # Port 8002 for video transmission
        self.socket_video = socket.socket()
        self.socket_video.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.socket_video.bind((host, 8002))
        self.socket_video.listen(1)

        # Port 5002 is used for instruction sending and receiving
        self.socket_data = socket.socket()
        self.socket_data.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.socket_data.bind((host, 5002))
        self.socket_data.listen(1)
        logging.info('Server address: ' + host)

    def turn_off_server(self):
        try:
            self.connection_video.close()
            self.connection_data.close()
        except:
            logging.debug('\n' + "No client connection")

    def reset_server(self):
        self.turn_off_server()
        self.turn_on_server()
        self.create_threads()

    def create_threads(self):
        self.video = Thread(target=self.transmission_video)
        self.instructions = Thread(target=self.receive_instruction)
        self.video.start()
        self.instructions.start()
        logging.debug("Threads created")

    def transmission_video(self):
        try:
            self.connection_video, self.client_address = self.socket_video.accept()
            self.connection_video = self.connection_video.makefile('wb')
        except:
            pass

        self.socket_video.close()

        logging.info("Socket video connected")

        camera = Picamera2()
        camera.configure(camera.create_video_configuration(main={"size": (400, 300)}))

        output = StreamingOutput()
        encoder = JpegEncoder(q=90)

        camera.start_recording(encoder, FileOutput(output), quality=Quality.VERY_HIGH)

        while True:
            with output.condition:
                output.condition.wait()
                frame = output.frame

            try:
                len_frame = len(output.frame)
                # logging.debug("output .length: ", len_frame)
                length_bin = struct.pack('<I', len_frame)
                self.connection_video.write(length_bin)
                self.connection_video.write(frame)
            except:
                camera.stop_recording()
                camera.close()
                logging.debug("End transmitting video")
                break

    def check_power(self):
        try:
            battery_voltage = self.adc.batteryPower()
            command = cmd.CMD_POWER + "#" + str(battery_voltage[0]) + "#" + str(
                battery_voltage[1]) + "\n"
            logging.debug(f"CMD_POWER command: {command}")
            self.send_data(self.connection_data, command)

            if battery_voltage[0] < 5.5 or battery_voltage[1] < 6:
                logging.debug(
                    f"Battery voltage is too low: LOAD {battery_voltage[0]}, RASPI {battery_voltage[1]}")

                if not self.debug_mode:
                    logging.warning(
                        "Battery voltage is too low, buzzer will be on for 3 times")
                    for i in range(3):
                        self.buzzer.run("1")
                        time.sleep(0.15)
                        self.buzzer.run("0")
                        time.sleep(0.1)
        except:
            pass

    def process_led(self, data):
        try:
            stop_thread(self.thread_led)
        except:
            pass
        self.thread_led = Thread(target=self.led.light, args=(data,))
        self.thread_led.start()

    def process_sonic(self):
        command = cmd.CMD_SONIC + "#" + str(self.sonic.getDistance()) + "\n"
        self.send_data(self.connection_data, command)

    def process_camera(self, data):
        if len(data) == 3:
            x = self.control.restriction(int(data[1]), 50, 180)
            y = self.control.restriction(int(data[2]), 0, 180)
            self.servo.setServoAngle(0, x)
            self.servo.setServoAngle(1, y)

    def process_relax(self):
        if not self.control.relax_flag:
            self.control.relax(True)
            self.control.relax_flag = True
        else:
            self.control.relax(False)
            self.control.relax_flag = False

    def process_servo_power(self, data):
        if data[1] == "0":
            GPIO.output(self.control.GPIO_4, True)
        else:
            GPIO.output(self.control.GPIO_4, False)

    def process_head(self, data):
        if len(data) == 3:
            self.servo.setServoAngle(int(data[1]), int(data[2]))

    def process_instruction(self, cmd_array):
        for oneCmd in cmd_array:
            data = oneCmd.split("#")

            if data is None or data[0] == '':
                continue
            elif cmd.CMD_BUZZER in data:
                self.buzzer.run(data[1])
            elif cmd.CMD_POWER in data:
                self.check_power()
            elif cmd.CMD_LED in data:
                self.process_led(data)
            elif cmd.CMD_LED_MOD in data:
                self.process_led(data)
            elif cmd.CMD_SONIC in data:
                self.process_sonic()
            elif cmd.CMD_HEAD in data:
                self.process_head(data)
            elif cmd.CMD_CAMERA in data:
                self.process_camera(data)
            elif cmd.CMD_RELAX in data:
                self.process_relax()
            elif cmd.CMD_SERVOPOWER in data:
                self.process_servo_power(data)
            else:
                self.control.order = data
                self.control.timeout = time.time()

    def receive_instruction(self):
        try:
            self.connection_data, self.client_address = self.socket_data.accept()
            logging.info(f"Client {self.client_address} connected successfully!")
        except:
            logging.info("Connection failed")

        self.socket_data.close()

        while True:
            try:
                all_data = self.connection_data.recv(1024).decode('utf-8')
            except:
                if self.tcp_flag:
                    self.reset_server()
                    break
                else:
                    break

            if all_data == "" and self.tcp_flag:
                self.reset_server()
                break
            else:
                cmd_array = all_data.split('\n')
                logging.debug(f"Received instructions data: {cmd_array}")

                # todo: Have no idea why this is needed
                if cmd_array[-1] != "":
                    cmd_array == cmd_array[:-1]

            self.process_instruction(cmd_array)

        try:
            stop_thread(self.thread_led)
        except:
            pass

        try:
            stop_thread(self.thread_sonic)
        except:
            pass

        logging.info("Close receive instructions and video transmission threads")


if __name__ == '__main__':
    pass
