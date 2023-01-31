# -*- coding: utf-8 -*-
import sys
import logging
import argparse
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from Server import *
from ui_server import Ui_server


class MyWindow(QMainWindow, Ui_server):
    def __init__(self):
        self.user_ui = True
        self.start_tcp = False
        self.args = self.create_parser()

        self.process_args(self.args)
        self.server = Server()

        if self.args.debug:
            self.server.debug_mode = self.args.debug

        if self.user_ui:
            self.app = QApplication(sys.argv)
            super(MyWindow, self).__init__()
            self.setupUi(self)
            self.pushButton_On_And_Off.clicked.connect(self.on_and_off_server)
            self.on_and_off_server()

        if self.start_tcp:
            self.server.turn_on_server()
            self.server.tcp_flag = True

            self.video = threading.Thread(target=self.server.transmission_video)
            self.video.start()

            self.instruction = threading.Thread(target=self.server.receive_instruction)
            self.instruction.start()

            if self.user_ui:
                self.pushButton_On_And_Off.setText('Off')
                self.states.setText('On')

    @staticmethod
    def create_parser():
        parser = argparse.ArgumentParser(description='Hexapod Server')
        group_args_global = parser.add_argument_group("Global parameters")
        group_args_global.add_argument(
            '--no-ui', dest='no_ui', action='store_true', default=False,
            help='No UI mode')
        group_args_global.add_argument(
            '--tcp', dest='tcp', action='store_true', default=False,
            help='Open TCP')
        group_args_global.add_argument(
            '--debug', dest='debug', action='store_true', default=False,
            help='Debug mode + verbose logging')

        args = parser.parse_args()

        return args

    def process_args(self, args):
        if args.debug:
            logging.basicConfig(level=logging.DEBUG, force=True)
            logging.debug("Verbose logging enabled")
        if args.tcp:
            self.start_tcp = True
            logging.debug("Open TCP")
        if args.no_ui:
            self.user_ui = False
            logging.debug("No UI mode enabled")

    def on_and_off_server(self):
        if self.pushButton_On_And_Off.text() == 'On':
            self.pushButton_On_And_Off.setText('Off')
            self.states.setText('On')
            self.server.turn_on_server()
            self.server.tcp_flag = True
            self.video = threading.Thread(target=self.server.transmission_video)
            self.video.start()
            self.instruction = threading.Thread(target=self.server.receive_instruction)
            self.instruction.start()
        else:
            self.pushButton_On_And_Off.setText('On')
            self.states.setText('Off')
            self.server.tcp_flag = False
            try:
                stop_thread(self.video)
                stop_thread(self.instruction)
            except Exception as e:
                logging.info(e)
            self.server.turn_off_server()
            logging.info("Closing the server")

    def closeEvent(self, event):
        try:
            stop_thread(self.video)
            stop_thread(self.instruction)
        except:
            pass

        try:
            self.server.socket_video.shutdown(2)
            self.server.socket_data.shutdown(2)
            self.server.turn_off_server()
        except:
            pass

        if self.user_ui:
            QCoreApplication.instance().quit()
        sys.exit(0)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting server...")

    try:
        my_window = MyWindow()
        user_ui = my_window.user_ui

        if user_ui:
            my_window.show()
            sys.exit(my_window.app.exec_())

        while True:
            pass

    except KeyboardInterrupt:
        logging.info('Ctrl+C pressed. Exiting.')
        if user_ui:
            my_window.close()
        sys.exit(1)

    except:
        logging.info('Error. Exiting.')
        sys.exit(1)
