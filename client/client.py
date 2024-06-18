#!/usr/bin/env python3

import string
import asyncio
import pygame
import pygame_gui
import websockets
import time
import threading
import logging
from multiprocessing import Process, active_children
import os
from io import StringIO

import queue
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication


pygame.init()
pygame.joystick.init()

pygame.display.set_caption('Quick Start')
window_surface = pygame.display.set_mode((800, 600))

background = pygame.Surface((800, 600))
background.fill(pygame.Color('#000000'))

manager = pygame_gui.UIManager((800, 600))

left_pad = 25

enable_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((left_pad, 75), (100, 50)),
                                            text='ENABLE',
                                            manager=manager)

disable_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((left_pad, 125), (100, 50)),
                                            text='DISABLE',
                                            manager=manager)

addr_entry = pygame_gui.elements.ui_text_entry_line.UITextEntryLine(relative_rect=pygame.Rect((left_pad, 175), (100, 50)), manager=manager)
addr_entry.set_allowed_characters([*(string.ascii_lowercase + string.digits + '.')])

connect_button = pygame_gui.elements.UIButton(relative_rect=pygame.Rect((left_pad + 100, 175), (100, 50)), text='CONNECT', manager=manager)

status_line = 'status: {}\nping: {}\ncontroller status: {}'
status_box = pygame_gui.elements.ui_text_box.UITextBox(relative_rect=pygame.Rect((left_pad, 250), (300, 100)), html_text=status_line.format('not connected', 'N/A', 'not connected'), manager=manager)

console_box = pygame_gui.elements.ui_text_box.UITextBox(relative_rect=pygame.Rect((400, 100), (350, 400)), html_text='', manager=manager)


clock = pygame.time.Clock()

class LogStream(object):
    def __init__(self):
        self.logs = ''

    def write(self, str):
        console_box.append_html_text(str)
        self.logs += str

    def flush(self):
        pass

    def __str__(self):
        return self.logs

log_stream = LogStream()
logging.basicConfig(stream=log_stream, level=logging.INFO)
#logging.basicConfig(level=logging.NOTSET)

port = int(os.environ.get('PORT', '8000'))

addr = 'raspberrypi.local' # default ip


enabled = False

connected = False
controller_connected = False

js1 = None




q = queue.Queue()



class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.orientationPlot = pg.PlotWidget()
        self.rawDataPlot = pg.PlotWidget()

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.orientationPlot)
        layout.addWidget(self.rawDataPlot, 1)
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)

        self.setCentralWidget(widget)

        self.x = []  # 100 time points
        self.roll = []  # 100 data points
        self.pitch = []  # 100 data points
        self.yaw = []
        self.data4 = []

        self.accel = []  # 100 data points
        self.gyro = []  # 100 data points

        self.orientationPlot.setBackground('w')
        self.rawDataPlot.setBackground('w')

        pen1 = pg.mkPen(color=(255, 0, 0))
        pen2 = pg.mkPen(color=(0, 0, 255))
        pen3 = pg.mkPen(color=(0, 0, 0))
        pen4 = pg.mkPen(color=(0, 255, 0))
        self.data_line1 = self.orientationPlot.plot(self.x, self.roll, pen=pen1)
        self.data_line2 = self.orientationPlot.plot(self.x, self.pitch, pen=pen2)
        #self.data_line3 = self.orientationPlot.plot(self.x, self.yaw, pen=pen3)
        #self.data_line4 = self.orientationPlot.plot(self.x, self.data4, pen=pen4)

        self.accel_plot = self.rawDataPlot.plot(self.x, self.accel, pen=pen1)
        self.gyro_plot = self.rawDataPlot.plot(self.x, self.gyro, pen=pen2)

        
    def update_plot_data(self):
        if len(self.x) > 100:
            self.x = self.x[1:]  # Remove the first y element.

        if len(self.x) == 0:
            self.x = [0]
        else:
            self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.

        if len(self.roll) > 100:
            self.roll = self.roll[1:]  # Remove the first
            self.pitch = self.pitch[1:]
            self.yaw = self.yaw[1:]
            self.data4 = self.data4[1:]
            self.accel = self.accel[1:]
            self.gyro = self.gyro[1:]

        sensor_data = q.get()
        axis = 2
        self.roll.append(sensor_data[0])  # Add a new value.
        self.pitch.append(sensor_data[1])
        # self.yaw.append(sensor_data[2])
        # self.data4.append(sensor_data[3])

        self.accel.append(sensor_data[2])
        self.gyro.append(sensor_data[3])

        self.data_line1.setData(self.x, self.roll)  # Update the data.
        self.data_line2.setData(self.x, self.pitch)
        #self.data_line3.setData(self.x, self.yaw)
        #self.data_line4.setData(self.x, self.data4)

        self.accel_plot.setData(self.x, self.accel)
        self.gyro_plot.setData(self.x, self.gyro)

app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
#app.exec()

try:
    js1 = pygame.joystick.Joystick(int(os.environ.get('CRONCH', '0')))
    js1.init()
    logging.info('joystick connected: ' + str(js1.get_name()))
except:
    pass

def format_status():
    str1 = 'connected' if connected else 'not connected'
    str2 = 'N/A'
    str3 = 'connected' if controller_connected else 'not connected'
    return status_line.format(str1, str2, str3)

async def get_joy():
    pygame.event.pump()
    x_axis = js1.get_axis(3) # right stick x
    y_axis = js1.get_axis(1) # left stick y

    start_auto = js1.get_button(1)
    estop = js1.get_button(0)


    return (x_axis, y_axis, start_auto, estop)

def format_ws_data(x_axis, y_axis, start_auto, estop):
    return '{enabled}:{start_auto}:{x_axis}:{y_axis}:{estop}'.format(enabled=bool(enabled), start_auto=bool(start_auto), x_axis=x_axis, y_axis=y_axis, estop=bool(estop))

async def async_send(websocket):
    global start_auto
    global estop

    while True:
        x_axis, y_axis, start_auto, estop = await get_joy()

        ws_data = format_ws_data(x_axis, y_axis, start_auto, estop)
        logging.debug(ws_data)
        # print("WS_DATA: ", ws_data)

        await websocket.send(ws_data)
        await asyncio.sleep(60/1000)

async def async_recv(websocket):
    while True:
        response = await websocket.recv()
        logging.debug('response ' + response)
        data = [float(numeric_string) for numeric_string in response.split(", ")]
        q.put(data)
        w.update_plot_data()

        await asyncio.sleep(200/1000)

async def test():
    async for websocket in websockets.connect('ws://' + addr + ':' + str(port), max_queue=1024):
        try:
            connected = True
            await asyncio.gather(async_recv(websocket), async_send(websocket))
        except websockets.ConnectionClosed:
            connected = False
            break

def asyncio_run_wrapper():
    asyncio.run(test())

if __name__ == '__main__':
    is_running = True

    disable_button.disable()

    
    #p = Process(target=lambda: asyncio.run(test()))
    #p = Process(target=idk_i_am_dying)
    p = threading.Thread(target=asyncio_run_wrapper)
    p.start()

    while is_running:
        time_delta = clock.tick(60) / 1000.0

        status_box.html_text = format_status()

        if not controller_connected and pygame.joystick.get_count() != 0:
            js1 = pygame.joystick.Joystick(int(os.environ.get('CRONCH', '0')))
            js1.init()
            logging.info('joystick connected: ' + str(js1.get_name()))
            controller_connected = True

        if controller_connected and pygame.joystick.get_count() == 0:
            controller_connected = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                is_running = False


            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == enable_button and not enabled:
                    logging.info('Enabling')
                    enabled = True
                    enable_button.disable()
                    disable_button.enable()

                if event.ui_element == disable_button and enabled:
                    logging.info('Disabling')
                    enabled = False
                    disable_button.disable()
                    enable_button.enable()

                if event.ui_element == connect_button:
                    addr = addr_entry.get_text()
                    logging.info('Setting addr to {}'.format(addr))
                    logging.info('Restarting connection')
                    #p.kill()
                    #p.stop()
                    #p = Process(target=idk_i_am_dying)
                    p = threading.Thread(target=asyncio_run_wrapper)
                    p.start()


            if event.type == pygame.KEYDOWN:
                if not enabled and pygame.key.get_pressed()[pygame.K_BACKSLASH] and pygame.key.get_pressed()[pygame.K_LEFTBRACKET] and pygame.key.get_pressed()[pygame.K_RIGHTBRACKET]:
                    logging.info('Enabling')
                    enabled = True
                    enable_button.disable()
                    disable_button.enable()
                if event.key == pygame.K_SPACE and enabled:
                    logging.info('Disabling')
                    enabled = False
                    disable_button.disable()
                    enable_button.enable()
                if event.key == pygame.K_q:
                    is_running = False


            manager.process_events(event)

        manager.update(time_delta)

        window_surface.blit(background, (0, 0))
        manager.draw_ui(window_surface)

        pygame.display.update()

    active = active_children()
    for child in active:
        child.kill()
    os._exit(0)
