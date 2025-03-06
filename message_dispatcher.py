# -*- coding:utf8 -*-
# !/usr/bin/env python3
# Copyright 2025 Octadero.
# Author Volodymyr Pavliukevych

"""This is a MessageDispatcher
"""

import sys
from queue import Queue
from threading import Thread

from time import time
import traceback

from fly_controller import FlyController
from environment import FlyState, FlyTask, FlyControlException, FlyControlExceptionCode

class MessageDispatcher():
    def __init__(self, controller: FlyController):
        self.controller = controller

        self.queue = Queue(maxsize=0)
        self.dispath_thread = Thread(target=self.dispath, name="MessageDispathThread", daemon=True)
        self.dispath_thread.start()
     
    def dispath(self): 
        while True:
            try:
                message = self.queue.get()
                start_exec_time = time()
                self.process_message_received(message)
            except Exception as error:
                #self.logger.critical("Critical Error at dispatch function: {error}".format(error=error))
                exc_info = sys.exc_info()
                traceback.print_exception(*exc_info)
            end_exec_time = time()
            exec_time = end_exec_time - start_exec_time
            if exec_time > 2.0:
                # self.logger.warn(f"Execution time: {exec_time:0.2f}".format(exec_time=exec_time))
                print(f"Execution time: {exec_time:0.2f}".format(exec_time=exec_time))
            self.queue.task_done()

    def did_receive_message(self, message):
        self.queue.put(message)

    def process_message_received(self, message):

        match message.get_type():
            case "GLOBAL_POSITION_INT":
                self.controller.did_receive_global_position(message)
            case "OPTICAL_FLOW":
                self.controller.did_receive_optical_flow(message)
            case "HEARTBEAT":
                pass    
                # case "POSITION_TARGET_LOCAL_NED":
                #     print(">>POSITION_TARGET_LOCAL_NED")
                # case "LOCAL_POSITION_NED":
                #     print(">>LOCAL_POSITION_NED")
            case _:
                print(f"Skip message: {message.get_type()}")

