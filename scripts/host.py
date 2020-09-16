#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

from robot_gui_debug.srv import ParameterShow, ParameterShowRequest
import os
from robot_gui_debug.msg import ParameterAdjustMsg
import webbrowser
import roslib
import rospy
import tkinter as tk
import functools
import ConfigParser
import sys
import matplotlib.pyplot as plt
import threading
import time
'Robomaster gui调参客户端'

__author__ = 'ningfeng'

__email__ = '2387143434@qq.com'
'''
19.12.12更新 
配置文件不再需要指明绝对路径,
增加了网页打开按钮和退出按钮
优化了界面布局
'''
'''
19.12.19更新
增加了变量回传示波功能
'''
# 由于ros运行时是在 home下，故调用ros命令找到包的位置
packageName = 'robot_gui_debug'
loc = "/res/gui_debug_host.ini"
pathGet = os.popen('rospack find '+packageName)
path = pathGet.read()[:-1]+loc
pathGet.close()


class Parameter:
    # 存储参数和发送topic
    def __init__(self, publisher):
        self.cf = ConfigParser.ConfigParser()
        self.cf.read(path)
        self.IP = self.cf.get("Base", "IP")
        self.intItems = [[item[0], item[1]] for item in self.cf.items("Int")]
        self.floatItems = [[item[0], item[1]]
                           for item in self.cf.items("Float")]
        self.guiDebugPublisher = publisher

    # 发送topic
    def send(self, loc, value, parType):
        try:
            msg = ParameterAdjustMsg()
            if (parType == 'int'):
                msg.name = self.intItems[loc][0]
                self.intItems[loc][1] = value
                msg.intValue = int(value)
            else:
                msg.name = self.floatItems[loc][0]
                self.floatItems[loc][1] = value
                msg.floatValue = float(value)
            self.guiDebugPublisher.publish(msg)
        except rospy.ROSInterruptException:
            pass

    # 保存
    def save(self):
        for item in self.intItems:
            self.cf.set("Int", item[0], item[1])
        for item in self.floatItems:
            self.cf.set("Float", item[0], item[1])
        self.cf.write(open(path, "w"))
        try:
            msg = ParameterAdjustMsg()
            msg.name = 'write'
            self.guiDebugPublisher.publish(msg)
        except rospy.ROSInterruptException:
            pass


class Gui:
    # GUI界面相关
    def __init__(self, parameter, guiDebugClient=None):
        self.parameter = parameter
        self.window = tk.Tk()
        self.window.title("Robomaster gui debug v1.2")
        self.intButtons = []
        self.intEntrys = []
        self.floatButtons = []
        self.floatEntrys = []
        self.message = tk.StringVar()
        self.need_value_show = False
        self.x = []
        self.y = []
        if guiDebugClient != None:
            self.need_value_show = True
            self.guiDebugClient = guiDebugClient
            self.sampling_time = float(parameter.cf.get('Base','sampling_time'))

    # button发送

    def send(self, loc, parType):
        if (parType == 'int'):
            value = self.intEntrys[loc].get()
        else:
            value = self.floatEntrys[loc].get()
        parameter.send(loc, value, parType)
        self.message.set("Send successfully!")

    # 打开网页
    def web(self):
        webbrowser.open("http://"+parameter.IP + ":8080")
        self.message.set("Open the video web...")

    def draw(self):
        if not self.need_value_show:
            self.message.set('未开启绘图功能，请检查配置文件！')
            return
        response = self.guiDebugClient(0)
        responseList = response.parameterToShow.split(' ')
        # print responseList
        valueNames = responseList[:int(len(responseList) / 2)]
        valueLocs = responseList[int(len(responseList) / 2):-1]
        x = []
        y = [[] for i in range(len(valueNames))]

        y_loc = [int(valueLoc) for valueLoc in valueLocs]
        plotNum = max(y_loc)
        plots = [[] for i in range(plotNum)] #按位置画图
        for i in range(plotNum):
            for j,loc in enumerate(y_loc):
                if loc==i+1:
                    plots[i].append(j)
        plt.ion()
        response=self.guiDebugClient(1)
        valueList=response.parameterToShow[:-1].split(' ')
        x.append(float(valueList[0]))
        for i,value in enumerate(valueList[1:]):
            y[i].append(float(value))
        while True:
            plt.clf()
            response=self.guiDebugClient(1)
            valueList=response.parameterToShow[:-1].split(' ')
            # print valueList
            if x[-1]>=float(valueList[0]):
                time.sleep(0.01)
                continue
            x.append(float(valueList[0]))
            for i,value in enumerate(valueList[1:]):
                y[i].append(float(value))
            if x[-1]-x[0]>10:
                x=x[1:]
                for i,y_v in enumerate( y):
                    y[i]=y_v[1:]
            # print x
            # print y
            # print plots
            for i in range(plotNum):
                for j in plots[i]:
                    # print y[j]
                    plt.subplot(plotNum,1,i+1).plot(x,y[j],label=valueNames[j])
                plt.legend()
            plt.pause(self.sampling_time)
        plt.ioff()
        plt.show()
            



    # 模块初始化，绘制各个模块

    def moduleInit(self):
        row = 0
        tk.Label(self.window, text="The integer parameters:", font=('Arial', 12)).grid(row=row, column=3,
                                                                                       columnspan=3, sticky=tk.N + tk.S,
                                                                                       pady=10)
        row += 1
        column = 0
        for i, item in enumerate(parameter.intItems):
            tk.Label(self.window, text=item[0]).grid(
                row=row, column=column, sticky=tk.E)
            column += 1
            self.intEntrys.append(tk.Entry(self.window, show=None))
            self.intEntrys[i].insert(0, item[1])
            self.intEntrys[i].grid(row=row, column=column)
            column += 1
            self.intButtons.append(
                tk.Button(master=self.window, text="set", command=functools.partial(self.send, i, 'int')))
            self.intButtons[i].grid(
                row=row, column=column, padx=10, sticky=tk.W)
            column += 1
            if column == 9:
                row += 1
                column = 0
        row += 1
        tk.Label(self.window, text="The float parameters:", font=('Arial', 12)).grid(row=row, column=3,
                                                                                     columnspan=3, sticky=tk.N + tk.S,
                                                                                     pady=10)
        row += 1
        column = 0
        for i, item in enumerate(parameter.floatItems):
            tk.Label(self.window, text=item[0]).grid(row=row, column=column)
            column += 1
            self.floatEntrys.append(tk.Entry(self.window, show=None))
            self.floatEntrys[i].insert(0, item[1])
            self.floatEntrys[i].grid(row=row, column=column)
            column += 1
            self.floatButtons.append(
                tk.Button(master=self.window, text="set", command=functools.partial(self.send, i, 'float')))
            self.floatButtons[i].grid(row=row, column=column)
            column += 1
            if column == 9:
                row += 1
                column = 0
        row += 1
        tk.Label(self.window, text="The commends:", font=('Arial', 12)).grid(row=row, column=3, pady=10,
                                                                             columnspan=3, sticky=tk.N + tk.S)
        row += 1
        tk.Button(master=self.window, text='save', command=self.parameter.save).grid(
            row=row, column=0, columnspan=3,)
        self.message.set("No message. ")
        tk.Button(master=self.window, text='openWeb', command=self.web).grid(
            row=row, column=3, columnspan=3)
        tk.Button(master=self.window, text='exit', command=exit).grid(
            row=row, column=6, columnspan=3)
        row += 1
        tk.Label(self.window, textvariable=self.message).grid(
            row=row, columnspan=9)

    def windowShow(self):
        self.window.mainloop()


if __name__ == '__main__':
    cf = ConfigParser.ConfigParser()
    cf.read(path)
    rospy.init_node(cf.get('Base', 'node_name'))
    guiDebugPublisher = rospy.Publisher(
        cf.get('Base', 'topic_name'), ParameterAdjustMsg, queue_size=1)
    parameter = Parameter(guiDebugPublisher)
    if int(cf.get('Base', 'need_value_show')) == 1:
        server_name = cf.get('Base', 'server_name')
        rospy.wait_for_service(server_name)
        guiDebugClient = rospy.ServiceProxy(server_name, ParameterShow)
        gui = Gui(parameter, guiDebugClient)
        draw_thread=threading.Thread(target=gui.draw)
        draw_thread.start()
    else:
        gui = Gui(parameter)

    gui.moduleInit()
    gui.windowShow()
