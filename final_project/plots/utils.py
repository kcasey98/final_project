#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import sys
import math
import os

# To read text files and plot them


# Get data from txt file, and return array
def getData(dataDir):
    data = np.loadtxt(dataDir, skiprows=0)
    time = np.arange(np.shape(data)[0])
    time = time/100
    return time, data

def autoscale_axes(ax,margin=0.1):

    def getYMinMax(line):
        yd = line.get_ydata()
        h = np.abs(np.max(yd) - np.min(yd))
        bot = np.min(yd)
        top = np.max(yd)
        return bot,top,h

    def getXMinMax(line):
        xd = line.get_xdata()
        w = np.abs(np.max(xd) - np.min(xd))
        left = np.min(xd)
        right = np.max(xd)
        return left,right,w

    lines = ax.get_lines()
    bot, top, h = np.inf, -np.inf, 0
    l, r, w = np.inf, -np.inf, 0

    for line in lines:
        new_bot, new_top, new_h = getYMinMax(line)
        new_l, new_r, new_w = getXMinMax(line)
        if new_bot < bot: bot = new_bot
        if new_top > top: top = new_top
        if new_h > h: h = new_h
        if new_l < l: l = new_l
        if new_r > r: r = new_r
        if new_w > w: w = new_w

    ax.set_ylim(bot-margin*h,top+margin*h)
    ax.set_xlim(l-margin/4*w,r+margin/4*w)

# Plot x, y, z, 
def plotXYZ(namePlots, data, desiredData, time, data_label, data_des_label):
    
    namePlots[0].plot(time, data[:,0], 'r', label=data_label[0])
    if type(desiredData) is np.ndarray:
        namePlots[0].plot(time, desiredData[:,0], 'r--', label=data_des_label[0])
    autoscale_axes(namePlots[0])
    # namePlots[0].autoscale(enable=True, axis='both', tight=False)
    lgd = namePlots[0].legend(loc=(0.8,0.5),frameon=True)

    namePlots[1].plot(time, data[:,1], 'g', label=data_label[1])
    if type(desiredData) is np.ndarray:
        namePlots[1].plot(time, desiredData[:,1], 'g--', label=data_des_label[1])
    autoscale_axes(namePlots[1])
    lgd = namePlots[1].legend(loc=(0.8,0.5),frameon=True)

    namePlots[2].plot(time, data[:,2], 'b', label=data_label[2])
    if type(desiredData) is np.ndarray:
        namePlots[2].plot(time, desiredData[:,2], 'b--', label=data_des_label[2])
    autoscale_axes(namePlots[2])
    lgd = namePlots[2].legend(loc=(0.8,0.5),frameon=True)


def simpleSubplot(namePlot, data, time, data_label, lineType, legendLoc=(0.8,0.5)):

    numC = np.asarray(data).ndim

    for i in range(numC):
        if numC == 1: namePlot.plot(time, data, lineType[i], label=data_label[i])
        else: namePlot.plot(time, data[:,i], lineType[i], label=data_label[i])

    autoscale_axes(namePlot)
    lgd = namePlot.legend(loc=legendLoc,frameon=True)


# # Get the current working directory
# cwd = os.getcwd()

# # Print the current working directory
# print("Current working directory: {0}".format(cwd))

# # data file to read 
# file_name_5a = "../data_files/question_5a.txt"
# data_5a = np.loadtxt(file_name_5a, skiprows=0)

# file_name_5b = "../data_files/question_5b.txt"
# data_5b = np.loadtxt(file_name_5b, skiprows=0)

# file_name_5c = "../data_files/question_5c.txt"
# data_5c = np.loadtxt(file_name_5c, skiprows=0)

# # q3_dense = np.linspace(-90,90,251)
# # q3 = (-90, -60, -30, 0, 30, 60, 90)

# # d2_dense = np.linspace(0,2,251)
# # d2 = (0, 0.5, 1, 1.5, 2)

# # question e

# time_1 = np.arange(np.shape(data_5a)[0])
# time_1 = time_1/100
# force = data_5a[:,0:3]

# fig0 = plt.figure(0,figsize=(7,8))

# fig0.add_subplot(311)
# plt.plot(time_1, force[:,0], 'r', label=r'$F_{x}$')
# # plt.title("Force x", FontSize=24)
# # plt.ylim((-2.0,2.0))
# # lgd = plt.legend(loc=(0.8,0.4),frameon=1)

# fig0.add_subplot(312)
# plt.plot(time_1, force[:,1], 'g', label=r'$F_{y}$')
# # plt.ylabel('Force y', FontSize=18)
# # plt.ylim((-0.2,0.1))
# # lgd = plt.legend(loc=(0.8,0.2),frameon=1)

# fig0.add_subplot(313)
# plt.plot(time_1, force[:,2], 'b', label=r'$F_{z}$')
# # plt.ylim((-2.26,-2.16))

# plt.xlabel('Time')
# fig0.tight_layout()
# fig0.savefig('HW3_Q5a_force.png')

# # plt.show()

# # question e
# time_1 = np.arange(np.shape(data_5b)[0])
# time_1 = time_1/100
# force = data_5b[:,0:3]

# fig1 = plt.figure(1,figsize=(7,8))

# fig1.add_subplot(311)
# plt.plot(time_1, force[:,0], 'r', label=r'$F_{x}$')
# # plt.title("Force x", FontSize=24)
# # plt.ylim((-2.0,2.0))
# # lgd = plt.legend(loc=(0.8,0.4),frameon=1)

# fig1.add_subplot(312)
# plt.plot(time_1, force[:,1], 'g', label=r'$F_{y}$')
# # plt.ylabel('Force y', FontSize=18)
# # plt.ylim((-0.2,0.1))
# # lgd = plt.legend(loc=(0.8,0.2),frameon=1)

# fig1.add_subplot(313)
# plt.plot(time_1, force[:,2], 'b', label=r'$F_{z}$')
# # plt.ylim((-2.26,-2.16))

# plt.xlabel('Time')
# fig1.tight_layout()
# fig1.savefig('HW3_Q5b_force.png')

# # question e
# time_1 = np.arange(np.shape(data_5c)[0])
# time_1 = time_1/100
# force = data_5c[:,0:3]

# fig2 = plt.figure(2,figsize=(7,8))

# fig2.add_subplot(311)
# plt.plot(time_1, force[:,0], 'r', label=r'$F_{x}$')
# # plt.title("Force x", FontSize=24)
# # plt.ylim((-2.0,2.0))
# # lgd = plt.legend(loc=(0.8,0.4),frameon=1)

# fig2.add_subplot(312)
# plt.plot(time_1, force[:,1], 'g', label=r'$F_{y}$')
# # plt.ylabel('Force y', FontSize=18)
# # plt.ylim((-0.2,0.1))
# # lgd = plt.legend(loc=(0.8,0.2),frameon=1)

# fig2.add_subplot(313)
# plt.plot(time_1, force[:,2], 'b', label=r'$F_{z}$')
# # plt.ylim((-2.26,-2.16))

# plt.xlabel('Time')
# fig2.tight_layout()
# fig2.savefig('HW3_Q5c_force.png')

# # plt.show()

