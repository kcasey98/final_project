#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import sys
import math
import os

from utils import getData, plotXYZ, simpleSubplot

# To read text files and plot them
def question1(dataDir, imageName):
    
    time, data = getData(dataDir)
    x = data[:,0:3]
    x_des = data[:,3:6]
    dx = data[:,6:9]
    dx_des = data[:,9:12]

    data_label = ["x", "y", "z"]
    data_des_label = ["$x_d$", "$y_d$", "$z_d$"]
    fig, plots = plt.subplots(3, figsize=(5, 6))
    plotXYZ(plots,dx,dx_des, time, data_label, data_des_label)

    title = "Question 1" + "\nx vs x_desired"
    plots[0].set_title(title, fontsize=13)
    fig.supylabel('End effector positions (m)', fontsize=11,  font="monospace")
    plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
    fig.tight_layout()
    fig.savefig(imageName)

    # data_label2 = ["dx", "dy", "dz"]
    # data_des_label2 = ["$dx_d$", "$dy_d$", "$dz_d$"]
    # fig2, plots2 = plt.subplots(3, figsize=(5, 6))
    # plotXYZ(plots, dx, dx_des, time, data_label2, data_des_label2)

    # title2 = "Question 1" + "\ndx vs dx_desired"
    # plots2[0].set_title(title2, fontsize=13)
    # fig2.supylabel('End effector positions (m)', fontsize=11,  font="monospace")
    # plots2[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
    # fig2.tight_layout()
    # fig2.savefig(imageName2)

    plt.close('all')

# def question2(dataDir, subNum, imageName, imageName2):
    
#     time, data = getData(dataDir)
    
#     # part 1
#     x = data[:,0:3]
#     x_des = data[:,3:6]

#     data_label = ["x", "y", "z"]
#     data_des_label = ["$x_d$", "$y_d$", "$z_d$"]
#     fig, plots = plt.subplots(3, figsize=(5, 6))
#     plotXYZ(plots, x, x_des, time, data_label, data_des_label)

#     title = "Question 2" + subNum + "\nx vs x_desired"
#     plots[0].set_title(title, fontsize=13,  font="monospace")
#     fig.supylabel('End effector positions (m)', fontsize=11,  font="monospace")
#     plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName)

#     # part 2
#     q4_data = data[:,6:9]
#     q6_data = data[:,9:12]

#     fig, plots = plt.subplots(2, figsize=(5, 6))

#     data_label = [r"$q_{4}$", r"$q_{4_{low}}$", r"$q_{4_{high}}$"]
#     lineType = ["c", "c--", "c-."]
#     simpleSubplot(plots[0], q4_data, time, data_label, lineType)
#     plots[0].set_title(title, fontsize=13,  font="monospace")

#     data_label = [r"$q_{6}$", r"$q_{6_{low}}$", r"$q_{6_{high}}$"]
#     lineType = ["m", "m--", "m-."]
#     simpleSubplot(plots[1], q6_data, time, data_label, lineType)

#     title = "Question 2" + subNum + "\nJoint Angle with Joint Limits"
#     fig.supylabel('Joint Angles (rad)', fontsize=11,  font="monospace")
#     plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName2)

#     plt.close('all')

# def question3(dataDir, imageName, imageName2):
    
#     time, data = getData(dataDir)
    
#     # part 1
#     x = data[:,0:3]
#     x_des = data[:,3:6]

#     data_label = ["x", "y", "z"]
#     data_des_label = ["$x_d$", "$y_d$", "$z_d$"]
#     fig, plots = plt.subplots(3, figsize=(5, 6))
#     plotXYZ(plots, x, x_des, time, data_label, data_des_label)

#     title = "Question 3\nx vs x_desired"
#     plots[0].set_title(title, fontsize=13,  font="monospace")
#     fig.supylabel('End effector positions (m)', fontsize=11,  font="monospace")
#     plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName)

#     # part 2
#     delta_phi = data[:,6:9]

#     data_label = ["x", "y", "z"]
#     fig, plots = plt.subplots(3, figsize=(5, 6))
#     plotXYZ(plots, delta_phi, 0, time, data_label, 0)

#     title = "Question 3\n$\delta$$\phi$"
#     plots[0].set_title(title, fontsize=13,  font="monospace")
#     fig.supylabel('Orientation error (rad)', fontsize=11,  font="monospace")
#     plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName2)

#     plt.close('all')

# def question4(dataDir, subNum, imageName, imageName2):
    
#     time, data = getData(dataDir)
    
#     # part 1
#     x = data[:,0:3]
#     x_des = data[:,3:6]

#     data_label = ["x", "y", "z"]
#     data_des_label = ["$x_d$", "$y_d$", "$z_d$"]
#     fig, plots = plt.subplots(3, figsize=(5, 6))
#     plotXYZ(plots, x, x_des, time, data_label, data_des_label)

#     title = "Question 4" + subNum + "\nx vs x_desired"
#     plots[0].set_title(title, fontsize=13,  font="monospace")
#     fig.supylabel('End effector positions (m)', fontsize=11,  font="monospace")
#     plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName)

#     # part 2
#     delta_phi = data[:,9:11]

#     fig, plots = plt.subplots(1, figsize=(5, 6))

#     data_label = ["velocity", r"$V_{max}$"]
#     lineType = ["c", "k--"]
#     simpleSubplot(plots, delta_phi, time, data_label, lineType)

#     title = "Question 4" + subNum + "\nEnd Effector Velocity"
#     plots.set_title(title, fontsize=13,  font="monospace")
#     fig.supylabel('Joint Angles (rad)', fontsize=11,  font="monospace")
#     plots.set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName2)

#     plt.close('all')


# def question5(dataDir, subNum, subtitle, imageName, imageName2, imageName3):
    
#     time, data = getData(dataDir)

#     pos_data = data[:,0:3]
#     pos_d_data = data[:,3:6]
#     force_data = data[:,6:9]

#     # part 1, trajectory in time
#     fig, plots = plt.subplots(3, figsize=(5, 6))

#     data_label = ["x", "y", "z"]
#     data_d_label = [r"$x_{d}$", r"$y_{d}$", r"$z_{d}$"]
#     plotXYZ(plots, data, pos_d_data, time, data_label, data_d_label)

#     title = "Question 5" + subNum + "\n" + subtitle + ": Trajectory in Time"
#     plots[0].set_title(title, fontsize=13, font="monospace")
#     fig.supylabel('Force applied on end effector (N)', fontsize=11,  font="monospace")
#     plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName)

#     # part 2, trajectory on plane
#     fig, plots = plt.subplots(1, 2, figsize=(9, 5))

#     data_label = ["trajectory, x-y plane"]
#     lineType = ["c"]
#     simpleSubplot(plots[0], pos_data[:,1], pos_data[:,0], data_label, lineType, legendLoc=(0.2, 1.005))
#     data_label = ["desired trajectory, x-y plane"]
#     lineType = ["k--"]
#     simpleSubplot(plots[0], pos_d_data[:,1], pos_d_data[:,0], data_label, lineType, legendLoc=(0.2, 1.005))
#     offset = 0.12
#     y_center = 0.1
#     x_center = 0.3
#     plots[0].set_ylim(y_center-offset,y_center+offset)
#     plots[0].set_xlim(x_center-offset,x_center+offset)
#     plots[0].set_ylabel('y (m)', fontsize=11,  font="monospace")
#     plots[0].set_xlabel('x (m)', fontsize=11,  font="monospace")
#     plots[0].set(aspect='equal')

#     data_label = ["trajectory, x-z plane"]
#     lineType = ["m"]
#     simpleSubplot(plots[1], pos_data[:,2], pos_data[:,0], data_label, lineType, legendLoc=(0.2, 1.005))
#     data_label = ["desired trajectory, x-z plane"]
#     lineType = ["k--"]
#     simpleSubplot(plots[1], pos_d_data[:,2], pos_d_data[:,0], data_label, lineType, legendLoc=(0.2, 1.005))
#     offset = 0.02
#     plots[1].set_ylabel('z (m)', fontsize=11,  font="monospace")
#     plots[1].set_xlabel('x (m)', fontsize=11,  font="monospace")
#     plots[1].set_xlim(x_center-offset,x_center+offset)
#     plots[1].set_ylim(0.52,0.62)
#     # plots[1].set(aspect='equal')

#     title = "Question 5" + subNum + "\n" + subtitle + ": Trajectory on Planes"
#     fig.suptitle(title, fontsize=13, font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName2)

#     # part 3, force in time
#     data_label = ["$F_x$", "$F_y$", "$F_z$"]
#     fig, plots = plt.subplots(3, figsize=(5, 6))
#     plotXYZ(plots, force_data, 0, time, data_label, 0)

#     title = "Question 5" + subNum + "\n" + subtitle + ": Force on EE"
#     plots[0].set_title(title, fontsize=13, font="monospace")
#     fig.supylabel('Force applied on end effector (N)', fontsize=11,  font="monospace")
#     plots[-1].set_xlabel('Time (seconds)', fontsize=11,  font="monospace")
#     fig.tight_layout()
#     fig.savefig(imageName3)

#     plt.close('all')


if __name__ == "__main__":

    question1("../data_files/question_1.txt", "project_q1.png")
    # question1("../data_files/question_1c.txt", "c", "hw3_q1c.png")
    # question2("../data_files/question_2d.txt", "d", "hw3_q2d_i.png", "hw3_q2d_ii.png")
    # question2("../data_files/question_2e.txt", "e", "hw3_q2e_i.png", "hw3_q2e_ii.png")
    # question2("../data_files/question_2f.txt", "f", "hw3_q2f_i.png", "hw3_q2f_ii.png")
    # question2("../data_files/question_2g.txt", "g", "hw3_q2g_i.png", "hw3_q2g_ii.png")
    # question3("../data_files/question_3.txt", "hw3_q3_i.png", "hw3_q3_ii.png")
    # question4("../data_files/question_4a.txt", "a", "hw3_q4a_i.png", "hw3_q4a_ii.png")
    # question4("../data_files/question_4b.txt", "b", "hw3_q4b_i.png", "hw3_q4b_ii.png")
    # question5("../data_files/question_5a.txt", "a", "Impedance Control", "hw3_q5a_time_traj.png", "hw3_q5a_plane_traj.png", "hw3_q5a_force.png")
    # question5("../data_files/question_5b.txt", "b", "Op Space Control", "hw3_q5b_time_traj.png", "hw3_q5b_plane_traj.png", "hw3_q5b_force.png")
    # question5("../data_files/question_5c.txt", "c", "Unified F-M Control", "hw3_q5c_time_traj.png", "hw3_q5c_plane_traj.png", "hw3_q5c_force.png")
