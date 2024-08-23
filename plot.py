import matplotlib.pyplot as plt
import csv
import math
import os
import glob
import sys

use_relative = True


def compare_angles(ang1: float, ang2: float):
    diff = abs(ang1) + abs(ang2)
    if diff > 355 and diff < 365:
        return True
    return False


def plot_wheeled_base(run_num: str, dir_path: str):
    path_data = dir_path + f'/wheeledData/run{run_num}.txt'
    f = open(path_data, mode="r")
    csv_file = csv.reader(f)
    lin_set_vel = []
    ang_set_vel = []
    time_set_vel = []
    lin_vel_move_straight = []
    ang_vel_move_straight = []
    posX_move_straight = []
    posY_move_straight = []
    time_move_straight = []
    spin_lin_vel = []
    spin_ang_vel = []
    spin_theta = []
    time_spin = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "sv":
                lin_set_vel.append(float(lines[1]))
                ang_set_vel.append(float(lines[2]))
                time_set_vel.append(float(lines[3])/1000)
            case "ms":
                lin_vel_move_straight.append(float(lines[1]))
                ang_vel_move_straight.append(float(lines[2]))
                time_move_straight.append(float(lines[3])/1000)
                posX_move_straight.append(float(lines[4])*1000)
                posY_move_straight.append(float(lines[5])*1000)
            case "s":
                spin_lin_vel.append(float(lines[1]))
                spin_ang_vel.append(float(lines[2]))
                time_spin.append(float(lines[3])/1000)
                spin_theta.append(float(lines[6]))

    path_desired = dir_path + f'/wheeledDes/run{run_num}.txt'
    f2 = open(path_desired, mode="r")
    csv_file = csv.reader(f2)
    lin_set_vel_des = []
    ang_set_vel_des = []
    time_set_vel_des = []
    lin_vel_move_straight_des = []
    ang_vel_move_straight_des = []
    posX_move_straight_des = []
    posY_move_straight_des = []
    time_move_straight_des = []
    spin_lin_vel_des = []
    spin_ang_vel_des = []
    spin_theta_des = []
    time_spin_des = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "sv":
                lin_set_vel_des.append(float(lines[1]))
                ang_set_vel_des.append(float(lines[2]))
                time_set_vel_des.append(float(lines[3])/1000)
            case "ms":
                lin_vel_move_straight_des.append(float(lines[1]))
                ang_vel_move_straight_des.append(float(lines[2]))
                time_move_straight_des.append(float(lines[3])/1000)
                posX_move_straight_des.append(float(lines[4])/10)
                posY_move_straight_des.append(float(lines[5])/10)
            case "s":
                spin_lin_vel_des.append(float(lines[1]))
                spin_ang_vel_des.append(float(lines[2]))
                time_spin_des.append(float(lines[3])/1000)
                if float(lines[6]) == -30:
                    spin_theta_des.append(360 + float(lines[6]))
                elif float(lines[6]) < -355:
                    spin_theta_des.append(0)
                else:
                    spin_theta_des.append(float(lines[6]))

    _, axs = plt.subplots(2)
    plt.title("SetVelocity Linear and Angular Velocities (Wheeled Base)")
    axs[0].set_ylabel("linear velocity(mm/sec)")
    axs[0].plot(time_set_vel, lin_set_vel, 'r.')
    axs[0].plot(time_set_vel, lin_set_vel, 'r', label="actual")
    axs[0].plot(time_set_vel_des, lin_set_vel_des, 'b', label="desired")
    axs[1].set_ylabel("angular velocity(degs/sec)")
    axs[1].plot(time_set_vel, ang_set_vel, 'r.')
    axs[1].plot(time_set_vel, ang_set_vel, 'r', label="actual")
    axs[1].plot(time_set_vel_des, ang_set_vel_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/wheeled_sv_vels.jpg")

    _, axs1 = plt.subplots(2)
    plt.title("MoveStraight Linear and Angular Velocities (Wheeled Base)")
    axs1[0].set_ylabel("linear velocity(mm/sec)")
    axs1[0].plot(time_move_straight, lin_vel_move_straight, 'r.')
    axs1[0].plot(time_move_straight, lin_vel_move_straight, 'r', label="actual")
    axs1[0].plot(time_move_straight_des, lin_vel_move_straight_des, 'b', label="desired")
    axs1[1].set_ylabel("angular velocity(degs/sec)")
    axs1[1].plot(time_move_straight, ang_vel_move_straight, 'r.')
    axs1[1].plot(time_move_straight, ang_vel_move_straight, 'r', label="actual")
    axs1[1].plot(time_move_straight_des, ang_vel_move_straight_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/wheeled_ms_vels.jpg")

    _, axsPos = plt.subplots(1)
    plt.title("MoveStraight Position (Wheeled Base)")
    axsPos.set_ylabel("Meters/Y")
    axsPos.set_xlabel("Meters/X")
    axsPos.plot(posY_move_straight, posX_move_straight, 'r.')
    axsPos.plot(posY_move_straight, posX_move_straight, 'r', label="actual")
    axsPos.plot(posY_move_straight_des, posX_move_straight_des, 'b', label="desired")
    axsPos.axis("equal")
    plt.legend()
    plt.savefig("./savedImages/wheeled_ms_pos.jpg")

    _, axs2 = plt.subplots(1)
    plt.title("Spin Angle (Wheeled Base)")
    axs2.set_ylabel("theta (degrees)")
    axs2.plot(time_spin, spin_theta, 'r.')
    axs2.plot(time_spin, spin_theta, 'r', label="actual")
    axs2.plot(time_spin_des, spin_theta_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/wheeled_spin_degs.jpg")


def plot_sensor_base(run_num: str, dir_path: str):
    path_data = dir_path + f'/sensorData/run{run_num}.txt'
    f = open(path_data, mode="r")
    csv_file = csv.reader(f)
    lin_set_vel = []
    ang_set_vel = []
    time_set_vel = []
    lin_vel_move_straight = []
    ang_vel_move_straight = []
    posX_move_straight = []
    posY_move_straight = []
    time_move_straight = []
    spin_lin_vel = []
    spin_ang_vel = []
    spin_theta = []
    time_spin = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "sv":
                lin_set_vel.append(float(lines[1]))
                ang_set_vel.append(float(lines[2]))
                time_set_vel.append(float(lines[3])/1000)
            case "ms":
                lin_vel_move_straight.append(float(lines[1]))
                ang_vel_move_straight.append(float(lines[2]))
                time_move_straight.append(float(lines[3])/1000)
                posX_move_straight.append(float(lines[4])*1000)
                posY_move_straight.append(float(lines[5])*1000)
            case "s":
                spin_lin_vel.append(float(lines[1]))
                spin_ang_vel.append(float(lines[2]))
                time_spin.append(float(lines[3])/1000)
                spin_theta.append(float(lines[6]))

    path_desired = dir_path + f'/sensorDes/run{run_num}.txt'
    f2 = open(path_desired, mode="r")
    csv_file = csv.reader(f2)
    lin_set_vel_des = []
    ang_set_vel_des = []
    time_set_vel_des = []
    lin_vel_move_straight_des = []
    ang_vel_move_straight_des = []
    posX_move_straight_des = []
    posY_move_straight_des = []
    time_move_straight_des = []
    spin_lin_vel_des = []
    spin_ang_vel_des = []
    spin_theta_des = []
    time_spin_des = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "sv":
                lin_set_vel_des.append(float(lines[1]))
                ang_set_vel_des.append(float(lines[2]))
                time_set_vel_des.append(float(lines[3])/1000)
            case "ms":
                lin_vel_move_straight_des.append(float(lines[1]))
                ang_vel_move_straight_des.append(float(lines[2]))
                time_move_straight_des.append(float(lines[3])/1000)
                posX_move_straight_des.append(float(lines[4])/10)
                posY_move_straight_des.append(float(lines[5])/10)
            case "s":
                spin_lin_vel_des.append(float(lines[1]))
                spin_ang_vel_des.append(float(lines[2]))
                time_spin_des.append(float(lines[3])/1000)
                if float(lines[6]) == -30:
                    spin_theta_des.append(360 + float(lines[6]))
                elif float(lines[6]) < -355:
                    spin_theta_des.append(0)
                else:
                    spin_theta_des.append(float(lines[6]))

    _, axs = plt.subplots(2)
    plt.title("SetVelocity Linear and Angular Velocities (Sensor Base)")
    axs[0].set_ylabel("linear velocity(mm/sec)")
    axs[0].plot(time_set_vel, lin_set_vel, 'r.')
    axs[0].plot(time_set_vel, lin_set_vel, 'r', label="actual")
    axs[0].plot(time_set_vel_des, lin_set_vel_des, 'b', label="desired")
    axs[1].set_ylabel("angular velocity(degs/sec)")
    axs[1].plot(time_set_vel, ang_set_vel, 'r.')
    axs[1].plot(time_set_vel, ang_set_vel, 'r', label="actual")
    axs[1].plot(time_set_vel_des, ang_set_vel_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/sensor_sv_vels.jpg")

    _, axs1 = plt.subplots(2)
    plt.title("MoveStraight Linear and Angular Velocities (Sensor Base)")
    axs1[0].set_ylabel("linear velocity(mm/sec)")
    axs1[0].plot(time_move_straight, lin_vel_move_straight, 'r.')
    axs1[0].plot(time_move_straight, lin_vel_move_straight, 'r', label="actual")
    axs1[0].plot(time_move_straight_des, lin_vel_move_straight_des, 'b', label="desired")
    axs1[1].set_ylabel("angular velocity(degs/sec)")
    axs1[1].plot(time_move_straight, ang_vel_move_straight, 'r.')
    axs1[1].plot(time_move_straight, ang_vel_move_straight, 'r', label="actual")
    axs1[1].plot(time_move_straight_des, ang_vel_move_straight_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/sensor_ms_vels.jpg")

    _, axsPos = plt.subplots(1)
    plt.title("MoveStraight Position (Sensor Base)")
    axsPos.set_ylabel("Meters/Y")
    axsPos.set_xlabel("Meters/X")
    axsPos.plot(posY_move_straight, posX_move_straight, 'r.')
    axsPos.plot(posY_move_straight, posX_move_straight, 'r', label="actual")
    axsPos.plot(posY_move_straight_des, posX_move_straight_des, 'b', label="desired")
    axsPos.axis("equal")
    plt.savefig("./savedImages/sensor_ms_pos.jpg")

    _, axs2 = plt.subplots(1)
    plt.title("Spin Angle (Sensor Base)")
    axs2.set_ylabel("theta (degrees)")
    axs2.plot(time_spin, spin_theta, 'r.')
    axs2.plot(time_spin, spin_theta, 'r', label="actual")
    axs2.plot(time_spin_des, spin_theta_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/sensor_spin_degs.jpg")


def plot_encoded_motor(run_num: str, dir_path: str):
    path_data = dir_path + f'/encodedData/run{run_num}.txt'
    f = open(path_data, mode="r")
    csv_file = csv.reader(f)
    rpm_go_for = []
    pos_go_for = []
    time_go_for = []
    rpm_go_to = []
    pos_go_to = []
    time_go_to = []
    rpm_set_rpm = []
    time_set_rpm = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "gf":
                rpm_go_for.append(float(lines[1]))
                pos_go_for.append(float(lines[2]))
                time_go_for.append(float(lines[3])/1000)
            case "gt":
                rpm_go_to.append(float(lines[1]))
                pos_go_to.append(float(lines[2]))
                time_go_to.append(float(lines[3])/1000)
            case "rpm":
                rpm_set_rpm.append(float(lines[1]))
                time_set_rpm.append(float(lines[3])/1000)

    path_desired = dir_path + f'/encodedDes/run{run_num}.txt'
    f2 = open(path_desired, mode="r")
    csv_file = csv.reader(f2)
    rpm_go_for_des = []
    pos_go_for_des = []
    time_go_for_des = []
    rpm_go_to_des = []
    pos_go_to_des = []
    time_go_to_des = []
    rpm_set_rpm_des = []
    time_set_rpm_des = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "gf":
                rpm_go_for_des.append(float(lines[1]))
                pos_go_for_des.append(float(lines[2]))
                time_go_for_des.append(float(lines[3])/1000)
            case "gt":
                rpm_go_to_des.append(float(lines[1]))
                pos_go_to_des.append(float(lines[2]))
                time_go_to_des.append(float(lines[3])/1000)
            case "rpm":
                rpm_set_rpm_des.append(float(lines[1]))
                time_set_rpm_des.append(float(lines[3])/1000)

    _, axs1 = plt.subplots(1)
    plt.title("GoFor RPM (Encoded Motor)")
    axs1.set_ylabel("RPM")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_for, rpm_go_for, 'r.')
    axs1.plot(time_go_for, rpm_go_for, 'r', label="actual")
    axs1.plot(time_go_for_des, rpm_go_for_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/encoded_go_for_rpm.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("GoFor Position (Encoded Motor)")
    axs1.set_ylabel("Position")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_for, pos_go_for, 'r.')
    axs1.plot(time_go_for, pos_go_for, 'r', label="actual")
    axs1.plot(time_go_for_des, pos_go_for_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/encoded_go_for_pos.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("GoTo RPM (Encoded Motor)")
    axs1.set_ylabel("RPM")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_to, rpm_go_to, 'r.')
    axs1.plot(time_go_to, rpm_go_to, 'r', label="actual")
    axs1.plot(time_go_to_des, rpm_go_to_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/encoded_go_to_rpm.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("GoTo Position (Encoded Motor)")
    axs1.set_ylabel("Position")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_to, pos_go_to, 'r.')
    axs1.plot(time_go_to, pos_go_to, 'r', label="actual")
    axs1.plot(time_go_to_des, pos_go_to_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/encoded_go_to_pos.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("SetRPM RPM (Encoded Motor)")
    axs1.set_ylabel("RPM")
    axs1.set_xlabel("Time")
    axs1.plot(time_set_rpm, rpm_set_rpm, 'r.')
    axs1.plot(time_set_rpm, rpm_set_rpm, 'r', label="actual")
    axs1.plot(time_set_rpm_des, rpm_set_rpm_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/encoded_set_rpm_rpm.jpg")


def plot_controlled_motor(run_num: str, dir_path: str):
    path_data = dir_path + f'/controlledData/run{run_num}.txt'
    f = open(path_data, mode="r")
    csv_file = csv.reader(f)
    rpm_go_for = []
    pos_go_for = []
    time_go_for = []
    rpm_go_to = []
    pos_go_to = []
    time_go_to = []
    rpm_set_rpm = []
    time_set_rpm = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "gf":
                rpm_go_for.append(float(lines[1]))
                pos_go_for.append(float(lines[2]))
                time_go_for.append(float(lines[3])/1000)
            case "gt":
                rpm_go_to.append(float(lines[1]))
                pos_go_to.append(float(lines[2]))
                time_go_to.append(float(lines[3])/1000)
            case "rpm":
                rpm_set_rpm.append(float(lines[1]))
                time_set_rpm.append(float(lines[3])/1000)

    path_desired = dir_path + f'/controlledDes/run{run_num}.txt'
    f2 = open(path_desired, mode="r")
    csv_file = csv.reader(f2)
    rpm_go_for_des = []
    pos_go_for_des = []
    time_go_for_des = []
    rpm_go_to_des = []
    pos_go_to_des = []
    time_go_to_des = []
    rpm_set_rpm_des = []
    time_set_rpm_des = []

    for i, lines in enumerate(csv_file):
        match lines[0]:
            case "gf":
                rpm_go_for_des.append(float(lines[1]))
                pos_go_for_des.append(float(lines[2]))
                time_go_for_des.append(float(lines[3])/1000)
            case "gt":
                rpm_go_to_des.append(float(lines[1]))
                pos_go_to_des.append(float(lines[2]))
                time_go_to_des.append(float(lines[3])/1000)
            case "rpm":
                rpm_set_rpm_des.append(float(lines[1]))
                time_set_rpm_des.append(float(lines[3])/1000)

    _, axs1 = plt.subplots(1)
    plt.title("GoFor RPM (Controlled Motor)")
    axs1.set_ylabel("RPM")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_for, rpm_go_for, 'r.')
    axs1.plot(time_go_for, rpm_go_for, 'r', label="actual")
    axs1.plot(time_go_for_des, rpm_go_for_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/controlled_go_for_rpm.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("GoFor Position (Controlled Motor)")
    axs1.set_ylabel("Position")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_for, pos_go_for, 'r.')
    axs1.plot(time_go_for, pos_go_for, 'r', label="actual")
    axs1.plot(time_go_for_des, pos_go_for_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/controlled_go_for_pos.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("GoTo RPM (Controlled Motor)")
    axs1.set_ylabel("RPM")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_to, rpm_go_to, 'r.')
    axs1.plot(time_go_to, rpm_go_to, 'r', label="actual")
    axs1.plot(time_go_to_des, rpm_go_to_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/controlled_go_to_rpm.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("GoTo Position (Controlled Motor)")
    axs1.set_ylabel("Position")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_to, pos_go_to, 'r.')
    axs1.plot(time_go_to, pos_go_to, 'r', label="actual")
    axs1.plot(time_go_to_des, pos_go_to_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/controlled_go_to_pos.jpg")

    _, axs1 = plt.subplots(1)
    plt.title("SetRPM RPM (Controlled Motor)")
    axs1.set_ylabel("RPM")
    axs1.set_xlabel("Time")
    axs1.plot(time_go_to, rpm_go_to, 'r.')
    axs1.plot(time_go_to, rpm_go_to, 'r', label="actual")
    axs1.plot(time_go_to_des, rpm_go_to_des, 'b', label="desired")
    plt.legend()
    plt.savefig("./savedImages/controlled_set_rpm_rpm.jpg")

def plot_grid_test(run_num: str, dir_path: str):
    path_data = dir_path + f'/gridData/run{run_num}.txt'
    f = open(path_data, mode="r")
    csv_file = csv.reader(f)
    posX = []
    posY = []

    # Skip the first line, its a header row
    for i, lines in enumerate(csv_file):
        if i == 0:
            continue
        posX.append(float(lines[0])*1000)
        posY.append(float(lines[1])*1000)
    
    path_desired = dir_path + f'/gridDes/run{run_num}.txt'
    f2 = open(path_desired, mode="r")
    csv_file = csv.reader(f2)
    posX_des = []
    posY_des = []

    # Skip the first line, its a header row
    for i, lines in enumerate(csv_file):
        if i == 0:
            continue
        posX_des.append(float(lines[0]))
        posY_des.append(float(lines[1]))
    
    _, axsPos = plt.subplots(1)
    plt.title("Grid Test (Sensor Base)")
    axsPos.set_ylabel("Meters/Y")
    axsPos.set_xlabel("Meters/X")
    axsPos.plot(posY, posX, 'r.')
    axsPos.plot(posY, posX, 'r', label="actual")
    axsPos.plot(posY_des, posX_des, 'b', label="desired")
    axsPos.axis("equal")
    plt.savefig("./savedImages/grid_test.jpg")

if __name__ == '__main__':
    # get the current directory
    dir_path = cwd = os.getcwd()

    list_of_files = glob.glob(dir_path+'/wheeledDes/*.txt') # * means all if need specific format then *.csv
    datafile = max(list_of_files, key=os.path.getctime)
    filesplit = datafile.split("/")
    runName = filesplit[-1].split(".")[0].split("run")[1]

    plot_wheeled_base(runName, dir_path)

    list_of_files = glob.glob(dir_path+'/sensorDes/*.txt') # * means all if need specific format then *.csv
    datafile = max(list_of_files, key=os.path.getctime)
    filesplit = datafile.split("/")
    runName = filesplit[-1].split(".")[0].split("run")[1]

    plot_sensor_base(runName, dir_path)

    list_of_files = glob.glob(dir_path+'/encodedDes/*.txt') # * means all if need specific format then *.csv
    datafile = max(list_of_files, key=os.path.getctime)
    filesplit = datafile.split("/")
    runName = filesplit[-1].split(".")[0].split("run")[1]

    plot_encoded_motor(runName, dir_path)

    list_of_files = glob.glob(dir_path+'/controlledDes/*.txt') # * means all if need specific format then *.csv
    datafile = max(list_of_files, key=os.path.getctime)
    filesplit = datafile.split("/")
    runName = filesplit[-1].split(".")[0].split("run")[1]

    plot_controlled_motor(runName, dir_path)

    list_of_files = glob.glob(dir_path+'/gridDes/*.txt') # * means all if need specific format then *.csv
    datafile = max(list_of_files, key=os.path.getctime)
    filesplit = datafile.split("/")
    runName = filesplit[-1].split(".")[0].split("run")[1]

    plot_grid_test(runName, dir_path)
