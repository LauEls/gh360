import rclpy
from rclpy.node import Node
import tkinter.ttk as ttk
import tkinter as tk
import re
import time
import TKinterModernThemes as TKMT
import random
import os
import subprocess
import signal
import rosbag2_py
import copy
from rclpy.serialization import deserialize_message

from gh360_interfaces.msg import SpaceMouse, PortStatus, SetMotorVelocities, SetVelocity
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
# may raise PackageNotFoundError


class RecordDemos(Node, TKMT.ThemedTKinterFrame):
    def __init__(self): 
        Node.__init__(self,"record_demo_gui")
        TKMT.ThemedTKinterFrame.__init__(self, "Record Demonstrations", "sun-valley", "dark")

        self.path_ros2_ws = '/home/laurenz/phd_project/ros2_gh360_ws'
        self.path_demo_dir = f'{self.path_ros2_ws}/src/gh360/gh360_examples/data/spacemouse_demonstrations/'
        self.path_venv = '~/phd_project/robosuite_venv'
    
        self.rosbag_reader = rosbag2_py.SequentialReader()
        self.data_read = False
        self.replay_record = False
        self.step_replay = False

        # print(_path)
        # self.AccentButton("Add Data", self.addData, row=0, col=0)
        self.notebook = self.Notebook("Test Notebook", row=0, col=0, rowspan=3, colspan=2)
        self.tab_1 = self.notebook.addTab("Record Data")

        self.tab_1.Text("Environment", row=0, col=0)
        self.env_option_menu_list = ["No Environment", "Door"]
        self.record_env = tk.StringVar(value=self.env_option_menu_list[0])
        self.tab_1.OptionMenu(self.env_option_menu_list, self.record_env, row=0, col=1)
        # cbox_environment["state"] = "readonly"self.textinputvar = tk.StringVar(value="Type text here.")
        self.record_filename = tk.StringVar(value="")
        self.tab_1.Text("File Name:", row=1, col=0)
        self.tab_1.Entry(self.record_filename, row=1, col=1)
        self.btn_start_rec = self.tab_1.AccentButton("Start Recording", self.start_record, row=2, col=0)
        self.btn_stop_rec = self.tab_1.AccentButton("Stop Recording", self.stop_record, row=2, col=1)
        self.btn_stop_rec["state"] = "disabled"

        self.tab_2 = self.notebook.addTab("Replay Data")
        self.file_tree = self.parse_file_tree()

        self.tree_view = self.tab_2.Treeview(["File Name", "Date", "Time"], [150,120,120], 10, self.file_tree, 'files', ["name", "date", "time"], row=0, col=0, rowspan=1, colspan=2, anchor="center")
        self.btn_start_replay = self.tab_2.AccentButton("Play", lambda: self.start_replay(record=False), row=1, col=0)
        self.btn_stop_replay = self.tab_2.AccentButton("Stop", self.stop_replay, row=1, col=1)
        self.btn_stop_replay["state"] = "disabled"
        self.btn_start_replay_record = self.tab_2.AccentButton("Play and Record", lambda: self.start_replay(record=True), row=2, col=0, colspan=2)
        self.btn_start_step_replay = self.tab_2.AccentButton("Play Steps", lambda: self.start_step_replay(gym=False), row=3, col=0, colspan=2)
        self.btn_start_step_replay = self.tab_2.AccentButton("Play Gym Steps", lambda: self.start_step_replay(gym=True), row=4, col=0, colspan=2)

        self.tab_3 = self.notebook.addTab("Visualize Data")
        self.tree_view_2 = self.tab_3.Treeview(["File Name", "Date", "Time"], [150,120,120], 10, self.file_tree, 'files', ["name", "date", "time"], row=0, col=0, rowspan=1, colspan=2, anchor="center")
        self.btn_read_data = self.tab_3.AccentButton("Read Data", self.read_data, row=1, col=0, colspan=2, rowspan=1)

        self.replay_files_frame = self.tab_3.addLabelFrame("Replay Files", row=2, col=0)
        self.cbtn_replay_var = []
        self.cbtn_replay = []        

        self.vis_data_options = ["Motor 1 Position", "Motor 2 Position", "Motor 3 Position", "Motor 4 Position", 
                                 "Motor 5 Position", "Motor 6 Position", "Motor 7 Position", "Motor 8 Position", 
                                 "Motor 9 Position", "Motor 10 Position", "Motor 11 Position", "Motor 12 Position", 
                                 "Motor 13 Position", "Motor 1 Goal Velocity", "Motor 2 Goal Velocity", "Motor 3 Goal Velocity",
                                 "Motor 4 Goal Velocity", "Motor 5 Goal Velocity", "Motor 6 Goal Velocity", "Motor 7 Goal Velocity",
                                 "Motor 8 Goal Velocity", "Motor 9 Goal Velocity", "Motor 10 Goal Velocity", "Motor 11 Goal Velocity",
                                 "Motor 12 Goal Velocity", "Motor 13 Goal Velocity", "Shoulder Yaw Position", "Shoulder Roll Position", 
                                 "Shoulder Pitch Position", "Upperarm Roll Position", "Elbow Position", "Lowerarm Roll Position", 
                                 "Wrist Pitch Position"]
        self.vis_data_var = tk.StringVar(value=self.vis_data_options[0])
        self.vis_data_var.trace_add("write", self.vis_data_changed)
        self.tab_3.OptionMenu(self.vis_data_options, self.vis_data_var, row=1, col=2)
        self.graphframe = self.tab_3.addLabelFrame("2D Graph", row=0, col=2)
        self.canvas, fig, self.ax, background, self.accent = self.graphframe.matplotlibFrame("Graph Frame Test")

        self.notebook.notebook.bind("<<NotebookTabChanged>>", self.tab_changed)
        
        self.run()

    def parse_file_tree(self):
        file_tree = []
        
        # print("Parsing File Tree")
        dir_lvl = 0
        for root, dirs, files in os.walk(self.path_demo_dir):
            result_string = re.sub(self.path_demo_dir, "", root)

            if not re.search('/', result_string):
                dir_lvl = 0
                split_string = result_string.split('/')
                env = split_string[0]
            else:
                dir_lvl = result_string.count('/')
                split_string = result_string.split('/')
                env = split_string[0]
                filename = split_string[1]

            if dir_lvl == 0 and result_string != "":
                new_environment = {}
                new_environment["name"] = env
                new_environment["files"] = []
                file_tree.append(new_environment)
            if dir_lvl == 1:
                if re.search('replay', filename) or re.search('step', filename):
                    continue
                new_file = {}
                name, date, time = self.filename_to_name_date_time(filename)
                new_file["name"] = name
                new_file["date"] = date
                new_file["time"] = time
                for tree_node in file_tree:
                    if tree_node["name"] == env:
                        tree_node["files"].append(new_file)
                        break

        return file_tree
    
    def find_replay_files(self, bagname):
        self.cbtn_replay_var = []
        self.cbtn_replay_var.append(tk.BooleanVar(value=True))
        self.cbtn_replay_var.append(tk.BooleanVar(value=False))
        self.cbtn_replay = []
        self.cbtn_replay.append(self.replay_files_frame.Checkbutton("Original Recording", self.cbtn_replay_var[0], command=self.draw_graph))
        self.cbtn_replay.append(self.replay_files_frame.Checkbutton("Steps", self.cbtn_replay_var[1], command=self.draw_graph))

        replay_bags = []

        for root, dirs, files in os.walk(self.path_demo_dir):
            for dir in dirs:
                if re.search(bagname, dir) and re.search('replay', dir):
                    print(f"Found: {dir}")
                    replay_bags.append(dir)
                    self.cbtn_replay_var.append(tk.BooleanVar(value=False))
                    self.cbtn_replay.append(self.replay_files_frame.Checkbutton(f"Replay {dir[-15:]}", self.cbtn_replay_var[-1], command=self.draw_graph))
                elif re.search(bagname, dir) and re.search('gym_step', dir):
                    print(f"Found: {dir}")
                    replay_bags.append(dir)
                    self.cbtn_replay_var.append(tk.BooleanVar(value=False))
                    self.cbtn_replay.append(self.replay_files_frame.Checkbutton(f"Gym Step Replay {dir[-15:]}", self.cbtn_replay_var[-1], command=self.draw_graph))
                elif re.search(bagname, dir) and re.search('step', dir):
                    print(f"Found: {dir}")
                    replay_bags.append(dir)
                    self.cbtn_replay_var.append(tk.BooleanVar(value=False))
                    self.cbtn_replay.append(self.replay_files_frame.Checkbutton(f"Step Replay {dir[-15:]}", self.cbtn_replay_var[-1], command=self.draw_graph))

        return replay_bags

    def tab_changed(self, event):
        notebook = event.widget
        tab_id = notebook.select()
        tab_text = notebook.tab(tab_id, "text")
        # print(f"Tab Changed to: {tab_text}")
        if tab_text == "Replay Data":
            self.update_treeview(self.tree_view)

        if tab_text == "Visualize Data":
            self.update_treeview(self.tree_view_2)
            pass

    def read_bag(self, rosbag_uri):
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=rosbag_uri,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.rosbag_reader.open(storage_options, converter_options)

        bag_data = {}

        bag_data["motor_positions"] = {}
        bag_data["motor_velocities"] = {}
        bag_data["motor_goal_velocities"] = {}
        for i in range(1, 14):
            bag_data["motor_positions"][f"motor_{i}"] = []
            bag_data["motor_velocities"][f"motor_{i}"] = []
            bag_data["motor_goal_velocities"][f"motor_{i}"] = []
        bag_data["joint_positions"] = {}
        bag_data["joint_positions"]["shoulder_yaw"] = []
        bag_data["joint_positions"]["shoulder_roll"] = []
        bag_data["joint_positions"]["shoulder_pitch"] = []
        bag_data["joint_positions"]["upperarm_roll"] = []
        bag_data["joint_positions"]["elbow"] = []
        bag_data["joint_positions"]["lowerarm_roll"] = []
        bag_data["joint_positions"]["wrist_pitch"] = []
        bag_data["gym_stepping"] = []
        bag_data["time"] = {}
        bag_data["time"]["joint_positions"] = []
        bag_data["time"]["shoulder_motors"] = []
        bag_data["time"]["upperarm_motors"] = []
        bag_data["time"]["lowerarm_motors"] = []
        bag_data["time"]["shoulder_motor_goal_velocities"] = []
        bag_data["time"]["upperarm_motor_goal_velocities"] = []
        bag_data["time"]["lowerarm_motor_goal_velocities"] = []
        bag_data["time"]["gym_stepping"] = []
        while self.rosbag_reader.has_next():
            topic, msg, t = self.rosbag_reader.read_next()

            if topic.endswith("motor_status"):
                msg_dec = deserialize_message(msg, PortStatus)
                for motor in msg_dec.motors:
                    if motor.motor_id < 14:
                        bag_data["motor_positions"][f"motor_{motor.motor_id}"].append(motor.present_position)
                        bag_data["motor_velocities"][f"motor_{motor.motor_id}"].append(motor.present_velocity)
                port = topic.split("/")[1]
                bag_data["time"][f"{port}_motors"].append(t)
            elif topic.endswith("motor_goal_velocity"):
                msg_dec = deserialize_message(msg, SetMotorVelocities)
                port = topic.split("/")[1]
                # print(f"Message: {msg_dec}")
                for motor_vel in msg_dec.motor_goal_velocities:
                    # print(f"Motor {motor_vel.id} Goal Velocity: {motor_vel.velocity}")
                    # if motor_vel.id < 14:
                    if (port == "shoulder" and motor_vel.id < 7) or (port == "upperarm" and 6 < motor_vel.id < 11) or (port == "lowerarm" and 10 < motor_vel.id < 14):
                        bag_data["motor_goal_velocities"][f"motor_{motor_vel.id}"].append(motor_vel.velocity)
                # print(f"Port: {port}")
                bag_data["time"][f"{port}_motor_goal_velocities"].append(t)
            elif topic == "/gh360_joint_states":
                msg_dec = deserialize_message(msg, JointState)
                bag_data["joint_positions"]["shoulder_yaw"].append(msg_dec.position[0])
                bag_data["joint_positions"]["shoulder_roll"].append(msg_dec.position[1])
                bag_data["joint_positions"]["shoulder_pitch"].append(msg_dec.position[2])
                bag_data["joint_positions"]["upperarm_roll"].append(msg_dec.position[3])
                bag_data["joint_positions"]["elbow"].append(msg_dec.position[4])
                bag_data["joint_positions"]["lowerarm_roll"].append(msg_dec.position[5])
                bag_data["joint_positions"]["wrist_pitch"].append(msg_dec.position[6])
                bag_data["time"]["joint_positions"].append(t)
            elif topic == "/gym_stepping":
                msg_dec = deserialize_message(msg, Bool)
                bag_data["gym_stepping"].append(msg_dec.data)
                bag_data["time"]["gym_stepping"].append(t)

        return bag_data

    def read_data(self):
        if self.tree_view_2.item(self.tree_view_2.selection())["values"] == "":
            self.get_logger().error("Please select a file to replay")
            return
        
        for cbtn in self.cbtn_replay:
            cbtn.destroy()

        name = self.tree_view_2.item(self.tree_view_2.selection())["text"]
        date = self.tree_view_2.item(self.tree_view_2.selection())["values"][0]
        time = self.tree_view_2.item(self.tree_view_2.selection())["values"][1]
        rosbag_name = self.name_date_time_to_filename(name, date, time)
        env = self.tree_view_2.item(self.tree_view_2.parent(self.tree_view_2.selection()))["text"]
        rosbag_uri = f'{self.path_demo_dir}{env}/{rosbag_name}'

        self.bag_data = []
        original_bag_data = self.read_bag(rosbag_uri)
        # self.bag_data.append(original_bag_data)
        filtered_og_bag_data = self.filter_bag_data(copy.deepcopy(original_bag_data))        
        self.bag_data.append(filtered_og_bag_data)
        self.bag_data.append(self.generate_step_data(original_bag_data))

        print(f"Goal Velocitiy length: {len(self.bag_data[0]['motor_goal_velocities']['motor_1'])}")
        print(f"Shoulder Velocity time length: {len(self.bag_data[0]['time']['shoulder_motor_goal_velocities'])}")
        print(f"Upperarm Velocity time length: {len(self.bag_data[0]['time']['upperarm_motor_goal_velocities'])}")
        print(f"Lowerarm Velocity time length: {len(self.bag_data[0]['time']['lowerarm_motor_goal_velocities'])}")
        # self.pre_process_data(self.bag_data[0])

        replay_bags = self.find_replay_files(rosbag_name)
        for bag_name in replay_bags:
            rosbag_uri = f'{self.path_demo_dir}{env}/{bag_name}'
            new_bag_data = self.read_bag(rosbag_uri)
            if re.search('gym_step', bag_name):
                # self.bag_data.append(self.filter_gym_bag_data(new_bag_data))
                self.bag_data.append(self.filter_bag_data(new_bag_data))
            else:
                self.bag_data.append(self.filter_bag_data(new_bag_data))

        self.data_read = True

        self.draw_graph()

    def filter_gym_bag_data(self, bag_data):
        start_move_time = bag_data["time"]["gym_stepping"][0]
        end_move_time = bag_data["time"]["gym_stepping"][-1]

        for i in range(1, 14):
            if i < 7:
                time_list_pos = bag_data["time"]["shoulder_motors"]
            elif i < 11:
                time_list_pos = bag_data["time"]["upperarm_motors"]
            else:
                time_list_pos = bag_data["time"]["lowerarm_motors"]

            for z in range(len(time_list_pos) - 1, -1, -1):
                if time_list_pos[z] < start_move_time or time_list_pos[z] > end_move_time:
                    bag_data["motor_positions"][f"motor_{i}"].pop(z)
                    if i == 6:
                        bag_data["time"]["shoulder_motors"].pop(z)
                    elif i == 10:
                        bag_data["time"]["upperarm_motors"].pop(z)
                    elif i == 13:
                        bag_data["time"]["lowerarm_motors"].pop(z)

        return bag_data

    def filter_bag_data(self, bag_data):
        goal_velocities = bag_data["motor_goal_velocities"]
        start_move_time = 10e20
        end_move_time = 0

        for i in range(1, 14):
            if i < 7:
                time_list = bag_data["time"]["shoulder_motor_goal_velocities"]
            elif i < 11:
                time_list = bag_data["time"]["upperarm_motor_goal_velocities"]
            else:
                time_list = bag_data["time"]["lowerarm_motor_goal_velocities"]

            for z, t in enumerate(time_list):
                if goal_velocities[f"motor_{i}"][z] != 0.0 and t < start_move_time:
                    start_move_time = t
                    # start_move_time = time_list[z-1]
                    break

            for t in reversed(time_list):
                z = time_list.index(t)
                if goal_velocities[f"motor_{i}"][z] != 0.0 and t > end_move_time:
                    end_move_time = t
                    # end_move_time = time_list[z+1]
                    break

        for i in range(1, 14):
            if i < 7:
                time_list = bag_data["time"]["shoulder_motor_goal_velocities"]
                time_list_pos = bag_data["time"]["shoulder_motors"]
            elif i < 11:
                time_list = bag_data["time"]["upperarm_motor_goal_velocities"]
                time_list_pos = bag_data["time"]["upperarm_motors"]
            else:
                time_list = bag_data["time"]["lowerarm_motor_goal_velocities"]
                time_list_pos = bag_data["time"]["lowerarm_motors"]

            for z in range(len(time_list) - 1, -1, -1):
                if time_list[z] < start_move_time or time_list[z] > end_move_time:
                    # bag["time"][time_list].pop(i)
                    bag_data["motor_goal_velocities"][f"motor_{i}"].pop(z)
                    if i == 6:
                        bag_data["time"]["shoulder_motor_goal_velocities"].pop(z)
                    elif i == 10:
                        bag_data["time"]["upperarm_motor_goal_velocities"].pop(z)
                    elif i == 13:
                        bag_data["time"]["lowerarm_motor_goal_velocities"].pop(z)

            for z in range(len(time_list_pos) - 1, -1, -1):
                if time_list_pos[z] < start_move_time or time_list_pos[z] > end_move_time:
                    bag_data["motor_positions"][f"motor_{i}"].pop(z)
                    if i == 6:
                        bag_data["time"]["shoulder_motors"].pop(z)
                    elif i == 10:
                        bag_data["time"]["upperarm_motors"].pop(z)
                    elif i == 13:
                        bag_data["time"]["lowerarm_motors"].pop(z)
            
        time_list = bag_data["time"]["joint_positions"]
        for z in range(len(time_list)-1, -1, -1):
            if time_list[z] < start_move_time or time_list[z] > end_move_time:
                bag_data["joint_positions"]["shoulder_yaw"].pop(z)
                bag_data["joint_positions"]["shoulder_roll"].pop(z)
                bag_data["joint_positions"]["shoulder_pitch"].pop(z)
                bag_data["joint_positions"]["upperarm_roll"].pop(z)
                bag_data["joint_positions"]["elbow"].pop(z)
                bag_data["joint_positions"]["lowerarm_roll"].pop(z)
                bag_data["joint_positions"]["wrist_pitch"].pop(z)
                bag_data["time"]["joint_positions"].pop(z)

            
        return bag_data

    def generate_step_data(self, original_bag_data):
        bag_data = {}

        bag_data["motor_positions"] = {}
        bag_data["motor_goal_velocities"] = {}
        for i in range(1, 14):
            bag_data["motor_positions"][f"motor_{i}"] = []
            bag_data["motor_goal_velocities"][f"motor_{i}"] = []
        bag_data["joint_positions"] = {}
        bag_data["joint_positions"]["shoulder_yaw"] = []
        bag_data["joint_positions"]["shoulder_roll"] = []
        bag_data["joint_positions"]["shoulder_pitch"] = []
        bag_data["joint_positions"]["upperarm_roll"] = []
        bag_data["joint_positions"]["elbow"] = []
        bag_data["joint_positions"]["lowerarm_roll"] = []
        bag_data["joint_positions"]["wrist_pitch"] = []
        bag_data["time"] = {}
        bag_data["time"]["joint_positions"] = []
        bag_data["time"]["shoulder_motors"] = []
        bag_data["time"]["upperarm_motors"] = []
        bag_data["time"]["lowerarm_motors"] = []
        bag_data["time"]["shoulder_motor_goal_velocities"] = []
        bag_data["time"]["upperarm_motor_goal_velocities"] = []
        bag_data["time"]["lowerarm_motor_goal_velocities"] = []
        # bag_data["motor_pos_steps"] = {}

        for i in range(1, 14):
            step_cntr = 0
            # bag_data["motor_pos_steps"][f"motor_{i}"] = []
            
            if i < 7:
                timestamp = bag_data["time"]["shoulder_motors"]
                t_list = original_bag_data["time"]["shoulder_motors"] 
                t_list = [x - original_bag_data["time"]["shoulder_motors"][0] for x in t_list]
            elif i < 11:
                timestamp = bag_data["time"]["upperarm_motors"]
                t_list = original_bag_data["time"]["upperarm_motors"]
                t_list = [x - original_bag_data["time"]["upperarm_motors"][0] for x in t_list]
            elif i < 14:
                timestamp = bag_data["time"]["lowerarm_motors"]
                t_list = original_bag_data["time"]["lowerarm_motors"]
                t_list = [x - original_bag_data["time"]["lowerarm_motors"][0] for x in t_list]

            # t_init = bag_data[f"motor_{i}_time"][0]
            for z, t in enumerate(t_list):
                # t = t - t_init

                if t >= step_cntr*200e6:
                    # pos_steps[f"motor_{i}_pos_step"].append(bag_data[f"motor_{i}_position"][z])
                    bag_data["motor_positions"][f"motor_{i}"].append((original_bag_data[f"motor_positions"][f"motor_{i}"][z]))
                    # bag_data["motor_goal_velocities"][f"motor_{i}"].append((original_bag_data[f"motor_goal_velocities"][f"motor_{i}"][z]))
                    if i == 1 or i == 7 or i == 11:
                        timestamp.append(t)
                    # if t == 0:
                    #     prev_pos = bag_data[f"motor_{i}_position"][0]
                    # else:
                    #     # vel_steps[f"motor_{i}_vel_step"].append((bag_data[f"motor_{i}_position"][z] - prev_pos)/0.05)
                    #     pos_diff = (bag_data[f"motor_{i}_position"][z] - prev_pos)*10
                    #     motor_steps.append(pos_diff)
                    #     # motor_steps.append((pos_diff/2))
                    #     # motor_steps.append((pos_diff/2))
                    #     prev_pos = bag_data[f"motor_{i}_position"][z]

                    step_cntr += 1

            # pos_steps.append(motor_steps)

        return bag_data
        

    def draw_graph(self):
        self.ax.clear()
        for z in range(len(self.cbtn_replay_var)):
            if self.cbtn_replay_var[z].get():
                bag_data = self.bag_data[z]
                for i in range(1, 14):
                    if self.vis_data_var.get() == f"Motor {i} Position":
                        print(f"Motor {i} Positions:")
                        # data = self.bag_data[z]
                        data = bag_data["motor_positions"][f"motor_{i}"]
                        if i < 7:
                            t = bag_data["time"]["shoulder_motors"] 
                            t = [x - bag_data["time"]["shoulder_motors"][0] for x in t]
                        elif i < 11:
                            t = bag_data["time"]["upperarm_motors"]
                            t = [x - bag_data["time"]["upperarm_motors"][0] for x in t]
                        else:
                            t = bag_data["time"]["lowerarm_motors"]
                            t = [x - bag_data["time"]["lowerarm_motors"][0] for x in t]
                    elif self.vis_data_var.get() == f"Motor {i} Goal Velocity":
                        print(f"Motor {i} Goal Velocities:")
                        data = bag_data["motor_goal_velocities"][f"motor_{i}"]
                        if i < 7:
                            t = bag_data["time"]["shoulder_motor_goal_velocities"]
                            t = [x - bag_data["time"]["shoulder_motor_goal_velocities"][0] for x in t]
                        elif i < 11:
                            t = bag_data["time"]["upperarm_motor_goal_velocities"]
                            t = [x - bag_data["time"]["upperarm_motor_goal_velocities"][0] for x in t]
                        else:
                            t = bag_data["time"]["lowerarm_motor_goal_velocities"]
                            t = [x - bag_data["time"]["lowerarm_motor_goal_velocities"][0] for x in t]
                    elif self.vis_data_var.get() == "Shoulder Yaw Position":
                        data = bag_data["joint_positions"]["shoulder_yaw"]
                        t = bag_data["time"]["joint_positions"]
                        t = [x - bag_data["time"]["joint_positions"][0] for x in t]
                    elif self.vis_data_var.get() == "Shoulder Roll Position":
                        data = bag_data["joint_positions"]["shoulder_roll"]
                        t = bag_data["time"]["joint_positions"]
                        t = [x - bag_data["time"]["joint_positions"][0] for x in t]
                    elif self.vis_data_var.get() == "Shoulder Pitch Position":
                        data = bag_data["joint_positions"]["shoulder_pitch"]
                        t = bag_data["time"]["joint_positions"]
                        t = [x - bag_data["time"]["joint_positions"][0] for x in t]
                    elif self.vis_data_var.get() == "Upperarm Roll Position":
                        data = bag_data["joint_positions"]["upperarm_roll"]
                        t = bag_data["time"]["joint_positions"]
                        t = [x - bag_data["time"]["joint_positions"][0] for x in t]
                    elif self.vis_data_var.get() == "Elbow Position":
                        data = bag_data["joint_positions"]["elbow"]
                        t = bag_data["time"]["joint_positions"]
                        t = [x - bag_data["time"]["joint_positions"][0] for x in t]
                    elif self.vis_data_var.get() == "Lowerarm Roll Position":
                        data = bag_data["joint_positions"]["lowerarm_roll"]
                        t = bag_data["time"]["joint_positions"]
                        t = [x - bag_data["time"]["joint_positions"][0] for x in t]
                    elif self.vis_data_var.get() == "Wrist Pitch Position":
                        data = bag_data["joint_positions"]["wrist_pitch"]
                        t = bag_data["time"]["joint_positions"]
                        t = [x - bag_data["time"]["joint_positions"][0] for x in t]
                
                self.ax.plot(t, data, label=f"Replay {z}")

        self.canvas.draw()

    def vis_data_changed(self, *args):
        if self.data_read == False:
            return
        self.draw_graph()
        

    def start_replay(self, record=False):
        if self.tree_view.item(self.tree_view.selection())["values"] == "":
            self.get_logger().error("Please select a file to replay")
            return
        self.step_replay = False
        self.btn_start_replay["state"] = "disabled"
        self.btn_start_replay_record["state"] = "disabled"
        name = self.tree_view.item(self.tree_view.selection())["text"]
        date = self.tree_view.item(self.tree_view.selection())["values"][0]
        t = self.tree_view.item(self.tree_view.selection())["values"][1]
        rosbag_name = self.name_date_time_to_filename(name, date, t)
        pre_command = f'source {self.path_venv}/bin/activate; source {self.path_ros2_ws}/install/setup.bash;'
        replay_topics = "/shoulder/motor_goal_velocity /upperarm/motor_goal_velocity /lowerarm/motor_goal_velocity"
        env = self.tree_view.item(self.tree_view.parent(self.tree_view.selection()))["text"]
        process_command = f'{pre_command} ros2 bag play {self.path_demo_dir}{env}/{rosbag_name} --topics {replay_topics}'
        self.replay_process = subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        self.get_logger().info("Start Replay")
        self.btn_stop_replay["state"] = "enabled"

        if record:
            self.replay_record = True
            dt = time.strftime("%Y%m%d-%H%M%S")
            rosbag_name = f"{rosbag_name}_replay_{str(dt)}"
            self.record_process = self.start_record_process(env, rosbag_name)
        else:
            self.replay_record = False

    def start_step_replay(self, gym=False):
        if self.tree_view.item(self.tree_view.selection())["values"] == "":
            self.get_logger().error("Please select a file to replay")
            return
        
        self.btn_start_replay["state"] = "disabled"
        self.btn_start_replay_record["state"] = "disabled"
        self.btn_start_step_replay["state"] = "disabled"
        name = self.tree_view.item(self.tree_view.selection())["text"]
        date = self.tree_view.item(self.tree_view.selection())["values"][0]
        t = self.tree_view.item(self.tree_view.selection())["values"][1]
        rosbag_name = self.name_date_time_to_filename(name, date, t)
        env = self.tree_view.item(self.tree_view.parent(self.tree_view.selection()))["text"]
        pre_command = f'source {self.path_venv}/bin/activate; source {self.path_ros2_ws}/install/setup.bash;'
        if gym:
            # process_command = f"{pre_command} python {self.path_ros2_ws}/src/gh360/gh360_examples/gh360_examples/step_execute_recording.py --bag_file {self.path_demo_dir}{env}/{rosbag_name}"
            process_command = f"{pre_command} python {self.path_ros2_ws}/src/gh360/gh360_demonstration/gh360_demonstration/gym_step.py --bag_file {self.path_demo_dir}{env}/{rosbag_name}"
        else:
            # process_command = f"{pre_command} ros2 run gh360_examples pos_step_pub --ros-args -p bag_file_path:={self.path_demo_dir}{env}/{rosbag_name}"
            process_command = f"{pre_command} ros2 run gh360_demonstration step_pub_2 --ros-args -p bag_file_path:={self.path_demo_dir}{env}/{rosbag_name}"
        self.step_replay_process = subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        
        dt = time.strftime("%Y%m%d-%H%M%S")
        if gym:
            rosbag_name = f"{rosbag_name}_gym_step_{str(dt)}"
        else:
            rosbag_name = f"{rosbag_name}_step_{str(dt)}"
        self.record_process = self.start_record_process(env, rosbag_name)

        self.replay_record = True
        self.step_replay = True
        self.get_logger().info("Start Step Replay")
        self.btn_stop_replay["state"] = "enabled"



    def stop_replay(self):
        self.get_logger().info("Stop Replay")
        self.btn_stop_replay["state"] = "disabled"
        if not self.step_replay:
            if self.replay_process.poll() is None:
                os.killpg(os.getpgid(self.replay_process.pid), signal.SIGINT)
                self.replay_process.wait()
        else:
            if self.step_replay_process.poll() is None:
                os.killpg(os.getpgid(self.step_replay_process.pid), signal.SIGINT)
                self.step_replay_process.wait()

        if self.replay_record and self.record_process.poll() is None:
            os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
            self.record_process.wait()

        self.btn_start_replay["state"] = "enabled"
        self.btn_start_replay_record["state"] = "enabled"
        self.btn_start_step_replay["state"] = "enabled"

        

    def start_record_process(self, env, rosbag_name):
        pre_command = f'source {self.path_venv}/bin/activate; source {self.path_ros2_ws}/install/setup.bash;'
        topics = []
        topics.append("/spacemouse")
        topics.append("/shoulder/motor_goal_velocity")
        topics.append("/upperarm/motor_goal_velocity")
        topics.append("/lowerarm/motor_goal_velocity")
        topics.append("/shoulder/motor_status")
        topics.append("/upperarm/motor_status")
        topics.append("/lowerarm/motor_status")
        topics.append("/gh360_joint_states")
        topics.append("/gym_stepping")
        
        process_command = f'{pre_command} ros2 bag record -o {self.path_demo_dir}{env}/{rosbag_name}'
        for topic in topics:
            process_command += f' {topic}'
        record_process = subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)

        return record_process

    def start_record(self):
        invalid_chars = r'[#%&{}$!+=`<>:"/\\|?*\0\s]'
    
        # If the filename contains any invalid characters, return False
        if re.search(invalid_chars, self.record_filename.get()):
            self.get_logger().error("Please enter a valid file name")
            return 
        if self.record_filename.get() == "":
            self.get_logger().error("Please enter a file name")
            return
        if self.record_env.get() == "No Environment":
            env = "no_env"
        elif self.record_env.get() == "Door":
            env = "door"
        dt = time.strftime("%Y%m%d-%H%M%S")
        rosbag_name = self.record_filename.get()+"_"+str(dt)
        self.btn_start_rec["state"] = "disabled"

        self.record_process = self.start_record_process(env, rosbag_name)
        self.get_logger().info("Start Recording")
        self.btn_stop_rec["state"] = "enabled"

    def stop_record(self):
        self.get_logger().info("Stop Recording")
        self.btn_stop_rec["state"] = "disabled"
        if self.record_process.poll() is None:
            os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
            self.record_process.wait()
        self.btn_start_rec["state"] = "enabled"
        
    def filename_to_name_date_time(self, filename):
        datetime = filename.split("_")[-1]
        # print(f'Datetime: {datetime}')
        name = filename[:-len(datetime)-1]
        # print(f'Name: {name}')
        date = f"{datetime[6:8]}/{datetime[4:6]}/{datetime[:4]}"
        # print(f'Date: {date}')
        time = f"{datetime[9:11]}:{datetime[11:13]}:{datetime[13:]}"
        # print(f'Time: {time}')

        return name, date, time
    
    def name_date_time_to_filename(self, name, date, time):
        file_date = date[6:10]+date[3:5]+date[:2]
        file_time = time[:2]+time[3:5]+time[6:]
        return f"{name}_{file_date}-{file_time}"
    
    def update_treeview(self, tv: ttk.Treeview):
        self.file_tree = self.parse_file_tree()
        for child in tv.get_children():
            tv.delete(child)
        cntr = 0
        parent_cntr = 0
        for env in self.file_tree:
            tv.insert("", "end", cntr, text=env["name"], values="")
            tv.item(cntr, open=True)
            parent_cntr = cntr
            cntr += 1
            for file in env["files"]:
                tv.insert(parent_cntr, "end", cntr, text=file["name"], values=(file["date"], file["time"]))
                cntr += 1

def main(args=None):
    rclpy.init(args=args)

    gh360_monitor = RecordDemos()

    rclpy.spin(gh360_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gh360_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()