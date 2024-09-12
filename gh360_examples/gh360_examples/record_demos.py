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
from rclpy.serialization import deserialize_message

from gh360_interfaces.msg import SpaceMouse, PortStatus
from sensor_msgs.msg import JointState
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

        self.tree_view = self.tab_2.Treeview(["File Name", "Date", "Time"], [120,120,120], 10, self.file_tree, 'files', ["name", "date", "time"], row=0, col=0, rowspan=1, colspan=2)
        self.btn_start_replay = self.tab_2.AccentButton("Play", self.start_replay, row=1, col=0)
        self.btn_stop_replay = self.tab_2.AccentButton("Stop", self.stop_replay, row=1, col=1)
        self.btn_stop_replay["state"] = "disabled"

        self.tab_3 = self.notebook.addTab("Visualize Data")
        self.tree_view_2 = self.tab_3.Treeview(["File Name", "Date", "Time"], [120,120,120], 10, self.file_tree, 'files', ["name", "date", "time"], row=0, col=0, rowspan=1, colspan=2)
        self.btn_read_data = self.tab_3.AccentButton("Read Data", self.read_data, row=1, col=0)

        self.vis_data_options = ["Motor 1 Position", "Motor 2 Position", "Motor 3 Position", "Motor 4 Position", 
                                 "Motor 5 Position", "Motor 6 Position", "Motor 7 Position", "Motor 8 Position", 
                                 "Motor 9 Position", "Motor 10 Position", "Motor 11 Position", "Motor 12 Position", 
                                 "Motor 13 Position", "Shoulder Yaw Position", "Shoulder Roll Position", "Shoulder Pitch Position", 
                                 "Upperarm Roll Position", "Elbow Position", "Lowerarm Position", "Wrist Pitch Position"]
        self.vis_data_var = tk.StringVar(value=self.vis_data_options[0])
        self.vis_data_var.trace_add("write", self.vis_data_changed)
        self.tab_3.OptionMenu(self.vis_data_options, self.vis_data_var, row=2, col=0)
        self.graphframe = self.tab_3.addLabelFrame("2D Graph")
        self.canvas, fig, self.ax, background, self.accent = self.tab_3.matplotlibFrame("Graph Frame Test")

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
                # print(f'Environment: {env}, Filename: {filename}')

            if dir_lvl == 0 and result_string != "":
                new_environment = {}
                new_environment["name"] = env
                new_environment["files"] = []
                file_tree.append(new_environment)
            if dir_lvl == 1:
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

    def read_data(self):
        if self.tree_view_2.item(self.tree_view_2.selection())["values"] == "":
            self.get_logger().error("Please select a file to replay")
            return
        
        name = self.tree_view_2.item(self.tree_view_2.selection())["text"]
        date = self.tree_view_2.item(self.tree_view_2.selection())["values"][0]
        time = self.tree_view_2.item(self.tree_view_2.selection())["values"][1]
        rosbag_name = self.name_date_time_to_filename(name, date, time)
        env = self.tree_view_2.item(self.tree_view_2.parent(self.tree_view_2.selection()))["text"]
        rosbag_uri = f'{self.path_demo_dir}{env}/{rosbag_name}'

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=rosbag_uri,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.rosbag_reader.open(storage_options, converter_options)

        self.motor_positions = {}
        for i in range(1, 14):
            self.motor_positions[f"motor_{i}"] = []
        self.joint_positions = {}
        self.joint_positions["shoulder_yaw"] = []
        self.joint_positions["shoulder_roll"] = []
        self.joint_positions["shoulder_pitch"] = []
        self.joint_positions["upperarm_roll"] = []
        self.joint_positions["elbow"] = []
        self.joint_positions["lowerarm_roll"] = []
        self.joint_positions["wrist_pitch"] = []
        self.time = {}
        self.time["joint_positions"] = []
        self.time["shoulder_motors"] = []
        self.time["upperarm_motors"] = []
        self.time["lowerarm_motors"] = []
        while self.rosbag_reader.has_next():
            topic, msg, t = self.rosbag_reader.read_next()
            if topic == "/shoulder/motor_status":
                msg_dec = deserialize_message(msg, PortStatus)
                for motor in msg_dec.motors:
                    self.motor_positions[f"motor_{motor.motor_id}"].append(motor.present_position)
                self.time["shoulder_motors"].append(t)   
            elif topic == "/upperarm/motor_status":
                msg_dec = deserialize_message(msg, PortStatus)
                for motor in msg_dec.motors:
                    self.motor_positions[f"motor_{motor.motor_id}"].append(motor.present_position)
                self.time["upperarm_motors"].append(t)
            elif topic == "/lowerarm/motor_status":
                msg_dec = deserialize_message(msg, PortStatus)
                for motor in msg_dec.motors:
                    self.motor_positions[f"motor_{motor.motor_id}"].append(motor.present_position)
                self.time["lowerarm_motors"].append(t)
            elif topic == "/gh360_joint_states":
                msg_dec = deserialize_message(msg, JointState)
                self.joint_positions["shoulder_yaw"].append(msg_dec.position[0])
                self.joint_positions["shoulder_roll"].append(msg_dec.position[1])
                self.joint_positions["shoulder_pitch"].append(msg_dec.position[2])
                self.joint_positions["upperarm_roll"].append(msg_dec.position[3])
                self.joint_positions["elbow"].append(msg_dec.position[4])
                self.joint_positions["lowerarm_roll"].append(msg_dec.position[5])
                self.joint_positions["wrist_pitch"].append(msg_dec.position[6])
                self.time["joint_positions"].append(t)

                # print(f"Topic: {topic}, Message: {msg_str}, Time: {t}")

        self.data_read = True

        print("Spacemouse Messages:")
        for i in range(1, 14):
            if self.vis_data_var.get() == f"Motor {i} Position":
                print(f"Motor {i} Positions:")
                data = self.motor_positions[f"motor_{i}"]
                if i < 7:
                    t = self.time["shoulder_motors"]
                elif i < 11:
                    t = self.time["upperarm_motors"]
                else:
                    t = self.time["lowerarm_motors"]
        self.ax.clear()
        self.ax.plot(t, data)
        self.canvas.draw()

    def vis_data_changed(self, *args):
        if self.data_read == False:
            return
        for i in range(1, 14):
            if self.vis_data_var.get() == f"Motor {i} Position":
                print(f"Motor {i} Positions:")
                data = self.motor_positions[f"motor_{i}"]
                if i < 7:
                    t = self.time["shoulder_motors"]
                elif i < 11:
                    t = self.time["upperarm_motors"]
                else:
                    t = self.time["lowerarm_motors"]
        self.ax.clear()
        self.ax.plot(t, data)
        self.canvas.draw()

    def start_replay(self):
        if self.tree_view.item(self.tree_view.selection())["values"] == "":
            self.get_logger().error("Please select a file to replay")
            return
        
        self.btn_start_replay["state"] = "disabled"
        name = self.tree_view.item(self.tree_view.selection())["text"]
        date = self.tree_view.item(self.tree_view.selection())["values"][0]
        time = self.tree_view.item(self.tree_view.selection())["values"][1]
        rosbag_name = self.name_date_time_to_filename(name, date, time)
        pre_command = f'source {self.path_venv}/bin/activate; source {self.path_ros2_ws}/install/setup.bash;'
        env = self.tree_view.item(self.tree_view.parent(self.tree_view.selection()))["text"]
        process_command = f'{pre_command} ros2 bag play {self.path_demo_dir}{env}/{rosbag_name}'
        self.replay_process = subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        self.get_logger().info("Start Replay")
        self.btn_stop_replay["state"] = "enabled"

    def stop_replay(self):
        self.get_logger().info("Stop Replay")
        self.btn_stop_replay["state"] = "disabled"
        if self.replay_process.poll() is None:
            os.killpg(os.getpgid(self.replay_process.pid), signal.SIGINT)
            self.replay_process.wait()
        self.btn_start_replay["state"] = "enabled"

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
        self.btn_start_rec["state"] = "disabled"
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
        dt = time.strftime("%Y%m%d-%H%M%S")
        rosbag_name = self.record_filename.get()+"_"+str(dt)
        process_command = f'{pre_command} ros2 bag record -o {self.path_demo_dir}{env}/{rosbag_name}'
        for topic in topics:
            process_command += f' {topic}'
        self.record_process = subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
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

    # def addData(self):
    #     x = []
    #     y = []
    #     z = []

    #     for i in range(0, 100):
    #         for l in [x, y, z]:
    #             l.append(random.random() * 100)

    #     self.ax.scatter(x, y, c=self.accent)
    #     self.ax2.scatter(x, y, z, c=self.accent)
    #     self.canvas.draw()
    #     self.canvas2.draw()

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