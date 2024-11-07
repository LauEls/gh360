import rclpy
from rclpy.node import Node
import tkinter.ttk as ttk
import tkinter as tk
import re
import time
import TKinterModernThemes as TKMT
from tkinter.constants import *
import random
import os
import subprocess
import signal
import rosbag2_py
import copy
import numpy as np
from tkscrolledframe import ScrolledFrame
from rclpy.serialization import deserialize_message

from gh360_interfaces.msg import SpaceMouse, PortStatus, SetMotorVelocities, SetVelocity
from gh360_demonstration.rosbag_util import ROSBagUtil, JointNames
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
# may raise PackageNotFoundError


class RecordDemos(Node, TKMT.ThemedTKinterFrame):
    def __init__(self): 
        Node.__init__(self,"record_demo_gui")
        TKMT.ThemedTKinterFrame.__init__(self, "Record Demonstrations", "sun-valley", "dark")

        self.path_ros2_ws = '/home/laurenz/phd_project/ros2_gh360_ws'
        self.path_demo_dir = f'{self.path_ros2_ws}/src/gh360/gh360_examples/data/spacemouse_demonstrations/'
        self.path_learning_data_dir = f'{self.path_ros2_ws}/src/gh360/gh360_demonstration/data/learning_datasets/'
        self.path_venv = '~/phd_project/robosuite_venv'
    
        # self.rosbag_reader = rosbag2_py.SequentialReader()
        # self.rosbag_util = ROSBagUtil()
        self.data_read = False
        self.replay_record = False
        self.step_replay = False

        # self.root.geometry("1200x1040")

        # print(_path)
        # self.AccentButton("Add Data", self.addData, row=0, col=0)
        self.notebook = self.Notebook("Test Notebook", row=0, col=0, rowspan=3, colspan=2)

        ###################
        # Record Data Tab #
        ###################
        self.tab_1 = self.notebook.addTab("Record Data")
        self.tab_1.Text("Environment:", row=0, col=0)
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

        ###################
        # Replay Data Tab #
        ###################
        self.tab_2 = self.notebook.addTab("Replay Data")
        self.file_tree = self.parse_file_tree()
        self.tree_view = self.tab_2.Treeview(["File Name", "Date", "Time"], [150,120,120], 10, self.file_tree, 'files', ["name", "date", "time"], row=0, col=0, rowspan=1, colspan=2, anchor="center")
        self.btn_start_replay = self.tab_2.AccentButton("Play", lambda: self.start_replay(record=False), row=1, col=0)
        self.btn_stop_replay = self.tab_2.AccentButton("Stop", self.stop_replay, row=1, col=1)
        self.btn_stop_replay["state"] = "disabled"
        self.btn_start_replay_record = self.tab_2.AccentButton("Play and Record", lambda: self.start_replay(record=True), row=2, col=0, colspan=2)
        self.btn_start_step_replay = self.tab_2.AccentButton("Play Steps", lambda: self.start_step_replay(gym=False), row=3, col=0, colspan=2)
        self.btn_start_step_replay = self.tab_2.AccentButton("Play Gym Steps", lambda: self.start_step_replay(gym=True), row=4, col=0, colspan=2)

        ######################
        # Visualize Data Tab #
        ######################
        self.tab_3 = self.notebook.addTab("Visualize Data")
        self.tree_view_2 = self.tab_3.Treeview(["File Name", "Date", "Time"], [150,120,120], 10, self.file_tree, 'files', ["name", "date", "time"], row=0, col=0, rowspan=1, colspan=2, anchor="center")
        self.btn_read_data = self.tab_3.AccentButton("Read Data", self.read_data, row=1, col=0, colspan=2, rowspan=1)
        self.replay_files_frame = self.tab_3.addLabelFrame("Replay Files", row=3, col=0)
        replay_file_vscrollbar = ttk.Scrollbar(self.replay_files_frame.master, orient=VERTICAL)
        replay_file_vscrollbar.pack(fill=Y, side=RIGHT, expand=FALSE)
        self.replay_file_canvas = tk.Canvas(self.replay_files_frame.master, height=300, highlightthickness=0, bd=0, yscrollcommand=replay_file_vscrollbar.set)
        self.replay_file_canvas.pack(side=LEFT, fill=BOTH, expand=TRUE)
        replay_file_vscrollbar.config(command=self.replay_file_canvas.yview)
        self.replay_files_interior = tk.Frame(self.replay_file_canvas)
        self.replay_file_interior_id = self.replay_file_canvas.create_window(0, 0, window=self.replay_files_interior, anchor=NW)
        self.replay_files_interior.bind("<Configure>", 
            lambda event, canvas=self.replay_file_canvas, interior_frame=self.replay_files_interior: 
            self._configure_interior(event, canvas, interior_frame)
        )
        self.replay_file_canvas.bind("<Configure>", 
            lambda event, canvas=self.replay_file_canvas, interior_frame=self.replay_files_interior, interior_id=self.replay_file_interior_id: 
            self._configure_canvas(event, canvas, interior_frame, interior_id)
        )
        self.replay_file_canvas.bind_all("<Button-4>", 
            lambda event, scroll=-1, canvas=self.replay_file_canvas: 
            self._on_mousehwheel(event, canvas, scroll)
        )
        self.replay_file_canvas.bind_all("<Button-5>", 
            lambda event, scroll=1, canvas=self.replay_file_canvas: 
            self._on_mousehwheel(event, canvas, scroll)
        )
        
        self.replay_files_frame
        self.cbtn_replay_var = []
        self.cbtn_replay = []        
        self.vis_data_options = ["Motor Position", "Motor Velocity", "Motor Goal Velocity", "Joint Position", "Joint Velocity"]
        self.vis_data_options_2 = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13"]
        self.vis_data_var = tk.StringVar(value=self.vis_data_options[0])
        self.vis_data_var_2 = tk.StringVar(value=self.vis_data_options_2[0])
        self.tab_3.OptionMenu(self.vis_data_options, self.vis_data_var, row=1, col=2)
        self.option_submenu = self.tab_3.OptionMenu(self.vis_data_options_2, self.vis_data_var_2, row=2, col=2)
        self.vis_data_var.trace_add("write", self.vis_data_changed)
        self.vis_data_var_2.trace_add("write", self.vis_data_2_changed)
        self.graphframe = self.tab_3.addLabelFrame("Data Plot", row=0, col=2)
        self.canvas, fig, self.ax, background, self.accent = self.graphframe.matplotlibFrame("Graph Frame Test")

        ######################################
        # Generate Demonstration Dataset Tab #
        ######################################
        self.tab_4 = self.notebook.addTab("Generate Demonstration Dataset")
        self.tab_4.Text("Environment:", row=0, col=0)
        self.demo_env = tk.StringVar(value=self.env_option_menu_list[0])
        self.tab_4.OptionMenu(self.env_option_menu_list, self.demo_env, row=0, col=1)
        self.demo_env.trace_add("write", self.update_recordings)
        self.recordings_frame = self.tab_4.addLabelFrame("Recordings", row=1, col=0, colspan=2)
        recordings_vscrollbar = ttk.Scrollbar(self.recordings_frame.master, orient=VERTICAL)
        recordings_vscrollbar.pack(fill=Y, side=RIGHT, expand=FALSE)
        self.recordings_canvas = tk.Canvas(self.recordings_frame.master, height=300, highlightthickness=0, bd=0, yscrollcommand=recordings_vscrollbar.set)
        self.recordings_canvas.pack(side=LEFT, fill=BOTH, expand=TRUE)
        recordings_vscrollbar.config(command=self.recordings_canvas.yview)
        self.recordings_interior = tk.Frame(self.recordings_canvas)
        self.recordings_interior_id = self.recordings_canvas.create_window(0, 0, window=self.recordings_interior, anchor=NW)
        self.recordings_interior.bind("<Configure>", 
            lambda event, canvas=self.recordings_canvas, interior_frame=self.recordings_interior: 
            self._configure_interior(event, canvas, interior_frame)
        )
        self.recordings_canvas.bind("<Configure>", 
            lambda event, canvas=self.recordings_canvas, interior_frame=self.recordings_interior, interior_id=self.recordings_interior_id: 
            self._configure_canvas(event, canvas, interior_frame, interior_id)
        )
        self.recordings_canvas.bind_all("<Button-4>", 
            lambda event, scroll=-1, canvas=self.recordings_canvas: 
            self._on_mousehwheel(event, canvas, scroll)
        )
        self.recordings_canvas.bind_all("<Button-5>", 
            lambda event, scroll=1, canvas=self.recordings_canvas: 
            self._on_mousehwheel(event, canvas, scroll)
        )
        self.cbtn_recordings_var = []
        self.cbtn_recordings = [] 
        self.dataset_filename = tk.StringVar(value="")
        self.tab_4.Text("File Name:", row=3, col=0)
        self.tab_4.Entry(self.dataset_filename, row=3, col=1)
        self.btn_gen_demos = self.tab_4.AccentButton("Generate Dataset", self.generate_demonstration_dataset, row=4, col=0, colspan=2)

        # self.learning_datasets_frame = self.tab_4.addLabelFrame("Demonstration Datasets", row=1, col=3, colspan=2)
        # self.learning_datasets_interior = self.ScrollableFrame(self.learning_datasets_frame.master)
        # self.btn_gen_demos = self.tab_4.AccentButton("Generate Random Demonstrations", self.update_learning_datasets, row=4, col=0, colspan=2)


        self.notebook.notebook.bind("<<NotebookTabChanged>>", self.tab_changed)
        
        self.run()

    def ScrollableFrame(self, master, height= 300):
        vscrollbar = ttk.Scrollbar(master, orient=VERTICAL)
        vscrollbar.pack(fill=Y, side=RIGHT, expand=FALSE)
        canvas = tk.Canvas(master, height=height, highlightthickness=0, bd=0, yscrollcommand=vscrollbar.set)
        canvas.pack(side=LEFT, fill=BOTH, expand=TRUE)
        vscrollbar.config(command=canvas.yview)
        interior = tk.Frame(canvas)
        interior_id = canvas.create_window(0, 0, window=interior, anchor=NW)

        interior.bind("<Configure>", 
            lambda event, canvas=canvas, interior_frame=interior: 
            self._configure_interior(event, canvas, interior_frame)
        )
        canvas.bind("<Configure>", 
            lambda event, canvas=canvas, interior_frame=interior, interior_id=interior_id: 
            self._configure_canvas(event, canvas, interior_frame, interior_id)
        )
        canvas.bind_all("<Button-4>", 
            lambda event, scroll=-1, canvas=canvas: 
            self._on_mousehwheel(event, canvas, scroll)
        )
        canvas.bind_all("<Button-5>", 
            lambda event, scroll=1, canvas=canvas: 
            self._on_mousehwheel(event, canvas, scroll)
        )
        
        return interior
    

    def _on_mousehwheel(self, event, canvas, scroll):
        canvas.yview_scroll(scroll, "units")

    def _configure_interior(self, event, canvas, interior_frame):
        size = (interior_frame.winfo_reqwidth(), interior_frame.winfo_reqheight())
        canvas.config(scrollregion="0 0 %s %s" % size)
        if interior_frame.winfo_reqwidth() != canvas.winfo_width():
            canvas.config(width=interior_frame.winfo_reqwidth())
        self.recordings_canvas.update_idletasks()

    def _configure_canvas(self, event, canvas, interior_frame, interior_id):
        if interior_frame.winfo_reqwidth() != canvas.winfo_width():
            canvas.itemconfigure(interior_id, width=canvas.winfo_width())
        self.recordings_canvas.update_idletasks()
        
    



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
        padx_value = 5
        pady_value = 2
        self.cbtn_replay_var = []
        self.cbtn_replay_var.append(tk.BooleanVar(value=True))
        # self.cbtn_replay_var.append(tk.BooleanVar(value=False))
        self.cbtn_replay = []
        # self.cbtn_replay.append(self.replay_files_interior.Checkbutton("Original Recording", self.cbtn_replay_var[0], command=self.draw_graph))
        c_btn = ttk.Checkbutton(self.replay_files_interior, text="Original Recording", variable=self.cbtn_replay_var[0], command=self.draw_graph)
        c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
        # self.replay_files_frame.widgets.widgetlist.append(button)
        self.cbtn_replay.append(c_btn)
        # self.cbtn_replay.append(self.replay_files_frame.Checkbutton("Steps", self.cbtn_replay_var[1], command=self.draw_graph))
        # c_btn = ttk.Checkbutton(self.replay_files_interior, text="Steps", variable=self.cbtn_replay_var[1], command=self.draw_graph)
        # c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
        # # self.replay_files_frame.widgets.widgetlist.append(button)
        # # button.grid(row=1, column=0)
        # self.cbtn_replay.append(c_btn)

        replay_bags = []

        for root, dirs, files in os.walk(self.path_demo_dir):
            for dir in dirs:
                if re.search(bagname, dir) and re.search('replay', dir):
                    # print(f"Found: {dir}")
                    replay_bags.append(dir)
                    self.cbtn_replay_var.append(tk.BooleanVar(value=False))
                    # self.cbtn_replay.append(self.replay_files_frame.Checkbutton(f"Replay {dir[-15:]}", self.cbtn_replay_var[-1], command=self.draw_graph))
                    name, date, time = self.filename_to_name_date_time(dir[-20:])
                    # print(f"Name: {name}, Date: {date}, Time: {time}")
                    c_btn = ttk.Checkbutton(self.replay_files_interior, text=f"Replay - {date} {time}", variable=self.cbtn_replay_var[-1], command=self.draw_graph)
                    c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
                    self.cbtn_replay.append(c_btn)
                elif re.search(bagname, dir) and re.search('gym_step', dir):
                    # print(f"Found: {dir}")
                    replay_bags.append(dir)
                    self.cbtn_replay_var.append(tk.BooleanVar(value=False))
                    # self.cbtn_replay.append(self.replay_files_frame.Checkbutton(f"Gym Step Replay {dir[-15:]}", self.cbtn_replay_var[-1], command=self.draw_graph))
                    name, date, time = self.filename_to_name_date_time(dir[-20:])
                    c_btn = ttk.Checkbutton(self.replay_files_interior, text=f"Gym Step Replay - {date} {time}", variable=self.cbtn_replay_var[-1], command=self.draw_graph)
                    c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
                    self.cbtn_replay.append(c_btn)
                elif re.search(bagname, dir) and re.search('step', dir):
                    # print(f"Found: {dir}")
                    replay_bags.append(dir)
                    self.cbtn_replay_var.append(tk.BooleanVar(value=False))
                    # self.cbtn_replay.append(self.replay_files_frame.Checkbutton(f"Step Replay {dir[-15:]}", self.cbtn_replay_var[-1], command=self.draw_graph))
                    name, date, time = self.filename_to_name_date_time(dir[-20:])
                    c_btn = ttk.Checkbutton(self.replay_files_interior, text=f"Step Replay - {date} {time}", variable=self.cbtn_replay_var[-1], command=self.draw_graph)
                    c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
                    self.cbtn_replay.append(c_btn)

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
            
        if tab_text == "Generate Demonstration Dataset":
            self.file_tree = self.parse_file_tree()
            self.update_recordings()

    def cancel_gen_demos(self):
        if self.gen_random_paths_process.poll() is None:
            os.killpg(os.getpgid(self.gen_random_paths_process.pid), signal.SIGINT)
            self.gen_random_paths_process.wait()

        self.btn_cancel_gen_demos.destroy()
            
    def generate_demonstration_dataset(self):
        save_file_name = self.dataset_filename.get()
        if not self.check_file_name(save_file_name):
            return
        
        if self.demo_env.get() == "No Environment":
            env = "no_env"
        elif self.demo_env.get() == "Door":
            env = "door"
        
        paths = []
        max_current = np.zeros(13)
        for i, cbtn_var in enumerate(self.cbtn_recordings_var):
            if cbtn_var.get():
                split_string = self.cbtn_recordings[i]['text'].split(' ')
                name = split_string[0]
                date = split_string[2]
                time = split_string[3]
                rosbag_name = self.name_date_time_to_filename(name, date, time)
                
                rosbag_uri = f'{self.path_demo_dir}{env}/{rosbag_name}'
                new_bag = ROSBagUtil(rosbag_uri)
                for j in range(13):
                    # print(f"Motor Current Shape: {nmotor_current.shape}")
                    motor_current = [obj.data for obj in new_bag.motor_currents[f"motor_{j+1}"]]
                    new_max_current = np.max(np.abs(motor_current))
                    if new_max_current > max_current[j]:
                        max_current[j] = new_max_current
                new_paths = new_bag.generate_demonstration_set()
                paths.append(new_paths)

        max_current_reduced = np.zeros(7)
        for i in range(7):
            if i*2 == 10:
                max_current_reduced[i] = max_current[10]
            elif i*2 == 12:
                max_current_reduced[i] = np.max([max_current[11], max_current[12]])
            else:
                max_current_reduced[i] = np.max([max_current[i*2], max_current[i*2+1]])
        print(f"Max currents: {max_current}")
        print(f"Max currents reduced: {max_current_reduced}")

        paths = np.array(paths, dtype=object)
        print(f"Paths Shape: {paths.shape}")
        path_lengths = np.array([])
        max_action = np.zeros(7)
        min_action = np.ones(7)*1000
        max_joint_pos = np.zeros(7)
        min_joint_pos = np.ones(7)*1000
        # max_current = np.zeros(7)
        for i, path in enumerate(paths):
            path_lengths = np.append(path_lengths, path['actions'].shape[0])
            new_max_action = np.amax(path['actions'], axis=0)
            new_min_action = np.amin(path['actions'], axis=0)
            new_max_joint_pos = np.amax(path['observations'][:, 7:14], axis=0)
            new_min_joint_pos = np.amin(path['observations'][:, 7:14], axis=0)
            for j in range(7):
                if new_max_action[j] > max_action[j]:
                    max_action[j] = new_max_action[j]
                if new_min_action[j] < min_action[j]:
                    min_action[j] = new_min_action[j]
            for j in range(7):
                if new_max_joint_pos[j] > max_joint_pos[j]:
                    max_joint_pos[j] = new_max_joint_pos[j]
                if new_min_joint_pos[j] < min_joint_pos[j]:
                    min_joint_pos[j] = new_min_joint_pos[j]
            
            print(f"Subpath {i} Shape: {path['actions'].shape[0]}")
        
        print(f"Max: {max_action}")
        print(f"Min: {min_action}")
        print(f"Max Joint Pos: {max_joint_pos}")
        print(f"Min Joint Pos: {min_joint_pos}")
        max_joint_pos = max_joint_pos + 0.1
        min_joint_pos = min_joint_pos - 0.1
        max_action = max_action + 0.1
        min_action = min_action - 0.1
        episode_length = int(np.max(path_lengths))
        num_steps = int(np.sum(path_lengths))

        print(f"Episode Length: {episode_length}")
        print(f"Num Steps: {num_steps}")

        # num_steps = 938
        # episode_length = 64
        # save_file_name = "reach_handle_v4"
        # max_action = np.array([0.73132122, 0.63249396, 1.04867809, 0.82074198, 1.23940366, 0.73069696, 0.89224023])
        # min_action = np.array([-0.6797611, -0.46385764, -0.64722833,-0.40239734, -0.53581053, -0.49964198, -0.83811197])
        # max_joint_pos = np.array([0.24625256, 0.0, 0.34377382, 1.6515779, 1.93695643, 0.08437629, 0.22024793])
        # min_joint_pos = np.array([-0.11346592, -0.41125246, 0.0, 0.0, 0.0, -0.67034226, -0.74194147])
        # max_current_reduced = np.array([638.4, 739.2, 675.36, 705.6, 534.24, 151.2, 544.32])
        
        # self.btn_cancel_gen_demos = self.tab_4.AccentButton("Cancel", self.cancel_gen_demos, row=5, col=0, colspan=2)

        # pre_command = f'source {self.path_venv}/bin/activate; source {self.path_ros2_ws}/install/setup.bash;'
        # arguments = f'--num_steps {num_steps} --episode_length {episode_length} --save_file_name {save_file_name} --input_max "{max_action.tolist()}" --input_min "{min_action.tolist()}" --joint_max_pos "{max_joint_pos.tolist()}" --joint_min_pos "{min_joint_pos.tolist()}" --max_current "{max_current_reduced.tolist()}"'
        # process_command = f"{pre_command} python {self.path_ros2_ws}/src/gh360/gh360_demonstration/gh360_demonstration/gen_random_paths.py {arguments}"
        # self.gen_random_paths_process = subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)

        # self.gen_random_paths_process.wait()

        # self.btn_cancel_gen_demos.destroy()

        # save_file_path = os.path.join(self.path_learning_data_dir, env, save_file_name)
        # np.save(save_file_path, paths)
                

    def update_recordings(self, *args):
        # print("Update Recordings")
        padx_value = 10
        pady_value = 2

        for cbtn in self.cbtn_recordings:
                cbtn.destroy()

        self.cbtn_recordings_var = []
        self.cbtn_recordings = []

        # self.cbtn_recordings_var.append(tk.BooleanVar(value=True))
        # c_btn = ttk.Checkbutton(self.recordings_interior, text="Original Recording asdf asdf asdf sadf asdf asd asd", variable=self.cbtn_recordings_var[0])
        # c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
        # self.cbtn_recordings.append(c_btn)

        # self.cbtn_recordings_var.append(tk.BooleanVar(value=True))
        # c_btn = ttk.Checkbutton(self.recordings_interior, text="Original Recording", variable=self.cbtn_recordings_var[0])
        # c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
        # self.cbtn_recordings.append(c_btn)

        if self.demo_env.get() == "No Environment":
            env_name = "no_env"
        elif self.demo_env.get() == "Door":
            env_name = "door"

        for env in self.file_tree:
            if env["name"] == env_name:
                for file in env["files"]:
                    self.cbtn_recordings_var.append(tk.BooleanVar(value=False))
                    name = file["name"]
                    date = file["date"]
                    time = file["time"]
                    c_btn = ttk.Checkbutton(self.recordings_interior, text=f"{name} - {date} {time}", variable=self.cbtn_recordings_var[-1],)
                    c_btn.pack(padx=padx_value, pady=pady_value, anchor="w")
                    self.cbtn_recordings.append(c_btn)
            # for file in env["files"]:
            #     tv.insert(parent_cntr, "end", cntr, text=file["name"], values=(file["date"], file["time"]))
            #     cntr += 1

    def update_learning_datasets(self):
        pass

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
        original_bag = ROSBagUtil(rosbag_uri)
        # original_bag_data = 
        # original_bag_data = self.read_bag(rosbag_uri)
        # # self.bag_data.append(original_bag_data)
        # filtered_og_bag_data = self.filter_bag_data(copy.deepcopy(original_bag_data))        
        self.bag_data.append(original_bag)
        # self.bag_data.append(self.generate_step_data(original_bag_data))

        # print(f"Goal Velocitiy length: {len(self.bag_data[0]['motor_goal_velocities']['motor_1'])}")
        # print(f"Shoulder Velocity time length: {len(self.bag_data[0]['time']['shoulder_motor_goal_velocities'])}")
        # print(f"Upperarm Velocity time length: {len(self.bag_data[0]['time']['upperarm_motor_goal_velocities'])}")
        # print(f"Lowerarm Velocity time length: {len(self.bag_data[0]['time']['lowerarm_motor_goal_velocities'])}")
        # self.pre_process_data(self.bag_data[0])

        replay_bags = self.find_replay_files(rosbag_name)
        for bag_name in replay_bags:
            rosbag_uri = f'{self.path_demo_dir}{env}/{bag_name}'
            # new_bag_data = self.read_bag(rosbag_uri)
            new_bag = ROSBagUtil(rosbag_uri)
            if re.search('gym_step', bag_name):
                # self.bag_data.append(self.filter_gym_bag_data(new_bag_data))
                # self.bag_data.append(self.filter_bag_data(new_bag_data))
                self.bag_data.append(new_bag)
            else:
                # self.bag_data.append(self.filter_bag_data(new_bag_data))
                self.bag_data.append(new_bag)

        self.data_read = True

        self.draw_graph()
        
    def draw_graph(self):
        self.ax.clear()
        for z in range(len(self.cbtn_replay_var)):
            if self.cbtn_replay_var[z].get():
                bag_data = self.bag_data[z]
                bag_selected_data = []
                data = []
                t = []
                if self.vis_data_var.get() == "Motor Position":
                    # print(f"Motor {self.vis_data_var_2.get()} Positions")
                    bag_selected_data = bag_data.motor_positions[f"motor_{self.vis_data_var_2.get()}"]
                elif self.vis_data_var.get() == "Motor Velocity":
                    # print(f"Motor {self.vis_data_var_2.get()} Velocities")
                    bag_selected_data = bag_data.motor_velocities[f"motor_{self.vis_data_var_2.get()}"]
                elif self.vis_data_var.get() == "Motor Goal Velocity":
                    # print(f"Motor {self.vis_data_var_2.get()} Goal Velocities")
                    bag_selected_data = bag_data.motor_goal_velocities[f"motor_{self.vis_data_var_2.get()}"]
                elif "Joint" in self.vis_data_var.get():
                    joint_name = self.vis_data_var_2.get().upper().replace(" ", "")
                    joint_id = JointNames[joint_name].value
                    if self.vis_data_var.get() == "Joint Position":
                        # print(f"Joint {joint_name} Positions")
                        bag_selected_data = bag_data.joint_positions[f"joint_{joint_id}"]
                    elif self.vis_data_var.get() == "Joint Velocity":
                        # print(f"Joint {joint_name} Velocities")
                        bag_selected_data = bag_data.joint_velocities[f"joint_{joint_id}"]

                for msg in bag_selected_data:
                    data.append(msg.data)
                    t.append(msg.time - bag_selected_data[0].time)
                
                self.ax.plot(t, data, label=f"Replay {z}")

        self.canvas.draw()

    def vis_data_changed(self, *args):
        if "Motor" in self.vis_data_var.get():
            # print("Motor Selected")
            self.vis_data_options_2 = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13"]
        else:
            self.vis_data_options_2 = ["Shoulder Yaw", "Shoulder Roll", "Shoulder Pitch", "Upperarm Roll", "Elbow", "Forearm Roll", "Wrist Pitch"]

        self.option_submenu.set_menu(self.vis_data_options_2[0], *self.vis_data_options_2)

        if self.data_read == False:
            return
        # print("vis_data_changed")
        
        self.draw_graph()

    def vis_data_2_changed(self, *args):
        if self.data_read == False:
            return
        # print("vis_data_changed")
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
        
        process_command = f'{pre_command} ros2 bag record -o {self.path_demo_dir}{env}/{rosbag_name}'
        process_command += ' -a'
        record_process = subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)

        return record_process
    
    def check_file_name(self, filename):
        invalid_chars = r'[#%&{}$!+=`<>:"/\\|?*\0\s]'
    
        # If the filename contains any invalid characters, return False
        if re.search(invalid_chars, filename):
            self.get_logger().error("Please enter a valid file name")
            return False
        if filename == "":
            self.get_logger().error("Please enter a file name")
            return False
        
        return True

    def start_record(self):
        # invalid_chars = r'[#%&{}$!+=`<>:"/\\|?*\0\s]'
    
        # # If the filename contains any invalid characters, return False
        # if re.search(invalid_chars, self.record_filename.get()):
        #     self.get_logger().error("Please enter a valid file name")
        #     return 
        # if self.record_filename.get() == "":
        #     self.get_logger().error("Please enter a file name")
        #     return

        if not self.check_file_name(self.record_filename.get()):
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
        name = filename[:-len(datetime)-1]
        date = f"{datetime[6:8]}/{datetime[4:6]}/{datetime[:4]}"
        time = f"{datetime[9:11]}:{datetime[11:13]}:{datetime[13:]}"

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