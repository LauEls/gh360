# import TKinterModernThemes as TKMT
# import tkinter as tk
from random import choice

import customtkinter as ctk
import rclpy
from rclpy.node import Node
import json

from gh360_interfaces.srv import LogTime

class LeaderboardEntry:
    def __init__(self, app, row):
        self.position_label = ctk.CTkLabel(app, text=f"", font=ctk.CTkFont(size=24))
        self.position_label.grid(row=row, column=0, padx=20, pady=10)
        self.username_label = ctk.CTkLabel(app, text=f"", font=ctk.CTkFont(size=24))
        self.username_label.grid(row=row, column=1, padx=20, pady=10)
        self.time_label = ctk.CTkLabel(app, text=f"", font=ctk.CTkFont(size=24))
        self.time_label.grid(row=row, column=2, padx=20, pady=10)

class ERFLeaderboard(Node):
    def __init__(self):
        super().__init__('erf_leaderboard')

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk()
        self.app.title("ERF Leaderboard")
        # self.app.geometry("1920x1080")

        # Load leaderboard data from JSON file if it exists
        self.file_path = "/home/gh360/ros2_gh360_ws/src/gh360/gh360_examples/gh360_examples/leaderboard_data.json"
        try:
            with open(self.file_path, "r") as json_file:
                self.leaderboard_data = json.load(json_file)
        except FileNotFoundError:
            self.get_logger().info("Leaderboard data file not found. Using default data.")
            self.leaderboard_data = {}
        except json.JSONDecodeError:
            self.get_logger().error("Error decoding leaderboard data file. Using default data.")

        
        self.leaderboard_data = dict(sorted(self.leaderboard_data.items(), key=lambda item: item[1]['time']))

        

        leaderboard_title = ctk.CTkLabel(self.app, text="ERF Leaderboard", font=ctk.CTkFont(size=48, weight="bold"))
        leaderboard_title.grid(row=1, column=0, columnspan=3, padx=20, pady=20)

        column_labels = ["Position", "User", "Time"]
        for i, col_label in enumerate(column_labels):
            label = ctk.CTkLabel(self.app, text=col_label, font=ctk.CTkFont(size=24, weight="bold"))
            label.grid(row=2, column=i, padx=20, pady=10)

        self.leaderboard_labels = []
        for i in range(10):
            self.leaderboard_labels.append(LeaderboardEntry(self.app, row=3+i))

        for user in self.leaderboard_data:
            self.leaderboard_labels[list(self.leaderboard_data.keys()).index(user)].position_label.configure(text=f"{list(self.leaderboard_data.keys()).index(user)+1}.")
            self.leaderboard_labels[list(self.leaderboard_data.keys()).index(user)].username_label.configure(text=f"{user}")
            self.leaderboard_labels[list(self.leaderboard_data.keys()).index(user)].time_label.configure(text=f"{self.leaderboard_data[user]['time']}s")

        self.log_time_service = self.create_service(LogTime, 'erf_log_time', self.log_time_callback)

        self.timer = self.create_timer(0.1, self.update_leaderboard)
        self.app.update()

    def update_leaderboard(self):
        self.app.update()

    def create_user_window(self):
        dialog = ctk.CTkInputDialog(text="New User Name:", title="Create User", font=ctk.CTkFont(size=20))
        return dialog.get_input()

    def log_time_callback(self, request, response):
        self.get_logger().info(f"Received log time request from user: {request.username.data} with time: {request.time}s")
        user_name = request.username.data
        time = round(request.time, 4)

        if len(self.leaderboard_data) < 10 or time < max(entry['time'] for entry in self.leaderboard_data.values()) or user_name in self.leaderboard_data:
            if user_name == "":
                user_name = self.create_user_window()

            if user_name in self.leaderboard_data:
                if time < self.leaderboard_data[user_name]['time']:
                    self.leaderboard_data[user_name]['time'] = time
            elif len(self.leaderboard_data) == 10:
                if time < max(entry['time'] for entry in self.leaderboard_data.values()):
                    worst_user = max(self.leaderboard_data, key=lambda user: self.leaderboard_data[user]['time'])
                    del self.leaderboard_data[worst_user]
                    self.leaderboard_data[user_name] = {"time": time}
            else:
                self.leaderboard_data[user_name] = {"time": time}

            response.topten = True
        else:
            response.topten = False
            
        # Sort the leaderboard data by time
        self.leaderboard_data = dict(sorted(self.leaderboard_data.items(), key=lambda item: item[1]['time']))

        # Update the leaderboard display
        for user in self.leaderboard_data:
            self.leaderboard_labels[list(self.leaderboard_data.keys()).index(user)].position_label.configure(text=f"{list(self.leaderboard_data.keys()).index(user)+1}.")
            self.leaderboard_labels[list(self.leaderboard_data.keys()).index(user)].username_label.configure(text=f"{user}")
            self.leaderboard_labels[list(self.leaderboard_data.keys()).index(user)].time_label.configure(text=f"{self.leaderboard_data[user]['time']}s")

        self.save_leaderboard_to_json()

        return response

    def save_leaderboard_to_json(self):
        with open(self.file_path, "w") as json_file:
            json.dump(self.leaderboard_data, json_file, indent=4)

def main(args=None):
    rclpy.init(args=args)

    erf_leaderboard = ERFLeaderboard()

    rclpy.spin(erf_leaderboard)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    erf_leaderboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()