import rclpy
from rclpy.node import Node
import argparse
import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from gh360_interfaces.msg import SetMotorPositions, SetMotorVelocities, PortStatus, SetPosition, SetVelocity
from sensor_msgs.msg import JointState

class PositionStepPublisher(Node):

    def __init__(self):
        super().__init__('position_step_publisher')

        self.declare_parameter('bag_file_path','')
        bag_file_path = self.get_parameter('bag_file_path').get_parameter_value().string_value

        self.pos_steps, self.vel_steps, self.vel_goal_steps = self.parse_bag_file(bag_file_path)

        self.pos_msg = SetMotorPositions()
        self.vel_msg = SetMotorVelocities()
        for i in range(1, 14):
            set_pos = SetPosition()
            set_pos.id = i
            set_pos.position = self.pos_steps[0][i-1]
            self.pos_msg.motor_goal_positions.append(set_pos)

            set_vel = SetVelocity()
            set_vel.id = i
            set_vel.velocity = self.vel_steps[0][i-1]
            self.vel_msg.motor_goal_velocities.append(set_vel)

        self.cntr = 0

        self.shoulder_motor_pos_pub = self.create_publisher(SetMotorPositions, '/shoulder/motor_goal_position', 10)
        self.upperarm_motor_pos_pub = self.create_publisher(SetMotorPositions, '/upperarm/motor_goal_position', 10)
        self.lowerarm_motor_pos_pub = self.create_publisher(SetMotorPositions, '/lowerarm/motor_goal_position', 10)

        self.shoulder_motor_vel_pub = self.create_publisher(SetMotorVelocities, '/shoulder/motor_goal_velocity', 10)
        self.upperarm_motor_vel_pub = self.create_publisher(SetMotorVelocities, '/upperarm/motor_goal_velocity', 10)
        self.lowerarm_motor_vel_pub = self.create_publisher(SetMotorVelocities, '/lowerarm/motor_goal_velocity', 10)

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]

        self.reseted = False
        self.reset_cntr = 0

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
                    break

            for t in reversed(time_list):
                z = time_list.index(t)
                if goal_velocities[f"motor_{i}"][z] != 0.0 and t > end_move_time:
                    end_move_time = t
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

            
        return bag_data
    
    def read_bag(self, rosbag_uri):
        rosbag_reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=rosbag_uri,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        rosbag_reader.open(storage_options, converter_options)

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
        while rosbag_reader.has_next():
            topic, msg, t = rosbag_reader.read_next()

            if topic.endswith("motor_status"):
                msg_dec = deserialize_message(msg, PortStatus)
                for motor in msg_dec.motors:
                    if motor.motor_id < 14:
                        bag_data["motor_positions"][f"motor_{motor.motor_id}"].append(motor.present_position)
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

        return bag_data

    def parse_bag_file(self, bag_file_path):
        # rosbag_reader = rosbag2_py.SequentialReader()
        # storage_options = rosbag2_py._storage.StorageOptions(
        #         uri=bag_file_path,
        #         storage_id='sqlite3')
        # converter_options = rosbag2_py._storage.ConverterOptions('', '')
        # rosbag_reader.open(storage_options, converter_options)

        og_bag_data = self.filter_bag_data(self.read_bag(bag_file_path))
        bag_data = {}
        pos_steps = []
        vel_steps = []
        vel_goal_steps = []
        for i in range(1, 14):
            bag_data[f"motor_{i}_position"] = []
            bag_data[f"motor_{i}_time"] = []
            bag_data[f"motor_{i}_goal_velocity"] = []
            bag_data[f"motor_{i}_time_goal_velocity"] = []
            # pos_steps[f"motor_{i}_pos_step"] = []

            if i < 7:
                time_list = og_bag_data["time"]["shoulder_motors"]
                time_list_goal_vel = og_bag_data["time"]["shoulder_motor_goal_velocities"]
            elif i < 11:
                time_list = og_bag_data["time"]["upperarm_motors"]
                time_list_goal_vel = og_bag_data["time"]["upperarm_motor_goal_velocities"]
            else:
                time_list = og_bag_data["time"]["lowerarm_motors"]
                time_list_goal_vel = og_bag_data["time"]["lowerarm_motor_goal_velocities"]

            for z, t in enumerate(time_list):
                bag_data[f"motor_{i}_position"].append(og_bag_data["motor_positions"][f"motor_{i}"][z])
                bag_data[f"motor_{i}_time"].append(t)

            for z, t in enumerate(time_list_goal_vel):
                bag_data[f"motor_{i}_goal_velocity"].append(og_bag_data["motor_goal_velocities"][f"motor_{i}"][z])
                bag_data[f"motor_{i}_time_goal_velocity"].append(t)
        # for i in range(1, 14):
        #     bag_data[f"motor_{i}_position"] = []
        #     bag_data[f"motor_{i}_time"] = []
        #     # pos_steps[f"motor_{i}_pos_step"] = []
        
        # while rosbag_reader.has_next():
        #     topic, msg, t = rosbag_reader.read_next()
        #     if topic.endswith("motor_status"):
        #         msg_dec = deserialize_message(msg, PortStatus)
        #         for motor in msg_dec.motors:
        #             if motor.motor_id < 14:
        #                 bag_data[f"motor_{motor.motor_id}_position"].append(motor.present_position)
        #                 bag_data[f"motor_{motor.motor_id}_time"].append(t)

        print(f"Len time data: {len(bag_data[f'motor_1_time_goal_velocity'])}")
        print(f"Len vel data: {len(bag_data[f'motor_1_goal_velocity'])}")
        for i in range(1, 14):
            step_cntr = 0
            motor_pos_steps = []
            motor_vel_steps = []
            t_init = bag_data[f"motor_{i}_time"][0]
            
            for z, t in enumerate(bag_data[f"motor_{i}_time"]):
                t = t - t_init

                if t >= step_cntr*200e6:
                    # pos_steps[f"motor_{i}_pos_step"].append(bag_data[f"motor_{i}_position"][z])
                    motor_pos_steps.append(bag_data[f"motor_{i}_position"][z])
                    # if step_cntr > 0:
                    #     motor_vel_steps.append((bag_data[f"motor_{i}_position"][z] - bag_data[f"motor_{i}_position"][z-1])/0.2)
                    # else:
                    #     motor_vel_steps.append(0.0)
                    if step_cntr == 0:
                        prev_pos = bag_data[f"motor_{i}_position"][0]
                    else:
                        # vel_steps[f"motor_{i}_vel_step"].append((bag_data[f"motor_{i}_position"][z] - prev_pos)/0.05)
                        pos_diff = (bag_data[f"motor_{i}_position"][z] - prev_pos)
                        motor_vel_steps.append(pos_diff/0.2)
                        # motor_steps.append((pos_diff/2))
                        # motor_steps.append((pos_diff/2))
                        prev_pos = bag_data[f"motor_{i}_position"][z]

                    step_cntr += 1
                
            motor_vel_steps.append(0.0)
            motor_vel_steps.append(0.0)
            motor_vel_steps.append(0.0)
            pos_steps.append(motor_pos_steps)
            vel_steps.append(motor_vel_steps)
            
            t_init = bag_data[f"motor_{i}_time_goal_velocity"][0]
            motor_vel_goal_steps = []
            vel_sum = 0.0
            vel_cntr = 0
            step_cntr = 0
            for z,t in enumerate(bag_data[f"motor_{i}_time_goal_velocity"]):
                t = t - t_init
                vel_sum += bag_data[f"motor_{i}_goal_velocity"][z]
                vel_cntr += 1
                if t >= step_cntr*200e6:
                    motor_vel_goal_steps.append(vel_sum/vel_cntr)
                    vel_sum = 0.0
                    vel_cntr = 0
                    # motor_pos_steps.append(bag_data[f"motor_{i}_position"][z])
                    # motor_vel_steps.append(bag_data[f"motor_{i}_goal_velocity"][z])
                    step_cntr += 1

            motor_vel_goal_steps.append(0.0)
            motor_vel_goal_steps.append(0.0)
            motor_vel_goal_steps.append(0.0)
            vel_goal_steps.append(motor_vel_goal_steps)

        print(f"len original data: {len(bag_data[f'motor_1_position'])}")

        # Determine the minimum length of the sublists
        min_length = min(len(sublist) for sublist in pos_steps)
        # Truncate each sublist to the minimum length
        pos_steps = [sublist[:min_length] for sublist in pos_steps]

        min_length = min(len(sublist) for sublist in vel_steps)
        # Truncate each sublist to the minimum length
        vel_steps = [sublist[:min_length] for sublist in vel_steps]

        min_length = min(len(sublist) for sublist in vel_goal_steps)
        # Truncate each sublist to the minimum length
        vel_goal_steps = [sublist[:min_length] for sublist in vel_goal_steps]

        # for i in range(1, 14):
        #     print(f"Motor {i} last vel: {vel_steps[i-1][-1]}")
        # print(f"pos_steps: {pos_steps}")
        np_pos_steps = np.array(pos_steps)
        np_vel_steps = np.array(vel_steps)
        np_vel_goal_steps = np.array(vel_goal_steps)
        # print(f"len step data: {len(pos_steps[0])}")
        # print(f"np shape: {np_pos_steps.shape}")
        np_pos_steps = np.transpose(np_pos_steps)
        np_vel_steps = np.transpose(np_vel_steps)
        np_vel_goal_steps = np.transpose(np_vel_goal_steps)
        # print(f"np shape: {np_pos_steps.shape}")
        # print(f"len action: {len(np_pos_steps[0])}")
        print(f"pos_steps shape: {np_pos_steps.shape}")
        print(f"vel_steps shape: {np_vel_steps.shape}")
        print(f"vel_goal_steps shape: {np_vel_goal_steps.shape}")

        return np_pos_steps, np_vel_steps, np_vel_goal_steps
    

    def timer_callback(self):
        if not self.reseted:
            self.pos_msg = SetMotorPositions()
            for i in range(1, 14):
                set_pos = SetPosition()
                set_pos.id = i
                set_pos.position = self.robot_reset_pos[i-1]
                self.pos_msg.motor_goal_positions.append(set_pos)

            self.shoulder_motor_pos_pub.publish(self.pos_msg)
            self.upperarm_motor_pos_pub.publish(self.pos_msg)
            self.lowerarm_motor_pos_pub.publish(self.pos_msg)

            self.reset_cntr += 1

            if self.reset_cntr >= 30:
                self.reseted = True
            return
        # self.pos_msg = SetMotorPositions()
        self.vel_msg = SetMotorVelocities()
        for i in range(1, 14):
            # set_pos = SetPosition()
            # set_pos.id = i
            # set_pos.position = self.pos_steps[self.cntr][i-1]
            # self.pos_msg.motor_goal_positions.append(set_pos)

            set_vel = SetVelocity()
            set_vel.id = i
            # set_vel.velocity = self.vel_steps[self.cntr][i-1]
            set_vel.velocity = self.vel_goal_steps[self.cntr][i-1]
            self.vel_msg.motor_goal_velocities.append(set_vel)

        # print("Goal Velocity: ", self.vel_steps[self.cntr])

        if self.cntr < len(self.vel_steps)-1:
            self.cntr += 1
        
        # self.shoulder_motor_pos_pub.publish(self.msg)
        # self.upperarm_motor_pos_pub.publish(self.msg)
        # self.lowerarm_motor_pos_pub.publish(self.msg)

        self.shoulder_motor_vel_pub.publish(self.vel_msg)
        self.upperarm_motor_vel_pub.publish(self.vel_msg)
        self.lowerarm_motor_vel_pub.publish(self.vel_msg)

        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    pos_step_pub = PositionStepPublisher()

    rclpy.spin(pos_step_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pos_step_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()