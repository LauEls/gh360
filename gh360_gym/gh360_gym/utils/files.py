def safe_to_file(self):
        arm_status = np.concatenate((self.shoulder_motor_states_msg.motor_status, self.upperarm_motor_states_msg.motor_status, self.lowerarm_motor_states_msg.motor_status), axis=None)
        for motor_state in arm_status:
            for joint in self.arm:
                if type(joint) == SoftJoint:
                    if motor_state.motor_id == joint.id_right_motor:
                        joint.right_motor_pos = motor_state.present_position
                    elif motor_state.motor_id == joint.id_left_motor:
                        joint.left_motor_pos = motor_state.present_position
                else:
                    if motor_state.motor_id == joint.id_motor:
                        joint.joint_angle = motor_state.present_position


        timestamp = time.time()

        joint_pos = []
        motor_pos = []

        for joint in self.arm:
            joint_pos.append(joint.joint_angle)
            if type(joint) == SoftJoint:
                motor_pos.append(joint.right_motor_pos)
                motor_pos.append(joint.left_motor_pos)
            else:
                motor_pos.append(joint.joint_angle)



        joint_pos_write = np.concatenate((timestamp, joint_pos), axis=None)
        motor_pos_write = np.concatenate((timestamp, motor_pos), axis=None)
        # robot_state = [timestamp, self.joint_pos, self.ee_pos]
        # robot_state = np.insert(robot_state, 0, timestamp)

        f = open(self.joint_pos_file, 'a')
        data_writer = csv.writer(f)
        data_writer.writerow(joint_pos_write)
        f.close()

        f = open(self.motor_pos_file, 'a')
        data_writer = csv.writer(f)
        data_writer.writerow(motor_pos_write)
        f.close()