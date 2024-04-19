import os
import threading
import io
import subprocess
import signal

from PIL import Image, ImageDraw
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.Devices.StreamDeck import DialEventType, TouchscreenEventType


class StreamDeckGH360Control:
    def __init__(self):
        self.asset_path = '/home/laurenz/python-elgato-streamdeck/src/Assets'
        self.disable_deck = False

        
       
        self.colcon_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.encoder_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.bridge_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.gh360_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.monitor_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.gym_example_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.shoulder_move_home_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.upperarm_move_home_process = subprocess.Popen('', shell=True, executable="/bin/bash")
        self.lowerarm_move_home_process = subprocess.Popen('', shell=True, executable="/bin/bash")

        self.dial_0_state = 0
        self.dial_1_state = 0

        # image for idle state
        img = Image.new('RGB', (120, 120), color='black')
        self.released_icon = Image.open(os.path.join(self.asset_path, 'Released.png')).resize((80, 80))
        img.paste(self.released_icon, (20, 20), self.released_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_released_bytes = img_byte_arr.getvalue()

        # image for pressed state
        img = Image.new('RGB', (120, 120), color='black')
        self.pressed_icon = Image.open(os.path.join(self.asset_path, 'Pressed.png')).resize((80, 80))
        img.paste(self.pressed_icon, (20, 20), self.pressed_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_pressed_bytes = img_byte_arr.getvalue()

        #image stop sign
        img = Image.new('RGB', (120, 120), color='black')
        img1 = ImageDraw.Draw(img)
        img1.rectangle([30,45,90,75], fill="white", outline="white") 
        self.stop_icon = Image.open(os.path.join(self.asset_path, 'stop_sign_v2.png')).resize((80, 80))
        img.paste(self.stop_icon, (20, 20), self.stop_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_stop_bytes = img_byte_arr.getvalue()

        #image build icon
        img = Image.new('RGB', (120, 120), color='black')
        self.build_icon = Image.open(os.path.join(self.asset_path, 'build_icon.png')).resize((80, 80))
        img.paste(self.build_icon, (20, 20), self.build_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_build_bytes = img_byte_arr.getvalue()

        #image ros logo
        img = Image.new('RGB', (120, 120), color='black')
        self.ros_logo = Image.open(os.path.join(self.asset_path, 'ros_logo.png')).resize((80, 80))
        img.paste(self.ros_logo, (20, 20), self.ros_logo)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_ros_bytes = img_byte_arr.getvalue()

        #image home icon
        img = Image.new('RGB', (120, 120), color='black')
        self.home_icon = Image.open(os.path.join(self.asset_path, 'home_icon.png')).resize((80, 80))
        img.paste(self.home_icon, (20, 20), self.home_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_home_bytes = img_byte_arr.getvalue()

        #image door icon
        img = Image.new('RGB', (120, 120), color='black')
        self.door_icon = Image.open(os.path.join(self.asset_path, 'open_door_icon.png')).resize((80, 80))
        img.paste(self.door_icon, (20, 20), self.door_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_door_bytes = img_byte_arr.getvalue()

        #image exit icon

        img = Image.new('RGB', (120, 120), color='black')
        self.exit_icon = Image.open(os.path.join(self.asset_path, 'Exit.png')).resize((80, 80))
        img.paste(self.exit_icon, (20, 20), self.exit_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_exit_bytes = img_byte_arr.getvalue()

        #image gui on icon
        img = Image.new('RGB', (120, 120), color='black')
        self.gui_on_icon = Image.open(os.path.join(self.asset_path, 'gui_on_icon_v2.png')).resize((80, 80))
        img.paste(self.gui_on_icon, (20, 20), self.gui_on_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_gui_on_bytes = img_byte_arr.getvalue()

        #image gui off icon
        img = Image.new('RGB', (120, 120), color='black')
        self.gui_off_icon = Image.open(os.path.join(self.asset_path, 'gui_off_icon_v2.png')).resize((80, 80))
        img.paste(self.gui_off_icon, (20, 20), self.gui_off_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_gui_off_bytes = img_byte_arr.getvalue()

        #image motor off icon
        img = Image.new('RGB', (120, 120), color='black')
        self.motor_off_icon = Image.open(os.path.join(self.asset_path, 'motor_off_icon_v2.png')).resize((80, 80))
        img.paste(self.motor_off_icon, (20, 20), self.motor_off_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_motor_off_bytes = img_byte_arr.getvalue()

        #image motor on icon
        img = Image.new('RGB', (120, 120), color='black')
        self.motor_on_icon = Image.open(os.path.join(self.asset_path, 'motor_on_icon.png')).resize((80, 80))
        img.paste(self.motor_on_icon, (20, 20), self.motor_on_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_motor_on_bytes = img_byte_arr.getvalue()

        #image code off icon
        img = Image.new('RGB', (120, 120), color='black')
        self.code_off_icon = Image.open(os.path.join(self.asset_path, 'code_off_icon.png')).resize((80, 80))
        img.paste(self.code_off_icon, (20, 20), self.code_off_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_code_off_bytes = img_byte_arr.getvalue()

        #image code on icon
        img = Image.new('RGB', (120, 120), color='black')
        self.code_on_icon = Image.open(os.path.join(self.asset_path, 'code_on_icon.png')).resize((80, 80))
        img.paste(self.code_on_icon, (20, 20), self.code_on_icon)

        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='JPEG')
        self.img_code_on_bytes = img_byte_arr.getvalue()



        self.key_swap = [0,0,0,0,0,0,0,0]


        streamdecks = DeviceManager().enumerate()

        print("Found {} Stream Deck(s).\n".format(len(streamdecks)))

        self.deck = streamdecks[0]
        

        if self.deck.DECK_TYPE != 'Stream Deck +':
            print(self.deck.DECK_TYPE)
            print("Sorry, this controller only works with Stream Deck +")
            return

        self.deck.open()
        self.deck.reset()

        self.deck.set_key_callback(self.key_change_callback)
        self.deck.set_dial_callback(self.dial_change_callback)
        self.deck.set_touchscreen_callback(self.touchscreen_event_callback)

        print("Opened '{}' device (serial number: '{}')".format(self.deck.deck_type(), self.deck.get_serial_number()))

        # Set initial screen brightness to 30%.
        self.deck.set_brightness(100)

        for key in range(0, self.deck.KEY_COUNT):
            if key == 0:
                self.deck.set_key_image(key, self.img_code_off_bytes)
            elif key == 1:
                self.deck.set_key_image(key, self.img_motor_off_bytes)
            elif key == 2:
                self.deck.set_key_image(key, self.img_gui_off_bytes)
            elif key == 3: 
                self.deck.set_key_image(key, self.img_stop_bytes)
            elif key == 4:
                self.deck.set_key_image(key, self.img_build_bytes)
            elif key == 5:
                self.deck.set_key_image(key, self.img_home_bytes)
            elif key == 6:
                self.deck.set_key_image(key, self.img_door_bytes)
            #else:
                #self.deck.set_key_image(key, self.img_released_bytes)

        # build an image for the touch lcd
        # img = Image.new('RGB', (800, 100), 'black')
        # icon = Image.open(os.path.join(self.asset_path, 'Exit.png')).resize((80, 80))
        # img.paste(icon, (690, 10), icon)

        # for dial in range(0, self.deck.DIAL_COUNT - 1):
        #     img.paste(self.released_icon, (30 + (dial * 220), 10), self.released_icon)

        # img_bytes = io.BytesIO()
        # img.save(img_bytes, format='JPEG')
        # touchscreen_image_bytes = img_bytes.getvalue()

        img = Image.new('RGB', (800, 100), 'black')
        img.paste(self.exit_icon, (30,10), self.exit_icon)
        img.paste(self.exit_icon, (250,10), self.exit_icon)
                
        img_bytes = io.BytesIO()
        img.save(img_bytes, format='JPEG')
        touchscreen_image_bytes = img_bytes.getvalue()
        self.deck.set_touchscreen_image(touchscreen_image_bytes, 0, 0, 800, 100)

        # self.deck.set_touchscreen_image(touchscreen_image_bytes, 0, 0, 800, 100)

    def closeStreamDeck(self):
        self.processes = []
        self.processes.append(self.colcon_process)
        self.processes.append(self.encoder_process)
        self.processes.append(self.bridge_process)
        self.processes.append(self.gh360_process)
        self.processes.append(self.monitor_process)
        self.processes.append(self.gym_example_process)
        self.processes.append(self.shoulder_move_home_process)
        self.processes.append(self.upperarm_move_home_process)
        self.processes.append(self.lowerarm_move_home_process)

        for process in self.processes:
            if process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait()
                print("Process closed")
            else:
                print("Process already closed")
        self.deck.reset()
        self.deck.close()
    
    def key_change_callback(self, deck, key, key_state):
        print("Key: " + str(key) + " state: " + str(key_state))
        # if not key_state and self.key_swap[key] == 0:
        #     deck.set_key_image(key, self.img_pressed_bytes)
        #     self.key_swap[key] = 1
        # elif not key_state and self.key_swap[key] == 1:
        #     deck.set_key_image(key, self.img_released_bytes)
        #     self.key_swap[key] = 0

        if key == 0 and not key_state:
            if self.encoder_process.poll() is None:
                os.killpg(os.getpgid(self.encoder_process.pid), signal.SIGINT)
                self.encoder_process.wait()
                print("Encoder Process closed")
                os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGINT)
                self.bridge_process.wait()
                print("Bridge Process closed")
                self.deck.set_key_image(key, self.img_code_off_bytes)
            else:
                self.deck.set_key_image(key, self.img_code_on_bytes)
                self.encoder_startup()
        if key == 1 and not key_state:
            if self.gh360_process.poll() is None:
                os.killpg(os.getpgid(self.gh360_process.pid), signal.SIGINT)
                self.gh360_process.wait()
                print("GH360 Process closed")
                deck.set_key_image(key, self.img_motor_off_bytes)
            else:
                deck.set_key_image(key, self.img_motor_on_bytes)
                self.gh360_startup()
        if key == 2 and not key_state:
            if self.monitor_process.poll() is None:
                os.killpg(os.getpgid(self.monitor_process.pid), signal.SIGINT)
                self.monitor_process.wait()
                print("Monitor Process closed")
                self.deck.set_key_image(key, self.img_gui_off_bytes)
            else:
                self.deck.set_key_image(key, self.img_gui_on_bytes)
                self.gh360_monitor()
        if key == 3 and not key_state:
            self.motor_torque_off()
        if key == 4 and not key_state: 
            if self.colcon_process.poll() is None:
                os.killpg(os.getpgid(self.colcon_process.pid), signal.SIGINT)
                # self.colcon_process.wait()
            else:
                self.colcon_build()
        if key == 5 and not key_state: 
            if self.shoulder_move_home_process.poll() is None:
                os.killpg(os.getpgid(self.shoulder_move_home_process.pid), signal.SIGINT)
            if self.upperarm_move_home_process.poll() is None:
                os.killpg(os.getpgid(self.upperarm_move_home_process.pid), signal.SIGINT)
            if self.lowerarm_move_home_process.poll() is None:
                os.killpg(os.getpgid(self.lowerarm_move_home_process.pid), signal.SIGINT)
            if self.shoulder_move_home_process.poll() is not None and self.upperarm_move_home_process.poll() is not None and self.lowerarm_move_home_process.poll() is not None:
                self.move_to_home()
        if key == 6 and not key_state:
            if self.gym_example_process.poll() is None:
                os.killpg(os.getpgid(self.gym_example_process.pid), signal.SIGINT)
                self.gym_example_process.wait()
                print("Gym Example Process closed")
            else:
                self.run_gym_example()


        # if not key_state and key_s[key] == 0:
        #     deck.set_key_image(key, img_pressed_bytes)
        #     key_s[key] = 1
        # elif not key_state and key_s[key] == 1:
        #     deck.set_key_image(key, img_released_bytes)
        #     key_s[key] = 0

    # callback when dials are pressed or released
    def dial_change_callback(self, deck, dial, event, value):
        if event == DialEventType.PUSH:
            print(f"dial pushed: {dial} state: {value}")
            if dial == 3 and value:
                self.closeStreamDeck()
                pass
                # deck.reset()
                # deck.close()
            # else:
            #     # build an image for the touch lcd
            #     img = Image.new('RGB', (800, 100), 'black')
            #     icon = Image.open(os.path.join(self.asset_path, 'Exit.png')).resize((80, 80))
            #     img.paste(icon, (690, 10), icon)

            #     for k in range(0, deck.DIAL_COUNT - 1):
            #         img.paste(self.pressed_icon if (dial == k and value) else self.released_icon, (30 + (k * 220), 10),
            #                 self.pressed_icon if (dial == k and value) else self.released_icon)

            #     img_byte_arr = io.BytesIO()
            #     img.save(img_byte_arr, format='JPEG')
            #     img_byte_arr = img_byte_arr.getvalue()

            #     deck.set_touchscreen_image(img_byte_arr, 0, 0, 800, 100)
        elif event == DialEventType.TURN:
            # print(f"dial {dial} turned: {value}")
            if dial == 0:
                self.dial_0_state += value
                if self.dial_0_state > 1:
                    self.dial_0_state = 0
                elif self.dial_0_state < 0:
                    self.dial_0_state = 1

                # print(f"dial {dial} state: {self.dial_0_state}")
            elif dial == 1:
                self.dial_1_state += value
                if self.dial_1_state > 1:
                    self.dial_1_state = 0
                elif self.dial_1_state < 0:
                    self.dial_1_state = 1

                # print(f"dial {dial} state: {self.dial_1_state}")

                

            img = Image.new('RGB', (800, 100), 'black')
            if self.dial_0_state == 0:
                img.paste(self.exit_icon, (30,10), self.exit_icon)
            elif self.dial_0_state == 1:
                img.paste(self.door_icon, (30,10), self.door_icon)

            if self.dial_1_state == 0:
                img.paste(self.exit_icon, (250,10), self.exit_icon)
            elif self.dial_1_state == 1:
                img.paste(self.door_icon, (250,10), self.door_icon)
                    
            img_byte_arr = io.BytesIO()
            img.save(img_byte_arr, format='JPEG')
            img_byte_arr = img_byte_arr.getvalue()
            deck.set_touchscreen_image(img_byte_arr, 0, 0, 800, 100)

    # callback when lcd is touched
    def touchscreen_event_callback(self, deck, evt_type, value):
        if evt_type == TouchscreenEventType.SHORT:
            print("Short touch @ " + str(value['x']) + "," + str(value['y']))

        elif evt_type == TouchscreenEventType.LONG:

            print("Long touch @ " + str(value['x']) + "," + str(value['y']))

        elif evt_type == TouchscreenEventType.DRAG:

            print("Drag started @ " + str(value['x']) + "," + str(value['y']) + " ended @ " + str(value['x_out']) + "," + str(value['y_out']))

    def encoder_startup(self):
        # Start encoders
        if self.dial_0_state == 0:
            self.encoder_process = subprocess.Popen('source ~/phd_project/gh360_ws/devel/setup.bash; roslaunch gh360_control encoder_manager.launch', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        elif self.dial_0_state == 1:
            self.encoder_process = subprocess.Popen('source ~/phd_project/gh360_ws/devel/setup.bash; roslaunch gh360_control door_env.launch', shell=True, executable="/bin/bash", preexec_fn=os.setsid)

        # Start Bridge
        self.bridge_process = subprocess.Popen('source ~/phd_project/bridge_ws/install/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    
    def gh360_startup(self):
        # Start GH360
        if self.dial_1_state == 0:
            self.gh360_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 launch gh360 gh360_startup.launch.py', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        elif self.dial_1_state == 1:
            self.gh360_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 launch gh360 gh360_door_env.launch.py', shell=True, executable="/bin/bash", preexec_fn=os.setsid)

    def gh360_monitor(self):
        # Start GH360 Monitor GUI
        self.monitor_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 run gh360_examples monitor', shell=True, executable="/bin/bash", preexec_fn=os.setsid)


    def colcon_build(self):
        self.colcon_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; cd ~/phd_project/ros2_gh360_ws/; colcon build --symlink-install', shell=True, executable="/bin/bash", preexec_fn=os.setsid) #shell=True,
        # self.disable_deck = True
        # p.wait()
        # self.disable_deck = False

    def motor_torque_off(self):
        self.shoulder_torque_off_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 service call /shoulder/motor_set_torque std_srvs/srv/SetBool "{data: False}"', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        self.upperarm_torque_off_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 service call /upperarm/motor_set_torque std_srvs/srv/SetBool "{data: False}"', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        self.lowerarm_torque_off_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 service call /lowerarm/motor_set_torque std_srvs/srv/SetBool "{data: False}"', shell=True, executable="/bin/bash", preexec_fn=os.setsid)

    def move_to_home(self):
        self.shoulder_move_home_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 service call /shoulder/motor_move_home std_srvs/srv/SetBool "{data: True}"', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        self.upperarm_move_home_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 service call /upperarm/motor_move_home std_srvs/srv/SetBool "{data: True}"', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        self.lowerarm_move_home_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 service call /lowerarm/motor_move_home std_srvs/srv/SetBool "{data: True}"', shell=True, executable="/bin/bash", preexec_fn=os.setsid)

    def run_gym_example(self):
        self.gym_example_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; python ~/phd_project/ros2_gh360_ws/src/gh360/gh360_gym/example/test_real_robot.py', shell=True, executable="/bin/bash", preexec_fn=os.setsid)


if __name__ == "__main__":
    stream_deck_control = StreamDeckGH360Control()
    # stream_deck_control = StreamDeckGH360Control()
    # signal.signal(signal.SIGINT, stream_deck_control.closeStreamDeck)
    # Wait until all application threads have terminated (for this example,
    # this is when all deck handles are closed).
    for t in threading.enumerate():
        try:
            t.join()
        except RuntimeError:
            pass
            # print("Deleting stream deck control")
            # stream_deck_control.closeStreamDeck()
        except KeyboardInterrupt:
            print("Deleting stream deck control")
            stream_deck_control.closeStreamDeck()
        except:
            pass

    


