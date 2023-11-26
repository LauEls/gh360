import os
import threading
import io
import subprocess
import signal

from PIL import Image
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
            self.deck.set_key_image(key, self.img_released_bytes)

        # build an image for the touch lcd
        img = Image.new('RGB', (800, 100), 'black')
        icon = Image.open(os.path.join(self.asset_path, 'Exit.png')).resize((80, 80))
        img.paste(icon, (690, 10), icon)

        for dial in range(0, self.deck.DIAL_COUNT - 1):
            img.paste(self.released_icon, (30 + (dial * 220), 10), self.released_icon)

        img_bytes = io.BytesIO()
        img.save(img_bytes, format='JPEG')
        touchscreen_image_bytes = img_bytes.getvalue()

        self.deck.set_touchscreen_image(touchscreen_image_bytes, 0, 0, 800, 100)

    def key_change_callback(self, deck, key, key_state):
        print("Key: " + str(key) + " state: " + str(key_state))
        if not key_state and self.key_swap[key] == 0:
            deck.set_key_image(key, self.img_pressed_bytes)
            self.key_swap[key] = 1
        elif not key_state and self.key_swap[key] == 1:
            deck.set_key_image(key, self.img_released_bytes)
            self.key_swap[key] = 0

        if key == 0 and not key_state:
            if self.encoder_process.poll() is None:
                os.killpg(os.getpgid(self.encoder_process.pid), signal.SIGINT)
                os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGINT)
            else:
                self.encoder_startup()
        if key == 1 and not key_state:
            if self.gh360_process.poll() is None:
                os.killpg(os.getpgid(self.gh360_process.pid), signal.SIGINT)
            else:
                self.gh360_startup()
        if key == 2 and not key_state:
            if self.monitor_process.poll() is None:
                os.killpg(os.getpgid(self.monitor_process.pid), signal.SIGINT)
            else:
                self.gh360_monitor()
        if key == 4 and not key_state: 
            if self.colcon_process.poll() is None:
                os.killpg(os.getpgid(self.colcon_process.pid), signal.SIGINT)
                # self.colcon_process.wait()
            else:
                self.colcon_build()

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
                deck.reset()
                deck.close()
            else:
                # build an image for the touch lcd
                img = Image.new('RGB', (800, 100), 'black')
                icon = Image.open(os.path.join(self.asset_path, 'Exit.png')).resize((80, 80))
                img.paste(icon, (690, 10), icon)

                for k in range(0, deck.DIAL_COUNT - 1):
                    img.paste(self.pressed_icon if (dial == k and value) else self.released_icon, (30 + (k * 220), 10),
                            self.pressed_icon if (dial == k and value) else self.released_icon)

                img_byte_arr = io.BytesIO()
                img.save(img_byte_arr, format='JPEG')
                img_byte_arr = img_byte_arr.getvalue()

                deck.set_touchscreen_image(img_byte_arr, 0, 0, 800, 100)
        elif event == DialEventType.TURN:
            print(f"dial {dial} turned: {value}")


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
        self.encoder_process = subprocess.Popen('source ~/phd_project/gh360_ws/devel/setup.bash; roslaunch gh360_control encoder_manager.launch', shell=True, executable="/bin/bash", preexec_fn=os.setsid)

        # Start Bridge
        self.bridge_process = subprocess.Popen('source ~/phd_project/bridge_ws/install/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics', shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    
    def gh360_startup(self):
        # Start GH360
        self.gh360_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 launch gh360 gh360_startup.launch.py', shell=True, executable="/bin/bash", preexec_fn=os.setsid)

    def gh360_monitor(self):
        # Start GH360 Monitor GUI
        self.monitor_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 run gh360_examples monitor', shell=True, executable="/bin/bash", preexec_fn=os.setsid)


    def colcon_build(self):
        self.colcon_process = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; cd ~/phd_project/ros2_gh360_ws/; colcon build --symlink-install', shell=True, executable="/bin/bash", preexec_fn=os.setsid) #shell=True,
        # self.disable_deck = True
        # p.wait()
        # self.disable_deck = False





if __name__ == "__main__":
    stream_deck_control = StreamDeckGH360Control()

    # Wait until all application threads have terminated (for this example,
    # this is when all deck handles are closed).
    for t in threading.enumerate():
        try:
            t.join()
        except RuntimeError:
            pass


