import os
import threading
import io
import subprocess
import signal

from PIL import Image, ImageDraw
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.Devices.StreamDeck import DialEventType, TouchscreenEventType
from src.utils import load_icon
import src.action_library as action_library
from src.environments import Env


class StreamDeckGH360Control:
    def __init__(self):
        self.dial_0_state = 0
        self.dial_1_state = 0
        self.env = Env.no_env

        self.key_actions = []
        self.key_actions.append(action_library.encoder_startup())
        self.key_actions.append(action_library.motor_startup())
        self.key_actions.append(action_library.state_monitor())
        self.key_actions.append(action_library.motor_torque_off())
        self.key_actions.append(action_library.colcon_build())
        self.key_actions.append(action_library.move_home())
        self.key_actions.append(action_library.spacemouse_teleop())
        self.key_actions.append(action_library.rosbag_record())

        self.dial_1_actions = []
        # self.dial_1_actions.append(action_library.rosbag_record())
        # self.dial_1_actions.append(action_library.rosbag_play())

        self.icon_exit = load_icon('Exit.png')
        self.icon_door = load_icon('open_door_icon.png')
        self.icon_start_record = load_icon('record_icon.png')
        self.icon_stop_record = load_icon('stop_record_icon.png', white_background=True)
        self.icon_play_record = load_icon('play_icon.png', white_background=True)

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

        # for key in range(0, self.deck.KEY_COUNT):
        for key in range(0, len(self.key_actions)):
            image = self.key_actions[key].off_icon
            self.deck.set_key_image(key, image)

        img = Image.new('RGB', (800, 100), 'black')
        img.paste(self.icon_exit, (30,10), self.icon_exit)
        img.paste(self.icon_start_record, (250,10), self.icon_start_record)
                
        img_bytes = io.BytesIO()
        img.save(img_bytes, format='JPEG')
        touchscreen_image_bytes = img_bytes.getvalue()
        self.deck.set_touchscreen_image(touchscreen_image_bytes, 0, 0, 800, 100)

    def closeStreamDeck(self):
        for key_action in self.key_actions:
            key_action.close()

        self.deck.reset()
        self.deck.close()
    
    def key_change_callback(self, deck, key, key_state):
        print("Key: " + str(key) + " state: " + str(key_state))

        if not key_state and key < len(self.key_actions):
            print("Env: " + str(self.env))
            key_image = self.key_actions[key].keypress(self.env)
            deck.set_key_image(key, key_image)

    # callback when dials are pressed or released
    def dial_change_callback(self, deck, dial, event, value):
        if event == DialEventType.PUSH:
            print(f"dial pushed: {dial} state: {value}")
            if dial == 1 and value:
                if self.dial_1_state == 2:
                    self.dial_1_actions[0].close()
                    self.dial_1_actions[1].close()
                    self.dial_1_state = 0
                else:
                    self.dial_1_actions[self.dial_1_state].keypress()
                    self.dial_1_state = 2
            if dial == 3 and value:
                self.closeStreamDeck()
        elif event == DialEventType.TURN:
            # print(f"dial {dial} turned: {value}")
            if dial == 0:
                self.dial_0_state += value
                if self.dial_0_state > 1:
                    self.dial_0_state = 0
                elif self.dial_0_state < 0:
                    self.dial_0_state = 1
                self.env = Env(self.dial_0_state+1)
            if dial == 1:
                if self.dial_1_state != 2:
                    self.dial_1_state += value
                    if self.dial_1_state > 1:
                        self.dial_1_state = 0
                    elif self.dial_1_state < 0:
                        self.dial_1_state = 1
                 

        img = Image.new('RGB', (800, 100), 'black')
        if self.dial_0_state == 0:
            img.paste(self.icon_exit, (30,10), self.icon_exit)
        elif self.dial_0_state == 1:
            img.paste(self.icon_door, (30,10), self.icon_door)
        if self.dial_1_state == 0:
            img.paste(self.icon_start_record, (250,10), self.icon_start_record)
        elif self.dial_1_state == 1:
            img.paste(self.icon_play_record, (250,10), self.icon_play_record)
        elif self.dial_1_state == 2:
            img.paste(self.icon_stop_record, (250,10), self.icon_stop_record)
                
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

if __name__ == "__main__":
    stream_deck_control = StreamDeckGH360Control()
    # Wait until all application threads have terminated (for this example,
    # this is when all deck handles are closed).
    for t in threading.enumerate():
        try:
            t.join()
        except RuntimeError:
            pass
        except KeyboardInterrupt:
            print("Deleting stream deck control")
            stream_deck_control.closeStreamDeck()
        except:
            pass

    


