#!/usr/bin/env python
import keyboard
import rospy
import numpy as np
from std_msgs.msg import String, Bool
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from typing import Optional, Tuple


class KeyPressCommand():
    """
    EVO rqt widget actions
    """
    #_last_info_msg = Info()
    _publisher = None
    _subscriber = None
    _publisher_copilot = None

    def __init__(self, evo_namespace='evo'):


        if not evo_namespace:
            evo_namespace = 'evo'

        self._evo_namespace = evo_namespace
        self.register(evo_namespace)
        self.useEVO = False
        self.checkbox_map_expansion = True
        self.append_to_pc = True
        rospy.init_node('keypresser', anonymous=True)
        self.text_to_write = "\nt: reset evs tracking\n" + "m: switch map expansion\n" + "p: switch append to pointcloud\n" + "u: update map\n" + "e: switch copilot\n" +"r: reset rovio\n" + "k: quit this program\n" 


    def register(self, evo_namespace):
        """
        Subscribes to ROS Info topic and registers callbacks
        """
        # self._subscriber = rospy.Subscriber(evo_namespace+'/info', Info, self.info_cb)

        # Initialize Publisher
        self._publisher = rospy.Publisher(
            evo_namespace+'/remote_key', String, queue_size=1)
        self._publisher_copilot = rospy.Publisher(
            evo_namespace+'/copilot_remote', Bool, queue_size=1)

    def unregister(self):
        """
        Unregisters publishers
        """
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

        if self._publisher_copilot is not None:
            self._publisher_copilot.unregister()
            self._publisher_copilot = None

        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None


    def on_bootstrap_button_pressed(self):
        """
        Triggers bootstrap
        """
        print('BOOTSTRAPPING')
        self.send_command('bootstrap')

    def on_start_button_pressed(self):
        """
        Triggers reset button
        """
        print('START/RESET')
        self.send_command('reset')

    def on_update_button_pressed(self):
        """
        Triggers map update
        """
        print('UPDATE')
        self.send_command('update')

    def on_switch_button_pressed(self):
        """
        Turns on tracking thread
        """
        print('SWITCH TO TRACKING')
        self.send_command('switch')

    def on_map_expansion_changed(self):
        """
        Switches on and off the map expansion algorithm based on checkbox_map_expansion 
        """
        self.checkbox_map_expansion = not self.checkbox_map_expansion
        if self.checkbox_map_expansion:
            print('ENABLE MAP EXPANSION')
            self.send_command('enable_map_expansion')
        else:
            print('DISABLE EXPANSION')
            self.send_command('disable_map_expansion')

    def on_pointcloud_update_changed(self):
        self.send_command("global_pc_switch")
        self.append_to_pc = not self.append_to_pc
        print("APPEND TO POINTCLOUD: ", self.append_to_pc)

    def on_copilot_state_changed(self):
        """
        Switch from bootstrapping tracker to evo tracker
        """

        self.useEVO = not self.useEVO
        print('SWITCH COPILOT TO ' + ('EVO' if self.useEVO else 'INITIAL PILOT'))
        self._publisher_copilot.publish(Bool(self.useEVO))

    def send_command(self, cmd):
        """
        Utils to send remote command
        """
        if self._publisher is None:
            return
        self._publisher.publish(String(cmd))

    def write_image(self, image, additional_text = ""):
        if(additional_text!=""):
            print(additional_text)
        text_to_write = self.text_to_write + additional_text
        return add_text_to_image(image, text_to_write)



def add_text_to_image(
    image_rgb: np.ndarray,
    label: str,
    top_left_xy: Tuple = (0, 0),
    font_scale: float = 1,
    font_thickness: float = 3,
    font_face=cv2.FONT_HERSHEY_SIMPLEX,
    font_color_rgb: Tuple = (255, 255, 0),
    bg_color_rgb: Optional[Tuple] = None,
    outline_color_rgb: Optional[Tuple] = None,
    line_spacing: float = 1,
):
    """
    Adds text (including multi line text) to images.
    You can also control background color, outline color, and line spacing.

    outline color and line spacing adopted from: https://gist.github.com/EricCousineau-TRI/596f04c83da9b82d0389d3ea1d782592
    """
    OUTLINE_FONT_THICKNESS = 3 * font_thickness

    im_h, im_w = image_rgb.shape[:2]

    for line in label.splitlines():
        x, y = top_left_xy

        # ====== get text size
        if outline_color_rgb is None:
            get_text_size_font_thickness = font_thickness
        else:
            get_text_size_font_thickness = OUTLINE_FONT_THICKNESS

        (line_width, line_height_no_baseline), baseline = cv2.getTextSize(
            line,
            font_face,
            font_scale,
            get_text_size_font_thickness,
        )
        line_height = line_height_no_baseline + baseline

        if bg_color_rgb is not None and line:
            # === get actual mask sizes with regard to image crop
            if im_h - (y + line_height) <= 0:
                sz_h = max(im_h - y, 0)
            else:
                sz_h = line_height

            if im_w - (x + line_width) <= 0:
                sz_w = max(im_w - x, 0)
            else:
                sz_w = line_width

            # ==== add mask to image
            if sz_h > 0 and sz_w > 0:
                bg_mask = np.zeros((sz_h, sz_w, 3), np.uint8)
                bg_mask[:, :] = np.array(bg_color_rgb)
                image_rgb[
                    y : y + sz_h,
                    x : x + sz_w,
                ] = bg_mask

        # === add outline text to image
        if outline_color_rgb is not None:
            image_rgb = cv2.putText(
                image_rgb,
                line,
                (x, y + line_height_no_baseline),  # putText start bottom-left
                font_face,
                font_scale,
                outline_color_rgb,
                OUTLINE_FONT_THICKNESS,
                cv2.LINE_AA,
            )
        # === add text to image
        image_rgb = cv2.putText(
            image_rgb,
            line,
            (x, y + line_height_no_baseline),  # putText start bottom-left
            font_face,
            font_scale,
            font_color_rgb,
            font_thickness,
            cv2.LINE_AA,
        )
        top_left_xy = (x, y + int(line_height * line_spacing))

    return image_rgb

if __name__=="__main__":
    keypresser = KeyPressCommand()
    is_enabled = True
    cv_bridge = CvBridge()
    image_publisher = rospy.Publisher("/evo/commands", Image, queue_size=10)
    while True:
        additional_text = ""
        if keyboard.is_pressed("x"):
            is_enabled = not is_enabled
            print("listening to the keyboard is ", is_enabled)
            time.sleep(1)
        
        def write_and_publish(text, sleep=0.7):
            image = np.zeros((512,512,3), np.uint8)
            image = keypresser.write_image(image, text)
            image_publisher.publish(cv_bridge.cv2_to_imgmsg(image))
            time.sleep(sleep)

        if not is_enabled:
            continue
        if keyboard.is_pressed("t"):
            keypresser.on_start_button_pressed()
            additional_text += "\n\n\nReset tracking...\n"
            write_and_publish(additional_text)
            keypresser.on_switch_button_pressed()
        if keyboard.is_pressed("m"):
            keypresser.on_map_expansion_changed()
            additional_text += "\n\n\nSwitched map expansion: {}\n".format(keypresser.checkbox_map_expansion)
            write_and_publish(additional_text)
        if keyboard.is_pressed("e"):
            keypresser.on_copilot_state_changed()
            additional_text += "\n\n\nCopilot changed to: {}\n".format(keypresser.useEVO)
            write_and_publish(additional_text)
        if keyboard.is_pressed("u"):
            keypresser.on_update_button_pressed()
            additional_text += "\n\n\nUpdate map command sent\n"
            write_and_publish(additional_text)
        if keyboard.is_pressed("k"):
            break
        if keyboard.is_pressed("r"):
            keypresser.send_command("reset_rovio")
            additional_text += "\n\n\nResetting rovio..\n"
            print("resetting ROVIO")
            write_and_publish(additional_text)
        if keyboard.is_pressed("p"):
            keypresser.on_pointcloud_update_changed()
            additional_text += "\n\n\nAppend to pointcloud: {}".format(keypresser.append_to_pc)
            write_and_publish(additional_text)
            
        write_and_publish("", 0.001)
            
        


        


