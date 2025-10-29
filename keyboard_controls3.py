"""
TEAM ABC - Task 3
purpose:
    add keyboard control functionality and camera view of go2 robot dog using pygame
"""



"""IMPORT LIBRARIES"""
import asyncio
import logging
import json
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD, VUI_COLOR
from functions import *
import pygame
import cv2
import numpy as np

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)


#controls constants
UP = pygame.K_w
DOWN = pygame.K_s
LEFT = pygame.K_a
RIGHT = pygame.K_d
STRAFE_LEFT = pygame.K_q
STRAFE_RIGHT = pygame.K_e
TOGGLE_LIGHT = pygame.K_l
JUMP = pygame.K_SPACE
ESC = pygame.K_ESCAPE
GAIT_IDLE = pygame.K_0
GAIT_TROT = pygame.K_1
GAIT_TROT_FAST = pygame.K_2
GAIT_STABLE = pygame.K_3

class Robot:
    """
    Robot class for controlling the Go2 robot dog via keyboard and streaming video.

    This class provides high-level asynchronous methods to control the Go2 robot's movement, 
    gait, lighting, and video streaming. It communicates with the robot through the 
    Go2WebRTCConnection interface, handling all data publishing and video reception.

    Main Features:
    - Start and handle video stream from the robot.
    - Perform motion commands: move, stop, gait switching, jump.
    - Toggle robot lights.
    - Retrieve the latest camera frame for display.
    - Store and manage the current gait and associated movement speeds.

    Parameters
    ----------
    conn : Go2WebRTCConnection
        An active WebRTC connection to the robot.

    Attributes
    ----------
    conn : Go2WebRTCConnection
        Underlying connection to the Go2 robot.
    latest_frame : np.ndarray or None
        The most recent video frame from the robot's camera.
    light : bool
        The state of the robot's headlight.
    current_gait : int
        The current gait mode (0: idle, 1: trot, 2: trot fast, 3: stable/climb).
    gait_speeds : dict
        Parameters defining speed and style for each gait mode.

    Example Usage
    -------------
    robot = Robot(conn)
    await robot.start_video_capture()
    await robot.move(vx=1.0)
    await robot.stop()
    await robot.toggle_light(on=True)
    frame = robot.get_latest_frame()
    """
    def __init__(self, conn):
        self.conn = conn
        self.latest_frame = None
        self.light = False
        self.current_gait = 0

        # Speed settings for each gait mode
        self.gait_speeds = {
            0: {"move": 0, "strafe": 0, "turn": 0, "name": "Idle"},
            1: {"move": 0.6, "strafe": 0.4, "turn": 0.8, "name": "Trot"},
            2: {"move": 1, "strafe": 0.6, "turn": 1.0, "name": "Trot Fast"},
            3: {"move": 0.4, "strafe": 0.3, "turn": 0.5, "name": "Stable/Climb"}
        }

    async def start_video_capture(self):
        """Enable video stream and set up frame callback"""
        # Switch video channel on
        self.conn.video.switchVideoChannel(True)
        
        # Add callback to receive video frames
        self.conn.video.add_track_callback(self._video_frame_callback)
        
        print("✅ Video stream started")

    async def _video_frame_callback(self, track):
        """Callback function to receive video frames"""
        try:
            while True:
                frame = await track.recv()
                # Convert frame to numpy array (BGR format)
                img = frame.to_ndarray(format="bgr24")
                self.latest_frame = img
        except Exception as e:
            logging.error(f"Video callback error: {e}")

    def get_latest_frame(self):
        """Get the most recent video frame"""
        return self.latest_frame

    #MOVEMENT COMMANDS
    async def move(self, vx=0.0, vy=0.0, vyaw=0.0):
        """send movement command to robot"""
        await self.conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["Move"], "parameter": {"x": vx, "y": vy, "z": vyaw}})

    async def stop(self):
        """stop all movements to robot"""
        await self.conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StopMove"]})

    async def jump(self):
        """send frontjump command to robot"""
        await self.conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["FrontJump"], "parameter": {"data": True}})



    async def disconnect(self):
        """disconnect video channel"""
        # Turn off video before disconnecting
        try:
            self.conn.video.switchVideoChannel(False)
        except:
            pass
        await self.conn.disconnect()


    async def toggle_light(self):
        """
        turns light on if it is off, turns off if it is on
        changes class paramater to track light status
        currently only has bright white light or  off
        """

        if self.light == False:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["VUI"],
                {"api_id": 1005, "parameter": {"brightness": 10}}
            )
            """Toggle the light of the robot"""
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["VUI"], 
                {
                    "api_id": 1007,
                    "parameter": 
                    {
                        "color": VUI_COLOR.WHITE,
                    }
                }
            )
            self.light = True
        else:
            self.light = False
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["VUI"],
                {"api_id": 1005, "parameter": {"brightness": 0}}
            )
           

        return

    async def _set_gait(self, gait_type=1):
        """
        Set the robot's gait mode
        gait_type:
            0 = idle/stand
            1 = trot (recommended for general use - balanced)
            2 = trot running (faster)
            3 = climb stairs (very stable, slower)
            4 = trot obstacle (stable on rough terrain)
        """
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["SwitchGait"], "parameter": {"d": gait_type}}
        )
        print(f"Gait mode set to: {gait_type}")

    async def change_gait(self, gait_mode):
        """Change gait mode and update speeds stored in class parameters"""
        self.current_gait = gait_mode
        await self._set_gait(gait_mode)
        speeds = self.gait_speeds[gait_mode]
        print(f"Switched to {speeds['name']} mode")
        print(f"Speeds: move={speeds['move']}, strafe={speeds['strafe']}, turn={speeds['turn']}")


class KeyboardControls:
    """
    KeyboardControls class provides a keyboard-driven interface to control the robot dog in real-time.

    Features:
    ----------
    - Connects to a robot instance and enables teleoperation using your keyboard.
    - Handles gait changes (Idle, Trot, Fast Trot, Stable/Climb) via number keys (0-3).
    - Controls robot movement using WASD keys for forward/back/turn and Q/E for strafing.
    - Toggles headlight with 'L' and triggers jump with SPACEBAR.
    - Gracefully handles quitting with the ESC key or window close.
    - Continuously polls key states for fluid and responsive movement.
    - Displays helpful on-screen instructions.
    - Manages and updates robot's gait and speed internally to match keyboard input.

    Main Methods:
    -------------
    - __init__(robot):     Initializes with a robot instance.
    - control_loop():      Starts the main async input loop for handling all keyboard events and driving the robot.

    Example Usage:
    --------------
        controls = KeyboardControls(robot)
        await controls.control_loop()

    Keyboard Mapping Summary:
    ------------------------
        W/S         Move forward/backward
        A/D         Turn left/right
        Q/E         Strafe left/right
        0           Idle/Stand mode
        1           Trot mode (default, balanced)
        2           Trot Fast mode
        3           Stable/Climb mode
        L           Toggle robot headlight
        SPACE       Jump
        ESC         Quit control loop/close window
    """
    def __init__(self, robot):
        self.robot = robot
        self.running = True


    async def control_loop(self):
        """
        Asynchronously handles keyboard events and controls the robot in real-time.

        Main responsibilities:
            - Initializes the Pygame display and sets up the control window.
            - Polls for keyboard events (key down/up, window close, etc.).
            - Maps key presses (WASD, QE, 0-3, L, SPACE, ESC) to robot control actions:
                - Movement (forward, backward, turn, strafe)
                - Gait/mode switching
                - Headlight toggle
                - Jump trigger
                - Graceful exit
            - Continuously queries the state of control keys for fluid, real-time movement.
            - Shows keyboard help summary in the console.
            - Loops as long as self.running is True, updating robot state accordingly.
            - Displays live robot camera feed in the window if available (starts video capture).
            - Handles both discrete (gait/light/jump) and continuous (WASD/QE) actions.

        Usage:
            Await this method after instantiating KeyboardControls:
                await KeyboardControls(robot).control_loop()

        Returns:
            None
        """
        screen = pygame.display.set_mode((1280, 720))  # Match robot camera resolution
        pygame.display.set_caption("Robot Dog Control")
        clock = pygame.time.Clock()
        print("\nKeyboard Controls Active:")
        print("W/S - Forward/Backward | A/D - Turn")
        print("Q/E - Strafe Left/Right")
        print("0 - Idle Mode | 1 - Trot Mode | 2 - Trot Fast | 3 - Stable/Climb")
        print("L - Toggle Light | SPACE - Jump")
        print("ESC - Quit\n")
        await self.robot.start_video_capture()
        
        while self.running:
            vx = 0.0
            vy = 0.0
            vyaw = 0.0

            # Handle events (for key presses and quit)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == ESC:
                        self.running = False
                    elif event.key == JUMP:
                        await self.robot.jump()
                    elif event.key == TOGGLE_LIGHT:
                        await self.robot.toggle_light()
                    elif event.key == GAIT_IDLE:
                        await self.robot.change_gait(0)
                    elif event.key == GAIT_TROT:
                        await self.robot.change_gait(1)  # Trot mode
                    elif event.key == GAIT_TROT_FAST:
                        await self.robot.change_gait(2)  # Trot fast mode
                    elif event.key == GAIT_STABLE:
                        await self.robot.change_gait(3)  # Stable/rough terrain mode

            # Get continuous keyboard state (for movement)
            keys = pygame.key.get_pressed()

            # Get current speed settings based on gait mode
            speeds = self.robot.gait_speeds[self.robot.current_gait]

            # Forward/Backward (W/S)
            if keys[UP]:
                vx = speeds["move"]
            elif keys[DOWN]:
                vx = -speeds["move"]

            # Turn Left/Right (A/D)
            if keys[LEFT]:
                vyaw = speeds["turn"]
            elif keys[RIGHT]:
                vyaw = -speeds["turn"]

            # Strafe Left/Right (Q/E)
            if keys[STRAFE_LEFT]:
                vy = speeds["strafe"]
            elif keys[STRAFE_RIGHT]:
                vy = -speeds["strafe"]

            # Send movement command (even if 0,0,0 to stop)
            await self.robot.move(vx, vy, vyaw)

            # Display video feed
            frame = self.robot.get_latest_frame()
            if frame is not None:
                # Convert BGR to RGB (OpenCV uses BGR, pygame uses RGB)
                frame_rgb = np.flip(frame, axis=2)

                # Convert to pygame surface
                frame_surface = pygame.surfarray.make_surface(
                    np.transpose(frame_rgb, (1, 0, 2))
                )

                # Scale to fit screen if needed
                frame_surface = pygame.transform.scale(frame_surface, (1280, 720))

                # Draw video feed
                screen.blit(frame_surface, (0, 0))
            else:
                # No video yet - show black screen
                screen.fill((30, 30, 30))

            # Draw status overlay
            font = pygame.font.Font(None, 24)
            speeds = self.robot.gait_speeds[self.robot.current_gait]
            status = font.render(f"Mode: {speeds['name']} | vx:{vx:.2f} vy:{vy:.2f} vyaw:{vyaw:.2f}",
                                 True, (0, 255, 0))

            # Draw semi-transparent background for text
            status_bg = pygame.Surface((500, 30))
            status_bg.set_alpha(128)
            status_bg.fill((0, 0, 0))
            screen.blit(status_bg, (10, 10))
            screen.blit(status, (15, 15))

            pygame.display.flip()
            clock.tick(30)
            await asyncio.sleep(0.033)
        
        await self.robot.stop()



async def main():
    conn = None
    robot = None
    try:
        pygame.init()
        conn = await connect_to_robot()
        await start_up(conn)
        await toggle_light_off(conn)
        await asyncio.sleep(0.1)
        
        # Debug: Inspect the connection object
        print("\n=== Debugging Connection Object ===")
        print(f"Connection type: {type(conn)}")
        print(f"\nAll attributes with 'video' or 'track':")
        for attr in dir(conn):
            if 'video' in attr.lower() or 'track' in attr.lower() or 'stream' in attr.lower():
                print(f"  - {attr}")
        
        # Check if there's a peer connection
        if hasattr(conn, 'pc'):
            print(f"\nPeer connection exists: {conn.pc}")
            print(f"Receivers: {conn.pc.getReceivers()}")
        
        robot = Robot(conn)
        await robot.change_gait(1)
        await asyncio.sleep(0.3)
        controller = KeyboardControls(robot)
        await controller.control_loop()
        
    except ValueError as e:
        logging.error(f"An error occurred: {e}")
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        if robot is not None:
            try:
                await robot.stop()
                await asyncio.sleep(0.1)
                await robot.disconnect()
            except Exception as e:
                logging.error(f"Error during cleanup: {e}")
        pygame.quit()

if __name__ == "__main__":
    asyncio.run(main())