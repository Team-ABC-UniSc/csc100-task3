import asyncio
import logging
import json
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
from functions import *
import pygame
import cv2
import numpy as np

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

# Movement parameters
MOVE_SPEED = 1  # Forward/backward speed
STRAFE_SPEED = 0.5  # Left/right strafe speed
TURN_SPEED = 1  # Rotation speed

#controls
UP = pygame.K_w
DOWN = pygame.K_s
LEFT = pygame.K_a
RIGHT = pygame.K_d
STRAFE_LEFT = pygame.K_q
STRAFE_RIGHT = pygame.K_e
TOGGLE_LIGHT = pygame.K_l
JUMP = pygame.K_SPACE
ESC = pygame.K_ESCAPE

class Robot:
    def __init__(self, conn):
        self.conn = conn
        self.latest_frame = None
        self.light = False

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


    async def move(self, vx=0.0, vy=0.0, vyaw=0.0):
        await self.conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["Move"], "parameter": {"x": vx, "y": vy, "z": vyaw}})

    async def stop(self):
        await self.conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StopMove"]})

    async def toggle_light(self, on=True):
        await self.conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["VUI_COLOR"], {"api_id": VUI_COLOR["Light"], "parameter": {"color": "white" if on else "black"}})

    async def jump(self):
        await self.conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["FrontJump"], "parameter": {"data": True}})
    async def disconnect(self):
        # Turn off video before disconnecting
        try:
            self.conn.video.switchVideoChannel(False)
        except:
            pass
        await self.conn.disconnect()

    async def toggle_light(self):
        """ Args:
        conn: connection of robot
        brightness: set brightness, default 10 (max)
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


class KeyboardControls:
    def __init__(self, robot):
        self.robot = robot
        self.running = True
    async def control_loop(self):
        screen = pygame.display.set_mode((1280, 720))  # Match robot camera resolution
        pygame.display.set_caption("Robot Dog Control")
        clock = pygame.time.Clock()
        print("\nKeyboard Controls Active:")
        print("W - Forward | S - Backward")
        print("A - Turn Left | D - Turn Right")
        print("SPACE - Jump")
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
            
            # Get continuous keyboard state (for movement)
            keys = pygame.key.get_pressed()
            
            # Forward/Backward (W/S)
            if keys[UP]:
                vx = MOVE_SPEED
            elif keys[DOWN]:
                vx = -MOVE_SPEED
            
            # Turn Left/Right (A/D)
            if keys[LEFT]:
                vyaw = TURN_SPEED
            elif keys[RIGHT]:
                vyaw = -TURN_SPEED

            # Strafe Left/Right (Q/E)
            if keys[STRAFE_LEFT]:
                vy = STRAFE_SPEED
            elif keys[STRAFE_RIGHT]:
                vy = -STRAFE_SPEED

            # Toggle Light (L)
            if keys[TOGGLE_LIGHT]:
                await self.robot.toggle_light()

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
            status = font.render(f"vx:{vx:.2f} vyaw:{vyaw:.2f}", True, (0, 255, 0))
            
            # Draw semi-transparent background for text
            status_bg = pygame.Surface((200, 30))
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