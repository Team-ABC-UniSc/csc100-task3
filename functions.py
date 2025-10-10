"""Functions for the robot
"""

import asyncio
import logging
import json
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD, VUI_COLOR
import math

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

async def stop_move(conn):
    """stop the robot
    Args:
        conn: connection to the robot
    """
    await conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StopMove"]})
    await asyncio.sleep(2)
    return


async def connect_to_robot(ip="192.168.12.1"):
    """connect to robot over local STA
    Args: 
        ip: ip address of the robot
    Returns:
        conn: connection to the robot
    """
    conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip)
    await conn.connect()
    return conn

async def disconnect_from_robot(conn):
    """disconnect from robot
    Args:
        conn: connection to the robot
    """
    await conn.disconnect()

async def start_up(conn):
    """start up the robot
    Args:
        conn: connection to the robot
    """
    ####### NORMAL MODE ########
    print("Checking current motion mode...")

    # Get the current motion_switcher status
    response = await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["MOTION_SWITCHER"], 
        {"api_id": 1001}
    )

    if response['data']['header']['status']['code'] == 0:
        data = json.loads(response['data']['data'])
        current_motion_switcher_mode = data['name']
        print(f"Current motion mode: {current_motion_switcher_mode}")

    # Switch to "normal" mode if not already
    if current_motion_switcher_mode != "normal":
        print(f"Switching motion mode from {current_motion_switcher_mode} to 'normal'...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], 
            {
                "api_id": 1002,
                "parameter": {"name": "normal"}
            }
        )
        await asyncio.sleep(5)  # Wait while it stands up
    return


async def move(conn, distance, speed=1):
    """Move robot a specific distance in meters.
    
    Args:
        conn: connection to the robot
        distance: distance in meters (+forward, -backward)
        speed: velocity in m/s (recommend 0.2-0.5 for accuracy)
    """
    # Set velocity direction
    velocity = speed if distance >= 0 else -speed
    
    # Calculate how long to move
    duration = abs(distance) / speed
    
    direction = "forward" if distance > 0 else "backward"
    print(f"Moving {direction} {abs(distance)}m at {speed}m/s (duration: {duration:.2f}s)...")
    
    # Send velocity commands repeatedly (every 0.1s) for the duration
    start_time = asyncio.get_event_loop().time()
    
    try:
        while (asyncio.get_event_loop().time() - start_time) < duration:
            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": velocity, "y": 0, "z": 0}
                }
            )
            await asyncio.sleep(0.1)  # Send command every 100ms
    
    except Exception as e:
        print(f"Movement error: {e}")
    
    finally:
        # Always stop at the end
        await stop_move(conn)
    
    return

"""   
async def move(conn, distance, speed = 2.0):

    #set velocity, positive for forward, negative for backward
    if distance >= 0:
        velocity = speed
    else:
        velocity = -speed

    #calculate duration of the movement
    duration = abs(distance) / speed

    #set direction, positive for forward, negative for backward
    if distance > 0:
        direction = "forward"
    else:
        direction = "backward"
    print(f"Moving {direction}...")

    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"], 
        {
            "api_id": SPORT_CMD["Move"],
            "parameter": {"x": velocity, "y": 0, "z": 0}
        }
    )       
    await asyncio.sleep(duration)
    await stop_move(conn)
    return
"""

"""
async def turn(conn, degrees, yaw_rate_rad_s=0.7):
    #Turn the robot by a specific angle in degrees.
    #Args:
    #    conn: connection to the robot
    #    degrees: +right / -left turn angle in degrees
    #    yaw_rate_rad_s: angular velocity magnitude (rad/s)

    import math
    direction_sign = 1 if degrees >= 0 else -1
    duration_s = abs(math.radians(degrees)) / yaw_rate_rad_s

    print(f"Turning {'right' if degrees >= 0 else 'left'} {abs(degrees)}°...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {
            "api_id": SPORT_CMD["Move"],
            "parameter": {"x": 0, "y": 0, "z": direction_sign * yaw_rate_rad_s}
        }
    )
    await asyncio.sleep(duration_s)
    await stop_move(conn)
    return

async def jump_forward(conn):
    #Make the robot jump
    print("Jumping...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["FrontJump"], "parameter": {"data": True}}
    )
    await asyncio.sleep(2)
    await stop_move(conn)
    return
"""

async def turn(conn, degrees, yaw_rate_rad_s=0.5):
    """Turn the robot by a specific angle in degrees.
    
    Args:
        conn: connection to the robot
        degrees: angle to turn (+right / -left) in degrees
        yaw_rate_rad_s: angular velocity in rad/s (recommend 0.3-0.7)
    """
    import math
    
    direction_sign = 1 if degrees >= 0 else -1
    duration_s = abs(math.radians(degrees)) / yaw_rate_rad_s
    
    direction = "right" if degrees >= 0 else "left"
    print(f"Turning {direction} {abs(degrees)}° at {yaw_rate_rad_s} rad/s (duration: {duration_s:.2f}s)...")
    
    # Send continuous turn commands
    start_time = asyncio.get_event_loop().time()
    
    try:
        while (asyncio.get_event_loop().time() - start_time) < duration_s:
            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {"x": 0, "y": 0, "z": direction_sign * yaw_rate_rad_s}
                }
            )
            await asyncio.sleep(0.1)  # Send command every 100ms
    
    except Exception as e:
        print(f"Turn error: {e}")
    
    finally:
        await stop_move(conn)
    
    return

async def toggle_light_on(conn, brightness=10):
    """ Args:
    conn: connection of robot
    brightness: set brightness, default 10 (max)
    """
    """set brightness"""
    # set brightness (range 0–10)
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"],
        {"api_id": 1005, "parameter": {"brightness": brightness}}
    )
    """Toggle the light of the robot"""
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"], 
        {
            "api_id": 1007,
            "parameter": 
            {
                "color": VUI_COLOR.WHITE,
            }
        }
    )
    return

async def toggle_light_off(conn):
    """
    Args:
    conn: connection to the robot
    Sets light to off
    """
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"],
        {"api_id":1005, "parameter": {"brightness": 0}}
    )
    return



