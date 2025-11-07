import asyncio
import logging
import json
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
from functions import *

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

"""Maze path:
forward
turn right
forward
turn right
forward
turn right
forward
turn right
forward
jump"""
async def main():
    """Mian function to navigate robot through maze in predetermined path"""
    conn = None
    try:
        conn = await connect_to_robot()
        await start_up(conn)

        await toggle_light_off(conn)
        #first move
        await move(conn, 3.0)
        await asyncio.sleep(0.1)
        #through tunnel
        await turn(conn, -75)
        await toggle_light_on(conn)
        await move(conn, 5.5)
        await toggle_light_off(conn)
        #over ramp
        await turn(conn, -121)
        await move(conn,3.6)
        #walk to hurdle
        await turn(conn, -75)
        await move(conn, 4.9)
        #jump over hurdle
        await jump_forward(conn)

        await asyncio.sleep(1)


        await conn.disconnect()
    except ValueError as e:
        # Log any value errors that occur during the process.
        logging.error(f"An error occurred: {e}")
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        if conn is not None:
            try:
                await stop_move(conn)
            except Exception:
                pass
            try:
                await conn.disconnect()
            except Exception:
                pass

if __name__ == "__main__":
    asyncio.run(main())
