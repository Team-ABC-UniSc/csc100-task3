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

        await move(conn,0.5)
        await turn(conn,45)
        await move(conn,0.5)
        await turn(conn,45)
        await move(conn,0.5)
        await turn(conn,45)
        await move(conn,0.5)
        await turn(conn,45)
        await move(conn,0.5)
        await jump_forward(conn)
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