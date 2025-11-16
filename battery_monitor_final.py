#Battery Monitoring function 3 - editing audio - script that works

import asyncio
import logging
import json
import os
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.webrtc_audiohub import WebRTCAudioHub
from go2_webrtc_driver.constants import RTC_TOPIC, MCF_CMD

# Threshold for low battery (percent)
LOW_BATTERY_THRESHOLD = 90
# Hysteresis to avoid flapping
BATTERY_RECOVER_THRESHOLD = 95

# Audio file to play when battery is low
AUDIO_FILE_NAME = "dog-barking.wav"  # bundled example file in repo

# Configure logging (allow enabling debug via env var)
log_level = logging.DEBUG if os.environ.get("DEBUG_BATTERY") == "1" else logging.INFO
logging.basicConfig(level=log_level)
logger = logging.getLogger(__name__)

# Suppress verbose JSON logging from WebRTC driver modules
logging.getLogger("go2_webrtc_driver").setLevel(logging.WARNING)
logging.getLogger("go2_webrtc_driver.webrtc_datachannel").setLevel(logging.WARNING)
logging.getLogger("go2_webrtc_driver.msgs.pub_sub").setLevel(logging.WARNING)



async def main(ip: str | None = None):
    """Start the battery monitor.

    ip: optional Go2 device IP. If None, reads from GO2_IP env or defaults to 192.168.12.1
    """
    triggered = False

    if ip is None:
        ip = os.environ.get('GO2_IP', '192.168.12.1')

    conn = None
    try:
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip)
        logger.info("Connecting to Go2 via WebRTC (ip=%s)...", ip)
        await conn.connect()
        logger.info("Connected to Go2 via WebRTC")

        # Create audio hub instance
        audio_hub = WebRTCAudioHub(conn, logger)
        logger.info("Audio hub initialized")

        # Define audio file to upload and play
        audio_file = "dog-barking.wav"
        audio_file_path = os.path.join(os.path.dirname(__file__), audio_file)
        logger.info(f"Using audio file: {audio_file_path}")

        # Get the list of available audio files
        response = await audio_hub.get_audio_list()
        if response and isinstance(response, dict):
            data_str = response.get('data', {}).get('data', '{}')
            audio_list = json.loads(data_str).get('audio_list', []) 

            # Extract filename without extension
            filename = os.path.splitext(audio_file)[0]
            print(audio_list)
            # Check if file already exists by CUSTOM_NAME and store UUID
            existing_audio = next((audio for audio in audio_list if audio['CUSTOM_NAME'] == filename), None)
            if existing_audio:
                print(f"Audio file {filename} already exists, skipping upload")
                uuid = existing_audio['UNIQUE_ID']
            else:
                print(f"Audio file {filename} not found, proceeding with upload")
                uuid = None

                # Upload the audio file
                logger.info("Starting audio file upload...")
                await audio_hub.upload_audio_file(audio_file_path)
                logger.info("Audio file upload completed")
                response = await audio_hub.get_audio_list()
                existing_audio = next((audio for audio in audio_list if audio['CUSTOM_NAME'] == filename), None)
                uuid = existing_audio['UNIQUE_ID']                   

        # callback for lowstate updates
        def lowstate_callback(message):
            nonlocal triggered, uuid
            try:
                current_message = message['data']
                bms = current_message.get('bms_state', {})
                soc = bms.get('soc')
                if soc is None:
                    logger.debug("No SOC found in BMS state")
                    return

                logger.info(f"\033[32m\033[1mBattery SOC:  {soc}%\033[0m")

                if soc <= LOW_BATTERY_THRESHOLD and not triggered:
                    logger.info(f"\033[31m\033[1mBattery low ({soc}%). Triggering audio + lie-down\033[0m")

                    async def play_and_command():
                        try:
                            #play audio
                            print(f"Starting audio playback of file: {uuid}")
                            await audio_hub.play_by_uuid(uuid)
                            logger.info("Audio playback completed")

                            # Send lie-down / stand down command
                            logger.info("Sending StandDown command to robot")
                            await conn.datachannel.pub_sub.publish_request_new(
                                RTC_TOPIC["SPORT_MOD"],
                                {"api_id": MCF_CMD["StandDown"]}
                            )
                        except Exception:
                            logger.exception("Error during low-battery actions")

                    # schedule the coroutine
                    asyncio.create_task(play_and_command())
                    triggered = True

                # Reset triggered when battery recovers above hysteresis threshold
                if triggered and soc >= BATTERY_RECOVER_THRESHOLD:
                    logger.info(f"Battery recovered to {soc}%; resetting low-battery trigger")
                    triggered = False

            except Exception:
                logger.exception("Error processing lowstate message")

        # Subscribe to lowstate topic
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LOW_STATE'], lowstate_callback)
        logger.info("Subscribed to %s for lowstate updates", RTC_TOPIC['LOW_STATE'])

        # If FORCE_TRIGGER env var is set, simulate a lowstate message to validate actions
        if os.environ.get('FORCE_TRIGGER') == '1':
            logger.info("FORCE_TRIGGER enabled — simulating lowstate message")
            fake_msg = {
                'data': {
                    'bms_state': {'soc': LOW_BATTERY_THRESHOLD - 1},
                }
            }
            # call synchronously to exercise callback
            lowstate_callback(fake_msg)

        # Keep running indefinitely
        while True:
            await asyncio.sleep(3600)

    except Exception:
        logger.exception("Unhandled error in battery monitor")
    finally:
        if conn:
            try:
                await conn.disconnect()
            except Exception:
                pass


if __name__ == '__main__':
    try:
        ip_arg = None
        # allow passing ip as first arg
        if len(sys.argv) > 1:
            ip_arg = sys.argv[1]
        asyncio.run(main(ip=ip_arg))
    except KeyboardInterrupt:
        logger.info("Battery monitor stopped by user")
