import asyncio
import logging
import json
import os
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.webrtc_audiohub import WebRTCAudioHub
from go2_webrtc_driver.constants import RTC_TOPIC, MCF_CMD

# Threshold for low battery (percent)
LOW_BATTERY_THRESHOLD = 80
# Hysteresis to avoid flapping
BATTERY_RECOVER_THRESHOLD = 85

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





async def ensure_audio_uuid(audio_hub: WebRTCAudioHub, audio_path: str):
    """Ensure the audio file is uploaded to the robot and return its UUID.
    If the file already exists on the device (matched by CUSTOM_NAME), return the existing UUID.
    Otherwise upload and then fetch the list to find the UUID.
    """
    try:
        response = await audio_hub.get_audio_list()
        if not response or not isinstance(response, dict):
            logger.warning("Could not retrieve audio list from device")
            return None

        data_str = response.get('data', {}).get('data', '{}')
        audio_list = json.loads(data_str).get('audio_list', [])

        filename_no_ext = os.path.splitext(os.path.basename(audio_path))[0]
        existing = next((a for a in audio_list if a.get('CUSTOM_NAME') == filename_no_ext), None)
        if existing:
            uuid = existing.get('UNIQUE_ID')
            logger.info(f"Found existing audio on device: {filename_no_ext} (uuid={uuid})")
            return uuid

        # Upload the file
        logger.info(f"Uploading audio file {audio_path} ...")
        await audio_hub.upload_audio_file(audio_path)

        # Re-fetch list
        response = await audio_hub.get_audio_list()
        data_str = response.get('data', {}).get('data', '{}')
        audio_list = json.loads(data_str).get('audio_list', [])
        existing = next((a for a in audio_list if a.get('CUSTOM_NAME') == filename_no_ext), None)
        if existing:
            uuid = existing.get('UNIQUE_ID')
            logger.info(f"Uploaded audio available on device: {filename_no_ext} (uuid={uuid})")
            return uuid

        logger.warning("Uploaded audio not found in list after upload")
        return None

    except Exception as e:
        logger.exception(f"Error ensuring audio uuid: {e}")
        return None


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

        # initialize audio hub helper
        audio_hub = WebRTCAudioHub(conn, logger)

        # Resolve audio file path
        repo_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        audio_file_path = os.path.join(repo_dir, "examples", "audio", "mp3_player", AUDIO_FILE_NAME)
        if not os.path.isfile(audio_file_path):
            logger.warning("Audio file not found at %s; audio will be skipped", audio_file_path)
            audio_file_path = None

        audio_uuid = None
        if audio_file_path:
            audio_uuid = await ensure_audio_uuid(audio_hub, audio_file_path)
        logger.debug("Audio uuid resolved: %s", audio_uuid)

        # callback for lowstate updates
        def lowstate_callback(message):
            nonlocal triggered, audio_uuid
            try:
                current_message = message['data']
                bms = current_message.get('bms_state', {})
                soc = bms.get('soc')
                if soc is None:
                    logger.debug("No SOC found in BMS state")
                    return

                logger.info(f"Battery SOC: {soc}%")

                if soc <= LOW_BATTERY_THRESHOLD and not triggered:
                    logger.info(f"Battery low ({soc}%). Triggering whine + lie-down")

                    async def play_and_command():
                        try:
                            if audio_uuid:
                                logger.info(f"Playing low-battery audio (uuid={audio_uuid})")
                                await audio_hub.play_by_uuid(audio_uuid)
                            else:
                                logger.info("No audio uuid available; skipping playback")

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
