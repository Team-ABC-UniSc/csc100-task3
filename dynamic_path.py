"""
Blue Tape Line Following Navigation
Robot follows blue tape path through maze autonomously
"""

import asyncio
import numpy as np
import cv2
from queue import Queue
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
from aiortc import MediaStreamTrack

class LineFollowingNavigator:
    """
    Follow blue tape line through maze
    Simple, reliable, and impressive!
    """
    
    def __init__(self, conn):
        self.conn = conn
        
        # Video stream
        self.frame_queue = Queue(maxsize=5)
        self.video_active = False
        
        # Blue tape detection
        self.blue_hsv_lower = np.array([100, 100, 50])
        self.blue_hsv_upper = np.array([130, 255, 255])
        
        # Line following parameters
        self.target_speed = 0.25  # Move slowly for better tracking
        self.turn_gain = 0.02     # How aggressively to correct direction
        self.min_line_area = 500  # Minimum blue pixels to consider valid line
        
        # State tracking
        self.distance_traveled = 0.0
        self.movements_made = 0
        self.line_lost_counter = 0
    
    # ==================== VIDEO STREAM ====================
    
    async def start_video_stream(self):
        """Start video"""
        print("📹 Starting video stream...")
        try:
            self.conn.video.switchVideoChannel(True)
            self.conn.video.add_track_callback(self._recv_camera_stream)
            self.video_active = True
            await asyncio.sleep(1)
            print("✅ Video active")
        except Exception as e:
            print(f"❌ Video error: {e}")
            return False
        return True
    
    async def _recv_camera_stream(self, track: MediaStreamTrack):
        """Receive frames"""
        while self.video_active:
            try:
                frame = await track.recv()
                img = frame.to_ndarray(format="bgr24")
                
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except:
                        pass
                self.frame_queue.put(img)
            except:
                break
    
    def get_latest_frame(self):
        """Get latest frame"""
        frame = None
        while not self.frame_queue.empty():
            frame = self.frame_queue.get()
        return frame
    
    # ==================== LINE DETECTION ====================
    
    def detect_line(self, frame):
        """
        Detect blue tape line in frame
        Returns: (line_found, center_x_offset, line_area)
        
        center_x_offset: How far left/right the line is from center
                        Negative = line is left, turn left
                        Positive = line is right, turn right
                        0 = line is centered, go straight
        """
        if frame is None:
            return False, 0, 0
        
        height, width = frame.shape[:2]
        
        # Only look at bottom half of frame (where line is visible)
        roi_height = int(height * 0.6)  # Bottom 60% of image
        roi = frame[roi_height:, :]
        
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create blue mask
        mask = cv2.inRange(hsv, self.blue_hsv_lower, self.blue_hsv_upper)
        
        # Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Calculate line position
        blue_pixels = cv2.countNonZero(mask)
        
        if blue_pixels < self.min_line_area:
            return False, 0, 0
        
        # Find the center of the blue line
        # Use moments to find centroid
        M = cv2.moments(mask)
        
        if M["m00"] == 0:
            return False, 0, 0
        
        # Center x position of line
        line_center_x = int(M["m10"] / M["m00"])
        
        # Frame center
        frame_center_x = width // 2
        
        # Calculate offset from center
        # Positive = line is to the right
        # Negative = line is to the left
        offset = line_center_x - frame_center_x
        
        return True, offset, blue_pixels
    
    def check_at_end(self, frame):
        """
        Check if at end of line (blue square start/finish)
        Large blue area = we're at the square
        """
        if frame is None:
            return False
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_hsv_lower, self.blue_hsv_upper)
        
        total_pixels = frame.shape[0] * frame.shape[1]
        blue_pixels = cv2.countNonZero(mask)
        blue_percentage = blue_pixels / total_pixels
        
        # If >20% of frame is blue, we're at the square
        return blue_percentage > 0.2
    
    # ==================== LINE FOLLOWING LOGIC ====================
    
    async def follow_line(self, max_time=120, max_distance=30):
        """
        Main line following algorithm
        Robot continuously adjusts to stay on the blue tape
        
        Args:
            max_time: Maximum time in seconds (safety limit)
            max_distance: Maximum distance in meters (safety limit)
        """
        print("\n" + "="*60)
        print("🚗 STARTING LINE FOLLOWING NAVIGATION")
        print("="*60)
        print(f"Speed: {self.target_speed} m/s")
        print(f"Max time: {max_time}s")
        print(f"Max distance: {max_distance}m")
        
        start_time = asyncio.get_event_loop().time()
        
        while True:
            # Safety checks
            elapsed = asyncio.get_event_loop().time() - start_time
            
            if elapsed > max_time:
                print("\n⏱️ Time limit reached")
                break
            
            if self.distance_traveled > max_distance:
                print("\n📏 Distance limit reached")
                break
            
            # Get current frame
            frame = self.get_latest_frame()
            
            if frame is None:
                print("⚠️ No camera frame")
                await asyncio.sleep(0.3)
                continue
            
            # Check if at end (blue square)
            if self.check_at_end(frame):
                print("\n🏁 Reached end of path (blue square detected)!")
                await self._stop()
                break
            
            # Detect line
            line_found, offset, area = self.detect_line(frame)
            
            if not line_found:
                # Lost the line!
                self.line_lost_counter += 1
                print(f"⚠️ Line lost! (counter: {self.line_lost_counter})")
                
                if self.line_lost_counter > 10:
                    print("❌ Line lost for too long - stopping")
                    await self._stop()
                    break
                
                # Stop and search
                await self._stop()
                await asyncio.sleep(0.3)
                continue
            
            # Reset lost counter
            self.line_lost_counter = 0
            
            # Calculate steering
            # offset is in pixels, convert to turning speed
            turn_speed = -offset * self.turn_gain  # Negative because left is negative
            
            # Limit turn speed
            turn_speed = np.clip(turn_speed, -0.3, 0.3)
            
            # Status update every 20 movements
            if self.movements_made % 20 == 0:
                print(f"📍 Distance: {self.distance_traveled:.2f}m | "
                      f"Line offset: {offset:+4.0f} px | "
                      f"Turn: {turn_speed:+.3f}")
            
            # Send movement command
            # Move forward while turning to stay on line
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {
                    "api_id": SPORT_CMD["Move"],
                    "parameter": {
                        "x": self.target_speed,  # Forward speed
                        "y": 0,                   # No lateral movement
                        "z": turn_speed          # Turning speed (positive = right)
                    }
                }
            )
            
            # Update tracking
            self.movements_made += 1
            self.distance_traveled += self.target_speed * 0.1  # Approximate
            
            await asyncio.sleep(0.1)  # 10Hz update rate
        
        # Stop at end
        await self._stop()
        
        print("\n" + "="*60)
        print("✅ LINE FOLLOWING COMPLETE")
        print("="*60)
        print(f"Distance traveled: {self.distance_traveled:.2f}m")
        print(f"Time elapsed: {elapsed:.1f}s")
        print(f"Commands sent: {self.movements_made}")
        
        return True
    
    # ==================== RETURN TO START ====================
    
    async def return_to_start(self):
        """
        Return by following line backwards!
        Just turn around and follow the same line
        """
        print("\n🔄 Turning around to return...")
        
        # Turn 180 degrees
        await self._execute_turn(180)
        
        await asyncio.sleep(1)
        
        # Reset tracking
        self.distance_traveled = 0
        self.movements_made = 0
        self.line_lost_counter = 0
        
        # Follow line back
        print("\n🏠 Following line back to start...")
        await self.follow_line(max_time=120, max_distance=30)
        
        print("\n✅ Returned to start!")
    
    # ==================== HELPER FUNCTIONS ====================
    
    async def _execute_turn(self, degrees, yaw_rate=0.8):
        """Turn the robot"""
        import math
        
        if abs(degrees) < 2:
            return
        
        print(f"↻ Turning {degrees:.0f}°...")
        
        sign = 1 if degrees >= 0 else -1
        duration = abs(math.radians(degrees)) / yaw_rate
        start = asyncio.get_event_loop().time()
        
        while asyncio.get_event_loop().time() - start < duration:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["Move"], "parameter": {"x": 0, "y": 0, "z": sign * yaw_rate}}
            )
            await asyncio.sleep(0.1)
        
        await self._stop()
    
    async def _stop(self):
        """Stop the robot"""
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["StopMove"]}
        )
        await asyncio.sleep(0.2)
    
    async def stop_video_stream(self):
        """Stop video"""
        self.video_active = False
        self.conn.video.switchVideoChannel(False)


# ==================== DEMO ====================

async def line_following_demo():
    """
    Complete line following demonstration
    """
    
    print("="*60)
    print("  AUTONOMOUS LINE FOLLOWING MAZE NAVIGATION")
    print("  Blue Tape Path Following System")
    print("="*60)
    
    # Connect
    print("\n📡 Connecting to robot...")
    conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.12.1")
    await conn.connect()
    print("✅ Connected!")
    
    # Initialize
    nav = LineFollowingNavigator(conn)
    
    # Start video
    success = await nav.start_video_stream()
    if not success:
        print("❌ Failed to start video")
        await conn.disconnect()
        return
    
    # Signal ready (blue lights)
    print("\n💡 System ready - Blue light active")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"],
        {"api_id": 1007, "parameter": {"color": 0x0000FF}}
    )
    await asyncio.sleep(2)
    
    # Follow line through maze
    print("\n🚀 Beginning autonomous navigation...")
    await nav.follow_line(max_time=120, max_distance=30)
    
    print("\n🎯 Reached destination!")
    
    # Green light for success
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"],
        {"api_id": 1007, "parameter": {"color": 0x00FF00}}
    )
    
    await asyncio.sleep(2)
    
    # Return to start
    print("\n🏠 Returning to base...")
    await nav.return_to_start()
    
    print("\n🎉 Mission complete!")
    
    # Victory jump
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["SPORT_MOD"],
        {"api_id": SPORT_CMD["FrontJump"], "parameter": {"data": True}}
    )
    
    await asyncio.sleep(3)
    
    # Cleanup
    await nav.stop_video_stream()
    await conn.disconnect()
    
    print("\n✅ Demonstration complete!")


# ==================== SIMPLE TEST ====================

async def test_line_detection():
    """
    Quick test to verify line detection works
    Place robot on tape and run this
    """
    
    print("🧪 Testing line detection...")
    
    conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.12.1")
    await conn.connect()
    
    nav = LineFollowingNavigator(conn)
    await nav.start_video_stream()
    
    print("\nWaiting for frames...")
    await asyncio.sleep(2)
    
    for i in range(10):
        frame = nav.get_latest_frame()
        
        if frame is not None:
            line_found, offset, area = nav.detect_line(frame)
            
            print(f"Frame {i+1}: ", end="")
            if line_found:
                direction = "LEFT" if offset < 0 else "RIGHT" if offset > 0 else "CENTER"
                print(f"✅ Line detected | Offset: {offset:+4.0f} px ({direction}) | Area: {area}")
            else:
                print(f"❌ No line detected")
            
            # Save frame for inspection
            if i == 5:
                cv2.imwrite("test_frame.jpg", frame)
                print("   💾 Saved test_frame.jpg")
        
        await asyncio.sleep(0.5)
    
    await nav.stop_video_stream()
    await conn.disconnect()
    
    print("\n✅ Test complete!")


"""
HOW IT WORKS:

1. CAMERA SETUP:
   - Looks at bottom 60% of frame (where tape is visible)
   - Detects blue pixels
   - Calculates center of blue line

2. STEERING LOGIC:
   ┌─────────────────────────┐
   │                         │
   │    Line here →  🔵      │ Offset = +200px → Turn RIGHT
   │                         │
   └─────────────────────────┘
   
   ┌─────────────────────────┐
   │                         │
   │      🔵  ← Line here    │ Offset = -200px → Turn LEFT
   │                         │
   └─────────────────────────┘
   
   ┌─────────────────────────┐
   │                         │
   │          🔵             │ Offset = 0px → Go STRAIGHT
   │                         │
   └─────────────────────────┘

3. CONTROL LOOP:
   - Read frame (10 times per second)
   - Detect line position
   - Calculate correction
   - Move forward + turn simultaneously
   - Repeat!

4. END DETECTION:
   - When blue fills >20% of frame = reached square
   - Stop and celebrate!

5. RETURN:
   - Turn 180°
   - Follow same line back
   - Even easier going back!

ADVANTAGES:

✅ No hardcoded path needed
✅ Works with any tape layout
✅ Self-correcting (always adjusts to stay on line)
✅ Bi-directional (works both ways on same line)
✅ Simpler than corner detection
✅ More impressive than path reversal
✅ Robust to camera noise

SETUP:

1. Lay blue tape in continuous line:
   START [====curved====path====] END
   
2. Make line 2-3 inches wide (easier to see)

3. Avoid:
   - Sharp 90° corners (use curves instead)
   - Gaps in tape
   - Overlapping other blue objects

4. Test with:
   asyncio.run(test_line_detection())

5. Then run full demo:
   asyncio.run(line_following_demo())

CALIBRATION:

If robot drifts off line:
- Increase turn_gain (line 28): Makes corrections stronger
- Decrease target_speed (line 27): Gives more time to correct

If robot oscillates (wiggles):
- Decrease turn_gain: Makes corrections gentler
- Increase target_speed: Less time to overcorrect

If line not detected:
- Adjust HSV values (lines 22-23)
- Check lighting
- Verify tape width
"""

# Run it!
# asyncio.run(test_line_detection())  # Test first
# asyncio.run(line_following_demo())  # Then full demo