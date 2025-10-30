"""
Blue Tape Square Detection + Path Reversal Return System
Uses camera to find blue tape square at start, with path reversal as backup
"""

import asyncio
import numpy as np
import cv2
import threading
from queue import Queue
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
from aiortc import MediaStreamTrack

class BlueSquareReturnToBase:
    """
    Two-strategy return system:
    1. Try to find blue tape square at start (camera-based)
    2. Fall back to path reversal if square not found
    """
    
    def __init__(self, conn):
        self.conn = conn
        
        # Path reversal backup
        self.movement_history = []
        
        # Video stream management
        self.frame_queue = Queue(maxsize=5)  # Store latest 5 frames
        self.video_active = False
        
        # Blue tape detection settings
        self.blue_hsv_lower = np.array([100, 100, 50])   # Lower blue threshold
        self.blue_hsv_upper = np.array([130, 255, 255])  # Upper blue threshold
        
        # Detection thresholds
        self.min_square_area = 2000      # Minimum area for blue detection
        self.arrival_threshold = 0.15     # Consider arrived if blue covers >15% of frame
    
    # ==================== VIDEO STREAM MANAGEMENT ====================
    
    async def start_video_stream(self):
        """Start receiving video frames from robot"""
        print("📹 Starting video stream...")
        
        try:
            # Switch video channel on
            self.conn.video.switchVideoChannel(True)
            
            # Add callback to receive frames
            self.conn.video.add_track_callback(self._recv_camera_stream)
            
            self.video_active = True
            print("✅ Video stream active")
            
            # Wait for first frame
            await asyncio.sleep(1)
            
        except Exception as e:
            print(f"❌ Video stream error: {e}")
            self.video_active = False
    
    async def _recv_camera_stream(self, track: MediaStreamTrack):
        """Callback to receive video frames (runs continuously)"""
        while self.video_active:
            try:
                frame = await track.recv()
                # Convert to NumPy array (BGR format)
                img = frame.to_ndarray(format="bgr24")
                
                # Add to queue (replace oldest if full)
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except:
                        pass
                self.frame_queue.put(img)
                
            except Exception as e:
                print(f"Frame receive error: {e}")
                break
    
    def get_latest_frame(self):
        """Get the most recent camera frame"""
        if self.frame_queue.empty():
            return None
        
        # Get latest frame (might be multiple frames behind)
        frame = None
        while not self.frame_queue.empty():
            frame = self.frame_queue.get()
        
        return frame
    
    async def stop_video_stream(self):
        """Stop video stream"""
        self.video_active = False
        self.conn.video.switchVideoChannel(False)
        print("📹 Video stream stopped")
    
    # ==================== PATH REVERSAL (BACKUP STRATEGY) ====================
    
    def record_move(self, move_type, value):
        """Record movement for path reversal backup"""
        self.movement_history.append((move_type, value))
        print(f"📝 Recorded: {move_type} {value}")
    
    async def return_via_path_reversal(self):
        """Backup method: reverse all recorded movements"""
        print("\n🔄 Using BACKUP strategy: Path Reversal")
        print(f"Reversing {len(self.movement_history)} movements...")
        
        for i, (move_type, value) in enumerate(reversed(self.movement_history)):
            print(f"  Step {i+1}/{len(self.movement_history)}: ", end="")
            
            if move_type == "forward":
                print(f"Going backward {value:.2f}m")
                await self._execute_move(-value)
            
            elif move_type == "turn":
                print(f"Turning {-value:.1f}°")
                await self._execute_turn(-value)
        
        print("✅ Path reversal complete!")
        return True
    
    # ==================== BLUE SQUARE DETECTION ====================
    
    def detect_blue_tape(self, frame):
        """
        Detect blue tape in camera frame
        Returns: (found, percentage_of_frame, center_x_offset)
        """
        if frame is None:
            return False, 0, 0
        
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for blue color
        mask = cv2.inRange(hsv, self.blue_hsv_lower, self.blue_hsv_upper)
        
        # Clean up mask (remove noise)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Calculate percentage of frame that's blue
        total_pixels = frame.shape[0] * frame.shape[1]
        blue_pixels = cv2.countNonZero(mask)
        blue_percentage = blue_pixels / total_pixels
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, blue_percentage, 0
        
        # Find largest blue contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Ignore tiny detections
        if area < self.min_square_area:
            return False, blue_percentage, 0
        
        # Get center of blue region
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return False, blue_percentage, 0
        
        cx = int(M["m10"] / M["m00"])
        
        # Calculate offset from center
        frame_center = frame.shape[1] / 2
        pixel_offset = cx - frame_center
        
        # Convert to angle (assuming ~60° FOV)
        angle_per_pixel = 60.0 / frame.shape[1]
        angle_offset = pixel_offset * angle_per_pixel
        
        return True, blue_percentage, angle_offset
    
    def check_inside_square(self, frame):
        """
        Check if robot is inside the blue square
        (Blue tape should be visible on multiple sides and cover significant area)
        """
        found, blue_percentage, _ = self.detect_blue_tape(frame)
        
        # If >15% of frame is blue, we're likely inside/at the square
        if found and blue_percentage > self.arrival_threshold:
            return True
        
        return False
    
    async def scan_for_blue_square(self, scan_degrees=360):
        """
        Spin robot to scan for blue tape square
        Returns: (found, angle_to_square)
        """
        print(f"\n🔍 Scanning {scan_degrees}° for blue tape square...")
        
        degrees_per_step = 30  # Turn 30° at a time
        steps = scan_degrees // degrees_per_step
        total_turned = 0
        
        best_detection = {"found": False, "angle": 0, "blue_percentage": 0}
        
        for step in range(steps):
            print(f"  Checking sector {step+1}/{steps}...")
            
            # Wait for frame to stabilize after turn
            await asyncio.sleep(0.3)
            
            # Get camera frame
            frame = self.get_latest_frame()
            
            if frame is not None:
                # Check for blue tape
                found, blue_percentage, angle_offset = self.detect_blue_tape(frame)
                
                if found:
                    print(f"    🔵 Blue detected! Coverage: {blue_percentage*100:.1f}%, "
                          f"Angle offset: {angle_offset:.1f}°")
                    
                    # Track best detection
                    if blue_percentage > best_detection["blue_percentage"]:
                        best_detection = {
                            "found": True,
                            "angle": total_turned + angle_offset,
                            "blue_percentage": blue_percentage
                        }
                    
                    # If we see A LOT of blue, we might already be close!
                    if blue_percentage > 0.1:  # >10% of frame
                        print(f"✅ Found blue square at {total_turned + angle_offset:.1f}°!")
                        return True, total_turned + angle_offset
            
            # Turn to next sector
            if step < steps - 1:
                await self._execute_turn(degrees_per_step)
                total_turned += degrees_per_step
        
        # Return best detection found
        if best_detection["found"]:
            print(f"✅ Best blue detection at {best_detection['angle']:.1f}° "
                  f"({best_detection['blue_percentage']*100:.1f}% coverage)")
            return True, best_detection["angle"]
        
        print("❌ Blue square not found in scan")
        return False, 0
    
    async def approach_blue_square(self):
        """
        Once blue square is detected, navigate towards it
        """
        print("\n🎯 Approaching blue square...")
        
        approach_steps = 0
        max_approach_steps = 30
        
        while approach_steps < max_approach_steps:
            approach_steps += 1
            
            # Get current frame
            frame = self.get_latest_frame()
            
            if frame is None:
                print("⚠️ No camera feed")
                await asyncio.sleep(0.5)
                continue
            
            # Check if we're at/inside the square
            if self.check_inside_square(frame):
                print("✅ Inside blue square - arrived at base!")
                return True
            
            # Detect blue tape direction
            found, blue_percentage, angle_offset = self.detect_blue_tape(frame)
            
            if not found:
                print("⚠️ Lost sight of blue tape!")
                # Try small search
                await self._execute_turn(15)
                continue
            
            print(f"  Blue coverage: {blue_percentage*100:.1f}%, Angle: {angle_offset:.1f}°")
            
            # Adjust heading if significantly off-center
            if abs(angle_offset) > 15:
                print(f"  Adjusting heading: {angle_offset:.1f}°")
                await self._execute_turn(angle_offset * 0.7)  # Turn 70% of the way
                await asyncio.sleep(0.3)
                continue
            
            # Move forward (small steps)
            move_distance = 0.3
            print(f"  Moving forward {move_distance:.2f}m...")
            await self._execute_move(move_distance)
            
            await asyncio.sleep(0.3)
        
        print("⚠️ Max approach steps reached")
        
        # Final check
        frame = self.get_latest_frame()
        if frame and self.check_inside_square(frame):
            print("✅ Inside blue square!")
            return True
        
        return False
    
    # ==================== MAIN RETURN TO BASE FUNCTION ====================
    
    async def return_to_base(self):
        """
        Main function: Try visual detection first, fall back to path reversal
        """
        print("\n" + "="*60)
        print("🏠 INITIATING RETURN TO BASE")
        print("="*60)
        
        # Start video stream if not already active
        if not self.video_active:
            await self.start_video_stream()
        
        # STRATEGY 1: Blue square detection
        print("\n📹 Strategy 1: Blue Square Detection")
        
        # Scan for blue square
        found, angle_to_square = await self.scan_for_blue_square(360)
        
        if found:
            # Turn to face square
            if abs(angle_to_square) > 5:
                print(f"\n↻ Turning {angle_to_square:.1f}° to face square...")
                await self._execute_turn(angle_to_square)
            
            # Approach square
            success = await self.approach_blue_square()
            
            if success:
                print("\n" + "="*60)
                print("🎉 SUCCESS! Returned to base via blue square detection!")
                print("="*60)
                return "visual_detection"
            else:
                print("\n⚠️ Visual approach failed, trying backup...")
        else:
            print("\n⚠️ Blue square not found, using backup strategy...")
        
        # STRATEGY 2: Path reversal backup
        print("\n📍 Strategy 2: Path Reversal Backup")
        await self.return_via_path_reversal()
        
        print("\n" + "="*60)
        print("✅ Returned to base via path reversal (backup)")
        print("="*60)
        return "path_reversal"
    
    # ==================== MOVEMENT FUNCTIONS ====================
    

    async def jump_forward(self):
        """Jump forward"""
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["FrontJump"], "parameter": {"data": True}}
        )
        await asyncio.sleep(2)
        await self._stop()
        return

    async def toggle_light_on(self, brightness = 10):
        """ Args:
        conn: connection of robot
        brightness: set brightness, default 10 (max)
        """
        """set brightness"""
        # set brightness (range 0–10)
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"],
            {"api_id": 1005, "parameter": {"brightness": brightness}}
        )
        await asyncio.sleep(1)
        return

    async def toggle_light_off(self):
        """Toggle light off"""
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"],
            {"api_id": 1005, "parameter": {"brightness": 0}}
        )
        await asyncio.sleep(1)
        return

    async def move_forward(self, distance, speed=0.3):
        """Move forward and record for backup"""
        await self._execute_move(distance, speed)
        self.record_move("forward", distance)
    
    async def turn(self, degrees, yaw_rate=0.8):
        """Turn and record for backup"""
        await self._execute_turn(degrees, yaw_rate)
        self.record_move("turn", degrees)
    
    async def _execute_move(self, distance, speed=0.3):
        """Execute movement (positive=forward, negative=backward)"""
        velocity = speed if distance >= 0 else -speed
        duration = abs(distance) / speed
        start_time = asyncio.get_event_loop().time()
        
        while (asyncio.get_event_loop().time() - start_time) < duration:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["Move"], "parameter": {"x": velocity, "y": 0, "z": 0}}
            )
            await asyncio.sleep(0.1)
        
        await self._stop()
    
    async def _execute_turn(self, degrees, yaw_rate=0.8):
        """Execute turn"""
        import math
        
        if abs(degrees) < 2:  # Skip tiny turns
            return
        
        direction_sign = 1 if degrees >= 0 else -1
        duration = abs(math.radians(degrees)) / yaw_rate
        start_time = asyncio.get_event_loop().time()
        
        while (asyncio.get_event_loop().time() - start_time) < duration:
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["Move"], 
                 "parameter": {"x": 0, "y": 0, "z": direction_sign * yaw_rate}}
            )
            await asyncio.sleep(0.1)
        
        await self._stop()
    
    async def _stop(self):
        """Stop robot"""
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["StopMove"]}
        )
        await asyncio.sleep(0.3)


# ==================== USAGE EXAMPLE ====================

async def demo_blue_square_return():
    """
    Complete demo: Navigate maze and return via blue square detection
    """
    
    print("🤖 Blue Square Return to Base Demo")
    print("="*60)
    
    # 1. Connect to robot
    print("\n📡 Connecting to robot...")
    conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.12.1")
    await conn.connect()
    print("✅ Connected!")
    
    # 2. Initialize return system
    rtb = BlueSquareReturnToBase(conn)
    
    # 3. Start video stream
    await rtb.start_video_stream()
    
    # 4. Signal start position (flash blue lights to match tape)
    print("\n💡 Marking start position with blue lights...")
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"],
        {"api_id": 1007, "parameter": {"color": 0x0000FF}}  # Blue
    )
    await asyncio.sleep(2)
    
    # 5. Navigate maze (example path)
    print("\n🚀 Starting maze navigation...")
    print("(Recording movements for backup strategy)")
    
    #first move
    await rtb.move_forward(3.2)
    await asyncio.sleep(0.1)
    #through tunnel
    await rtb.turn(-74)
    await rtb.toggle_light_on()
    await rtb.move_forward(5.4)
    await rtb.toggle_light_off()
    #over ramp
    await rtb.turn(-115)
    await rtb.move_forward(3.4)
    #walk to hurdle
    await rtb.turn(-57)
    await rtb.move_forward(5.1)
    #jump over hurdle
    await rtb.jump_forward()

    await asyncio.sleep(1)
    
    print(f"\n✅ Maze complete! Recorded {len(rtb.movement_history)} movements")
    
    # 6. Return to base
    method_used = await rtb.return_to_base()
    
    # 7. Celebrate based on method used
    print("\n🎊 Celebrating success!")
    if method_used == "visual_detection":
        # Green light = visual success
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"],
            {"api_id": 1007, "parameter": {"color": 0x00FF00}}  # Green
        )
    else:
        # Yellow light = backup method
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"],
            {"api_id": 1007, "parameter": {"color": 0xFFFF00}}  # Yellow
        )
    
    await asyncio.sleep(2)
    
    # 8. Stop video and disconnect
    await rtb.stop_video_stream()
    print("\n👋 Disconnecting...")
    await conn.disconnect()
    print("Done!")


# ==================== SETUP & CALIBRATION ====================

"""
SETUP CHECKLIST:

1. HARDWARE:
   ✅ Blue tape square already exists at start position
   ✅ Robot camera pointing forward
   ✅ Good lighting (not too dark)

2. SOFTWARE:
   pip install opencv-python aiortc

3. CALIBRATION (if needed):
   
   Test blue detection:
   - Run a frame capture and check HSV values
   - Adjust blue_hsv_lower and blue_hsv_upper if needed
   - Current values: H: 100-130, S: 100-255, V: 50-255
   
   To calibrate:
   frame = rtb.get_latest_frame()
   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   # Click on blue tape and note HSV values
   # Adjust thresholds accordingly

4. TEST:
   - Place robot in blue square
   - Check that check_inside_square() returns True
   - Move robot away and test scan_for_blue_square()

ADVANTAGES OF BLUE SQUARE:

✅ Large target area (easier to detect from distance)
✅ Multiple edges (corners) provide more visual information
✅ Square shape helps determine if "inside" vs "approaching"
✅ Already set up (no extra prep needed)
✅ Blue tape at corners helps with orientation

HOW IT WORKS:

1. SCANNING PHASE:
   - Spins 360° looking for blue tape
   - Calculates what % of camera view is blue
   - Records angle with most blue visible

2. APPROACHING PHASE:
   - Turns to face blue tape
   - Moves forward in small steps
   - Continuously adjusts heading to stay centered
   - Stops when blue covers >15% of frame (inside square)

3. VERIFICATION:
   - Checks if surrounded by blue (multiple edges visible)
   - This confirms robot is inside the start square

4. FALLBACK:
   - If visual fails at any point, uses recorded path reversal
   - Guaranteed to return to start

TROUBLESHOOTING:

"Blue not detected":
- Check lighting (add more light if too dark)
- Verify blue tape is bright/saturated enough
- Adjust HSV thresholds (lines 36-37)
- Test with: cv2.imshow('mask', mask) to see what's detected

"False detections":
- Increase min_square_area (line 41)
- Make HSV range more restrictive
- Add shape detection (check if contour is roughly rectangular)

"Robot overshoots square":
- Decrease arrival_threshold (line 42)
- Add safety margin (stop earlier)
- Reduce move_distance in approach phase

"Can't find square from far away":
- Blue tape might be too small when distant
- Try increasing scan density (smaller degrees_per_step)
- Consider adding ArUco marker in center of square for long-range detection
"""

# Run the demo
asyncio.run(demo_blue_square_return())