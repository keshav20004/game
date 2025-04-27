import cv2
import mediapipe as mp
import numpy as np
import math
import pyautogui
import time

# Initialize MediaPipe Hands with optimized configuration
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=2, 
    min_detection_confidence=0.5, 
    min_tracking_confidence=0.3,  # Lower tracking confidence for smoother tracking
    model_complexity=0  # Use the lighter model for better performance
)
mp_draw = mp.solutions.drawing_utils

# Steering parameters
STEERING_RADIUS = 150
STEERING_THRESHOLD = 15
DEAD_ZONE = 5
COOLDOWN = 0.05

# Frame processing control
PROCESS_EVERY_N_FRAMES = 2  # Process every 2nd frame
FRAME_RESIZE_FACTOR = 0.75  # Resize frames to 75% for processing

def get_steering_angle(landmarks, frame_width, frame_height):
    # Use two hands if available, otherwise fall back to one hand
    if len(landmarks) >= 2:
        # Use the wrist landmarks of both hands
        left_wrist = landmarks[0].landmark[0]  # Wrist of first hand
        right_wrist = landmarks[1].landmark[0]  # Wrist of second hand
        left_x = left_wrist.x * frame_width
        left_y = left_wrist.y * frame_height
        right_x = right_wrist.x * frame_width
        right_y = right_wrist.y * frame_height
        center_x = (left_x + right_x) / 2
        center_y = (left_y + right_y) / 2
        dx = right_x - left_x
        dy = right_y - left_y
    else:
        # Fallback to single hand (wrist to middle finger)
        wrist = landmarks[0].landmark[0]
        middle_tip = landmarks[0].landmark[12]
        center_x = wrist.x * frame_width
        center_y = wrist.y * frame_height
        dx = (middle_tip.x - wrist.x) * frame_width
        dy = (middle_tip.y - wrist.y) * frame_height

    # Calculate angle
    angle = math.degrees(math.atan2(dy, dx))
    # Adjust angle for natural hand orientation (positive for right, negative for left)
    angle = angle - 90
    if angle > 180:
        angle -= 360
    if angle < -180:
        angle += 360

    return max(-45, min(45, angle)), (int(center_x), int(center_y))

def draw_steering_wheel(frame, angle, center):
    # Draw steering wheel (circle with cross, similar to the video)
    cv2.circle(frame, center, STEERING_RADIUS, (0, 255, 0), 2)
    cv2.line(frame, (center[0] - STEERING_RADIUS//2, center[1]),
             (center[0] + STEERING_RADIUS//2, center[1]), (0, 255, 0), 2)
    cv2.line(frame, (center[0], center[1] - STEERING_RADIUS//2),
             (center[0], center[1] + STEERING_RADIUS//2), (0, 255, 0), 2)
    angle_rad = math.radians(angle)
    end_x = int(center[0] + (STEERING_RADIUS - 20) * math.cos(angle_rad))
    end_y = int(center[1] + (STEERING_RADIUS - 20) * math.sin(angle_rad))
    cv2.line(frame, center, (end_x, end_y), (0, 0, 255), 2)

def control_game(steering_angle, last_press_time):
    current_time = time.time()
    if current_time - last_press_time < COOLDOWN:
        return last_press_time

    # Release all keys
    pyautogui.keyUp('left')
    pyautogui.keyUp('right')
    pyautogui.keyUp('up')
    pyautogui.keyDown('up')  # Continuous forward

    # Steering logic with debug output
    status = "steer: keeping straight"
    if abs(steering_angle) > DEAD_ZONE:
        if steering_angle < -STEERING_THRESHOLD:
            pyautogui.keyDown('left')
            status = "steer: turning left"
        elif steering_angle > STEERING_THRESHOLD:
            pyautogui.keyDown('right')
            status = "steer: turning right"
    
    print(f"Angle: {steering_angle:.2f} | Status: {status}")
    return current_time

def main():
    cap = cv2.VideoCapture(1)
    
    # Set lower FPS for better performance
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # Increased buffer size
    
    last_press_time = 0
    frame_count = 0
    last_angle = 0  # Store last valid angle for smoother transitions
    
    print("Starting in 3 seconds... Switch to your game window!")
    time.sleep(3)
    
    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                time.sleep(0.1)  # Wait a bit and try again
                continue
            
            frame_count += 1
            frame = cv2.flip(frame, 1)
            
            # Always display the current frame for visual feedback
            display_frame = frame.copy()
            frame_height, frame_width, _ = frame.shape
            center = (frame_width // 2, frame_height // 2)  # Default center
            
            # Process only every N frames to reduce CPU load
            if frame_count % PROCESS_EVERY_N_FRAMES == 0:
                # Resize frame for faster processing
                process_frame = cv2.resize(frame, (0, 0), fx=FRAME_RESIZE_FACTOR, fy=FRAME_RESIZE_FACTOR)
                rgb_frame = cv2.cvtColor(process_frame, cv2.COLOR_BGR2RGB)
                
                # Process with non-blocking
                results = hands.process(rgb_frame)
                
                if results.multi_hand_landmarks:
                    # Scale up coordinates to match original frame size
                    scaled_landmarks = results.multi_hand_landmarks
                    for hand_landmarks in scaled_landmarks:
                        for landmark in hand_landmarks.landmark:
                            landmark.x /= FRAME_RESIZE_FACTOR
                            landmark.y /= FRAME_RESIZE_FACTOR
                    
                    steering_angle, center = get_steering_angle(scaled_landmarks, frame_width, frame_height)
                    last_angle = steering_angle  # Update last known angle
                    last_press_time = control_game(steering_angle, last_press_time)
                    
                    # Draw landmarks on display frame
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_draw.draw_landmarks(display_frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Always draw the steering wheel using the most recent angle
            draw_steering_wheel(display_frame, last_angle, center)
            
            # Display status on frame
            status = "steer: keeping straight"
            if abs(last_angle) > DEAD_ZONE:
                if last_angle < -STEERING_THRESHOLD:
                    status = "steer: turning left"
                elif last_angle > STEERING_THRESHOLD:
                    status = "steer: turning right"
            
            cv2.putText(display_frame, f"Angle: {int(last_angle)}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"Status: {status}", (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"FPS: {int(cap.get(cv2.CAP_PROP_FPS))}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, "Q to quit", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow("Virtual Steering Control", display_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pyautogui.keyUp('left')
                pyautogui.keyUp('right')
                pyautogui.keyUp('up')
                break
                
        except Exception as e:
            print(f"Error: {e}")
            continue
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()