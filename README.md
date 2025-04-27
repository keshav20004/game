🕹️ Virtual Steering Wheel Controller
Control car games using just your hands via webcam tracking!
Built with OpenCV, MediaPipe, and pyautogui.

🚀 Features
Real-time hand tracking (1 or 2 hands)

Smooth steering based on wrist positions

Auto acceleration (up key hold)

Minimal CPU load (frame skipping, resizing)

Visual steering wheel + angle/status display

📦 Requirements
bash
Copy
Edit
pip install opencv-python mediapipe numpy pyautogui
🛠️ How it works
Track wrists (or wrist + middle finger if single hand).

Compute steering angle relative to center.

Press left/right arrow keys based on angle.

Always accelerate forward.

Visual feedback with steering wheel overlay.

🎮 Usage
Connect a webcam.

Run:

bash
Copy
Edit
python steering_control.py
Switch to your game window within 3 seconds.

Use your hands to steer!

Quit anytime: Press Q.

⚙️ Settings
STEERING_RADIUS: Visual wheel size.

STEERING_THRESHOLD: How much tilt triggers a turn.

PROCESS_EVERY_N_FRAMES: Lower for faster response, higher for CPU saving.

📋 Notes
Works best with simple driving games.

Lower model_complexity in Hands for speed over accuracy.

Make sure game is focused (pyautogui sends keys to active window).
