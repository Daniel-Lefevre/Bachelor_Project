import paramiko
import time

# --- CONFIGURATION ---
ROBOT_IP = "169.254.200.200"
USER = "niryo"
PASS = "robotics"
# The setup file path we found earlier
SETUP_FILE = "/home/niryo/catkin_ws/install/release/ned2/setup.bash"
# The specific camera topic for your robot version
TOPIC = "/niryo_robot_vision/compressed_video_stream"
# ---------------------

# The Python code we will run INSIDE the robot
# We inject this code directly so we don't rely on files existing there
ROBOT_SCRIPT = f"""
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

def callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite('remote_capture.jpg', img)
    rospy.signal_shutdown('Done')

rospy.init_node('remote_snapper', anonymous=True)
rospy.Subscriber('{TOPIC}', CompressedImage, callback)
print('Waiting for image...')
rospy.spin()
"""

def take_remote_snapshot():
    print(f"--- Connecting to {ROBOT_IP} ---")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(ROBOT_IP, username=USER, password=PASS)
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return

    print("✅ Connected. Turning on camera stream...")
    # 1. Wake up the camera (using the command that worked for you)
    stdin, stdout, stderr = client.exec_command(
        f"source {SETUP_FILE} && rosservice call /niryo_robot_vision/start_stop_video_streaming '{{value: true}}'"
    )
    time.sleep(2) # Give it a moment to wake up

    print("📸 Taking snapshot on robot...")
    # 2. Save the python code to a temporary file
    sftp = client.open_sftp()
    with sftp.file("temp_snap.py", "w") as f:
        f.write(ROBOT_SCRIPT)
    
    # 3. Run the script on the robot
    stdin, stdout, stderr = client.exec_command(f"source {SETUP_FILE} && python temp_snap.py")
    
    # Check for errors
    err = stderr.read().decode()
    if err and "Traceback" in err:
        print(f"❌ Error running script:\n{err}")
        client.close()
        return

    print("⬇️  Downloading image to your computer...")
    # 4. Download the resulting image
    try:
        sftp.get("remote_capture.jpg", "final_result.jpg")
        print("🎉 SUCCESS! Saved as 'final_result.jpg'")
    except FileNotFoundError:
        print("❌ Failed: The robot did not save the image. Is the camera plugged in?")
    
    # Cleanup
    sftp.close()
    client.close()

if __name__ == "__main__":
    take_remote_snapshot()