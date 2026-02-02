import paramiko
import time

# --- CONFIGURATION ---
ROBOT_IP = "169.254.200.200"
USER = "niryo"
PASS = "robotics"
SETUP_FILE = "/home/niryo/catkin_ws/install/release/ned2/setup.bash"
TOPIC = "/niryo_robot_vision/compressed_video_stream"

# --- THE FIX: ADDED TIMEOUT ---
# I changed 'rospy.spin()' (infinite loop) to 'rospy.wait_for_message' (smart wait)
ROBOT_SCRIPT = f"""
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

rospy.init_node('remote_snapper', anonymous=True)

print('Attempting to capture one image...')
try:
    # WAIT MAX 5 SECONDS. If no image, it crashes intentionally so we know.
    msg = rospy.wait_for_message('{TOPIC}', CompressedImage, timeout=5.0)

    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imwrite('remote_capture.jpg', img)
    print('SUCCESS: Image captured.')

except rospy.ROSException:
    print('FAILURE: Timeout. The camera is not publishing data.')
except Exception as e:
    print(f'FAILURE: {{e}}')
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

    # --- THE FIX: VERIFY CAMERA START ---
    print("✅ Connected. Enabling camera stream...")
    cmd = f"source {SETUP_FILE} && rosservice call /niryo_robot_vision/start_stop_video_streaming '{{value: true}}'"
    stdin, stdout, stderr = client.exec_command(cmd)

    # Wait for the command to finish and read the output
    exit_status = stdout.channel.recv_exit_status()
    output = stdout.read().decode().strip()

    if exit_status != 0 or "success: True" not in output:
        print(f"❌ CAMERA FAILED TO START. Robot said:\n{output}\n{stderr.read().decode()}")
        client.close()
        return
    else:
        print("✅ Camera service started successfully.")

    time.sleep(3) # Wait for camera warm-up

    print("📸 Running capture script...")
    sftp = client.open_sftp()
    with sftp.file("temp_snap.py", "w") as f:
        f.write(ROBOT_SCRIPT)

    stdin, stdout, stderr = client.exec_command(f"source {SETUP_FILE} && python temp_snap.py")

    # Read what happened
    result = stdout.read().decode()
    error_log = stderr.read().decode()

    print(f"--- Robot Output ---\n{result}\n--------------------")

    if "FAILURE" in result or "Traceback" in error_log:
        print("❌ Capture failed. Check the output above.")
    else:
        print("⬇️ Downloading image...")
        try:
            sftp.get("remote_capture.jpg", "final_result.jpg")
            print("🎉 SUCCESS! Saved as 'final_result.jpg'")
        except Exception as e:
            print(f"❌ Could not download file: {e}")

    sftp.close()
    client.close()

if __name__ == "__main__":
    take_remote_snapshot()
