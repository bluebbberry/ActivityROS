import rospy
import requests
import json
from flask import Flask, request
from mastodon import Mastodon
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Mastodon API Credentials
MASTODON_INSTANCE = "https://botsin.space"  # Replace with your Mastodon instance
ACCESS_TOKEN = "YOUR_ACCESS_TOKEN"

# Initialize Mastodon API
mastodon = Mastodon(access_token=ACCESS_TOKEN, api_base_url=MASTODON_INSTANCE)

# Flask Server for receiving ActivityPub commands
app = Flask(__name__)

# ROS Setup
rospy.init_node("activitypub_bridge")
cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Function to Post TurtleBot Status on Mastodon
def post_status(message, hashtags):
    status = f"{message} {' '.join([f'#{tag}' for tag in hashtags])}"
    mastodon.status_post(status)

# Function to Process Incoming Mastodon Commands
def process_commands():
    print("Listening for commands on Mastodon...")
    last_checked = None
    while not rospy.is_shutdown():
        timeline = mastodon.timeline_hashtag("TurtleBot", since_id=last_checked)
        for post in timeline:
            content = post["content"].lower()
            if "#moveforward" in content:
                move_robot(0.5, 0)  # Move forward
            elif "#movebackward" in content:
                move_robot(-0.5, 0)  # Move backward
            elif "#turnleft" in content:
                move_robot(0, 0.5)  # Turn left
            elif "#turnright" in content:
                move_robot(0, -0.5)  # Turn right
            last_checked = post["id"]
        rospy.sleep(5)  # Check Mastodon every 5 seconds

# Function to Move TurtleBot
def move_robot(linear, angular):
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    cmd_pub.publish(cmd)
    post_status(f"Moving: linear={linear}, angular={angular}", ["TurtleBot", "ROS"])

# ROS Subscriber for Position Updates
def odom_callback(msg):
    position = msg.pose.pose.position
    post_status(f"TurtleBot Position: x={position.x:.2f}, y={position.y:.2f}", ["TurtleBot", "ROS"])

rospy.Subscriber("/odom", Odometry, odom_callback)

# Start Command Listening in a Separate Thread
import threading
threading.Thread(target=process_commands, daemon=True).start()

# Run Flask to Receive Webhooks (Optional)
@app.route("/inbox", methods=["POST"])
def inbox():
    data = request.json
    if data and data.get("type") == "Create":
        content = data["object"].get("content", "")
        if "#moveforward" in content:
            move_robot(0.5, 0)
    return "OK", 200

app.run(host="0.0.0.0", port=5000)
