#!/usr/bin/env python
import rospy
from std_msgs.msg import String  # Change to your message type
from geometry_msgs.msg import PoseStamped
import json
import sys

class DataCollector:
    def __init__(self, topic_name, message_type, output_file):
        """
        Initialize the data collector.
        :param topic_name: The name of the ROS topic to subscribe to.
        :param message_type: The type of the ROS message.
        :param output_file: The file path where the data will be saved.
        """
        self.altitude=sys.argv[1]
        self.topic_name = "/uav/gimbal/Target_Relative_Pose"
        self.message_type = PoseStamped
        self.output_file = "/home/jeremie/Master-Thesis/output_files/altitude_{0}.json".format(self.altitude)
        self.data = [[],[]]

        # Initialize ROS node (anonymous=True ensures the node has a unique name, avoiding conflicts)
        rospy.init_node('data_collector_node', anonymous=True)
        
        # Subscribe to the topic
        rospy.Subscriber(self.topic_name, self.message_type, self.callback)
        
        # Control the duration we want to listen for messages
        self.listen_duration = rospy.Duration(9999)  # Listen for 60 seconds, adjust as needed

    def callback(self, msg):
        """
        Callback function for the topic subscriber. It collects data from each message.
        :param msg: The received message.
        """
        self.data[0].append(msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000)
        if msg.header.frame_id=="":
        	self.data[1].append(0)
        	self.save_data()
        	
        if msg.header.frame_id!="":
        	self.data[1].append(1)
        	self.save_data()
        	


    def collect_data(self):
        """
        Collect data from the topic for a specified duration.
        """
        print(f"Starting to collect data from {self.topic_name}")
        rospy.sleep(self.listen_duration)
        print("Finished collecting data.")

    def save_data(self):
        """
        Save the collected data to a file.
        """
        with open(self.output_file, 'w') as file:
            json.dump(self.data, file)
        print(f"Data saved to {self.output_file}")

if __name__ == '__main__':
    # Customize these variables
    topic = 'your_topic_name'
    msg_type = String  # Change to the appropriate message type
    output_path = 'output_data.json'  # Change the output file path and name as required

    collector = DataCollector(topic, msg_type, output_path)
    collector.collect_data()
    collector.save_data()

