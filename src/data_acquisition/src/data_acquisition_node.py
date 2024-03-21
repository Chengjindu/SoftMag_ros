#!/usr/bin/env python3
# This line tells the system to use Python to execute the script.

import rospy  # ROS Python API
from std_msgs.msg import String  # ROS standard message types
import socket  # Socket library for TCP communication
import threading  # Threading library to handle each client connection


# Define s globally so it's accessible in shutdown_hook and start_server
s = None

# Function to handle client connections
def client_handler(conn, pub):
    conn.settimeout(1.0)  # Set a timeout for blocking operations to allow periodic shutdown checks
    while not rospy.is_shutdown():  # Loop until ROS node is shutdown
        try:
            data = conn.recv(1024)  # Attempt to receive data from the client (up to 1024 bytes)
            if not data:
                break  # If no data is received, exit the loop
            rospy.loginfo("Received data: %s", data)  # Log received data for debugging
            pub.publish(data.decode())  # Publish the received data to the ROS topic after decoding it
        except socket.timeout:
            continue  # If a timeout occurs, continue the loop and check for shutdown again
        except Exception as e:
            rospy.logwarn("Error receiving data: %s", e)  # Log any exceptions that occur
            break  # Break the loop if an exception occurs

# Function to start the TCP server
def start_server(pub):
    global s  # Use the global server socket variable
    HOST = ''  # Symbolic name meaning all available interfaces
    PORT = 65432  # Arbitrary non-privileged port

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a new socket using IPv4 and TCP
    s.bind((HOST, PORT))  # Bind the socket to the host and port
    s.listen(1)  # Listen for incoming connections (1 connection at a time)
    rospy.loginfo("Server listening on port %s", PORT)  # Log that the server is listening

    while not rospy.is_shutdown():  # Loop until ROS node is shutdown
        try:
            conn, addr = s.accept()  # Accept a new connection
            rospy.loginfo('Connected by %s', addr)  # Log the address of the connected client
            # Start a new daemon thread to handle the client connection
            threading.Thread(target=client_handler, args=(conn, pub), daemon=True).start()
        except socket.error as e:
            if rospy.is_shutdown(): # Check if the exception is due to the shutdown process
                rospy.loginfo("Server socket closed, shutting down.")
            else:
                rospy.logerr("Socket error: %s", e)
            break


# Function to be called on ROS node shutdown
def shutdown_hook():
    print("Shutting down. Closing sockets.")
    if s:
        s.close()  # Close the server socket


if __name__ == '__main__':
    rospy.init_node('data_acquisition_node', anonymous=True)  # Initialize the ROS node
    data_pub = rospy.Publisher('sensor_data', String, queue_size=10)  # Create a ROS publisher for sending data

    rospy.on_shutdown(shutdown_hook)  # Register the shutdown hook

    try:
        start_server(data_pub)  # Start the TCP server
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down data acquisition node.")  # Log shutdown message
