#!/usr/bin/env python

from setuptools import setup

setup(
    name='global_pose_calculator',
    version='0.0.1',
    packages=['global_pose_calculator'],
    install_requires=[
        'rospy',
        'matplotlib',
        'numpy',
        'geometry_msgs',
        'nav_msgs',
        'tfe_msgs',
        'tf',
        'tf2_ros',
        'message_filters'
    ],
    scripts=['scripts/Global_pose_publisher.py'],
    author='Your Name',
    author_email='youremail@example.com',
    maintainer='Your Name',
    maintainer_email='youremail@example.com',
    url='https://github.com/yourusername/global_pose_calculator',
    license='MIT',
    description='ROS package for calculating global pose'
)
