#!/bin/env python
"""
This script launches a master GPU instance on EC2.
"""
from __future__ import division, print_function
import base64
import boto
import boto.ec2
import cv
import cv2
import json
import os

credential = json.load(open('./dev_credential.json'))

conn = boto.ec2.connect_to_region(
	'ap-northeast-1',
	aws_access_key_id = credential["key"],
	aws_secret_access_key = credential["secret"])

def show_instances():
	for reservation in conn.get_all_reservations():
		print(reservation, [(inst, inst.instance_type) for inst in reservation.instances])


print('Running instances BEFORE launching:')
show_instances()

conn.run_instances('ami-a73676a6',
	instance_type = 'g2.2xlarge',
	key_name = 'ec2_devel_key',
	security_groups = ['WebFrontend'])

print('Running instances AFTER launching:')
show_instances()
