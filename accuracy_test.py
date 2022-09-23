import modern_sys

import math
import logging
from random import randint
import time

logging.basicConfig(filename='accuracy_test.log', filemode = 'w', encoding='utf-8', level=logging.INFO)


logging.info("ACCURACY TESTING START----")


test_bot = modern_sys.Cam_Arm(
		base_pos = [0,0,70], 
		base_orient = [0,90],
		#Remeasure the distance from last segment to the camera
		seg_lengths = [125, 125, 80], 
		range_trans = [
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]]
			], 
		range_rot = [
			[[0,15 - 90], [180, 165 - 90]],
			[[0, 0 - 90], [0.01, 180 - 90]],
			[[0, 0 - 90], [0.01, 180 - 90]],
			[[0 - 90, 10],[180 - 90, 73]]
			],
		tol = 0.1,
		cam_orient = 0,
		calibUnit = 19.26,
		cam_num = 1,
		comm_line = 'COM5', baud_rate = 9600, 
		hard_run = True
		)

print(f"BASE ORIENT::{test_bot.manager.base_joint.orient}")
for joint_name, param in test_bot.manager.get_joint_param().items():
	print("JOINT ", joint_name, ":")
	print(param)

for seg_name, param in test_bot.manager.get_segment_param().items():
	print("SEGMENT ", seg_name, ":")
	print(param)


logging.info("STARTING CONFIGURATION---")
for joint_name, param in test_bot.manager.get_joint_param().items():
	logging.info("JOINT "+ joint_name + ":")
	logging.info(param)

for seg_name, param in test_bot.manager.get_segment_param().items():
	logging.info("SEGMENT "+ seg_name+ ":")
	logging.info(param)

test_bot.visualize(axis_range = [[-300,300],[-300,300],[0,300]],  plot_name = "STARTING_CONFIG.png", save = True)


input()

test_pos = [
	#[test_bot.home_pos],
	[125, 205, 125 + 70],
	[200, 200, 100 + 70]
]

for test in test_pos:
	test_bot.end_to_target(test)
	input("Waiting command...")
	for i in range(10):
		test_bot.capture_image(test, f"test_{test}_{i}.png")
		#test_bot.visualize(axis_range = [[-300,300],[-300,300],[0,300]],  plot_name = "test_{test}_{i}_vis.png", save = False)
		x = [randint(10, 200), randint(10, 200), randint(10, 100) + 70]
		print(x)

		test_bot.end_to_target(x)

		time.sleep(1)


