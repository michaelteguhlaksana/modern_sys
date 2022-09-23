import modern_sys

import math

import logging

logging.basicConfig(filename='cam_arm_test.log', filemode = 'w', encoding='utf-8', level=logging.INFO)


'''
test_bot = modern_sys.Cam_Arm(
		base_pos = [0,0,0], 
		base_orient = [0,90], 
		seg_lengths = [10, 10, 10], 
		range_trans = [
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]]
			], 
		range_rot = [
			[[0,-80], [360,80]],
			[[0,-90], [0,90]],
			[[0,-90], [0,90]],
			[[0, 0],[0, 360]]
			]
		)

print("INIT TESTING CAM BOT...")
for joint_name, param in test_bot.manager.get_joint_param().items():
	print("JOINT ", joint_name, ":")
	print(param)

for seg_name, param in test_bot.manager.get_segment_param().items():
	print("SEGMENT ", seg_name, ":")
	print(param)

print("INIT SUCCESS----")

target = [10,20,20]
reach = test_bot.end_to_target(target, iterate = 200)
diff = [tar-curr for tar, curr in zip(target, test_bot.manager.end_joint.pos)]

print(f"BASE ORIENT::{test_bot.manager.base_joint.orient}")
print(f"TEST 2 WITH:: TARGET:: {target} :: END:: {test_bot.manager.end_joint.pos}")
print(f"DIFF::{diff}")
for joint_name, param in test_bot.manager.get_joint_param().items():
	print("JOINT ", joint_name, ":")
	print(param)

for seg_name, param in test_bot.manager.get_segment_param().items():
	print("SEGMENT ", seg_name, ":")
	print(param)
test_bot.visualize(axis_range = [[-40,40],[-40,40],[0,40]], target = target, has_target = True)

test_bot.home()
test_bot.visualize(axis_range = [[-40,40],[-40,40],[0,40]], target = target, has_target = True)

'''

#HARD TESTING

logging.info("HARD TESTING START----")


test_bot_hard = modern_sys.Cam_Arm(
		base_pos = [0,0,70], 
		base_orient = [0,90], 
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
			[[0, 0],[180, 180]]
			],
		tol = 0.1,
		cam_orient = 0,
		comm_line = 'COM5', baud_rate = 9600, 
		hard_run = True
		)

print(f"BASE ORIENT::{test_bot_hard.manager.base_joint.orient}")
for joint_name, param in test_bot_hard.manager.get_joint_param().items():
	print("JOINT ", joint_name, ":")
	print(param)

for seg_name, param in test_bot_hard.manager.get_segment_param().items():
	print("SEGMENT ", seg_name, ":")
	print(param)


logging.info("STARTING CONFIGURATION---")
for joint_name, param in test_bot_hard.manager.get_joint_param().items():
	logging.info("JOINT "+ joint_name + ":")
	logging.info(param)

for seg_name, param in test_bot_hard.manager.get_segment_param().items():
	logging.info("SEGMENT "+ seg_name+ ":")
	logging.info(param)

test_bot_hard.visualize(axis_range = [[-300,300],[-300,300],[0,300]],  plot_name = "STARTING_CONFIG.png", save = True)

max_l = sum([125, 125, 80])

test_pos = [
	[max_l* math.cos(math.pi/180 * 45) * math.sin(math.pi/180 * 45), max_l* math.cos(math.pi/180 * 45) * math.cos(math.pi/180 * 45), max_l* math.sin(math.pi/180 * 45) + 70],# Test 45deg on all axis
	[125, 205, 125 + 70], #Some random numbers
	[200, 200, 100 + 70]
	#[0,0,max_l + 70] #Test straight up. Does not work?!
]



input()

for test in test_pos:
	test_bot_hard.end_to_target(test, iterate = 200)

	print("------------------------------------------------------------")
	print(f"BASE ORIENT::{test_bot_hard.manager.base_joint.orient}")
	print(f"HOME:: {test_bot_hard.home_pos} ORIENT:: {test_bot_hard.home_orient}")
	print(f"TEST WITH:: TARGET:: {test} :: END:: {test_bot_hard.manager.end_joint.pos}")
	diff = [tar-curr for tar, curr in zip(test, test_bot_hard.manager.end_joint.pos)]
	print(f"DIFF::{diff}")
	for joint_name, param in test_bot_hard.manager.get_joint_param().items():
		print("JOINT ", joint_name, ":")
		print(param)

	for seg_name, param in test_bot_hard.manager.get_segment_param().items():
		print("SEGMENT ", seg_name, ":")
		print(param)
	

	logging.info("------------------------------------------------------------")
	logging.info(f"---TEST WITH:: TARGET:: {test}\n-------END:: {test_bot_hard.manager.end_joint.pos}\n")
	logging.info(f"---BASE ORIENT::{test_bot_hard.manager.base_joint.orient}")
	logging.info(f"---HOME:: {test_bot_hard.home_pos} ORIENT:: {test_bot_hard.home_orient}")
	
	diff = [tar-curr for tar, curr in zip(test, test_bot_hard.manager.end_joint.pos)]
	logging.info(f"---DIFF::{diff}\n")

	logging.info("JOINTS===")
	for joint_name, param in test_bot_hard.manager.get_joint_param().items():
		logging.info("-------JOINT " + joint_name +":")
		logging.info(param)

	logging.info("SEGMENTS===")
	for seg_name, param in test_bot_hard.manager.get_segment_param().items():
		logging.info("------SEGMENT "+ seg_name+ ":")
		logging.info(param)
	test_bot_hard.visualize(axis_range = [[-300,300],[-300,300],[0,300]], target = test, has_target = True,  plot_name = F"TAR_{test}.png", save = True)



test_bot_hard.home()
print("------------------------------------------------------------")
print(f"BASE ORIENT::{test_bot_hard.manager.base_joint.orient}")
print(f"TEST WITH:: TARGET:: HOME :: END:: {test_bot_hard.manager.end_joint.pos}")
print(f"HOME:: {test_bot_hard.home_pos} ORIENT:: {test_bot_hard.home_orient}")
print(f"DIFF::{diff}")
for joint_name, param in test_bot_hard.manager.get_joint_param().items():
	print("JOINT ", joint_name, ":")
	print(param)

for seg_name, param in test_bot_hard.manager.get_segment_param().items():
	print("SEGMENT ", seg_name, ":")
	print(param)

logging.info("------------------------------------------------------------")
logging.info(f"---TEST WITH:: TARGET:: HOME\n-------END:: {test_bot_hard.manager.end_joint.pos}\n")
logging.info(f"---BASE ORIENT::{test_bot_hard.manager.base_joint.orient}")
logging.info(f"---HOME:: {test_bot_hard.home_pos} ORIENT:: {test_bot_hard.home_orient}")

diff = [tar-curr for tar, curr in zip(test, test_bot_hard.manager.end_joint.pos)]
logging.info(f"---DIFF::{diff}\n")

logging.info("JOINTS===")
for joint_name, param in test_bot_hard.manager.get_joint_param().items():
	logging.info("-------JOINT " + joint_name +":")
	logging.info(param)

logging.info("SEGMENTS===")
for seg_name, param in test_bot_hard.manager.get_segment_param().items():
	logging.info("------SEGMENT "+ seg_name+ ":")
	logging.info(param)
test_bot_hard.visualize(axis_range = [[-300,300],[-300,300],[0,300]],  plot_name = F"TAR_HOME.png", save = True)



test_bot_hard.update_cam_orient(90)
