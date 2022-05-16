import modern_sys

test_bot = modern_sys.Cam_Arm(
		base_pos = [0,0,0], 
		base_orient = [0,0], 
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

#HARD TESTING

test_bot_hard = modern_sys.Cam_Arm(
		base_pos = [0,0,70], 
		base_orient = [0,0], 
		seg_lengths = [125, 125, 80], 
		range_trans = [
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]],
			[[0,0,0], [0,0,0]]
			], 
		range_rot = [
			[[0,-70 - 90], [360, 70 - 90]],
			[[0, -180], [0, 0]],
			[[0, -180], [0, 0]],
			[[0, 0],[0, 360]]
			],
		tol = 0.1,
		comm_line = 'COM3', baud_rate = 115200, 
		hard_run = True
		)

test_bot_hard.visualize(axis_range = [[-300,300],[-300,300],[0,300]])
