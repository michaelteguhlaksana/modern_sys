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

target = [10,20,10]
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
test_bot.visualize(axis_range = [[-40,40],[-40,40],[0,40]])
