'''
MODERN
-------
By: MTL

An inverse kinematics lib for robotic arms
Designed for the modifications of the PRIMITV system

IMPORTANT: ADD ROLL TO THE ORIENTATIONS
'''

import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class Joint(object):
	"""
	A robotic joint. Acts as the actuator/endeffector of the next segment.
	"""
	def __init__(self, name, dof, pos, orient, range_trans, range_rot, rot_rate = 1500, trans_rate = 0):
		"""
		The init for Joint class

		name (str): The name of the joint
		dof (list(list(bool))) : The available axis for the joint to move. First list for translation and second list for rotation, bot in order of xyz and alpha,beta
			Example, for robots that translates in x axis only and rotates in alpha and beta axis, use [[1,0,0],[1,1]]
		pos (list(float)): The position of the joint, in xyz axis
		orient(list(float)): The angles of the joint (degrees). First angle (alpha) for the xy-plane, wrt to x axis; 
			the second angle (beta) wrt to the xy plane (tan beta = z/xy_plane).
		range_trans(list(float)): A list of 2 lists (min, max), 3 components each (x,y,z), the range of translation allowed relative to initial position.
		range_rot(list(float)):  A list of 2 lists (min, max), 2 components each (alpha, beta), the range of rotation allowed relative to initial orientation.


		"""
		super(Joint, self).__init__()
		self.name = name
		self.dof = dof
		self.pos = pos
		self.orient = orient
		self.range_trans = range_trans
		self.range_rot = range_rot
		self.rot_rate = rot_rate
		self.trans_rate = trans_rate

		self.x = pos[0]
		self.y = pos[1]
		self.z = pos[2]

		self.alpha = (orient[0] + 360) % 360
		self.beta = (orient[1] + 360) % 360

		self.min_trans = [sum(mini) for mini in zip(self.pos, range_trans[0])]
		self.max_trans = [sum(maxi) for maxi in zip(self.pos, range_trans[1])]

		self.min_rot = [(sum(mini) + 360) % 360 for mini in zip(self.orient, range_rot[0])]
		self.max_rot = [(sum(maxi) + 360) % 360 for maxi in zip(self.orient, range_rot[1])]

		self.inverted = [False, False]

		for ind, (maxi, mini) in enumerate(zip(self.max_rot,self.min_rot)):
			self.inverted[ind] = mini > maxi

	def set_pos(self, pos):
		self.pos = [round(p, 2) for p in pos]

		self.x = pos[0]
		self.y = pos[1]
		self.z = pos[2]

	def set_orient(self, orient):
		self.orient = [round(o, 2) for o in orient]

		self.alpha = (orient[0] + 360) % 360
		self.beta = (orient[1] + 360) % 360

	def update_range(self):
		self.min_trans = [round(sum(mini),2) for mini in zip(self.pos, self.range_trans[0])]
		self.max_trans = [round(sum(maxi),2) for maxi in zip(self.pos, self.range_trans[1])]

		self.min_rot = [round((sum(mini) + 360) % 360, 2) for mini in zip(self.orient, self.range_rot[0])]
		self.max_rot = [round((sum(maxi) + 360) % 360, 2) for maxi in zip(self.orient, self.range_rot[1])]

		for ind, (maxi, mini) in enumerate(zip(self.max_rot,self.min_rot)):
			self.inverted[ind] = mini > maxi

	def move(self, trans, rot, trans_error = 0, rot_error = 0):
		new_pos = self.pos.copy()
		new_orient = self.orient.copy()

		tot_trans = []
		tot_rot = []

		t_dof, r_dof = self.dof

		for ind, t_allow in enumerate(t_dof):
			if t_allow:
				new_pos[ind] = max(min(self.pos[ind] + trans[ind] + trans_error, self.max_trans[ind]), self.min_trans[ind])
			tot_trans.append(new_pos[ind] - self.pos[ind])

		self.set_pos(new_pos)

		for ind, r_allow in enumerate(r_dof):
			#REPAIR LOGIC LATER> THIS ONE SUCKS, TOO CONVOLUTED
			if r_allow:
				if self.inverted[ind]:
					target = (self.orient[ind] + rot[ind] + rot_error + 360) % 360
					if target >= self.min_rot[ind] or target <= self.max_rot[ind]:
						new_orient[ind] = target
					else:
						if self.min_rot[ind] - target < target - self.max_rot[ind] : 
							new_orient[ind] = self.min_rot[ind]
						else:
							new_orient[ind] = self.max_rot[ind]

				else:
					if self.min_rot[ind] == self.max_rot[ind] and self.max_rot[ind] == 0:
						new_orient[ind] = (self.orient[ind] + rot[ind] + rot_error + 360) % 360
					else:
						new_orient[ind] = max(min( (self.orient[ind] + rot[ind] + rot_error + 360) % 360, self.max_rot[ind]), self.min_rot[ind])
			tot_rot.append(new_orient[ind] - self.orient[ind])

		self.set_orient(new_orient)

		return tot_trans, tot_rot

	def shift(self, new_pos, new_orient):
		tot_trans = [n_pos-pos for n_pos, pos in zip(new_pos, self.pos)]
		tot_rot = [n_orient-orient for n_orient, orient in zip(new_orient, self.orient)]

		self.set_pos(new_pos)
		self.set_orient(new_orient)

		self.update_range()

		return tot_trans, tot_rot



class Segment(object):
	"""
	A rigid segmenet connecting 2 joints together
	Not a literal stick that connects the 2 joints,
	just a way to tell the program which joints connect to which
	"""
	def __init__(self, name, base, end):
		super(Segment, self).__init__()
		self.name = name
		self.base = base
		self.end = end

		self.x_len = self.end.x-self.base.x
		self.y_len = self.end.y-self.base.y
		self.z_len = self.end.z-self.base.z
		
		self.length = math.sqrt((self.x_len)**2 + (self.y_len)**2 + (self.z_len)**2)

		self.alpha = math.atan2(self.y_len, self.x_len) * 180/math.pi
		self.beta = math.atan2(self.z_len, math.sqrt((self.x_len)**2 + (self.y_len)**2) )* 180/math.pi
		# Angular correction so all angles are positive
		self.alpha = round((self.alpha + 360) % 360, 2)
		self.beta = round((self.beta + 360) % 360, 2)

	def update_orient(self, rot):
		self.alpha = round((self.alpha + rot[0] + 360) % 360, 2)
		self.beta = round((self.beta + rot[1] + 360) % 360, 2)

	def update_length(self):
		self.x_len = self.end.x-self.base.x
		self.y_len = self.end.y-self.base.y
		self.z_len = self.end.z-self.base.z
		
		self.length = math.sqrt((self.x_len)**2 + (self.y_len)**2 + (self.z_len)**2)

	def get_end_move(self, tot_base_trans, tot_base_rot):
		tot_end_orient = [(sum(angles) + 360) % 360 for angles in zip(self.end.orient, tot_base_rot)]

		#tot_end_pos = [sum(transl) for transl in zip(self.end.pos, tot_base_trans)]

		tot_end_pos = [0,0,0]
		tot_end_pos[0] = self.length * math.cos(self.beta * (math.pi/180)) * math.cos(self.alpha * (math.pi/180)) + self.base.pos[0]
		tot_end_pos[1] = self.length * math.cos(self.beta * (math.pi/180)) * math.sin(self.alpha * (math.pi/180)) + self.base.pos[1]
		tot_end_pos[2] = self.length * math.sin(self.beta * (math.pi/180)) + self.base.pos[2]

		#tot_end_pos = [sum(transl) for transl in zip(tot_base_trans, delta_pos_rot)]


		#print(self.end.name, tot_end_pos, tot_end_orient)

		return tot_end_pos, tot_end_orient


	def move_base(self, trans, rot, trans_error_base = 0, rot_error_base = 0, trans_error_end = 0, rot_error_end = 0):
		tot_base_trans, tot_base_rot = self.base.move(trans, rot, trans_error_base, rot_error_base)
		#print(tot_base_trans, tot_base_rot)
		
		#Update the segment
		self.update_orient(tot_base_rot)

		#Get movements for the end according to the base
		tot_end_pos, tot_end_orient = self.get_end_move(tot_base_trans, tot_base_rot)
		#print(tot_end_pos, tot_end_orient)

		#self.end.shift(tot_end_pos, tot_end_orient)
		return tot_end_pos, tot_end_orient

	def shift_base(self, new_pos, new_orient):
		tot_base_trans, tot_base_rot = self.base.shift(new_pos, new_orient)

		self.update_orient(tot_base_rot)

		tot_end_pos, tot_end_orient = self.get_end_move(tot_base_trans, tot_base_rot)

		return tot_end_pos, tot_end_orient


		'''
		tot_end_orient = [(sum(angles) + 360) % 360 for angles in zip(self.end.orient, tot_base_rot)]

		tot_end_trans = [sum(transl) for transl in zip(self.end.pos, tot_base_trans)]

		delta_pos_rot = [0,0,0]
		delta_pos_rot[0] = self.length * math.cos(self.alpha * (math.pi/180))
		delta_pos_rot[1] = self.length * math.sin(self.alpha * (math.pi/180))
		delta_pos_rot[2] = self.length * math.sin(self.beta * (math.pi/180))

		tot_end_trans = [sum(transl) for transl in zip(tot_end_trans, delta_pos_rot)]

		tot_end_trans, tot_end_rot = self.end.move(tot_end_trans, tot_end_orient, trans_error_end, rot_error_end)

		self.end.update_range()
		'''

		#self.update_length()




class SegmentManager(object):
	"""
	Manages the movements of segments and joints.
	Made because moving the end of a segment means moving
	base of another segment.
	"""
	def __init__(self, segments, base_joint, end_joint):
		super(SegmentManager, self).__init__()
		self.segments = segments
		self.segments_dict = {}
		self.base_to_segment = {}
		self.end_to_segment = {}

		for seg in segments:
			self.segments_dict[seg.name] = seg
			self.base_to_segment[seg.base.name] = seg.name
			self.end_to_segment[seg.end.name] = seg.name

		self.base_joint = base_joint
		self.end_joint = end_joint
			
	def move_base(self, segment_name, trans, rot, trans_error = 0, rot_error = 0):
		segment = self.segments_dict[segment_name]

		tot_end_pos, tot_end_orient = segment.move_base(trans, rot, trans_error, rot_error)

		seg_end = segment.end.name

		while seg_end in self.base_to_segment.keys():
			segment = self.segments_dict[self.base_to_segment[seg_end]]
			
			tot_end_pos, tot_end_orient = segment.shift_base(tot_end_pos, tot_end_orient)
			seg_end = segment.end.name

		#Move end-effector

		segment.end.shift(tot_end_pos, tot_end_orient)

	def get_joint_param(self):
		param = {}
		for joint_name, seg_name in self.base_to_segment.items():
			segment = self.segments_dict[seg_name]
			joint = segment.base
			param[joint_name] = {
				"pos" : joint.pos,
				"orient" : joint.orient,
				"DOF" : joint.dof,
				"trans_range" : [joint.min_trans, joint.max_trans],
				"rot_range" : [joint.min_rot, joint.max_rot]
			}
			seg_end = segment.end

		param[seg_end.name] = {
				"pos" : seg_end.pos,
				"orient" : seg_end.orient,
				"DOF" : seg_end.dof,
				"trans_range" : [seg_end.min_trans, seg_end.max_trans],
				"rot_range" : [seg_end.min_rot, seg_end.max_rot]
			}

		return param

	def get_segment_param (self):
		param = {}
		for seg_name, seg in self.segments_dict.items():
			param[seg_name] = {
				"length" : seg.length,
				"alpha" : seg.alpha,
				"beta" : seg.beta,
				"base" : seg.base.name,
				"end" : seg.end.name
			}

		return param

	def visualize(self, axis_range = [[0,2],[0,2],[0,2]], plot_name = None, save = False):
		fig = plt.figure()
		ax = plt.axes(projection='3d')

		label = []

		x_pos = []
		y_pos = []
		z_pos = []

		for joint_name, seg_name in self.base_to_segment.items():
			segment = self.segments_dict[seg_name]
			joint = segment.base

			label.append(joint.name)
			x_pos.append(joint.pos[0])
			y_pos.append(joint.pos[1])
			z_pos.append(joint.pos[2])

			seg_end = segment.end

		label.append(seg_end.name)
		x_pos.append(seg_end.pos[0])
		y_pos.append(seg_end.pos[1])
		z_pos.append(seg_end.pos[2])

		ax.plot3D(x_pos, y_pos, z_pos, marker = "o")

		ax.set_xlim3d(axis_range[0])
		ax.set_xlabel('X')

		ax.set_ylim3d(axis_range[1])
		ax.set_ylabel('Y')

		ax.set_zlim3d(axis_range[2])
		ax.set_zlabel('Z')

		if save:
			fig.savefig(plot_name)


		plt.show()






if __name__ == "__main__":
	## Joints
	Base = Joint(
		name = "Base", 
		dof = [[1,0,0],[1,0]], 
		pos = [0,0,0],
		orient = [0,0],
		range_trans = [[-1, -1, -1], [1, 1, 1]],
		range_rot = [[-90, -90],[90, 90]]
		)

	End = Joint(
		name = "End", 
		dof = [[0,1,0],[0,0]], 
		pos = [2,0,0],
		orient = [90, 0],
		range_trans = [[0,0,0], [0,0,0]],
		range_rot = [[0, 0],[0, 360]]
		)

	#Segments

	Connect1 = Segment("connector1", Base, End)

	print("TESTING 1")
	print("----------------------")
	print("STARTING POSITION")
	print(f"{Base.name} :: pos:{Base.pos} :: orient:{Base.orient}" )
	print(f"{End.name} :: pos:{End.pos} :: orient:{End.orient}" )
	print(f"Segment :: length:{Connect1.length} :: alpha:{Connect1.alpha} :: beta:{Connect1.beta}" )

	#Moving Base 
	tot_end_pos, tot_end_orient = Connect1.move_base(trans = [0, 0, 0.5], rot = [45, 0])
	Connect1.end.shift(tot_end_pos, tot_end_orient)

	print("AFTER MOVING POSITION")
	#print(f"{Base.name} :: pos:{Base.pos} :: orient:{Base.orient}" )
	#print(f"{End.name} :: pos:{End.pos} :: orient:{End.orient}" )
	print(f"Segment :: length:{Connect1.length} :: alpha:{Connect1.alpha} :: beta:{Connect1.beta}" )
	print("----- Segment components")
	print(f"Connect1 :: {Connect1.base.name} :: pos:{Connect1.base.pos} :: orient:{Connect1.base.orient}" )
	print(f"Connect1 :: {Connect1.end.name} :: pos:{Connect1.end.pos} :: orient:{Connect1.end.orient}" )


	print("")
	print("TESTING 2")
	print("----------------------")

	print("INIT JOINTS...")

	Base1 = Joint(
		name = "base", 
		dof = [[0,0,0],[1,1]], 
		pos = [0,0,0],
		orient = [0,90],
		range_trans = [[0,0,0], [0,0,0]],
		range_rot = [[0,-90],[360, 90]]
		)

	Elbow1 = Joint(
		name = "e1", 
		dof = [[0,0,0],[0,1]], 
		pos = [0,0,1],
		orient = [0,0],
		range_trans = [[0,0,0], [0,0,0]],
		range_rot = [[0,-45],[360, 180 + 45]]
		)

	Elbow2 = Joint(
		name = "e2", 
		dof = [[0,0,0],[0,1]], 
		pos = [1,0,1],
		orient = [0,0],
		range_trans = [[0,0,0], [0,0,0]],
		range_rot = [[0,180-45],[360, 180 + 45]]
		)

	End = Joint(
		name = "end", 
		dof = [[0,0,0],[0,1]], 
		pos = [2,0,1],
		orient = [0,-90],
		range_trans = [[0,0,0], [0,0,0]],
		range_rot = [[0,0],[0, 360]]
		)
	print("INIT JOINT SUCCESS")
	print("INIT SEGMENTS...")
	Seg1 = Segment("base_e1", Base1, Elbow1)
	Seg2 = Segment("e1_e2", Elbow1, Elbow2)
	Seg3 = Segment("e2_end", Elbow2, End)
	print("INIT SEGMENTS SUCCESS")

	print("INIT SEG_MANAGER...")
	Robot = SegmentManager([Seg1, Seg2, Seg3], Seg1.base, Seg3.end)
	print("INIT SEG_MANAGER SUCCESS")
	for joint_name, param in Robot.get_joint_param().items():
		print("JOINT ", joint_name, ":")
		print(param)

	for seg_name, param in Robot.get_segment_param().items():
		print("SEGMENT ", seg_name, ":")
		print(param)


	Robot.visualize()
	print("-------------------------------------")
	print("Rotating base by 90 deg...")
	Robot.move_base("base_e1", [0,0,0], [90, 0])
	for joint_name, param in Robot.get_joint_param().items():
		print("JOINT ", joint_name, ":")
		print(param)

	for seg_name, param in Robot.get_segment_param().items():
		print("SEGMENT ", seg_name, ":")
		print(param)

	Robot.visualize()




