from modern import Joint, Segment, SegmentManager

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math
import serial
import time
import cv2
import PySimpleGUI as sg
import pandas as pd

'''
import logging

logging.basicConfig(filename='modern_sys.log', encoding='utf-8', level=logging.DEBUG)
'''


class Cam_Arm(object):
	"""
	A Robot arm model for holding the camera for the object recognition

	"""
	def __init__(self, base_pos, base_orient, seg_lengths, range_trans, range_rot, tol = 0.01, cam_orient = 270, comm_line = None, baud_rate = None, cam_num = None, hard_run = False):
		super(Cam_Arm, self).__init__()
		self.home_base_pos = base_pos
		self.home_base_orient = base_orient
		self.seg_lengths = seg_lengths
		self.range_trans = range_trans
		self.range_rot = range_rot
		self.tol = tol

		self.cam_num = cam_num
		self.cam = cv2.VideoCapture(self.cam_num)

		self.hard_run = hard_run
		self.baud_rate = baud_rate
		self.comm_line = comm_line
		if self.hard_run:
			self.cnc = serial.Serial(self.comm_line, self.baud_rate, timeout=1)

		self.home_pos = {
			"base" : self.home_base_pos.copy(),
			"e1" : [sum(pos) for pos in zip([0,0,seg_lengths[0]], self.home_base_pos)],
			"e2" : [sum(pos) for pos in zip([seg_lengths[1],0,seg_lengths[0]], self.home_base_pos)],
			"end" : [sum(pos) for pos in zip([seg_lengths[1] + seg_lengths[2], 0, seg_lengths[0]], self.home_base_pos)]

			}

		self.home_orient = {
			"base" : self.home_base_orient.copy(),
			"e1" : [self.home_base_orient[0],(self.home_base_orient[1]-90 +360)%360],
			"e2" : [self.home_base_orient[0],(self.home_base_orient[1]-90 +360)%360],
			"end" : [self.home_base_orient[0], cam_orient]

			}

		self.Base = Joint(
			name = "base", 
			dof = [[0,0,0],[1,1]], 
			pos = self.home_pos["base"].copy(),
			orient = self.home_orient["base"].copy(),
			range_trans = self.range_trans[0],
			range_rot = self.range_rot[0]
			)

		self.Elbow1 = Joint(
			name = "e1", 
			dof = [[0,0,0],[0,1]], 
			pos = self.home_pos["e1"].copy(),
			orient = self.home_orient["e1"].copy(),
			range_trans = self.range_trans[1],
			range_rot = self.range_rot[1]
			)

		self.Elbow2 = Joint(
			name = "e2", 
			dof = [[0,0,0],[0,1]], 
			pos = self.home_pos["e2"].copy(),
			orient = self.home_orient["e2"].copy(),
			range_trans = self.range_trans[2],
			range_rot = self.range_rot[2]
			)

		self.End = Joint(
			name = "end", 
			dof = [[0,0,0],[1,1]], 
			pos = self.home_pos["end"].copy(),
			orient = self.home_orient["end"].copy(),
			range_trans = self.range_trans[3],
			range_rot = self.range_rot[3]
			)

		self.Seg1 = Segment("base_e1", self.Base, self.Elbow1)
		self.Seg2 = Segment("e1_e2", self.Elbow1, self.Elbow2)
		self.Seg3 = Segment("e2_end", self.Elbow2, self.End)

		self.manager = SegmentManager([self.Seg1, self.Seg2, self.Seg3], self.Seg1.base, self.Seg3.end)

		if self.hard_run:
			self.setup_connection()
			self.send_angles()

	def visualize(self, axis_range = [[0,2],[0,2],[0,2]], target = None, has_target = False, plot_name = None, save = False):
		#self.manager.visualize(axis_range,plot_name, save)
		fig = plt.figure()
		ax = plt.axes(projection='3d')

		label = []

		x_pos = []
		y_pos = []
		z_pos = []


		for joint_name, seg_name in self.manager.base_to_segment.items():
			segment = self.manager.segments_dict[seg_name]
			joint = segment.base

			label.append(joint.name)
			x_pos.append(joint.pos[0])
			y_pos.append(joint.pos[1])
			z_pos.append(joint.pos[2])
		
			x_or = [joint.pos[0], joint.pos[0] + abs(axis_range[0][1]-axis_range[0][0]) * 0.05 * math.cos(joint.beta * (math.pi/180)) * math.cos(joint.alpha * (math.pi/180))]
			y_or = [joint.pos[1], joint.pos[1] + abs(axis_range[1][1]-axis_range[1][0]) * 0.05 * math.cos(joint.beta * (math.pi/180)) * math.sin(joint.alpha * (math.pi/180))]
			z_or = [joint.pos[2], joint.pos[2] + abs(axis_range[2][1]-axis_range[2][0]) * 0.05 * math.sin(joint.beta * (math.pi/180))]

			ax.plot3D(x_or, y_or, z_or, marker = "o")

			seg_end = segment.end

		label.append(seg_end.name)
		x_pos.append(seg_end.pos[0])
		y_pos.append(seg_end.pos[1])
		z_pos.append(seg_end.pos[2])

		ax.plot3D(x_pos, y_pos, z_pos, marker = "o")

		if has_target:
			ax.plot3D(target[0], target[1], target[2], marker = "x")
			ax.plot3D([0,target[0], target[0]],[0,target[1], target[1]],[0,0,target[2]])

		ax.set_xlim3d(axis_range[0])
		ax.set_xlabel('X')

		ax.set_ylim3d(axis_range[1])
		ax.set_ylabel('Y')

		ax.set_zlim3d(axis_range[2])
		ax.set_zlabel('Z')

		if save:
			fig.savefig(plot_name)

		plt.show()


	def rotate_base(self, target):
		t_x, t_y, t_z = target
		t_alpha = math.atan2(t_y - self.Base.pos[1], t_x - self.Base.pos[0]) * 180/math.pi
		rot_alpha = t_alpha - self.Base.alpha
		self.manager.move_base("base_e1", [0,0,0], [rot_alpha, 0])

	def IK_2D(self, target):
		#Returns the change in beta, not the final beta
		t_x, t_y, t_z = target
		end = self.manager.end_joint
		seg = self.manager.segments_dict[self.manager.end_to_segment[end.name]]

		t_beta = math.atan2(t_z - seg.base.pos[2], math.sqrt((t_x - seg.base.pos[0])**2 + (t_y - seg.base.pos[1])**2)) * 180/math.pi
		
		#print(f"TARGET::{target}")
		#print(f"TBETA::{t_beta}")

		beta_res = {seg.name: t_beta - seg.beta}

		# Shift the end to be at the target, the base follows with fixed segment length
		# Shift the target to the base of the last segment
		# Shift the segment to the previous segment
		# Calculate the beta for the new segment
		# Repeat until base

		while seg.base.name in self.manager.end_to_segment.keys():
			t_x = t_x - seg.length * math.cos(t_beta * (math.pi/180)) * math.cos(seg.alpha * (math.pi/180))
			t_y = t_y - seg.length * math.cos(t_beta * (math.pi/180)) * math.sin(seg.alpha * (math.pi/180))
			t_z = t_z - seg.length * math.sin(t_beta * (math.pi/180))

			seg = self.manager.segments_dict[self.manager.end_to_segment[seg.base.name]]

			t_beta = math.atan2(t_z - seg.base.pos[2], math.sqrt((t_x - seg.base.pos[0])**2 + (t_y - seg.base.pos[1])**2)) * 180/math.pi
			
			'''
			print(f"SEG::{seg.base.pos}")
			print(f"TARGET::{[t_x,t_y,t_z]}")
			print(f"DIFF::{[t_x - seg.base.pos[0],t_y - seg.base.pos[1],t_z - seg.base.pos[2]]}")
			print(f"TBETA::{t_beta}")
			'''
			
			beta_res[seg.name] = t_beta - seg.beta

		#reach = all([target - self.tol <= posi <= target + self.tol for target, posi in zip([t_x, t_y, t_z], self.manager.base_joint.pos)])
		reach = self.tol < math.sqrt(sum([(tar-posi) ** 2 for tar, posi in zip([t_x, t_y, t_z], self.manager.base_joint.pos)]))
		#print(f"REACH::{reach} , {[t - self.tol <= posi <= t + self.tol for t, posi in zip([t_x, t_y, t_z], self.manager.base_joint.pos)]}")
			
		return beta_res, reach

		
	def end_to_target(self, target, iterate = 100):
		'''
		Since the robot could not translate on all joints
		1. Rotate the base such that the end is in-plane with the target
		2. Align end to the target, imagin the whole segment move to the target
		3. New target is the base of the last segment, repeat for the previous segment
		4. Repeat until the base is reached
		5. Shift the base and all other segment to the base's original position
		'''
		#target = [tar - base for tar, base in zip(target, self.manager.base_joint.pos)]
		t_x, t_y, t_z = target

		# 1. Rotate the base to target to allow 2D IK
		self.rotate_base(target)

		for i in range(iterate):
			# Get the angles needed to get to target
			beta_tar, reach = self.IK_2D(target)

			if not reach:
				pass
				#print("TARGET OUT OF REACH!")
			
			for seg_name, beta_rot in beta_tar.items():
				# Forward feedback as the movements of the base affect the endeffectors
				# And the t_beta made by IK_2D is the final angles
				seg = self.manager.segments_dict[seg_name]

				if seg.end.name in self.manager.base_to_segment.keys():
					check_seg = self.manager.segments_dict[self.manager.base_to_segment[seg.end.name]]
					beta_tar[check_seg.name] = beta_tar[check_seg.name] - beta_rot

			# Move the joints according to the target angles
			for seg_name, beta_rot in beta_tar.items():
				self.manager.move_base(seg_name, [0,0,0], [0, beta_rot])

		if self.hard_run:
			self.send_angles()

		return reach


	def upate_cam_orient (angle):
		self.manager.end_joint.move(trans = [0,0,0], rot = [angle-self.manager.end_joint.orient[0], 0])
		if self.hard_run:
			self.send_angles()

	def home(self):
		for joint_name, seg_name in self.manager.base_to_segment.items():
			seg = self.manager.segments_dict[seg_name]
			trans = [home - curr for home, curr in zip(self.home_pos[joint_name], seg.base.pos)]
			rot = [home - curr for home, curr in zip(self.home_orient[joint_name], seg.base.orient)]
			self.manager.move_base(seg_name, trans, rot)

		if self.hard_run:
			self.send_angles()


	def setup_connection(self):
		self.cnc.close()
		self.cnc.open()

		time.sleep(5)   # Wait for grbl to initialize
		self.cnc.flushInput()

		#Homing cycle
		self.send_angles()
		#print(self.cnc.readline())

		time.sleep(10)

	def send_angles(self):
		'''
		joint_dict = self.manager.get_joint_param()
		base_alpha = joint_dict["base"]["orient"][0]
		base_beta = joint_dict["base"]["orient"][1]
		e1_beta = (joint_dict["e1"]["orient"][1] + base_beta + 360) % 360
		e2_beta = (joint_dict["e2"]["orient"][1] + base_beta + e1_beta + 360) % 360
		end_alpha = joint_dict["end"]["orient"][0]
		end_beta = (joint_dict["end"]["orient"][1] + base_beta + e1_beta + e2_beta + 360) % 360
		'''
		#To compensate for the changing frame of reference for each servo
		#Take the beta difference between adjacent segment  and add 90 (as the 0 is 90 degree from the segment)
		seg_dict = self.manager.get_segment_param()
		base_alpha = seg_dict["base_e1"]["alpha"]
		base_beta = seg_dict["base_e1"]["beta"]
		e1_beta = (seg_dict["e1_e2"]["beta"] - base_beta + 90 + 360) % 360
		e2_beta = (seg_dict["e2_end"]["beta"] - seg_dict["e1_e2"]["beta"] + 90 + 360) % 360
		end_alpha = self.manager.end_joint.orient[0]
		end_beta = self.manager.end_joint.orient[1]

		command = bytes(f"{base_alpha};{base_beta};{e1_beta};{e2_beta};{end_beta};{end_alpha}\n", 'utf-8')
		print(F"SENDING COMMAND:: {command}")

		self.cnc.write(command)
		time.sleep(2)

	def capture(self):
		pass

	def detect(self):
		pass
		

class HaarCascadeClassifier(object):
	""" 
	A Haar Cascade Classifier UI for detecting devices.
	Adapted from the PRIMITV system

	"""
	def __init__(self, mask_path, result_folder, vid_cap):
		super(HaarCascadeClassifier, self).__init__()
		self.cascade_device = cv2.CascadeClassifier(mask_path)
		self.cam = vid_cap
		self.result_folder = result_folder
		self.calibUnit = calibUnit

		self.probe_pos = probe_pos
		self.cam_pos = cam_pos
		self.offset = [probe + cam for probe, cam in zip(self.probe_pos, self.cam_pos)]
		

		self.layout = [
			# Line 1: Brightness
			[
				sg.Text("Brightness:", size=(20, 0), justification="left"), 
				sg.Slider((0, 100), 0, orientation="h", size=(50, 15), key="-BRIGHTNESS SLIDER-")
			],
			# Line 2: Contrast
			[
				sg.Text("Contrast:", size=(20, 0), justification="left"),
				sg.Slider((0, 5), 1, orientation="h", size=(50, 15), key="-CONTRAST SLIDER-")
			],
			# Line 3: Gaussian Blurring Kernel Size
			[
				sg.Text("Gaussian Blurring Kernel:", size=(20, 0), justification="left"), 
				sg.Radio('Disable', 'Radio1', size=(10, 1), key="-KERNEL DISABLE-", default=True),
				sg.Radio('3x3 Size', 'Radio1', size=(10, 1), key="-3x3 KERNEL SIZE-"),
				sg.Radio('5x5 Size', 'Radio1', size=(10, 1), key="-5x5 KERNEL SIZE-"),
				sg.Radio('9x9 Size', 'Radio1', size=(10, 1), key="-9x9 KERNEL SIZE-"),
			],
			[sg.Text("", size=(80, 0), justification="center")],
			# Line 4
			[sg.Text("Haar Cascade Object Detection", size=(80, 0), justification="center", font=('Arial', 10, 'bold'))],
			# Line 5
			[sg.Text("Scale Factor", size=(50, 0), justification="center"),
			 sg.Text("Min Neighbour", size=(20, 0), justification="center"),
			],
			# Line 6-10: Scale Factor for Cross, Circle, Square, Triangle and Device
			[
				sg.Text('Cross', size=(8, 0), justification="left"),
				sg.Slider((1, 1000), 800, orientation="h", size=(30, 15), key="-CROSS SCALE SLIDER-"),
				sg.Slider((0, 20), 12, orientation="h", size=(30, 15), key="-CROSS NEIGHBOUR SLIDER-"),
			],
			[
				sg.Text('Circle', size=(8, 0), justification="left"),
				sg.Slider((1, 1000), 800, orientation="h", size=(30, 15), key="-CIRCLE SCALE SLIDER-"),
				sg.Slider((0, 20), 12, orientation="h", size=(30, 15), key="-CIRCLE NEIGHBOUR SLIDER-")
			],
			[
				sg.Text('Square', size=(8, 0), justification="left"),
				sg.Slider((1, 1000), 800, orientation="h", size=(30, 15), key="-SQUARE SCALE SLIDER-"),
				sg.Slider((0, 20), 12, orientation="h", size=(30, 15), key="-SQUARE NEIGHBOUR SLIDER-")
			],
			[
				sg.Text('Triangle', size=(8, 0), justification="left"),
				sg.Slider((1, 1000), 800, orientation="h", size=(30, 15), key="-TRIANGLE SCALE SLIDER-"),
				sg.Slider((0, 20), 12, orientation="h", size=(30, 15), key="-TRIANGLE NEIGHBOUR SLIDER-")
			],
			[
				sg.Text('Device', size=(8, 0), justification="left"),
				sg.Slider((1, 2000), 800, orientation="h", size=(30, 15), key="-DEVICE SCALE SLIDER-"),
				sg.Slider((0, 20), 12, orientation="h", size=(30, 15), key="-DEVICE NEIGHBOUR SLIDER-")
			],
			[sg.Text("", size=(80, 0), justification="center")],
			# Line 11
			[sg.Text("Device Contour Detection", size=(80, 0), justification="center", font=('Arial', 10, 'bold'))],
			# Line 12: Thresholding
			[
				sg.Text("Thresholding Value:", size=(20, 1), justification="left"),
				sg.Slider((0, 255), 127, orientation="h", size=(50, 15), key="-THRESHOLD SLIDER-")
			],
			# Line 13-14: Morphological Transformations / Noise Reduction Iterations
			[
				sg.Text("Iterations for Opening:", size=(20, 1), justification="left"),
				sg.Slider((0, 5), 0, orientation="h", size=(50, 15), key="-OPENING SLIDER-")
			],
			[
				sg.Text("Iterations for Closing:", size=(20, 1), justification="left"),
				sg.Slider((0, 5),0, orientation="h", size=(50, 15), key="-CLOSING SLIDER-")
			],
			# Line 15
			[sg.Button("OK", size=(10, 1))],
		]

		self.window = sg.Window("Controller", self.layout, location=(700, 100))


	def detect(self):

		data = {}
		while (1):
			event, values = self.window.read(timeout=0)

			# Values from PySimpleGUI
			scaleFactor_device = 1+ (int(values["-DEVICE SCALE SLIDER-"]) / 1000)

			neig_device = int(values["-DEVICE NEIGHBOUR SLIDER-"])

			beta = int(values["-BRIGHTNESS SLIDER-"])
			alpha = int(values["-CONTRAST SLIDER-"])
			threshold = int(values["-THRESHOLD SLIDER-"])
			opening_iter = int(values["-OPENING SLIDER-"])
			closing_iter = int(values["-CLOSING SLIDER-"])

			if values["-KERNEL DISABLE-"]:
				applyBlur = False
			else:
				applyBlur = True
				if values["-3x3 KERNEL SIZE-"]:
					blur_kernel = 3
				elif values["-5x5 KERNEL SIZE-"]:
					blur_kernel = 5
				elif values["-9x9 KERNEL SIZE-"]:
					blur_kernel = 9

			# Capture the frame
			_, frame = self.cam.read()
			center_x = int(frame.shape[1] / 2)
			center_y = int(frame.shape[0] / 2)

			# Brightness & Contrast  
			frame = cv2.addWeighted(frame, alpha, np.zeros(frame.shape, frame.dtype), 0, beta)

			# Blurring
			if applyBlur:
				frame = cv2.GaussianBlur(frame,(blur_kernel,blur_kernel),0)

			gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# Detect the object using the Cascade
			object_device = cascade_device.detectMultiScale(image=gray_img,scaleFactor=scaleFactor_device,minNeighbors=neig_device)
					
			# Sorting the devices from left-to-right and top-to-bottom
			if len(object_device) < 1:
				pass
			else:
				df = pd.DataFrame(object_device)
				df.rename(columns={0: 'x', 1: 'y', 2: 'w', 3: 'h'},inplace=True)
				df.sort_values(by='y', ascending=True, inplace=True)
				df.reset_index(inplace = True, drop = True)
				df['row'] = np.ones(len(df), dtype='int64')
				row_num = 1
				for i in np.arange(1,len(df)): 
					# If diff in y-coordinates > 30, assign next row (adjustable)
					if (abs(df.loc[i,'y'] - df.loc[i-1,'y']) > 30):             
						row_num += 1
						df.loc[i,'row'] = row_num
					else:
						df.loc[i,'row'] = row_num
				df.sort_values(by=['row','x'], ascending=[True,True], inplace=True) 
				df.reset_index(inplace = True, drop = True)
				object_device = df

			for i in range(len(object_device)):
				x = object_device.loc[i,'x']
				y = object_device.loc[i,'y']
				w = object_device.loc[i,'w']
				h = object_device.loc[i,'h']
				area = w*h
				if area >= 36*36:
					cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
					cv2.circle(frame, (int(x+(w/2)), int(y+(h/2))), 3, (0,0,255), -1)
					cv2.putText(frame, '{}'.format(i+1), (x-8, y-4), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.6, (255,255,255), 1)
					roi_color = frame[y:y+h, x:x+w]
					roi_gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)

					# Morphological Transformations - Noise Removal
					kernel = np.ones((3,3),np.uint8)
					roi_gray = cv2.morphologyEx(roi_gray,cv2.MORPH_OPEN,kernel,iterations=opening_iter)
					roi_gray = cv2.morphologyEx(roi_gray,cv2.MORPH_CLOSE,kernel,iterations=closing_iter)

					# Image Thresholding: Inverse Binary Thresholding + the OTSU method
					ret, thresh = cv2.threshold(roi_gray, threshold, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
					
					# Contour Detection
					contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
					contours = contours[1:]
					cv2.drawContours(roi_color, contours, -1, (255,0,0), 2)
					data['C'+str(i+1)] = [int(x+(w/2)), int(y+(h/2))]
						
			cv2.imshow("Result", frame)
			cv2.imshow("Grayscale", gray_img)
			
			if event == "OK" or event == sg.WIN_CLOSED or event is None:
				break

			k = cv2.waitKey(5) & 0xFF
			if k == 27:         # wait for ESC key to exit
				break

		self.cam.release()
		cv2.destroyAllWindows()
		self.window.close()

		img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		fig = plt.figure(figsize=(8, 6))
		plt.imshow(img)
		plt.xlabel('x')
		plt.ylabel('y')
		plt.title('Result')
		plt.show()

		fig.savefig(os.path.join(self.result_folder + '/','haarcascade.png'), dpi=fig.dpi)

		# Save CSV file: Coordinates
		data = pd.DataFrame(data)
		data = data.T
		data = data.copy()
		data.rename(columns={0: 'x', 1: 'y'},inplace=True)
		data.to_csv(os.path.join(self.result_folder + '/','from_origin_pixel.csv'))

		# Save CSV file: Coordinates from Center
		data['x'] = (data['x'] - center_x) / calibUnit
		data['y'] = (center_y - data['y']) / calibUnit
		data.to_csv(os.path.join(self.result_folder + '/','from_center.csv'))

		# Save CSV file: Coordinates from Camera
		data['x'] = data['x'] + cam_x 
		data['y'] = data['y'] + cam_y
		data.to_csv(os.path.join(self.result_folder + '/','from_cam.csv'))

		# Save CSV file: Coordinates from Probe
		data['x'] = data['x'] + probe_x
		data['y'] = data['y'] + probe_y
		data.to_csv(os.path.join(self.result_folder + '/','from_probe.csv'))


	def get_pos(self, filename):
		positions = pd.read_csv(filename, index_col = 0)

		positions.columns=["X","Y"]

		return [[positions.iat[i-1, 0],positions.iat[i-1, 1]] for i in range(len(positions)) ]






		



class Probe_Arm(object):
	"""
		The implementation for a robotic arm for managing the probe.
	"""
	def __init__(self, base_pos, base_orient, seg_lengths, range_trans, tol = 0.01, comm_line = None, baud_rate = None, hard_run = False):
		super(Probe_Arm, self).__init__()
		self.home_base_pos = base_pos
		self.home_base_orient = base_orient
		self.seg_lengths = seg_lengths
		self.range_trans = range_trans
		self.tol = tol

		self.hard_run = hard_run
		self.baud_rate = baud_rate
		self.comm_line = comm_line
		if self.hard_run:
			self.cnc = serial.Serial(self.comm_line, self.baud_rate)

		self.home_pos = {

			"base" : self.home_base_pos.copy(),
			"e1" : self.home_base_pos.copy(),
			"end" : self.home_base_pos.copy()
		}

		self.home_orient = {
			"base" : self.home_base_orient.copy(),
			"e1" : self.home_base_orient.copy(),
			"end" : self.home_base_orient.copy()
		}


		self.Base = Joint(
			name = "base", 
			dof = [[1,0,0],[0,0]], 
			pos = self.home_pos["base"],
			orient = self.home_orient["base"],
			range_trans = self.range_trans[0],
			range_rot = [[0,0],[0.01,0.01]]
			)

		self.Elbow1 = Joint(
			name = "e1", 
			dof = [[0,1,0],[0,0]], 
			pos = self.home_pos["e1"],
			orient = self.home_orient["e1"],
			range_trans = self.range_trans[1],
			range_rot = [[0,0],[0.01,0.01]]
			)

		self.End = Joint(
			name = "end", 
			dof = [[0,0,0],[0,0]], 
			pos = self.home_pos["end"],
			orient = self.home_orient["end"],
			range_trans = self.range_trans[2],
			range_rot = self.range_rot[2]
			)

		self.Seg1 = Segment("base_e1", self.Base, self.Elbow1)
		self.Seg2 = Segment("e1_end", self.Elbow1, self.End)

		self.manager = SegmentManager([self.Seg1, self.Seg2], self.Seg1.base, self.Seg2.end)

		if self.hard_run:
			self.setup_connection()
			self.send_pos()

	def setup_connection(self):
		self.cnc.close()
		self.cnc.open()

		'''
		
		time.sleep(2)   # Wait for grbl to initialize
		self.cnc.flushInput()
		'''

		#Homing cycle
		#self.cnc.write(self.home())
		self.send_pos()
		#print(self.cnc.readline())

		time.sleep(10)

	def end_to_target(self, target):
		t_x, t_y, t_z = target

		trans = [tar-curr for tar, curr in zip(target, self.manager.end.pos)]





	def send_pos(self):
		pass



		



if __name__ == "__main__":
	test_bot = Cam_Arm(
		base_pos = [0,0,0], 
		base_orient = [0,0], 
		seg_lengths = [1, 1, 1], 
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

	tol = 0.01

	def in_range(pos, reach, tar, tol):
		'''
		if reach:
			return all([target - tol <= posi <= target + tol for target, posi in zip(tar, pos)])
		return True
		'''
		return tol < math.sqrt(sum([(target-posi) ** 2 for target, posi in zip(tar, pos)]))
		#return all([target - tol <= posi <= target + tol for target, posi in zip(tar, pos)])

	# Test in the shape of a circle with radius 2 and height 1
	print("TEST 1 STARTED----")
	test_num = 4
	z = 1
	length = 2
	for n in range(1, test_num):
		tar_alpha = n*360/test_num
		target = [length * math.cos(n*360/test_num * (math.pi/180)), length * math.sin(n*360/test_num * (math.pi/180)), z]
		reach = test_bot.end_to_target(target)

		'''
		diff = [tar-curr for tar, curr in zip(target, test_bot.manager.end_joint.pos)]
		print(f"BASE ORIENT::{test_bot.manager.base_joint.orient}")
		print(f"TEST 1 #{n} WITH:: TARGET:: {target} :: END:: {test_bot.manager.end_joint.pos}")
		print(f"DIFF::{diff}")
		test_bot.visualize(axis_range = [[-4,4],[-4,4],[0,4]], target = target, has_target = True)
		'''
		test_bot.visualize(axis_range = [[-4,4],[-4,4],[0,4]], target = target, has_target = True)

		if not in_range(test_bot.manager.end_joint.pos, reach, target, tol):
			diff = [tar-curr for tar, curr in zip(target, test_bot.manager.end_joint.pos)]
			print(f"FAILURE AT TEST 1 #{n} WITH:: TARGET:: {target} :: END:: {test_bot.manager.end_joint.pos}")
			print(f"DIFF::{diff}")

			print("\n")
			for joint_name, param in test_bot.manager.get_joint_param().items():
				print("JOINT ", joint_name, ":")
				print(param)
			test_bot.visualize(axis_range = [[-4,4],[-4,4],[0,4]], target = target, has_target = True)
			break
	print("TEST 1 ENDED----")

	z = 3/math.sqrt(2)
	length = 3/math.sqrt(2)
	#target = [2, 2, 1]
	#target = [length, 0, z]
	target = [0,0,3]
	reach = test_bot.end_to_target(target)
	diff = [tar-curr for tar, curr in zip(target, test_bot.manager.end_joint.pos)]

	print(f"BASE ORIENT::{test_bot.manager.base_joint.orient}")
	print(f"TEST 2 WITH:: TARGET:: {target} :: END:: {test_bot.manager.end_joint.pos}")
	print(f"DIFF::{diff}")
	for joint_name, param in test_bot.manager.get_joint_param().items():
		print("JOINT ", joint_name, ":")
		print(param)
	test_bot.visualize(axis_range = [[-4,4],[-4,4],[0,4]], target = target, has_target = True)


	
	#Test with arm fully stretched in circle
	'''
	print("TEST 2 STARTED----")
	test_num = 4
	z = 3/math.sqrt(2)
	length = 3/math.sqrt(2)

	for n in range(1, test_num):
		tar_alpha = n*360/test_num
		target = [length * math.cos(n*360/test_num * (math.pi/180)), length * math.sin(n*360/test_num * (math.pi/180)), z]
		reach = test_bot.end_to_target(target)

		
		diff = [tar-curr for tar, curr in zip(target, test_bot.manager.end_joint.pos)]
		print(f"BASE ORIENT::{test_bot.manager.base_joint.orient}")
		print(f"TEST 2 #{n} WITH:: TARGET:: {target} :: END:: {test_bot.manager.end_joint.pos}")
		print(f"DIFF::{diff}")
		test_bot.visualize(axis_range = [[-4,4],[-4,4],[0,4]], target = target, has_target = True)
		


		if not in_range(test_bot.manager.end_joint.pos, reach, target, tol):
			diff = [tar-curr for tar, curr in zip(target, test_bot.manager.end_joint.pos)]
			print(f"FAILURE AT TEST 2 #{n} WITH:: TARGET:: {target} :: END:: {test_bot.manager.end_joint.pos}")
			print(f"DIFF::{diff}")

			print("\n")
			for joint_name, param in test_bot.manager.get_joint_param().items():
				print("JOINT ", joint_name, ":")
				print(param)
			test_bot.visualize(axis_range = [[-4,4],[-4,4],[0,4]], target = target, has_target = True)
			break
	print("TEST 2 ENDED----")
	'''
		















