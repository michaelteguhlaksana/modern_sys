import modern_sys
import cv2
import os
'''
#testing from static image
haar  = modern_sys.HaarCascadeClassifier(mask_path = "./masks/haarcascade_device.xml", result_folder =  "./Results", calibUnit = 19.26)

raw_pos, frame_center = haar.detect()
offset = 0
haar.save_pos(raw_pos, offset, frame_center)
print(haar.get_pos(os.path.join(haar.result_folder + '/','from_cam.csv')))
'''

#testing from camera
haar  = modern_sys.HaarCascadeClassifier(mask_path = "./masks/haarcascade_device.xml", result_folder =  "./Results", calibUnit = 19.26, vid_cap = cv2.VideoCapture(1))

raw_pos, frame_center = haar.detect()
offset = (0,0)
haar.save_pos(raw_pos, offset, frame_center)
print(haar.get_pos(os.path.join(haar.result_folder + '/','from_cam.csv')))

