
from time import time, strftime
import cv2
from cv2 import aruco
import numpy as np
import yaml
from cv_bridge import CvBridge, CvBridgeError
import rospy
from marker_attitude.msg import marker_info
from geometry_msgs.msg import Vector3
import MarkerInfo
import math
import rospkg

def calc_angles(rot):
	roll = math.atan2(rot[1,0],rot[0,0])*180/math.pi
	pitch = math.atan2(-rot[2,0],math.sqrt(rot[2,1]**2+rot[2,2]**2))*180/math.pi
	yaw = math.atan2(rot[2,1],rot[2,2])*180/math.pi

	# rospy.loginfo("roll: {}, pitch: {}, yaw: {}".format(roll,pitch,yaw))

	return roll, pitch, yaw

class VideoMarkerFinder:

	def __init__(self, dict, video_device):
		self.camera = cv2.VideoCapture(video_device)
		#self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		#self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
		self.aruco_dict = dict
		self.arucoParams = aruco.DetectorParameters_create()
		self.running = True

		self.prev_yaw = 0

		rospack = rospkg.RosPack()
		self.path = rospack.get_path('marker_attitude')
		print(self.path)

		ret, self.current_frame = self.camera.read()
		self.t0 = time()
		self.t1 = time()
		self.time_diff = time()
		self.rvec = None
		self.tvec = None
		self.ros_topic = "/markerlocator/attitude"
		self.marker_attitude_pub = rospy.Publisher(self.ros_topic, marker_info, queue_size=0)

		self.markers_detected = False

		self.calibrated = True
		if self.calibrated:
			self.maker_length = 7.3
			self.calibration()

	def publish_to_ros(self, id):
		#print(self.rvec)
		msgRvec = Vector3(self.rvec[0,0,0],self.rvec[0,0,1],self.rvec[0,0,2])
		msgTvec = Vector3(self.tvec[0,0,0],self.tvec[0,0,1],self.tvec[0,0,2])
		msg = marker_info(rvec = msgRvec, tvec = msgTvec, time = self.time_diff, id = id)
		try:
			self.marker_attitude_pub.publish(msg)
		except CvBridgeError as e:
			print(e)

	def do_the_magic(self):
		self.find_markers()
		self.draw_markers()
		self.show_video_with_markers()

	def calibration(self):
		with open(self.path+'/calibration_webcam_stdsize.yaml') as f:
			self.calibration_params = yaml.load(f)
		self.camera_matrix = np.array(self.calibration_params.get('camera_matrix'))
		self.dist_coeffs = np.array(self.calibration_params.get('dist_coeff'))
		self.image_height = self.calibration_params.get('height')
		self.image_width = self.calibration_params.get('width')
		image_size = (self.image_width, self.image_height)
		self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, image_size, 1, image_size)
		self.map1, self.map2 = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, self.newcameramtx, image_size, 5)

	def find_markers(self):
		ret, self.current_frame = self.camera.read()
		if self.calibrated:
			img_remapped = cv2.remap(self.current_frame, self.map1, self.map2, cv2.INTER_LINEAR)
			self.corners, self.ids, self.rejected_img_points = aruco.detectMarkers(img_remapped, self.aruco_dict,
																				   parameters=self.arucoParams)
		else:
			self.corners, self.ids, self.rejected_img_points = aruco.detectMarkers(self.current_frame, self.aruco_dict,
																				   parameters=self.arucoParams)
		if self.corners:
			self.markers_detected = True
		else:
			self.markers_detected = False

	def draw_markers(self):
		self.img_with_aruco = aruco.drawDetectedMarkers(self.current_frame, self.corners, self.ids, (255, 0, 255))

		if self.calibrated and self.markers_detected:
			(self.t1, self.t0) = (self.t0, time())
			self.time_diff = self.t0 - self.t1
			for i in range(len(self.ids)):
				self.rvec, self.tvec, _objPoints = aruco.estimatePoseSingleMarkers(self.corners[i], self.maker_length,
																		 self.camera_matrix, self.dist_coeffs)
				self.calc_rotation_matrix()

				# only publish and display if the orientation is correct
				if not self.z_axis_flipped():
					self.img_with_aruco = aruco.drawAxis(self.img_with_aruco, self.camera_matrix,
														 self.dist_coeffs, self.rvec, self.tvec, 6)
					self.publish_to_ros(self.ids[i])
					rospy.loginfo("Here")

				#self.some_vec = np.cross(self.rmat[:,0], self.rmat[:,2])
				#print self.rvec
				#print 'marker{}'.format(i)
				#print tvec

	def z_axis_flipped(self):
		# if the angle changes too quickly, then the axis has flipped
		_,_,yaw = calc_angles(self.rmat)

		dyaw = (yaw - self.prev_yaw) / self.time_diff
		rospy.loginfo(dyaw)
		self.prev_yaw = yaw

		if abs(dyaw) > 1000:
			return True

		return False


	def calc_rotation_matrix(self):
		self.rmat, _ = cv2.Rodrigues(self.rvec)

	def show_video_with_markers(self):
		self.img_with_aruco = cv2.flip(self.img_with_aruco, 1)
		cv2.imshow('Markers', self.img_with_aruco)

	def handle_keyboard_events(self):
		# Listen for keyboard events and take relevant actions.
		key = cv2.waitKey(1)
		# Discard higher order bit, http://permalink.gmane.org/gmane.comp.lib.opencv.devel/410
		key = key & 0xff
		if key == 27:  # Esc
			self.running = False
		if key == 114:  # R
			print("Resetting...")
		if key == 115:  # S
			# save image
			print("Saving image...")
			filename = strftime("%Y-%m-%d %H-%M-%S")
			cv2.imwrite("output/%s.png" % filename, self.current_frame)

	def close_all_windows(self):
		self.camera.release()
		cv2.destroyAllWindows()


def main():
	print 'Starting...'
	rospy.init_node('marker_attitude')
	#rospy.sleep(1)

	videoDevice = 0
	aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)
	vmf = VideoMarkerFinder(aruco_dict, videoDevice)

	while vmf.running == True:#not rospy.is_shutdown():
		vmf.do_the_magic()
		vmf.handle_keyboard_events()

	# When everything done, release the capture
	vmf.close_all_windows()



if __name__ == "__main__":
	main()
