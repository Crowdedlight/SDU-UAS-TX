import rospy
import numpy as np
from remote_control.msg import set_controller
from marker_attitude.msg import marker_info
import cv2
import math

def calc_angles(rot):
	roll = math.atan2(rot[1,0],rot[0,0])*180/math.pi
	pitch = math.atan2(-rot[2,0],math.sqrt(rot[2,1]**2+rot[2,2]**2))*180/math.pi
	yaw = math.atan2(rot[2,1],rot[2,2])*180/math.pi

	# rospy.loginfo("roll: {}, pitch: {}, yaw: {}".format(roll,pitch,yaw))

	return roll, pitch, yaw

class Controller:
	def __init__(self):
		self.command_pub = rospy.Publisher("/remote_control/set_controller", set_controller, queue_size=10)

		rospy.Subscriber("/markerlocator/attitude", marker_info, self.cb_marker_update)
		rospy.loginfo("Initializing")
		# rospy.Service("drone_controller/new_setpoint", Setpoint, self.new_setpoint)

		self.thrust = 50
		self.busy = False
		self.follow_camera = False
		self.is_landing = False
		self.landed = False

		self.GSD = 2.0/800
		self.setpoint = np.array([[0],[0],[40],[180]])
		self.current_pose = np.array([[0],[0],[0],[0]])
		self.marker_quality = 0

		self.P = 0.4
		self.I = 0#.002
		self.D = 0.001

		self.int_sum = np.array([[0],[0],[0],[0]])
		self.prev_error = np.array([[0],[0],[0],[0]])
		self.prev_time = None

	def cb_marker_update(self, msg):
		rvec = msg.rvec
		x = msg.tvec.x
		y = msg.tvec.y
		z = msg.tvec.z
		time_stamp = msg.time
		id = msg.id
		r = cv2.Rodrigues(np.array([rvec.x,rvec.y,rvec.z]))
		rot = r[0]
		_,_,yaw = calc_angles(rot)
		if yaw < 0:
			yaw += 360

		self.current_pose = np.array([[x],[y],[z],[yaw]])
		error = self.setpoint - self.current_pose


		camera2marker = np.array([[rot[0,0],rot[0,1],rot[0,2],0],
								[rot[1,0],rot[1,1],rot[1,2],0],
								[rot[2,0],rot[2,1],rot[2,2],0],
								[0,0,0,1]])

		marker2drone = np.array([[0,0,-1,0],
								 [-1,0,0,0],
								 [0,1,0,0],
								 [0,0,0,1]])

		camera2drone = np.matmul(camera2marker, marker2drone)
		drone2camera = np.linalg.inv(camera2drone)

		# rospy.loginfo("x: {}, y: {}, z: {}, yaw: {}".format(error[0],error[1],error[2],error[3]))
		# rospy.loginfo("x: {}, y: {}, z: {}, yaw: {}".format(cmd[0], cmd[1], cmd[2], cmd[3]))

		p = np.array([error[0],error[1],error[2],[1]])
		p = np.matmul(drone2camera,p)

		rospy.loginfo("x: {}, y: {}, z: {}, yaw: {}".format(p[0],p[1],p[2],error[3]))

		error = np.array([p[0],p[1],p[2],error[3]])
		cmd = self.pid_control(error)

		new_thrust = self.thrust + cmd[2]

		if new_thrust > 100:
			self.thrust = 100
		elif new_thrust < 0:
			self.thrust = 0
		else:
			self.thrust = new_thrust

		msg = set_controller(thrust = self.thrust, roll = cmd[0,0], pitch = cmd[1,0], yaw = cmd[3,0])
		self.command_pub.publish(msg)
		# rospy.loginfo(p)
		# cmd = self.transform_cmd(cmd,camera2drone)

	def transform_cmd(self, cmd, transform):
		p_cam = np.array([cmd[0],cmd[1],cmd[2],[1]])

		p_drone = np.matmul(transform,p_cam)

		return np.array([p_drone[0],p_drone[1],p_drone[2],cmd[3]])

	def run(self):
		pass


	def pid_control(self,error):
		curr_time = rospy.get_time()
		dedt = np.array([[0],[0],[0],[0]])
		de = error - self.prev_error
		if self.prev_time != None:
			dt = curr_time - self.prev_time
			if dt != 0:
				dedt = de / dt

			self.int_sum = self.int_sum + error*dt

		self.prev_time = curr_time

		cmd = self.P*error + self.I*self.int_sum + self.D*dedt

		return cmd
	'''
	def run(self):

		error = np.array([[0],[0],[0],[0]])

		rospy.loginfo(self.setpoint)

		if self.follow_camera:# and self.marker_quality > 0.2:
			error = self.marker_error

			# adjust height slowly if landing
			if self.is_landing and self.current_pose[2] < 2.5:
				self.setpoint[2] = self.setpoint[2] - 0.005

		else:
			# calculate angle and translation from 2D homogenous transformations
			x,y,angle = transform2D(self.current_pose,self.setpoint)
			error = self.setpoint - self.current_pose
			error[0] = x; error[1] = y; error[3] = -angle

		if abs(error[0]) < 0.5 and abs(error[1]) < 0.5 and abs(error[2]) < 0.5 and abs(error[3]) < 0.5 and self.busy:
			self.busy = False

			if self.follow_camera:
				self.ready_to_land = True

				if self.is_landing:
					# keep going
					self.busy = True

				if self.is_landing and self.current_pose[2] < 1:
					# stop following marker and land where you are
					self.setpoint = self.current_pose
					self.setpoint[2] = 0
					self.follow_camera = False
					self.is_landing = False


		cmd = self.pid_control(error)

		command = MoveDrone(x = cmd[0],#x_error,
							y = cmd[1],#y_error,
							z = cmd[2],#z_error,
							yaw = cmd[3])#angle_error)

		# don't try to regulate the drone if it has landed
		if not self.landed:
			self.command_pub.publish(command)

		#
		if self.current_pose[2] > 0.15:
			self.landed = False
		else:
			self.landed = True
			self.is_landing = False
'''
def main():
	rospy.init_node("camera_controller")
	rospy.sleep(1)

	controller = Controller()

	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		controller.run()
		rate.sleep()

	rospy.loginfo("Shutting down controller node")
