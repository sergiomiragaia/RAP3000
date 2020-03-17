
import numpy as np
import transforms3d as t3d
from copy import copy

servoNames = ['A', 'B', 'C', 'D', 'E', 'F']
dofNames = ['x', 'y', 'z', 'pitch', 'roll', 'yaw']

#		F A
#	
#	E		 B
#	 D		C

#		^ Y (roll)
#	  Z up > X (pitch)
#	(yaw)

class Plataform6dof():
	
	def __init__(self, axis_dist, base_length, servo_height, armA_length, armB_lenght, joint_dist, joint_length, plataform_height):
		self.axis_dist = axis_dist
		self.base_length = base_length
		self.servo_height = servo_height
		self.armA_length = armA_length
		self.armB_lenght = armB_lenght
		self.joint_dist = joint_dist
		self.joint_length = joint_length
		self.plataform_height = plataform_height
		self.resting_height = np.math.sin(np.math.acos((axis_dist/2-armA_length-joint_dist/2)/armB_lenght))*armB_lenght + servo_height + plataform_height
		self.plataformPos = {'x':0.0, 'y':0.0, 'z':self.resting_height, 'pitch':0.0, 'roll':0.0, 'yaw':0.0}

		self.servoR = {'A':['even',30], 'B':['odd',30], 'C':['even',270], 'D':['odd',270], 'E':['even',150], 'F':['odd',150]}

		self.plataform_PfixedPoints = self._get_fixedPoints((self.base_length*np.math.sqrt(3)/6 + self.joint_length), self.joint_dist/2, -self.plataform_height)
		self.base_BfixedPoints = self._get_fixedPoints(self.base_length*np.math.sqrt(3)/6, self.axis_dist/2, self.servo_height)
		self.servoAngles = self._get_servoAngles(self.plataformPos)

	def _get_fixedPoints(self, length, p_dist, height):

		p0even = np.array([length, p_dist, height])
		p0odd = np.array([length, -p_dist, height])

		fixedPoints = dict()
		for s in servoNames:

			R = t3d.euler.euler2mat(0, 0, np.math.radians(self.servoR[s][1]), 'syxz')
			if self.servoR[s][0] == 'even':
				fixedPoints[s] = R.dot(p0even)
			elif self.servoR[s][0] == 'odd':
				fixedPoints[s] = R.dot(p0odd)

		return fixedPoints

	def _get_servoAngles(self, plataformPos):
		
		T = [plataformPos['x'], plataformPos['y'], plataformPos['z']]
		R = t3d.euler.euler2mat(
			np.math.radians(plataformPos['pitch']), 
			np.math.radians(plataformPos['roll']), 
			np.math.radians(plataformPos['yaw']), 
			'sxyz')
		M_P2B = t3d.affines.compose(T,R,np.ones(3),np.zeros(3))

		plataform_BfixedPoints = dict()
		servoAngles = dict()
		for s in servoNames:
			plataform_BfixedPoints[s] = M_P2B.dot(np.append(self.plataform_PfixedPoints[s],1.0))
			if self.servoR[s][0] == 'even':
				beta = -np.math.pi/2 + np.math.radians(self.servoR[s][1])
			elif self.servoR[s][0] == 'odd':
				beta = +np.math.pi/2 + np.math.radians(self.servoR[s][1])
			l = plataform_BfixedPoints[s][:3] - self.base_BfixedPoints[s]
			L = sum(l*l) - (self.armB_lenght**2 - self.armA_length**2) # L=l^2-(s^2-a^2)
			M = 2*self.armA_length*(plataform_BfixedPoints[s][2] - self.base_BfixedPoints[s][2]) # M = 2a(zp-zb)
			N = 2*self.armA_length*(
				(plataform_BfixedPoints[s][0] - self.base_BfixedPoints[s][0])*np.math.cos(beta)+
				(plataform_BfixedPoints[s][1] - self.base_BfixedPoints[s][1])*np.math.sin(beta)
			) # N=2a[cosB(xp-xb)+sinB(yp-yb)]
			alfa = np.math.asin(L/np.sqrt(M**2+N**2)) - np.math.atan(N/M)
			servoAngles[s] = np.math.degrees(alfa)
		return servoAngles

	def update_pos(self, x,y,z,pitch,roll,yaw):
		self.plataformPos['x'] += x
		self.plataformPos['y'] += y
		self.plataformPos['z'] += z
		self.plataformPos['pitch'] += pitch
		self.plataformPos['roll'] += roll
		self.plataformPos['yaw'] += yaw

		self.servoAngles = self._get_servoAngles(self.plataformPos)

def main():
	plataform = Plataform6dof(
		axis_dist = 320, 
		base_length = 580, 
		servo_height = 30, 
		armA_length = 50, 
		armB_lenght = 270, 
		joint_dist = 20, 
		joint_length = 31.5, #24.5+7
		plataform_height = 10)

	print(f"Pos: {plataform.plataformPos}\n")
	print(f"plataform_PfixedPoints: {plataform.plataform_PfixedPoints}\n")
	print(f"base_BfixedPoints: {plataform.base_BfixedPoints}\n")
	print(f"servoAngles: {plataform.servoAngles}\n")

	plataform.update_pos(0,5,0,0,0,0)

	print(f"Pos: {plataform.plataformPos}\n")
	print(f"plataform_PfixedPoints: {plataform.plataform_PfixedPoints}\n")
	print(f"base_BfixedPoints: {plataform.base_BfixedPoints}\n")
	print(f"servoAngles: {plataform.servoAngles}\n")

if __name__ == "__main__":
	main()
