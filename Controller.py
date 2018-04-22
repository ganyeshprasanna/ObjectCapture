from ABB import ABB_IRB_140
import numpy as np

class Controller(ABB_IRB_140):
	def __init__(self,desiredPosition):
		ABB_IRB_140.__init__(self)
		self.transformation_d 		 = np.identity(4)
		self.transformation_d[0:3,3] = desiredPosition
		self.desiredAngles 			 = self.inverseKinematics(self.transformation_d)
		self.Kp			 			 = np.identity(3)*10
		self.Kd			             = np.identity(3)*2.5
		self.desiredVelocity	  	 = np.zeros(3)

	def pdController(self,angles,velocity):
		angleError    = np.array(angles) - self.desiredAngles
		velocityError = np.array(velocity) - self.desiredVelocity
		torques 	  = np.dot(self.Kp,angleError) + np.dot(self.Kv,velocityError)
		return list(torques)