function sysCall_init()
	torques = {0,0,0,0,0,0}
	joints={-1,-1,-1}
    for i=1,3,1 do
        joints[i]=sim.getObjectHandle('IRB140_joint'..i)
    end
    pub = simROS.advertise('sensorABB', 'std_msgs/Float32MultiArray')--'sensor_msgs/JointState')
    simROS.publisherTreatUInt8ArrayAsString(pub) -- idk if this is needed
    sub = simROS.subscribe('torques','std_msgs/Float32MultiArray', 'torqueMessage_callback')
    simROS.subscriberTreatUInt8ArrayAsString(sub) -- idk if this is needed
    joint_angles={0,0,0}
    joint_velocities={0,0,0}
end

function torqueMessage_callback(msg)
	torques = msg.data
end

function sysCall_sensing()
	for i=1,3,1 do
		joint_velocities[i]=sim.getObjectFloatParameter(joints[i],2012)
		joint_angles[i]=sim.getJointPosition(joint[i])
	end
	for i=1,#joint_velocities do
        joint_angles[#joint_angles+1] = joint_velocities[i]
    end
	simROS.publish(pub,joint_angles)
end

function sysCall_actuation()
    for i=1,3,1 do
    	local epsilon = 1e-2
    	while math.abs(torques[i])>epsilon do
            sim.SetJointTargetVelocity(joints[i],0)
            sim.SetJointForce(joints[i],9999)
        end
        sim.SetJointTargetVelocity(clientID,joints[i],0,sim._opmode_oneshot)
        sim.SetJointForce(clientID,joints[i],9999,sim._opmode_oneshot)
	end
end	