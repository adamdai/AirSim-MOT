import setup_path
import airsim
import pprint

import sys
import time

print("""This script is designed to fly on the streets of the Neighborhood environment
and assumes the unreal position of the drone is [160, -1500, 120].""")

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")

print("arming the drone...")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")

state = client.getMultirotorState(vehicle_name="Drone1")

print("taking off...")
f1 = client.takeoffAsync(vehicle_name="Drone1")
f2 = client.takeoffAsync(vehicle_name="Drone2")
f1.join()
f2.join()

time.sleep(1)

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7
print("ascending to hover altitude")
f1 = client.moveToZAsync(z, 1, vehicle_name="Drone1")
f2 = client.moveToZAsync(z, 1, vehicle_name="Drone2")
f1.join()
f2.join()

state1 = client.getMultirotorState(vehicle_name="Drone1").kinematics_estimated.position
print("Drone1 state: %s" % pprint.pformat(state1))

home = client.getHomeGeoPoint(vehicle_name="Drone1")
print("Drone1 home: %s" % pprint.pformat(home))

state2 = client.getMultirotorState(vehicle_name="Drone2").kinematics_estimated.position
print("Drone2 state: %s" % pprint.pformat(state2))

# pose1 = client.simGetVehiclePose(vehicle_name="Drone1")
# pose1.orientation.y_val += 45
# client.simSetVehiclePose(pose1, True, vehicle_name="Drone1")


print("flying on path...")
f1 = client.moveOnPathAsync([airsim.Vector3r(125,0,z),
                                airsim.Vector3r(125,-130,z),
                                airsim.Vector3r(0,-130,z),
                                airsim.Vector3r(0,0,z)],
                        7, 120,
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name="Drone1")

f2 = client.moveOnPathAsync([airsim.Vector3r(125-10,0,z),
                                airsim.Vector3r(125-10,-130,z),
                                airsim.Vector3r(-10,-130,z),
                                airsim.Vector3r(-10,0,z)],
                        7, 120,
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name="Drone2")

f1.join()
f2.join()


print("disarming...")
client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
print("resetting")
client.reset()
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")
print("done.")


if __name__ == "__main__":
