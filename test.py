from polymetis import RobotInterface, GripperInterface
robot = RobotInterface(ip_address="192.168.1.10", enforce_version=False)
gripper = GripperInterface(ip_address="192.168.1.10")

print(gripper.get_state().width)