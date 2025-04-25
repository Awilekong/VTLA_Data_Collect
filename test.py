from polymetis import RobotInterface, GripperInterface

gripper = GripperInterface(ip_address="192.168.1.100")

print(gripper.get_state().width)