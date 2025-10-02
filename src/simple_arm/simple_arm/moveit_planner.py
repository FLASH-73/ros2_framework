import rclpy
from rclpy.node import Node
from moveit_planning_interface import MoveItPy  # Or moveit_commander if preferred
from geometry_msgs.msg import PoseStamped

class MoveItPlannerNode(Node):
    def __init__(self):
        super().__init__('moveit_planner_node')
        self.moveit_py = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit_py.get_planning_component("arm")  # From your SRDF

    def plan_and_execute(self):
        # Example: Plan to a pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.4
        # Set orientation...

        self.arm.set_goal(target_pose, "end_effector_link")  # Tweak for your EE
        plan = self.arm.plan()
        if plan:
            self.arm.execute()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItPlannerNode()
    node.plan_and_execute()  # Run once; add loop for ongoing
    rclpy.shutdown()

if __name__ == '__main__':
    main()