#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from moveit.planning import MoveItPy
import time

class DualArmHoming(Node):
    def __init__(self):
        super().__init__('dual_arm_homing_node')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.fr3 = MoveItPy(node_name="moveit_py")
        self.left_arm = self.fr3.get_planning_component("left_fr3_arm")
        self.right_arm = self.fr3.get_planning_component("right_fr3_arm")

    def plan_and_execute(self, arm_component, arm_name):
        self.get_logger().info(f"Planning 'home' for {arm_name}...")
        arm_component.set_start_state_to_current_state()
        arm_component.set_goal_state(configuration_name="home")
        plan_result = arm_component.plan()
        if plan_result:
            self.get_logger().info(f"Executing plan for {arm_name}...")
            execution_result = self.fr3.execute(plan_result.trajectory, controllers=[])
            if execution_result:
                self.get_logger().info(f"{arm_name} homed successfully!")
            else:
                self.get_logger().error(f"Execution failed for {arm_name}")
        else:
            self.get_logger().error(f"Planning failed for {arm_name}")

    def go_home(self):
        self.plan_and_execute(self.left_arm, "Left Arm")
        self.plan_and_execute(self.right_arm, "Right Arm")

def main(args=None):
    rclpy.init(args=args)
    homing_node = DualArmHoming()
    homing_node.go_home()
    homing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()