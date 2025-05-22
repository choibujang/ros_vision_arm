import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ros_interfaces.action import DispatchManipulationTask


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager_node')
        self._action_client = ActionClient(self, DispatchManipulationTask, 'dispatch_manipulation_task')

    def send_goal(self, item_names, item_quantities):
        goal_msg = DispatchManipulationTask.Goal()
        goal_msg.item_names = item_names
        goal_msg.item_quantities = item_quantities

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"success: {result.success}, \
                               error code: {result.error_code}, \
                               error msg: {result.error_msg}")
        rclpy.shutdown()

    

def main(args=None):
    rclpy.init(args=args)

    action_client = TaskManagerNode()

    future = action_client.send_goal(["apple", "banana", "carrot", "date"], [3,2,1,30])

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()