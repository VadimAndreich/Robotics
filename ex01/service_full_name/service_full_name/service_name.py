from service_full_name_interfaces.srv import SummFullName

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SummFullName, 'full_name', self.handle_sum_full_name)

    def handle_sum_full_name(self, request, response):
        response.full_name = request.surname + " " + request.name + " " + request.patronymic
        self.get_logger().info('\nIncoming request:\nSurname: %s\nName: %s\nPatronymic: %s\n' % (request.surname, request.name, request.patronymic))
        self.get_logger().info('\nSending Answer: %s\n' % response.full_name)

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()