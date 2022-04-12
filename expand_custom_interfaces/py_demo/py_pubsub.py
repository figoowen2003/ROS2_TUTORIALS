#! /usr/bin/env python3 
# 使用ros2 run运行python文件时，需要在py文件添加Shebang行，用于指定该脚本能够像可执行文件一样执行，但是不能使用#!/bin/sh

from asyncio.log import logger
from email.headerregistry import Address
import rclpy
from rclpy.node import Node
from expand_custom_interfaces.msg import AddressBook

class AddressBookPub(Node):

    def __init__(self):
        super().__init__('address_book_publiser')
        self.pub_ = self.create_publisher(AddressBook, 'address_book', 10)
        timer_period = 0.5 # second
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = AddressBook()
        msg.first_name = 'John'
        msg.last_name = 'Doe'
        msg.age = 30
        msg.gender = msg.MALE
        msg.address = 'Shanghai'
        self.pub_.publish(msg)
        self.get_logger().info('Publishing: first name "%s", address "%s"' % (msg.first_name, 
            msg.address))

def main(args=None):
    rclpy.init(args=args)
    myPub = AddressBookPub()
    rclpy.spin(myPub)

    myPub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()