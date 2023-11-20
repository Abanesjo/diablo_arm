# Copyright 2019 Bold Hearts
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from cm730driver_msgs.srv import Ping

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('ping_motors_node')
    ping_client = node.create_client(Ping, '/cm730/ping')

    while not ping_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for CM730 driver to appear...')

    for device_id in range(21):
        req = Ping.Request()
        req.device_id = device_id
        future = ping_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        e = future.result().error
        print("Device {}:\t{}".format(device_id, "✔" if e == 0 else "✘: {}".format(e)))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
