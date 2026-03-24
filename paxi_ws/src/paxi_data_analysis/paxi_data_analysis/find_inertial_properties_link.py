# Copyright 2026 JustASimpleCoder
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


from dataclasses import dataclass

from typing import List
from typing import Optional
from typing import Union

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import String


@dataclass
class InertialMomentTensor:
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0
    izx: float = 0.0
    izy: float = 0.0
    iyx: float = 0.0


@dataclass
class LinkCommon:
    name: str
    geometry_type: Optional[str] = None
    mass: float = 0.0
    moment_tensor: InertialMomentTensor = None


@dataclass
class SphereLink(LinkCommon):
    geometry_type = 'sphere'
    radius: float = 0.0

    def __print_str__(self) -> str:
        return (
            f'link [{self.name}] | type [{self.geometry_type}]\n'
            f'     radius [{self.radius}]\n'
            f'     Tensor [{self.moment_tensor}]\n'
        )


@dataclass
class CylinderLink(LinkCommon):
    geometry_type = 'cylinder'
    radius: float = 0.0
    length: float = 0.0

    def __print_str__(self) -> str:
        return (
            f'link [{self.name}] | type [{self.geometry_type}]\n'
            f'     radius [{self.radius}]\n'
            f'     length [{self.length}]\n'
            f'     Tensor [{self.moment_tensor}]\n'
        )


@dataclass
class BoxLink(LinkCommon):
    geometry_type = 'box'
    radius: float = 0.0
    length: float = 0.0  # x
    width: float = 0.0  # y
    height: float = 0.0  # z

    def __print_str__(self) -> str:
        return (
            f'link [{self.name}] | type [{self.geometry_type}]\n'
            f'     length [{self.length}]\n'
            f'     width  [{self.width}]\n'
            f'     height [{self.width}]\n'
            f'     Tensor [{self.moment_tensor}]'
        )


class GetInertialMoment:

    def cylinder(self, radius: float, length: float, mass: float) -> InertialMomentTensor:
        moment = InertialMomentTensor()

        moment.ixx = (1 / 12) * mass * (3 * (radius ** 2) + length ** 2)
        moment.iyy = moment.ixx
        moment.izz = (1 / 2) * mass * (radius ** 2)
        return moment

    def sphere(self, radius: float, mass: float) -> InertialMomentTensor:
        moment = InertialMomentTensor()

        moment.ixx = (2 / 5) * mass * (radius ** 2)
        moment.iyy = moment.ixx
        moment.izz = moment.ixx

        return moment

    def box(self, x: float, y: float, z: float, mass: float) -> InertialMomentTensor:
        moment = InertialMomentTensor()

        moment.ixx = (1 / 12) * mass * (y ** 2 + z ** 2)
        moment.iyy = (1 / 12) * mass * (x ** 2 + z ** 2)
        moment.izz = (1 / 12) * mass * (x ** 2 + y ** 2)

        return moment


# TODO(jacob): maybe turn this into dynaimcally changing xacro urdf file
# class UrdfIntertialWriter():
#     def __init__(self, filename):
#         self.home_dir = os.path.expanduser('~')
#         #to do make this a parameter
#         self.path_to_paxi_ws = '/robotics/paxi_bot_dev'

#         self.path_to_urdf = '/paxi_bot/paxi_ws/src/paxi_description/urdf/simulation'

#         self.full_xml_path = os.path.join(self.path_to_urdf, filename)
#         self.et = ET.ElementTree(file=self.full_xml_path)

#     def write_inertials(self, I : InertialMomentTensor, link_name : List[str]):
#         root = self.et.find(link_name)


# 'paxi_base_inertials.xacro'
# 'paxi_imu_inertials.xacro'
# 'paxi_imu_slamtec__inertials.xacro'
# 'paxi_lidar_inertials.xacro'
# 'paxi_wheels_inertials.xacro'


class UrdfParser(Node):

    def __init__(self):
        super().__init__('inertial_node')
        self.get_logger().info('Starting the linear regression node')

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.subscription = self.create_subscription(
            String, '/robot_description', self.robot_description_listener, qos
        )

        self.subscription

        self.links_info: List[Union[SphereLink, CylinderLink, BoxLink]] = []
        self.etree = ET.ElementTree()
        self.moment_cal = GetInertialMoment()

    def robot_description_listener(self, robot_description_xml_msg: String = None):
        try:
            self.etree = ET.fromstring(robot_description_xml_msg.data)
            self.get_logger().info('Successfully parsed Tree')

        except ET.ParseError as exc:
            self.get_logger().error(f'Failed to pars URDF xlm as Element Tree f{exc}')
        # else:
        #     self.get_logger().fatal(f'Failed to parse robot')

        if self.etree is not None:
            self.parse_links()
            self._print_inertials()

    def parse_links(self):
        self.etree.iter('link')
        for link_el in self.etree.iter('link'):
            link_name = link_el.get('name')
            visual_el = link_el.find('visual')
            if visual_el is None:
                continue

            geometry_el = visual_el.find('geometry')

            self._parse_visual(link_name, geometry_el)

        return

    def _parse_visual(self, link_name: str, geometry_el: ET.Element):
        cylinder = geometry_el.find('cylinder')
        box = geometry_el.find('box')
        sphere = geometry_el.find('sphere')

        if cylinder is not None:
            radius = float(cylinder.get('radius', 0.0))
            length = float(cylinder.get('length', 0.0))

            link = CylinderLink(
                name=link_name,
                mass=0.0,
                radius=radius,
                length=length,
                moment_tensor=self.moment_cal.cylinder(radius, length, 1),
            )

            self.links_info.append(link)

        if sphere is not None:
            radius = float(sphere.get('radius', 0.0))

            link = SphereLink(
                name=link_name,
                mass=0.0,
                radius=radius,
                moment_tensor=self.moment_cal.sphere(radius, 1.0),
            )

            self.links_info.append(link)

        if box is not None:
            size = box.get('size', 0.0)
            size = list(size.split(' '))
            x = float(size[0])
            y = float(size[1])
            z = float(size[2])
            link = BoxLink(
                name=link_name,
                mass=0.0,
                length=x,
                width=y,
                height=z,
                moment_tensor=self.moment_cal.box(x, y, z, 1.0),
            )

            self.links_info.append(link)

    def _print_inertials(self):
        self.get_logger().info(f'Parsed {len(self.links_info)} link(s):')
        if self.links_info is None:
            self.get_logger().info('Failed to print intertials')
            return

        for i in range(0, len(self.links_info)):
            self.get_logger().info(self.links_info[i].__print_str__())


def main(args=None):

    rclpy.init(args=args)
    rclpy.spin(UrdfParser())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
