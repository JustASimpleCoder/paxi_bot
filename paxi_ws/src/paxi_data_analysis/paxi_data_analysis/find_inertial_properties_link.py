# Copyright 2026 JustASimpleCoder
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String

from typing import Optional

from dataclasses import dataclass, field

import xml.etree.ElementTree as ET

@dataclass
class InertialMomentTensor:
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

@dataclass
class LinkCommon:
    name: str
    geometry_type: Optional[str] = None
    mass: float = 0.0

@dataclass
class SphereLink(LinkCommon):
    geometry_type = "sphere"
    radius: float = 0.0
    def __print_str__(self) -> str:
        return (
            f"link [{self.name}] | type [{self.geometry_type}]"
            f"     radius [{self.radius}]"
        )

@dataclass
class CylinderLink(LinkCommon):
    geometry_type = "cylinder"
    radius: float = 0.0
    length: float = 0.0
    
    def __print_str__(self) -> str:
        return (
            f"link [{self.name}] | type [{self.geometry_type}]"
            f"     radius [{self.radius}]"
            f"     length [{self.length}]"
        )  

@dataclass
class BoxLink(LinkCommon):
    geometry_type = "box"
    radius: float = 0.0
    length: float = 0.0 # x
    width:  float = 0.0 # y
    height: float = 0.0 # z
    def __print_str__(self) -> str:
        return (
            f"link [{self.name}] | type [{self.geometry_type}]"
            f"     length [{self.length}]"
            f"     width  [{self.width}]"
            f"     height [{self.width}]"
        )


class GetInertialMoment:
    def cylinder(self, r:  float, l: float, m : float) -> list[float]:
 
        I_xx = (1/12)*m*( 3*(r**2)+ l**2) 
        I_yy = I_xx
        I_zz = (1/2)*m*(r**2)
        return [I_xx, I_yy, I_zz]
    
    def sphere(self, r: float, m: float) -> list[float]:
 
        I_xx = (2/5)*m*(r**2)
        I_yy = I_xx
        I_zz = I_xx

        return [I_xx, I_yy, I_zz]
     
    def box(self, x: float, y: float, z: float, m: float) -> list[float]:

        I_xx = (1/12)*m*(y**2 + z**2)
        I_yy = (1/12)*m*(x**2 + z**2)
        I_zz = (1/12)*m*(x**2 + y**2)
        return [I_xx, I_yy, I_zz]

class UrdfParser(Node):
    def __init__(self):
        super().__init__("inertial_node")
        self.get_logger().info("Starting the linear regression node")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.subscription = self.create_subscription(
            String, 
            '/robot_description',
            self.robot_description_listener,
            qos)
        
        self.subscription

        self.links_info = list()
        self.etree = ET.ElementTree()
        self.moment_cal = GetInertialMoment()

        
    def robot_description_listener(self, robot_description_xml_msg : String = None):
        try:
            self.etree = ET.fromstring(robot_description_xml_msg.data)
            self.get_logger().info("Successfully parsed Tree")

        except ET.ParseError as exc:
            self.get_logger().error(f"Failed to pars URDF xlm as Element Tree f{exc}")
        # else:
        #     self.get_logger().fatal(f"Failed to parse robot")

        if self.etree is not None:
            self.parse_links()
            self._print_inertials()
            
 
    def parse_links(self):
        self.etree.iter("link")
        for link_el in self.etree.iter("link"):
            link_name = link_el.get("name")
            visual_el = link_el.find("visual")
            if visual_el is None: 
                continue

            geometry_el = visual_el.find("geometry")

            self._parse_visual(link_name, geometry_el)
        
        return 
    

    def _parse_visual(self, link_name : str, geometry_el : ET.Element):
        cylinder = geometry_el.find("cylinder")
        box = geometry_el.find("box")
        sphere = geometry_el.find("sphere")

        if cylinder is not None:
            radius = float(cylinder.get("radius", 0.0))
            length = float(cylinder.get("length", 0.0))

            link = CylinderLink(
                name=link_name,
                mass= 0.0,
                radius=radius,
                length=length)
            
            self.links_info.append(link)

        if sphere is not None:
            radius = float(sphere.get("radius", 0.0))

            link = SphereLink(
                name=link_name,
                mass= 0.0,
                radius= radius)
            
            link.name = link_name
            self.links_info.append(link)

        if box is not None:
            size = box.get("size", 0.0)
            size = list(size.split(" "))
            x = float(size[0])
            y = float(size[1])
            z = float(size[2])
            link = BoxLink(
                name=link_name,
                mass= 0.0,
                length=x,
                width=y,
                height=z)

    def _print_inertials(self):
        self.get_logger().info(f"Parsed {len(self.links_info)} link(s):")
        if self.links_info is None:
            self.get_logger().info("Failed to print intertials")
            return

        for i in range(0, len(self.links_info)):
            self.get_logger().info(self.links_info[i].__print_str__())


def main(args=None):
    
    rclpy.init(args=args)
    rclpy.spin(UrdfParser())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
