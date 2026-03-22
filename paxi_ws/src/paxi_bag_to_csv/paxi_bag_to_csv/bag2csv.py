
import rclpy

import sqlite3

from dataclasses import dataclass

from typing import Optional

from typing import Tuple

from typing import List

from rclpy.serialization import deserialize_message

from rosidl_runtime_py.utilities import get_message

import csv
import yaml as yl

yam = yl.full_load()


@dataclass
class baginfo:
    file_uri: str  = ""
    folder_path: str = "" 
    full_path: str = file_uri + folder_path

class bag_to_csv():
    def __init__(self, bag_folder_path : str, file_uri : str, topic_names: List[str] ):
        
        self.bag_info = baginfo(file_uri=file_uri, folder_path=bag_folder_path, full_path=bag_folder_path + "/" + file_uri)

        print(f"connecting to path {self.bag_info.full_path}")
        self.conn = sqlite3.connect(self.bag_info.full_path)
        self.cursor = self.conn.cursor()

        self.topic_names = topic_names
        self.csv_path= self.bag_info.folder_path
    
    def generate_csv(self):

        for topic in self.topic_names:
            print(f"generating csv for {topic}...")
            if topic[0] == '/':
                topic = topic[1:]

            type_str = self._get_msg_type_str(topic)
            msg_type = get_message(type_str)

            self.cursor.execute(
            '''
                SELECT timestamp, data 
                FROM messages
                WHERE topic_id = (
                    SELECT id
                    FROM topics
                    WHERE name==?
                );
            ''', (topic,))

            # sql_column_names = [column_info[0] for column_info in self.cursor.description]
            row = self.cursor.fetchone()
            if row is None:
                print(f"failed to get any data for topic [{topic}]")
                continue


            msg = deserialize_message(bytes(row[-1]), msg_type)
            topic_column_names = ['timestamp']
            topic_column_names += (msg.get_fields_and_field_types().keys())
            print(topic_column_names)
            
            with open(self.csv_path + "_" + topic.replace("/","_"), mode ='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(topic_column_names)
                counter = 0
                for timestamp, data in self.cursor:
                    msg = deserialize_message(bytes(data), msg_type)
                    row = [timestamp] + [getattr(msg, field) for field in msg.get_fields_and_field_types().keys()]
                    writer.writerow(row)
                    counter += 1

                print(f"wrote {counter}")

        self.conn.close()

    def _get_msg_type_str(self, topic : str) -> str:
        if self.cursor is None:
            self.cursor = self.conn.cursor()

        self.cursor.execute('''
            SELECT type
            FROM topics
            WHERE name==?
        ''', (topic,))
        row = self.cursor.fetchone()
        return row[0]



def main():
    
    bag_2_csv = bag_to_csv(
        "/home/j/robotics/paxi_bot_dev/paxi_bot/paxi_ws/sensors_bag_test_ACML_default_3_2026-03-14", 
        "sensors_bag_test_ACML_default_3_2026-03-14_0.db3",
        [   "scan",
            "paxi/imu_raw",
            "imu/data",
            "joint_states",
            "hoverboard_base_controller/odom",
            "odometry/filtered",
            "tf",
            "tf_static"
        ])
    bag_2_csv.generate_csv()



if __name__ == '__main__':
    main()
