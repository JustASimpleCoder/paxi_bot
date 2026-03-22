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

import sqlite3

from dataclasses import dataclass
from dataclasses import field

from typing import Optional

from typing import Tuple

from rclpy.serialization import deserialize_message

from rosidl_runtime_py.utilities import get_message

import csv

import yaml

import multiprocessing as mp


@dataclass
class BagInfo:
    file_uri: str = ""
    folder_path: str = ""
    full_path: str = field(default=folder_path + file_uri, init=False)

    def __post_init__(self):
        self.full_path = self.folder_path + "/" + self.file_uri


class MetadataYaml:
    def __init__(self, folder_path: str):

        self.metada_file_path = folder_path + "/metadata.yaml"
        self.topic_data = list(tuple())
        print(f"metadata path {self.metada_file_path}")

    def get_topic_metadata(self):
        with open(self.metada_file_path, "r") as file:
            data = yaml.safe_load(file)
            for topic_data_with_msg_count in data["rosbag2_bagfile_information"][
                "topics_with_message_count"
            ]:
                self.topic_data.append(
                    (
                        topic_data_with_msg_count["topic_metadata"]["name"],
                        topic_data_with_msg_count["topic_metadata"]["type"],
                    )
                )


class bag_to_csv:
    def __init__(
        self, bag_folder_path: str, file_uri: str, out_csv_folder_path: Optional[str]
    ):

        self.bag_info = BagInfo(file_uri=file_uri, folder_path=bag_folder_path)

        print(f"connecting to path [{self.bag_info.full_path}]")

        # self.topic_names = topic_names
        self.csv_path = (
            self.bag_info.folder_path
            if out_csv_folder_path is None
            else out_csv_folder_path
        )

    def generate_csv_for_a_topic(self, topic_and_topic_msg_type: Tuple[str, str]):

        conn = sqlite3.connect(self.bag_info.full_path)
        cursor = conn.cursor()

        topic, topic_msg_type = topic_and_topic_msg_type

        print(f"generating csv for {topic}...")
        if topic[0] == "/":
            topic = topic[1:]

        # type_str = self._get_msg_type_str(topic, cursor)
        msg_type = get_message(topic_msg_type)

        cursor.execute(
            """
            SELECT timestamp, data 
            FROM messages
            WHERE topic_id = (
                SELECT id
                FROM topics
                WHERE name==?
            );
        """,
            (topic,),
        )

        # sql_column_names = [column_info[0] for column_info in self.cursor.description]
        row = cursor.fetchone()
        if row is None:
            print(f"failed to get any data for topic [{topic}]")
            return

        msg = deserialize_message(bytes(row[-1]), msg_type)
        topic_column_names = ["timestamp"]
        topic_column_names += msg.get_fields_and_field_types().keys()
        print(topic_column_names)

        with open(
            self.csv_path + "_" + topic.replace("/", "_") + ".csv", mode="w", newline=""
        ) as file:
            writer = csv.writer(file)
            writer.writerow(topic_column_names)
            counter = 0
            for timestamp, data in cursor:
                msg = deserialize_message(bytes(data), msg_type)
                # TODO(Jacob): Get the actual field data into columns
                row = [timestamp] + [
                    getattr(msg, field)
                    for field in msg.get_fields_and_field_types().keys()
                ]
                writer.writerow(row)
                counter += 1

            print(f"wrote {counter}")

        conn.close()


def main():
    # TODO(Jacob): make this not-hardcoded
    bag_folder = "/home/j/robotics/paxi_bot_dev/paxi_bot/paxi_ws/sensors_bag_test_ACML_default_3_2026-03-14"
    file_uri = "sensors_bag_test_ACML_default_3_2026-03-14_0.db3"

    bag_2_csv = bag_to_csv(
        bag_folder,
        file_uri,
        "/home/j/robotics/paxi_bot_dev/paxi_bot/paxi_ws/bag_data_csv/",
    )

    meta = MetadataYaml(bag_2_csv.bag_info.folder_path)
    meta.get_topic_metadata()

    with mp.Pool(processes=None) as p:
        p.map(bag_2_csv.generate_csv_for_a_topic, meta.topic_data)


if __name__ == "__main__":
    main()
