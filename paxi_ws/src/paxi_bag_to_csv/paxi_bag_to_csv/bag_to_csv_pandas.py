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

from collections import OrderedDict

from dataclasses import dataclass
from dataclasses import field

import multiprocessing as mp

import os

import sqlite3

from typing import Optional
from typing import Tuple

import pandas as pd

from rclpy.serialization import deserialize_message

import rosidl_runtime_py
import rosidl_runtime_py.utilities as rosutil

import yaml


@dataclass
class BagInfo:

    file_uri: str = ''
    folder_path: str = ''
    full_path: str = field(default='', init=False)

    def __post_init__(self):
        self.full_path = os.path.join(self.folder_path, self.file_uri)


class BagPathYaml:

    def __init__(self, config_path: str):

        self.config_file_path = os.path.join(config_path, 'paxi_bag_path_info.yaml')
        self.bag_folder_path = ''
        self.bag_names = []

        self._get_bag_info()

    def _get_bag_info(self):
        with open(self.config_file_path, 'r') as file:
            data = yaml.safe_load(file)
            data = data['paxi_bag']
            self.bag_folder_path = data['bag_folder_path']
            self.bag_names = data['bag_names']


class MetadataYaml:

    def __init__(self, folder_path: str):

        self.metada_file_path = os.path.join(folder_path, 'metadata.yaml')
        self.topic_data = []
        self._get_topic_metadata()

    def _get_topic_metadata(self):
        with open(self.metada_file_path, 'r') as file:
            data = yaml.safe_load(file)
            for topic_data_with_msg_count in data['rosbag2_bagfile_information'][
                'topics_with_message_count'
            ]:

                self.topic_data.append(
                    (
                        topic_data_with_msg_count['topic_metadata']['name'],
                        topic_data_with_msg_count['topic_metadata']['type'],
                    )
                )


@staticmethod
def recursive_items(ordered_dict: OrderedDict, prefix: str = ''):

    for key, value in ordered_dict.items():
        full_key = f'{prefix}.{key}' if prefix else key

        if type(value) is OrderedDict:
            yield from recursive_items(value, full_key)

        # elif isinstance(value, list):
        #     for item in value:
        #         if isinstance(item, OrderedDict):
        #             frame = item.get('child_frame_id', '')
        #             sub_prefix = f'{full_key}.{frame}' if frame else full_key
        #             yield from recursive_items(item, sub_prefix)

        #         else:
        #             yield (full_key, item)

        else:
            yield (full_key, value)


class BagToCsv:

    def __init__(
        self, bag_folder_path: str, file_uri: str, out_csv_folder_path: Optional[str]
    ):

        self.bag_info = BagInfo(file_uri=file_uri, folder_path=bag_folder_path)

        print(f'connecting to path [{self.bag_info.full_path}]')

        # self.topic_names = topic_names
        self.csv_path = (
            self.bag_info.folder_path
            if out_csv_folder_path is None
            else out_csv_folder_path
        )

    def create_out_csv_folder_path(self) -> bool:
        if os.path.exists(self.csv_path):
            return True

        if not os.path.exists(self.csv_path):
            parent = os.path.exists(os.path.dirname(self.csv_path))
            if not parent:
                print(f'No Parent folder found for {parent}')
                return False

        print(f'Creating CSV folder in {self.bag_info.folder_path}')
        os.mkdir(self.csv_path)
        print(f'created path {self.csv_path}')
        return True

    def generate_csv_for_a_topic(self, topic_and_topic_msg_type: Tuple[str, str]):

        topic_name = topic_and_topic_msg_type[0]
        topic_msg_type = rosutil.get_message(topic_and_topic_msg_type[1])
        try:
            conn = sqlite3.connect(self.bag_info.full_path)
            bag_data_df_query = pd.read_sql_query(
                sql=f"""
                    SELECT timestamp, data
                    FROM messages
                        WHERE topic_id = (
                            SELECT id
                            FROM topics
                            WHERE name=='{topic_name}'
                        );
                    """,
                con=conn,
            )
            conn.close()
        except sqlite3.Error as er:
            print(
                f'Failed to connect to sql data base for path {self.bag_info.full_path}'
            )
            print(f'SQLite3 raised error {er}')
            print(f'Skipping csv creation for {topic_name} with type {topic_msg_type}')
            print()
            conn.close()
            return

        except pd.errors.DatabaseError as er:
            print(f'Failed to get bag data from pandas query for topic {topic_name}')
            print(f'Pandas raised an exception for swql query {er}')
            print(f'Skipping csv creation for {topic_name} with type {topic_msg_type}')
            print()
            conn.close()
            return

        rows = []
        timestamps = bag_data_df_query['timestamp'].tolist()
        raw_data = bag_data_df_query['data'].to_list()

        for timestamp, raw in zip(timestamps, raw_data):
            msg = deserialize_message(bytes(raw), topic_msg_type)
            flat = {'timestamp': timestamp}
            for key, value in recursive_items(
                rosidl_runtime_py.message_to_ordereddict(msg)
            ):
                flat[key] = value

            rows.append(flat)

        self._create_csv_file_from_df(pd.DataFrame(rows), topic_name)

    def _create_csv_file_from_df(self, topic_df: pd.DataFrame, topic_name: str):

        topic_name = topic_name.replace('/', '_')
        topic_name = topic_name[1:] if topic_name.startswith('_') else topic_name
        topic_df.to_csv(os.path.join(self.csv_path, topic_name + '.csv'), index=False)


def main():

    home_dir = os.path.expanduser('~')
    script_dir = os.path.dirname(os.path.realpath(__file__))
    config_dir = os.path.join(script_dir, 'config')

    bags_path_info = BagPathYaml(config_dir)

    for bag_name in bags_path_info.bag_names:

        bag_folder_path = os.path.join(
            home_dir, bags_path_info.bag_folder_path, bag_name
        )
        # database file same as bag name but with _0.db3 added
        file_uri = bag_name + '_0.db3'

        bag_2_csv = BagToCsv(
            bag_folder_path=bag_folder_path,
            file_uri=file_uri,
            out_csv_folder_path=os.path.join(bag_folder_path, 'csv'),
        )

        if not bag_2_csv.create_out_csv_folder_path():
            print(f'skipping csv generation for this bag {bag_name}')
            continue

        meta = MetadataYaml(bag_2_csv.bag_info.folder_path)

        print(f'Creating csvs for bag {bag_name}..... ')

        with mp.Pool(processes=None) as p:
            p.map(bag_2_csv.generate_csv_for_a_topic, meta.topic_data)

        print(f'..... finished creating csvs for {bag_name}!')
        print()


if __name__ == '__main__':
    main()
