# Copyright 2026 JustASimpleCoder
#
# Licensed under the Apache License, Version 2.0 (the "License");
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

from paxi_data_analysis.linear_regression import CSVLinRegression as lin_reg

import rclpy
from rclpy.node import Node


class LinRegNode(Node):

    def __init__(self):
        super().__init__('lin_reg')
        self.get_logger().info('Starting the linear regression node')

        self.l_wheel_lin_reg = lin_reg('Left_wheel.csv')
        self.r_wheel_lin_reg = lin_reg('right_wheel.csv')

    def get_data(self):
        self.l_wheel_lin_reg.get_data_from_csv()
        self.r_wheel_lin_reg.get_data_from_csv()

    def fit_model(self):
        self.r_wheel_lin_reg.fit_model()
        self.l_wheel_lin_reg.fit_model()

    def log_line_reg_data_right(self, model_type: str):
        idx = 0
        if model_type == 'pos':
            idx = 0
        if model_type == 'neg':
            idx = 1

        print(f'**** LEFT WHEEL DATA {model_type.upper()} ****')
        self.get_logger().info(
            f'R-squared Left {model_type} [{self.l_wheel_lin_reg.get_score()[idx]}]'
        )
        self.get_logger().info(
            f'Intercept Left {model_type} [{self.l_wheel_lin_reg.get_y_intercept()[idx]}]'
        )
        self.get_logger().info(
            f'Slope Left {model_type}     [{self.l_wheel_lin_reg.get_coefficient()[idx]}]'
        )

    def log_line_reg_data_left(self, model_type: str):
        idx = 0
        if model_type == 'pos':
            idx = 0
        if model_type == 'neg':
            idx = 1
        print(f'**** LEFT WHEEL DATA {model_type.upper()} ****')
        self.get_logger().info(
            f'R-squared Left {model_type} [{self.r_wheel_lin_reg.get_score()[idx]}]'
        )
        self.get_logger().info(
            f'Intercept Left {model_type} [{self.r_wheel_lin_reg.get_y_intercept()[idx]}]'
        )
        self.get_logger().info(
            f'Slope Left {model_type}     [{self.r_wheel_lin_reg.get_coefficient()[idx]}]'
        )


def main(args=None):

    rclpy.init(args=args)

    lin_reg_node = LinRegNode()
    # TODO: poll user input and wait for run model or something
    # rclpy.spin_once(lin_reg_node)

    lin_reg_node.get_data()
    lin_reg_node.get_data()

    lin_reg_node.fit_model()
    lin_reg_node.fit_model()

    lin_reg_node.log_line_reg_data_left('pos')
    lin_reg_node.log_line_reg_data_left('neg')

    lin_reg_node.log_line_reg_data_right('pos')
    lin_reg_node.log_line_reg_data_right('neg')

    lin_reg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# example usage
# def main():
#     l_wheel_lin_reg = csv_to_lin_regression('Left_wheel.csv')
#     r_wheel_lin_reg = csv_to_lin_regression('right_wheel.csv')

#     l_wheel_lin_reg.create_np_data()
#     r_wheel_lin_reg.create_np_data()

#     l_wheel_lin_reg.fit_model()
#     r_wheel_lin_reg.fit_model()

#     # l_wheel_y_pred = l_wheel_lin_reg.get_pred_values()
#     # r_wheel_y_pred = r_wheel_lin_reg.get_pred_values()

#     print('**** LEFT WHEEL DATA POS ****')
#     print(f'R-squared Left Pos [{l_wheel_lin_reg.get_score()[0]}]')
#     print(f'Intercept Left Pos [{l_wheel_lin_reg.get_y_intercept()[0]}]')
#     print(f'Slope Left Pos     [{l_wheel_lin_reg.get_coefficient()[0]}]')
#     print()

#     print('**** LEFT WHEEL DATA NEG ****')
#     print(f'R-squared Left Pos [{l_wheel_lin_reg.get_score()[1]}]')
#     print(f'Intercept Left Pos [{l_wheel_lin_reg.get_y_intercept()[1]}]')
#     print(f'Slope Left Pos     [{l_wheel_lin_reg.get_coefficient()[1]}]')
#     print()

#     print('**** Right WHEEL DATA POS****')
#     print(f'R-squared Right Pos [{r_wheel_lin_reg.get_score()[0]}]')
#     print(f'Intercept Right Pos [{r_wheel_lin_reg.get_y_intercept()[0]}]')
#     print(f'Slope Right Pos     [{r_wheel_lin_reg.get_coefficient()[0]}]')
#     print()

#     print('**** Right WHEEL DATA NEG****')
#     print(f'R-squared Right Pos [{r_wheel_lin_reg.get_score()[1]}]')
#     print(f'Intercept Right Pos [{r_wheel_lin_reg.get_y_intercept()[1]}]')
#     print(f'Slope Right Pos     [{r_wheel_lin_reg.get_coefficient()[1]}]')
#     print()
#     # print(r_wheel_lin_reg.df[r_wheel_lin_reg.df['target_rpm'] > 0].describe())


# if __name__ == '__main__':
#     main()
