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

import pandas as pd

# import numpy as np
from sklearn.linear_model import LinearRegression


class csv_to_lin_regression:
    def __init__(self, csv_filename):
        self.filename = csv_filename
        self.df = pd.read_csv(self.filename)

        self.X_pos = None
        self.X_neg = None
        self.Y_pos = None
        self.Y_neg = None

        self.model_pos = LinearRegression()
        self.model_neg = LinearRegression()

    def get_X_Data(self):
        return [self.X_pos, self.X_neg]

    def get_Y_data(self):
        return [self.Y_pos, self.Y_neg]

    def create_np_data(self):
        pos_df = self.df[self.df["target_rpm"] > 0]

        self.X_pos = pos_df["feedback_rpm"].values.reshape(-1, 1)
        self.Y_pos = pos_df["target_rpm"].values

        neg_df = self.df[self.df["target_rpm"] < 0]

        self.X_neg = neg_df["feedback_rpm"].values.reshape(-1, 1)
        self.Y_neg = neg_df["target_rpm"].values

    def fit_model(self):
        self.model_pos.fit(self.X_pos, self.Y_pos)
        self.model_neg.fit(self.X_neg, self.Y_neg)

    def get_score(self):
        return [
            self.model_pos.score(self.X_pos, self.Y_pos),
            self.model_neg.score(self.X_neg, self.Y_neg),
        ]

    def get_pred_values(self):
        return [self.model_pos.predict(self.X_pos), self.model_neg.predict(self.X_neg)]

    def get_y_intercept(self):
        return [self.model_pos.intercept_, self.model_neg.intercept_]

    def get_coefficient(self):
        return [self.model_pos.coef_, self.model_neg.coef_]


def main():
    l_wheel_lin_reg = csv_to_lin_regression("Left_wheel.csv")
    r_wheel_lin_reg = csv_to_lin_regression("right_wheel.csv")

    l_wheel_lin_reg.create_np_data()
    r_wheel_lin_reg.create_np_data()

    l_wheel_lin_reg.fit_model()
    r_wheel_lin_reg.fit_model()

    # l_wheel_y_pred = l_wheel_lin_reg.get_pred_values()
    # r_wheel_y_pred = r_wheel_lin_reg.get_pred_values()

    print("**** LEFT WHEEL DATA POS ****")
    print(f"R-squared Left Pos [{l_wheel_lin_reg.get_score()[0]}]")
    print(f"Intercept Left Pos [{l_wheel_lin_reg.get_y_intercept()[0]}]")
    print(f"Slope Left Pos     [{l_wheel_lin_reg.get_coefficient()[0]}]")
    print()

    print("**** LEFT WHEEL DATA NEG ****")
    print(f"R-squared Left Pos [{l_wheel_lin_reg.get_score()[1]}]")
    print(f"Intercept Left Pos [{l_wheel_lin_reg.get_y_intercept()[1]}]")
    print(f"Slope Left Pos     [{l_wheel_lin_reg.get_coefficient()[1]}]")
    print()

    print("**** Right WHEEL DATA POS****")
    print(f"R-squared Right Pos [{r_wheel_lin_reg.get_score()[0]}]")
    print(f"Intercept Right Pos [{r_wheel_lin_reg.get_y_intercept()[0]}]")
    print(f"Slope Right Pos     [{r_wheel_lin_reg.get_coefficient()[0]}]")
    print()

    print("**** Right WHEEL DATA NEG****")
    print(f"R-squared Right Pos [{r_wheel_lin_reg.get_score()[1]}]")
    print(f"Intercept Right Pos [{r_wheel_lin_reg.get_y_intercept()[1]}]")
    print(f"Slope Right Pos     [{r_wheel_lin_reg.get_coefficient()[1]}]")
    print()
    # print(r_wheel_lin_reg.df[r_wheel_lin_reg.df['target_rpm'] > 0].describe())


if __name__ == "__main__":
    main()
