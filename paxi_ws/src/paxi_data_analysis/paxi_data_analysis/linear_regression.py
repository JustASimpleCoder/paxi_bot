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

import pandas as pd

# import numpy as np
from sklearn.linear_model import LinearRegression


class CSVLinRegression:
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

    def get_data_from_csv(self):
        pos_df = self.df[self.df['target_rpm'] > 0]

        self.X_pos = pos_df['feedback_rpm'].values.reshape(-1, 1)
        self.Y_pos = pos_df['target_rpm'].values

        neg_df = self.df[self.df['target_rpm'] < 0]

        self.X_neg = neg_df['feedback_rpm'].values.reshape(-1, 1)
        self.Y_neg = neg_df['target_rpm'].values

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
