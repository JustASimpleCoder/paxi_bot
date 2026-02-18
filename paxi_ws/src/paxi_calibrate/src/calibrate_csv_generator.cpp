// Copyright 2026 JustASimpleCoder
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "paxi_calibrate/calibrate_csv_generator.hpp"
#include <fstream>

CSVGenerator::CSVGenerator(std::string f_name_l, std::string f_name_r)
: filename_l_{f_name_l},
  filename_r_{f_name_r},
  csv_file_l_{},
  csv_file_r_{}
{
  csv_file_l_.open(filename_l_);
  csv_file_r_.open(filename_r_);

  csv_file_l_ << CSV_L_HEADER << std::endl;
  csv_file_r_ << CSV_R_HEADER << std::endl;
}

void CSVGenerator::add_line_l(
  double linear,
  double angular,
  const std::vector<double> & target,
  const std::vector<double> & feedback,
  const std::vector<double> & diff,
  const std::vector<double> & tf,
  const std::vector<double> & ft)
{
  if (linear == 0.0 && angular == 0.0) {
    // Skip rest wheel conditions
    return;
  }

  std::size_t size = diff.size();

  for (std::size_t i = 0u; i < size; ++i) {
    if (feedback[i] == 0.0) {
      // Remove garbage data
      continue;
    }

    csv_file_l_ << std::to_string(linear) << ",";
    csv_file_l_ << std::to_string(angular) << ",";
    csv_file_l_ << std::to_string(target[i]) << ",";
    csv_file_l_ << std::to_string(feedback[i]) << ",";
    csv_file_l_ << std::to_string(diff[i]) << ",";
    csv_file_l_ << std::to_string(tf[i]) << ",";
    csv_file_l_ << std::to_string(ft[i]) << std::endl;
  }
}

void CSVGenerator::add_line_r(
  double linear,
  double angular,
  const std::vector<double> & target,
  const std::vector<double> & feedback,
  const std::vector<double> & diff,
  const std::vector<double> & tf,
  const std::vector<double> & ft)
{
  if (linear == 0.0 && angular == 0.0) {
    // Skip reset wheel conditions
    return;
  }

  std::size_t size = diff.size();

  for (std::size_t i = 0u; i < size; ++i) {
    if (feedback[i] == 0.0) {
      // Remove garbage data
      continue;
    }

    csv_file_r_ << std::to_string(linear) << ",";
    csv_file_r_ << std::to_string(angular) << ",";
    csv_file_r_ << std::to_string(target[i]) << ",";
    csv_file_r_ << std::to_string(feedback[i]) << ",";
    csv_file_r_ << std::to_string(diff[i]) << ",";
    csv_file_r_ << std::to_string(tf[i]) << ",";
    csv_file_r_ << std::to_string(ft[i]) << std::endl;
  }
}

void CSVGenerator::close_files()
{
  csv_file_l_.close();
  csv_file_r_.close();
}
