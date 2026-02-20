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

CSVGenerator::CSVGenerator(std::string f_name)
: filename_{f_name},
  csv_file_{}
{
  csv_file_.open(filename_);
  csv_file_ << CSV_HEADER << std::endl;
}

void CSVGenerator::add_line(
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

  std::size_t size = std::min(
    {target.size(), feedback.size(), diff.size(), tf.size(), ft.size()}
  );

  for (std::size_t i = 0u; i < size; ++i) {
    if (feedback[i] == 0.0) {
      // Remove garbage data
      continue;
    }
    if (check_int16_overflow(target[i], feedback[i])) {
      // skip if an overflow happened
      continue;
    }

    csv_file_ << std::to_string(linear) << ",";
    csv_file_ << std::to_string(angular) << ",";
    csv_file_ << std::to_string(target[i]) << ",";
    csv_file_ << std::to_string(feedback[i]) << ",";
    csv_file_ << std::to_string(diff[i]) << ",";
    csv_file_ << std::to_string(tf[i]) << ",";
    csv_file_ << std::to_string(ft[i]) << std::endl;
  }
}

bool CSVGenerator::check_int16_overflow(double target_sample, double feedback_sample)
{
  if (target_sample > 0.0) {
    if (std::abs(feedback_sample - INT16_MIN) < OVERFLOW_THRESHOLD) {
      return true;
    }
  }
  if (target_sample < 0.0) {
    if (std::abs(feedback_sample - INT16_MAX) < OVERFLOW_THRESHOLD) {
      return true;
    }
  }
  return false;
}

void CSVGenerator::close_file()
{
  csv_file_.close();
}
