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

#ifndef PAXI_CALIBRATE__CALIBRATE_CSV_GENERATOR_HPP_
#define PAXI_CALIBRATE__CALIBRATE_CSV_GENERATOR_HPP_


#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "paxi_calibrate/utility.hpp"

class CSVGenerator
{
public:
  CSVGenerator(std::string f_name_l, std::string f_name_r);
  ~CSVGenerator() = default;

  void add_line_l(
    double linear,
    double angular,
    const std::vector<double> & target,
    const std::vector<double> & feedback,
    const std::vector<double> & diff,
    const std::vector<double> & tf,
    const std::vector<double> & ft
  );

  void add_line_r(
    double linear,
    double angular,
    const std::vector<double> & target,
    const std::vector<double> & feedback,
    const std::vector<double> & diff,
    const std::vector<double> & tf,
    const std::vector<double> & ft
  );

  void close_files();

private:
  std::string filename_l_;
  std::string filename_r_;

  std::ofstream csv_file_l_;
  std::ofstream csv_file_r_;
};
#endif  // PAXI_CALIBRATE__CALIBRATE_CSV_GENERATOR_HPP_
