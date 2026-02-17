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

  csv_file_l_ << CSV_L_HEADER;
  csv_file_r_ << CSV_L_HEADER;
}

void CSVGenerator::add_line_l(
  double linear, double angular, double target, double feedback,
  std::vector<double> diff, std::vector<double> tf, std::vector<double> ft)
{
  std::size_t size = diff.size();

  for (std::size_t i = 0u; i < size; ++i) {

    csv_file_l_ << std::to_string(linear) << ",";
    csv_file_l_ << std::to_string(angular) << ",";
    csv_file_l_ << std::to_string(target) << ",";
    csv_file_l_ << std::to_string(feedback) << ",";
    csv_file_l_ << std::to_string(diff[i]) << ",";
    csv_file_l_ << std::to_string(tf[i]) << ",";
    csv_file_l_ << std::to_string(ft[i]) << std::endl;
  }
}

void CSVGenerator::add_line_r(
  double linear, double angular, double target, double feedback,
  std::vector<double> diff, std::vector<double> tf, std::vector<double> ft)
{

  std::size_t size = diff.size();

  for (std::size_t i = 0u; i < size; ++i) {

    csv_file_r_ << std::to_string(linear) << ",";
    csv_file_r_ << std::to_string(angular) << ",";
    csv_file_r_ << std::to_string(target) << ",";
    csv_file_r_ << std::to_string(feedback) << ",";
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
