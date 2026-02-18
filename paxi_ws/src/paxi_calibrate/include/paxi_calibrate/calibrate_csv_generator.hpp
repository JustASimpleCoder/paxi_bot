#ifndef PAXI_CALIBRATE__CALIBRATE_CSV_GENERATOR
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
#endif  // PAXI_CALIBRATE__CALIBRATE_NODE_HPP_
