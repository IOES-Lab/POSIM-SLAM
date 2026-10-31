// SPDX-License-Identifier: Apache-2.0
// Run the caller-selected, unchanged POSIM RaySurfaceNormals.hh.
#include "RaySurfaceNormals.hh"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream in(argv[1], std::ios::binary);
  uint32_t rows=0, cols=0;
  double maximum=0;
  in.read(reinterpret_cast<char *>(&rows), sizeof(rows));
  in.read(reinterpret_cast<char *>(&cols), sizeof(cols));
  in.read(reinterpret_cast<char *>(&maximum), sizeof(maximum));
  if (!rows || !cols || rows > 512 || cols > 512) return 3;
  std::vector<float> az(cols), el(rows);
  cv::Mat ranges(rows, cols, CV_32FC1);
  in.read(reinterpret_cast<char *>(az.data()), cols*sizeof(float));
  in.read(reinterpret_cast<char *>(el.data()), rows*sizeof(float));
  in.read(reinterpret_cast<char *>(ranges.data), rows*cols*sizeof(float));
  if (!in || in.peek() != EOF) return 4;
  const auto normals=gz::sensors::RaySurfaceNormals(ranges, az, el, maximum);
  std::ofstream out(argv[2], std::ios::binary);
  out.write(reinterpret_cast<const char *>(normals.data), rows*cols*3*sizeof(float));
  if (!out) return 5;
  std::cout << CV_VERSION << '\n';
}
