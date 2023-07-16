#pragma once

#include <fstream>
#include <istream>
#include <sstream>
#include <string>
#include <vector>

namespace mcpt {

// handle `\n' and `\r\n' correctly
std::string SafelyGetLineString(std::istream& is);
std::stringstream SafelyGetLineStream(std::istream& is);

// remap depth image to RGB color and save
void DepthToPPM(unsigned int w, unsigned int h, const float im[], std::ofstream& ofs);
// remap raw image to RGB color with gamma corrected and save
void RawToPPM(unsigned int w, unsigned int h, float gamma, const float im[], std::ofstream& ofs);

}  // namespace mcpt
