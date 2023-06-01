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

// remap raw image to RGB color and save
void SaveRawToPPM(unsigned int w, unsigned int h, const float im[], std::ofstream& ofs);

}  // namespace mcpt
