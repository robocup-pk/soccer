#ifndef READ_FILE_H
#define READ_FILE_H

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace util {
  std::string ReadFile(const std::string& path);
  std::string GetExecutableDir();
}


#endif // READ_FILE_H