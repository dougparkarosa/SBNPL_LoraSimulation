#ifndef ANISI_H
#define ANISI_H

#include <string>

namespace Ansi {

const std::string reset("\033[0m");
const std::string black("\033[30m");
const std::string red("\033[31m");
const std::string green("\033[32m");
const std::string yellow("\033[33m");
const std::string blue("\033[34m");
const std::string magenta("\033[35m");
const std::string cyan("\033[36m");
const std::string white("\033[37m");
const std::string boldblack("\033[1m\033[30m");
const std::string boldred("\033[1m\033[31m");
const std::string boldgreen("\033[1m\033[32m");
const std::string boldyellow("\033[1m\033[33m");
const std::string boldblue("\033[1m\033[34m");
const std::string boldmagenta("\033[1m\033[35m");
const std::string boldcyan("\033[1m\033[36m");
const std::string boldwhite("\033[1m\033[37m");

inline std::string ansi(const std::string &code) { return "\033[" + code; }

inline std::string clear() { return ansi("2J"); }
inline std::string clear_line() { return ansi("K"); }

inline std::string cursor_up(int n) {
  std::stringstream s;
  s << n << "A";
  return ansi(s.str());
}

inline std::string cursor_down(int n) {
  std::stringstream s;
  s << n << "B";
  return ansi(s.str());
}

inline std::string cursor_forward(int n) {
  std::stringstream s;
  s << n << "C";
  return ansi(s.str());
}

inline std::string cursor_back(int n) {
  std::stringstream s;
  s << n << "D";
  return ansi(s.str());
}

} // namespace Ansi

#endif // ANISI_H