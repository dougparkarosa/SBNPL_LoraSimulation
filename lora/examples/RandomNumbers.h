#ifndef RANDOM_NUMBERS
#define RANDOM_NUMBERS
#include <random>

namespace Random {
extern std::mt19937 mG;
void seed(unsigned int);
template <typename T> T uniform_real(T min_num, T max_num) {
  std::uniform_real_distribution<T> uniform_dist(min_num, max_num);
  return uniform_dist(mG);
}
template <typename T> T uniform_int(T min_num, T max_num) {
  std::uniform_int_distribution<T> uniform_dist(min_num, max_num);
  return uniform_dist(mG);
}
} // namespace Random

#endif // RANDOM_NUMBERS
