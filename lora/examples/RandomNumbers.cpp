#include "RandomNumbers.h"

namespace Random {
std::mt19937 mG;
void seed(unsigned int seed_val) { mG.seed(seed_val); }
} // namespace Random
