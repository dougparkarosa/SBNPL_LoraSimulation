// Abstraction to allow using etl or std array.
#ifndef ARRAY_H
#define ARRAY_H

#include "LoraMeshConfiguration.h"

#if USE_ETL
#include <etl/array.h>
template <typename T, size_t N> using Array = etl::array<T, N>;
#else
#include <array>
template <typename T, size_t N> using Array = std::array<T, N>;
#endif

#endif // ARRAY_H