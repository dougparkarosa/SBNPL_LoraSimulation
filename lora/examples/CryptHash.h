#ifndef CRYPT_HASH_H
#define CRYPT_HASH_H

// Class to allow access to SHA hash.

#include <array>
#include <ios>
#include <iostream>
#include <vector>

namespace CryptHash {
using Byte = uint8_t;
const int SHALength = 32;
using HashSHA256 = std::array<Byte, SHALength>;

HashSHA256 SHA256Impl(const Byte data[], size_t len);
// Wrap up the scary SHA256 code so it is harder to mess up.
// We could create a more elaborate enable_if here to prevent other options.
template <typename ArrayOrVectorT>
HashSHA256 SHA256(const ArrayOrVectorT &data) {
  return SHA256Impl(&data[0], data.size());
}

} // namespace CryptHash

inline std::ostream &operator<<(std::ostream &os,
                                const CryptHash::HashSHA256 &hash) {
  os << std::hex;
  for (auto b : hash) {
    os << static_cast<unsigned>(b);
  }
  return os;
}

#endif // CRYPT_HASH_H