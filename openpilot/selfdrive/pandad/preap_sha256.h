#pragma once
#include <array>
#include <cstdint>
#include <string_view>

// SHA-256 over the bounded serialized startup CP; no external runtime library.
inline std::array<uint8_t, 32> preap_sha256(std::string_view input) {
  constexpr std::array<uint32_t, 64> k = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
  };
  std::array<uint32_t, 8> hash = {0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                                  0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
  const auto rotate = [](uint32_t value, unsigned bits) { return (value >> bits) | (value << (32U - bits)); };
  const uint64_t bits = static_cast<uint64_t>(input.size()) * 8U;
  const size_t blocks = input.size() / 64U + ((input.size() % 64U < 56U) ? 1U : 2U);
  for (size_t block = 0; block < blocks; ++block) {
    std::array<uint32_t, 64> words{};
    for (size_t byte = 0; byte < 64U; ++byte) {
      const size_t position = block * 64U + byte;
      uint8_t value = 0;
      if (position < input.size()) value = static_cast<uint8_t>(input[position]);
      else if (position == input.size()) value = 0x80U;
      else if (block == blocks - 1U && byte >= 56U) value = static_cast<uint8_t>(bits >> ((63U - byte) * 8U));
      words[byte / 4U] |= static_cast<uint32_t>(value) << ((3U - byte % 4U) * 8U);
    }
    for (size_t i = 16; i < words.size(); ++i) {
      const uint32_t s0 = rotate(words[i - 15], 7) ^ rotate(words[i - 15], 18) ^ (words[i - 15] >> 3);
      const uint32_t s1 = rotate(words[i - 2], 17) ^ rotate(words[i - 2], 19) ^ (words[i - 2] >> 10);
      words[i] = words[i - 16] + s0 + words[i - 7] + s1;
    }
    auto [a, b, c, d, e, f, g, h] = hash;
    for (size_t i = 0; i < words.size(); ++i) {
      const uint32_t s1 = rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25);
      const uint32_t choose = (e & f) ^ (~e & g);
      const uint32_t t1 = h + s1 + choose + k[i] + words[i];
      const uint32_t s0 = rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22);
      const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + s0 + majority;
    }
    hash[0] += a; hash[1] += b; hash[2] += c; hash[3] += d;
    hash[4] += e; hash[5] += f; hash[6] += g; hash[7] += h;
  }
  std::array<uint8_t, 32> result{};
  for (size_t i = 0; i < result.size(); ++i) result[i] = static_cast<uint8_t>(hash[i / 4U] >> ((3U - i % 4U) * 8U));
  return result;
}
