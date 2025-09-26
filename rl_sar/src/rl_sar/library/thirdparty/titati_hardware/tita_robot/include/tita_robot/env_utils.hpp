#ifndef TITATI_ROBOT__ENV_UTILS_HPP_
#define TITATI_ROBOT__ENV_UTILS_HPP_

#include <cstdint>
#include <cstdlib>
#include <string>

namespace can_device
{
namespace detail
{
inline std::string get_env_or_default(const char *name, const std::string &default_value)
{
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == 0) {
    return default_value;
  }
  return std::string(value);
}

inline uint8_t get_env_u8(const char *name, uint8_t default_value)
{
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == 0) {
    return default_value;
  }
  char *end = nullptr;
  long parsed = std::strtol(value, &end, 0);
  if (end == value || parsed < 0 || parsed > 255) {
    return default_value;
  }
  return static_cast<uint8_t>(parsed);
}
}  // namespace detail
}  // namespace can_device

#endif  // TITATI_ROBOT__ENV_UTILS_HPP_
