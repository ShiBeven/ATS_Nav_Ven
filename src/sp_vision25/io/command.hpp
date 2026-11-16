#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

#include <string>
#include <vector>

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;
};

// Shooter-specific mode (moved from cboard.hpp; enum values unchanged,
// not part of any communication protocol struct).
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

}  // namespace io

#endif  // IO__COMMAND_HPP