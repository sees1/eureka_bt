#pragma once
#include <deque>
#include <string>
#include <cmath>

struct Arrow {
  Arrow() = default;

  std::string direction;
  double distance;
  double angle;
};

class ArrowFilter {
public:
  ArrowFilter(size_t window,
              double allow_length_error = 2.0,
              double allow_angle_error = 3.0);

  void addArrow(Arrow&& arrow);
  Arrow getActualArrow();

  inline bool isBufferFull()
  {
    if (arrows_.size() == window_size_)
      return true;

    return false;
  }

private:
  size_t window_size_;
  double allow_length_error_;
  double allow_angle_error_;

  std::deque<Arrow> arrows_;
};