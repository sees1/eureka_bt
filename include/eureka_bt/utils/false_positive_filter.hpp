#pragma once
#include <deque>
#include <string>
#include <cmath>

struct Object {
  Object() = default;

  std::string direction;
  double distance;
  double angle;
};

class FalsePositiveFilter {
public:
  FalsePositiveFilter(size_t window,
              double allow_length_error = 2.0,
              double allow_angle_error = 3.0);

  void addObject(Object&& obj);
  Object getActualObject();

  inline bool isBufferFull()
  {
    if (objects_.size() == window_size_)
      return true;

    return false;
  }

private:
  size_t window_size_;
  double allow_length_error_;
  double allow_angle_error_;

  std::deque<Object> objects_;
};