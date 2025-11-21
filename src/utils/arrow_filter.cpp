#include "utils/arrow_filter.hpp"

ArrowFilter::ArrowFilter(size_t window,
                         double allow_length_error,
                         double allow_angle_error)
: window_size_(window),
  allow_length_error_(allow_length_error),
  allow_angle_error_(allow_angle_error)
{ }

void ArrowFilter::addArrow(Arrow&& arrow)
{
  if (isBufferFull())
    arrows_.pop_front();

  arrows_.push_back(arrow);
}

Arrow ArrowFilter::getActualArrow()
{
  if (!isBufferFull())
    return Arrow();

  size_t false_positive_counter = 0;

  size_t left_idx = 0;
  size_t right_idx = 1;

  while(right_idx < window_size_)
  {
    if (arrows_[right_idx].direction == "none")
    {  
      right_idx++;
      false_positive_counter++;
      continue;
    }
    if (arrows_[left_idx].direction == "none")
    {
      left_idx = right_idx;
      continue;
    }

    if (std::fabs(arrows_[left_idx].distance - arrows_[right_idx].distance) > allow_length_error_ ||
        std::fabs(arrows_[left_idx].angle - arrows_[right_idx].angle) > allow_angle_error_)
      false_positive_counter++;

    left_idx = right_idx;
    right_idx++;

    // too much false positive arrows in buffer
    if (static_cast<float>(false_positive_counter) / static_cast<float>(window_size_ - 1) > 0.4)
      return Arrow{"none", 0.0, 0.0};
  }

  // if buffer doesn't corrupted by false positive arrows
  // than take most recent arrow detection
  right_idx = window_size_ - 1;

  while(right_idx != 0)
  {
    if (arrows_[right_idx].direction == "none")
      right_idx--;
    else
      break;
  }
  
  return arrows_[right_idx];
}