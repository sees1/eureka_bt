#include "utils/false_positive_filter.hpp"

FalsePositiveFilter::FalsePositiveFilter(size_t window,
                         double allow_length_error,
                         double allow_angle_error)
: window_size_(window),
  allow_length_error_(allow_length_error),
  allow_angle_error_(allow_angle_error)
{ }

void FalsePositiveFilter::addObject(Object&& arrow)
{
  if (isBufferFull())
    objects_.pop_front();

  objects_.push_back(std::move(arrow));
}

Object FalsePositiveFilter::getActualObject()
{
  if (!isBufferFull())
    return Object{"none", 0.0, 0.0};

  size_t false_positive_counter = 0;

  size_t left_idx = 0;
  size_t right_idx = 1;

  while(right_idx < window_size_)
  {
    if (objects_[right_idx].direction == "none")
    {  
      right_idx++;
      false_positive_counter++;
      continue;
    }
    if (objects_[left_idx].direction == "none")
    {
      left_idx = right_idx;
      continue;
    }

    if (std::fabs(objects_[left_idx].distance - objects_[right_idx].distance) > allow_length_error_ ||
        std::fabs(objects_[left_idx].angle - objects_[right_idx].angle) > allow_angle_error_)
      false_positive_counter++;

    left_idx = right_idx;
    right_idx++;

    // too much false positive arrows in buffer
    if (static_cast<float>(false_positive_counter) / static_cast<float>(window_size_ - 1) > 0.4)
      return Object{"none", 0.0, 0.0};
  }

  // if buffer doesn't corrupted by false positive arrows
  // than take most recent arrow detection
  right_idx = window_size_ - 1;

  while(right_idx != 0)
  {
    if (objects_[right_idx].direction == "none")
      right_idx--;
    else
      break;
  }
  
  return objects_[right_idx];
}