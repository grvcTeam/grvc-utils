//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef HANDY_TOOLS_CIRCULAR_BUFFER_H
#define HANDY_TOOLS_CIRCULAR_BUFFER_H

#include <vector>
#include <limits>

namespace grvc { namespace utils {

class CircularBuffer {
public:
  void set_size(size_t _size) {
    buffer_size_ = _size;
    buffer_.clear();
    current_ = 0;
  }

  void reset() {
    buffer_.clear();
    current_ = 0;
  }

  // TODO: templatize
  void push(double _value) {
    if (buffer_.size() < buffer_size_) {
      buffer_.push_back(_value);
    } else {
      buffer_[current_] = _value;
      current_ = (current_ + 1) % buffer_size_;
    }
  }

  void fill_with(double _value) {
    for (int i = 0; i < buffer_size_; i++) {
        push(_value);
    }
  }

  bool get_stats(double& _min, double& _mean, double& _max) {
    if (buffer_.size() >= buffer_size_) {
      double min_value = +std::numeric_limits<double>::max();
      double max_value = -std::numeric_limits<double>::max();
      double sum = 0;
      for (int i = 0; i < buffer_.size(); i++) {
        if (buffer_[i] < min_value) { min_value = buffer_[i]; }
        if (buffer_[i] > max_value) { max_value = buffer_[i]; }
        sum += buffer_[i];
      }
      _min = min_value;
      _max = max_value;
      _mean = sum / buffer_.size();
      return true;
    }
      return false;
  }

  bool get_variance(double& _var) {
    if (buffer_.size() >= buffer_size_) {
      double sum = 0;
      for (int i = 0; i < buffer_.size(); i++) { sum += buffer_[i]; }
      double mean = sum / buffer_.size();
      _var = 0;
      for (int i = 0; i < buffer_.size(); i++) { _var += (buffer_[i] - mean) * (buffer_[i] - mean); }
      return true;
    }
    return false;
  }

protected:
  size_t buffer_size_ = 0;
  unsigned int current_ = 0;
  std::vector<double> buffer_;
};

}} // namespace grvc::utils

#endif  // HANDY_TOOLS_CIRCULAR_BUFFER_H
