// This file includes code from:
// https://github.com/RocShi/hungarian_optimizer
// Licensed under the MIT License.
// © Original author: RocShi

#pragma once

#include <algorithm>
#include <string>

#include <Eigen/Dense>

template <typename T>
class SecureMat {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  SecureMat() { Reserve(max_height_, max_width_); }

  size_t height() { return height_; }
  size_t width() { return width_; }

  /* @brief: reserve memory of SecureMat
   * @params[IN] reserve_height: height of reserve memory
   * @params[IN] reserve_width: width of reserve memory
   */
  void Reserve(const size_t reserve_height, const size_t reserve_width) {
    max_height_ = std::max(reserve_height, max_height_);
    max_width_ = std::max(reserve_width, max_width_);
    mat_.resize(max_height_, max_width_);
  }

  /* @brief: resize memory of SecureMat
   * @params[IN] resize_height: height of resize memory
   * @params[IN] resize_width: width of resize memory
   * @return nothing */
  void Resize(const size_t resize_height, const size_t resize_width) {
    height_ = resize_height;
    width_ = resize_width;
    if (resize_height <= max_height_ && resize_width <= max_width_) {
      return;
    }
    max_height_ = std::max(resize_height, max_height_);
    max_width_ = std::max(resize_width, max_width_);
    mat_.resize(max_height_, max_width_);
  }

  std::string ToString() const {
    std::ostringstream oss;
    for (size_t row = 0; row < height_; ++row) {
      for (size_t col = 0; col < width_; ++col) {
        oss << mat_(row, col) << "\t";
      }
      oss << "\n";
    }
    return oss.str();
  }

  inline const T& operator()(const size_t row, const size_t col) const {
    return mat_(row, col);
  }

  inline T& operator()(const size_t row, const size_t col) {
    return mat_(row, col);
  }

 private:
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_;
  size_t max_height_ = 1000;
  size_t max_width_ = 1000;
  size_t height_ = 0;
  size_t width_ = 0;
};  // class SecureMat
