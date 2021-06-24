/*
 * Copyright 2021 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autoware_utils
{
template <class T>
std::vector<T> arange(const T start, const T stop, const T step = 1)
{
  if (step == 0) {
    throw std::invalid_argument("step must be non-zero value.");
  }

  if (step > 0 && stop < start) {
    throw std::invalid_argument("must be stop >= start for positive step.");
  }

  if (step < 0 && stop > start) {
    throw std::invalid_argument("must be stop <= start for negative step.");
  }

  std::vector<T> out;
  const double max_i = std::ceil(static_cast<double>(stop - start) / step);
  for (size_t i = 0; i < static_cast<size_t>(max_i); ++i) {
    out.push_back(start + i * step);
  }

  return out;
}

template <class T>
std::vector<double> linspace(const T start, const T stop, const size_t num)
{
  const double start_double = static_cast<double>(start);
  const double stop_double = static_cast<double>(stop);

  if (num == 0) {
    return {};
  }

  if (num == 1) {
    return {start_double};
  }

  std::vector<double> out;
  out.reserve(num);
  const double step = (stop_double - start_double) / static_cast<double>(num - 1);
  for (size_t i = 0; i < num; i++) {
    out.push_back(start_double + static_cast<double>(i) * step);
  }

  return out;
}

}  // namespace autoware_utils
