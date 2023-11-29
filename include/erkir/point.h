// clang-format off
/**
 * MIT License
 *
 * Copyright (c) 2018-2023 Vahan Aghajanyan
 * Copyright (c) 2002-2018 Chris Veness (Latitude/Longitude spherical geodesy tools | https://www.movable-type.co.uk/scripts/latlong.html)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
// clang-format on

#ifndef ERKIR_POINT_H_
#define ERKIR_POINT_H_

#include <vector>

#include "erkir/coordinate.h"
#include "erkir/export.h"

namespace erkir {

/**
 * @brief Base class for all types of geodetic points.
 */
class ERKIR_EXPORT Point {
 public:
  /**
   * @brief Constructs an invalid point object.
   */
  Point();

  /**
   * @brief Constructs a point with the given @p latitude and @p longitude.
   *
   * @param latitude Latitude.
   * @param longitude Longitude.
   */
  Point(const Latitude &latitude, const Longitude &longitude);

  /**
   * @brief Returns the latitude of this point.
   *
   * @return Latitude.
   */
  const Latitude &latitude() const;

  /**
   * @brief Returns the longitude of this point.
   *
   * @return Longitude.
   */
  const Longitude &longitude() const;

  /**
   * @brief Check if this point is valid.
   *
   * @return True if valid, false otherwise.
   */
  bool isValid() const;

  /**
   * @brief Check if this point is equal to @p other.
   *
   * @param other Other point.
   *
   * @return True if equals, false otherwise.
   */
  bool operator==(const Point &other) const;

  /**
   * @brief Check if this point is different from @p other.
   *
   * @param other Other point.
   *
   * @return True if not equals, false otherwise.
   */
  bool operator!=(const Point &other) const;

 private:
  Latitude m_latitude;
  Longitude m_longitude;
  bool m_isValid;
};

}  // namespace erkir

#endif  // ERKIR_POINT_H_
