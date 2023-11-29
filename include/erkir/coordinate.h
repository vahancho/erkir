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

#ifndef ERKIR_COORDINATE_H_
#define ERKIR_COORDINATE_H_

#include "erkir/export.h"

namespace erkir {

/**
 * @brief Implements the geographical coordinate abstraction.
 */
class ERKIR_EXPORT Coordinate {
 public:
  /**
   * @brief Constructs a coordinate by the given decimal degrees.
   *
   * @param degrees Decimal degrees.
   */
  Coordinate(double degrees);

  /**
   * @brief Returns this coordinate's value in decimal degrees.
   *
   * @return Decimal degrees.
   */
  double degrees() const;

  /**
   * @brief Returns this coordinate's value in radians.
   *
   * @return Radians.
   */
  double radians() const;

  /**
   * @brief Helper function to convert radians to degrees.
   *
   * @param radians Radians.
   * @return Deciaml degrees.
   */
  static double toDegrees(double radians);

  /**
   * @brief Helper function to convert deciaml degrees to radians.
   *
   * @param degrees Decimal degrees.
   * @return Radians.
   */
  static double toRadians(double degrees);

  /**
   * @brief Returns Pi.
   *
   * @return Pi.
   */
  static double pi();

  /**
   * @brief  Constrain degrees to range from 0 to 360.0 (e.g. for bearings).
   * -1 => 359, 361
   *
   * @param degrees Decimal degrees.
   * @return Contrained decimal degrees.
   */
  static double wrap360(double degrees);

 private:
  double m_degrees;
};

/**
 * @brief Latitude.
 * Geographic coordinate that specifies the north–south position of a point on
 * the Earth's surface.
 */
class ERKIR_EXPORT Latitude : public Coordinate {
 public:
  /**
   * @brief Constructs a latitude by the given decimal degrees.
   *
   * @param degree Decimal degree from 0° to (+/–)90°
   * @throw std::out_of_range
   */
  Latitude(double degree);

  /**
   * @brief Check if this latitude is equal to @p other.
   *
   * @param other Other latitude.
   *
   * @return True if equals, false otherwise.
   */
  bool operator==(const Latitude &other) const;

  /**
   * @brief Check if this latitude is different from @p other.
   *
   * @param other Other latitude.
   *
   * @return True if not equals, false otherwise.
   */
  bool operator!=(const Latitude &other) const;
};

/**
 * @brief Longitude.
 * Measurement east or west of the prime meridian.
 */
class ERKIR_EXPORT Longitude : public Coordinate {
 public:
  /**
   * @brief Constructs a longitude by the given decimal degrees.
   *
   * @param degree Decimal degree from 0° to (+/–)180°
   * @throw std::out_of_range
   */
  Longitude(double degree);

  /**
   * @brief Check if this longitude is equal to @p other.
   *
   * @param other Other longitude.
   *
   * @return True if equals, false otherwise.
   */
  bool operator==(const Longitude &other) const;

  /**
   * @brief Check if this longitude is different from @p other.
   *
   * @param other Other longitude.
   *
   * @return True if not equals, false otherwise.
   */
  bool operator!=(const Longitude &other) const;
};

}  // namespace erkir

#endif  // ERKIR_COORDINATE_H_
