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

#define _USE_MATH_DEFINES

#include "erkir/coordinate.h"

#include <cmath>
#include <stdexcept>

#include "erkir/config.h"

constexpr double radiansInDegree = M_PI / 180.0;

namespace erkir {

Coordinate::Coordinate(double degrees) : m_degrees(degrees) {}

double Coordinate::degrees() const { return m_degrees; }

double Coordinate::radians() const { return toRadians(m_degrees); }

double Coordinate::toDegrees(double radians) {
  return radians / radiansInDegree;
}

double Coordinate::toRadians(double degrees) {
  return degrees * radiansInDegree;
}

double Coordinate::pi() { return M_PI; }

double Coordinate::wrap360(double degrees) {
  return std::fmod(degrees + 360.0, 360.0);
}

Latitude::Latitude(double degree) : Coordinate(degree) {
  if (degree > 90.0 || degree < -90.0) {
    throw std::out_of_range("Latitude measurements range from 0° to (+/-)90°.");
  }
}

bool Latitude::operator==(const Latitude& other) const {
  return std::abs(degrees() - other.degrees()) < ERKIR_EPSILON;
}

bool Latitude::operator!=(const Latitude& other) const {
  return !(*this == other);
}

Longitude::Longitude(double degree) : Coordinate(degree) {
  if (degree > 180.0 || degree < -180.0) {
    throw std::out_of_range(
        "Longitude measurements range from 0° to (+/-)180°.");
  }
}

bool Longitude::operator==(const Longitude& other) const {
  return std::abs(degrees() - other.degrees()) < ERKIR_EPSILON;
}

bool Longitude::operator!=(const Longitude& other) const {
  return !(*this == other);
}

}  // namespace erkir
