/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2018-2020 Vahan Aghajanyan <vahancho@gmail.com>                  *
*                                                                                 *
*  Geodesy tools for conversions between (historical) datums                      *
*  (c) Chris Veness 2005-2019                                                     *
*  www.movable-type.co.uk/scripts/latlong-convert-coords.html                     *
*  www.movable-type.co.uk/scripts/geodesy-library.html#latlon-ellipsoidal-datum   *
*                                                                                 *
*  Permission is hereby granted, free of charge, to any person obtaining a copy   *
*  of this software and associated documentation files (the "Software"), to deal  *
*  in the Software without restriction, including without limitation the rights   *
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      *
*  copies of the Software, and to permit persons to whom the Software is          *
*  furnished to do so, subject to the following conditions:                       *
*                                                                                 *
*  The above copyright notice and this permission notice shall be included in all *
*  copies or substantial portions of the Software.                                *
*                                                                                 *
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     *
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       *
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    *
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         *
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  *
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  *
*  SOFTWARE.                                                                      *
***********************************************************************************/

#include <cmath>

#include "ellipsoidalpoint.h"
#include "cartesianpoint.h"

namespace erkir
{

namespace ellipsoidal
{

Point::Point(const Latitude &latitude, const Longitude &longitude, double height,
             const Datum &datum)
  :
    erkir::Point(latitude, longitude),
    m_height(height),
    m_datum(datum)
{}

Datum Point::datum() const
{
  return m_datum;
}

double Point::height() const
{
  return m_height;
}

Point &Point::toDatum(Datum::Type targetDatum)
{
  auto currentDatum = datum();
  if (currentDatum.type() == targetDatum) {
    return *this;
  }

  auto cartesian = toCartesianPoint();
  cartesian->toDatum(targetDatum);
  *this = *cartesian->toGeoPoint();

  return *this;
}

std::unique_ptr<cartesian::Point> Point::toCartesianPoint()
{
  auto phi = latitude().radians();
  auto lambda = longitude().radians();
  auto h = m_height; // Height above ellipsoid.

  const auto &currentDatum = datum();
  auto ellips = currentDatum.ellipsoid();
  auto a = ellips.m_a;
  auto f = ellips.m_f;

  auto sinPhi = std::sin(phi);
  auto cosPhi = std::cos(phi);
  auto sinLambda = std::sin(lambda);
  auto cosLambda = std::cos(lambda);

  auto eSq = 2.0 * f - f * f;                           // 1st eccentricity squared = (aІ - bІ)/aІ
  auto nu = a / std::sqrt(1.0 - eSq * sinPhi * sinPhi); // Radius of curvature in prime vertical

  auto x = (nu + h) * cosPhi * cosLambda;
  auto y = (nu + h) * cosPhi * sinLambda;
  auto z = (nu * (1.0 - eSq) + h) * sinPhi;

  return std::make_unique<cartesian::Point>(x, y, z, currentDatum);
}

} // ellipsoidal

} // erkir

