/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2020 Vahan Aghajanyan <vahancho@gmail.com>                       *
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

#include "cartesianpoint.h"
#include "ellipsoidalpoint.h"

namespace erkir
{

namespace cartesian
{

Point::Point(double x, double y, double z, const ellipsoidal::Datum &datum)
  :
    Vector3d(x, y, z),
    m_datum(datum)
{
}

std::unique_ptr<ellipsoidal::Point> Point::toGeoPoint() const
{
  const auto &ellips = m_datum.ellipsoid();
  auto a = ellips.m_a;
  auto b = ellips.m_b;
  auto f = ellips.m_f;

  auto e2 = 2.0 * f - f * f;                 // 1st eccentricity squared  (aІ - bІ) / aІ
  auto epsilon2 = e2 / (1.0 - e2);           // 2nd eccentricity squared  (aІ - bІ) / bІ
  auto p = std::sqrt(x() * x() + y() * y()); // distance from minor axis
  auto R = std::sqrt(p * p + z() * z());     // polar radius

  // Parametric latitude (Bowring eqn.17, replacing tanBeta = z*a / p*b)
  auto tanBeta = (b * z()) / (a * p) * (1.0 + epsilon2 * b / R);
  auto sinBeta = tanBeta / std::sqrt( 1 + tanBeta * tanBeta );
  auto cosBeta = sinBeta / tanBeta;

  // Geodetic latitude (Bowring eqn.18)
  auto phi = std::isnan(cosBeta) ? 0 : std::atan2(z() + epsilon2 * b * sinBeta * sinBeta * sinBeta,
                                                  p - e2 * a * cosBeta * cosBeta * cosBeta);

  // Longitude
  auto lambda = std::atan2(y(), x());

  // Height above ellipsoid (Bowring eqn.7)
  auto sinPhi = std::sin(phi), cosphi = std::cos(phi);
  auto nu = a / std::sqrt(1 - e2 * sinPhi * sinPhi); // Length of the normal terminated by the minor axis
  auto h = p * cosphi + z() * sinPhi - (a * a / nu);

  /// TODO: Replace it with std::make_unique (since C++14)
  return std::unique_ptr<ellipsoidal::Point>(new ellipsoidal::Point(Coordinate::toDegrees( phi ),
                                                                    Coordinate::toDegrees( lambda ),
                                                                    h,
                                                                    m_datum));
}

Point &Point::toDatum(ellipsoidal::Datum::Type targetDatum)
{
  if (m_datum.type() == targetDatum) {
    return *this;
  }

  if (m_datum.type() != ellipsoidal::Datum::Type::WGS84 &&
      targetDatum != ellipsoidal::Datum::Type::WGS84) {
    // Neither this datum nor target datum are WGS84: convert this to WGS84 first
    toDatum(ellipsoidal::Datum::Type::WGS84);
  }

  m_datum.toDatum(*this, targetDatum);
  m_datum = { targetDatum };
  return *this;
}

} // cartesian

} // erkir

