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

#include "erkir/ellipsoidalpoint.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

#include "erkir/cartesianpoint.h"

namespace erkir {

namespace ellipsoidal {

Point::Point(const Latitude &latitude, const Longitude &longitude,
             double height, const Datum &datum)
    : erkir::Point(latitude, longitude), m_height(height), m_datum(datum) {}

const Datum &Point::datum() const { return m_datum; }

double Point::height() const { return m_height; }

Point &Point::toDatum(Datum::Type targetDatum) {
  auto currentDatum = datum();
  if (currentDatum.type() == targetDatum) {
    return *this;
  }

  auto cartesian = toCartesianPoint();
  cartesian->toDatum(targetDatum);
  *this = *cartesian->toGeoPoint();

  return *this;
}

std::unique_ptr<cartesian::Point> Point::toCartesianPoint() {
  auto phi = latitude().radians();
  auto lambda = longitude().radians();
  auto h = m_height;  // Height above ellipsoid.

  const auto &currentDatum = datum();
  const auto &ellips = currentDatum.ellipsoid();
  auto a = ellips.m_a;
  auto f = ellips.m_f;

  auto sinPhi = std::sin(phi);
  auto cosPhi = std::cos(phi);
  auto sinLambda = std::sin(lambda);
  auto cosLambda = std::cos(lambda);

  auto eSq = 2.0 * f - f * f;  // 1st eccentricity squared = (aІ - bІ)/aІ
  auto nu =
      a / std::sqrt(1.0 - eSq * sinPhi *
                              sinPhi);  // Radius of curvature in prime vertical

  auto x = (nu + h) * cosPhi * cosLambda;
  auto y = (nu + h) * cosPhi * sinLambda;
  auto z = (nu * (1.0 - eSq) + h) * sinPhi;

  /// TODO(vahancho) Replace with std::make_unique (since C++14)
  return std::unique_ptr<cartesian::Point>(
      new cartesian::Point(x, y, z, currentDatum));
}

double Point::distanceTo(const Point &point) const {
  try {
    return std::get<static_cast<int>(InverseField::Distance)>(inverse(point));
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

Point Point::destinationPoint(double distance, double initialBearing) const {
  return std::get<static_cast<int>(DirectField::Point)>(
      direct(distance, initialBearing));
}

double Point::initialBearingTo(const Point &point) const {
  try {
    auto brng = std::get<static_cast<int>(InverseField::InitialBearing)>(
        inverse(point));
    return brng;
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

double Point::finalBearingTo(const Point &point) const {
  try {
    auto brng =
        std::get<static_cast<int>(InverseField::FinalBearing)>(inverse(point));
    return brng;
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

double Point::finalBearingOn(double distance, double initialBearing) const {
  auto brng = std::get<static_cast<int>(DirectField::FinalBearing)>(
      direct(distance, initialBearing));
  return brng;
}

std::tuple<Point, double> Point::direct(double distance,
                                        double initialBearing) const {
  if (m_height != 0.0) {
    throw std::domain_error("Point must be on the surface of the ellipsoid");
  }

  const auto phi1 = latitude().radians();
  const auto lambda1 = longitude().radians();
  const auto alpha1 = Coordinate::toRadians(initialBearing);
  const auto s = distance;

  // Allow alternative ellipsoid to be specified
  const auto &ellipsoid = m_datum.ellipsoid();
  const auto a = ellipsoid.m_a;
  const auto b = ellipsoid.m_b;
  const auto f = ellipsoid.m_f;

  const auto sinAlpha1 = std::sin(alpha1);
  const auto cosAlpha1 = std::cos(alpha1);

  const auto tanU1 = (1.0 - f) * std::tan(phi1);
  const auto cosU1 = 1.0 / std::sqrt(1.0 + tanU1 * tanU1);
  const auto sinU1 = tanU1 * cosU1;
  const auto sigma1 =
      std::atan2(tanU1, cosAlpha1);  // sigma1 = angular distance on the sphere
                                     // from the equator to P1
  const auto sinAlpha =
      cosU1 * sinAlpha1;  // α = azimuth of the geodesic at the equator
  const auto cosSqAlpha = 1.0 - sinAlpha * sinAlpha;
  const auto uSq = cosSqAlpha * (a * a - b * b) / (b * b);
  const auto A =
      1.0 +
      uSq / 16384.0 * (4096.0 + uSq * (-768.0 + uSq * (320.0 - 175.0 * uSq)));
  const auto B =
      uSq / 1024.0 * (256.0 + uSq * (-128.0 + uSq * (74.0 - 47.0 * uSq)));

  auto sigma = s / (b * A);
  auto sinSigma = 0.0;
  auto cosSigma = 0.0;
  auto deltaSigma = 0.0;  // σ = angular distance P₁ P₂ on the sphere
  auto cos2SigmaM = 0.0;  // σₘ = angular distance on the sphere from the
                          // equator to the midpoint of the line

  auto sigmaPrim = 0.0;
  auto iterations = 0;
  do {
    cos2SigmaM = std::cos(2.0 * sigma1 + sigma);
    sinSigma = std::sin(sigma);
    cosSigma = std::cos(sigma);
    deltaSigma =
        B * sinSigma *
        (cos2SigmaM +
         B / 4.0 *
             (cosSigma * (-1.0 + 2.0 * cos2SigmaM * cos2SigmaM) -
              B / 6.0 * cos2SigmaM * (-3.0 + 4.0 * sinSigma * sinSigma) *
                  (-3.0 + 4.0 * cos2SigmaM * cos2SigmaM)));
    sigmaPrim = sigma;
    sigma = s / (b * A) + deltaSigma;
  } while (std::abs(sigma - sigmaPrim) > 1e-12 && ++iterations < 100);

  if (iterations >= 100) {
    throw std::domain_error(
        "Vincenty formula failed to converge");  // not possible?
  }

  const auto x = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1;
  const auto phi2 =
      std::atan2(sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1,
                 (1.0 - f) * std::sqrt(sinAlpha * sinAlpha + x * x));
  const auto lambda = std::atan2(
      sinSigma * sinAlpha1, cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1);
  const auto C = f / 16.0 * cosSqAlpha * (4.0 + f * (4.0 - 3.0 * cosSqAlpha));
  const auto L =
      lambda -
      (1.0 - C) * f * sinAlpha *
          (sigma + C * sinSigma *
                       (cos2SigmaM +
                        C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
  const auto lambda2 = lambda1 + L;

  const auto alpha2 = std::atan2(sinAlpha, -x);

  auto destinationPoint = Point(Coordinate::toDegrees(phi2),
                                Coordinate::toDegrees(lambda2), 0.0, m_datum);
  auto finalBearing = Coordinate::wrap360(Coordinate::toDegrees(alpha2));

  return std::make_tuple(destinationPoint, finalBearing);
}

std::tuple<double, double, double> Point::inverse(const Point &point) const {
  if (m_height != 0.0 || point.height() != 0.0) {
    throw std::domain_error("Point must be on the surface of the ellipsoid");
  }

  const auto phi1 = latitude().radians();
  const auto lambda1 = longitude().radians();
  const auto phi2 = point.latitude().radians();
  const auto lambda2 = point.longitude().radians();

  // Allow alternative ellipsoid to be specified
  const auto &ellipsoid = m_datum.ellipsoid();
  const auto a = ellipsoid.m_a;
  const auto b = ellipsoid.m_b;
  const auto f = ellipsoid.m_f;

  const auto L =
      lambda2 - lambda1;  // L = difference in longitude, U = reduced latitude,
                          // defined by tan U = (1 - f)·tanφ.
  const auto tanU1 = (1.0 - f) * std::tan(phi1);
  const auto cosU1 = 1.0 / std::sqrt(1.0 + tanU1 * tanU1);
  const auto sinU1 = tanU1 * cosU1;
  const auto tanU2 = (1.0 - f) * std::tan(phi2);
  const auto cosU2 = 1.0 / std::sqrt(1.0 + tanU2 * tanU2);
  const auto sinU2 = tanU2 * cosU2;

  const bool antipodal = std::abs(L) > Coordinate::pi() / 2.0 ||
                         std::abs(phi2 - phi1) > Coordinate::pi() / 2.0;

  auto lambda = L;
  auto sinLambda = 0.0;
  auto cosLambda = 0.0;  // λ = difference in longitude on an auxiliary sphere
  auto sigma = antipodal ? Coordinate::pi() : 0.0;
  auto sinSigma = 0.0;
  auto cosSigma = antipodal ? -1.0 : 1.0;
  auto sinSqSigma = 0.0;  // σ = angular distance P₁ P₂ on the sphere
  auto cos2SigmaM = 1.0;  // σₘ = angular distance on the sphere from the
                          // equator to the midpoint of the line
  auto sinAlpha = 0.0;
  auto cosSqAlpha = 1.0;  // alpha = azimuth of the geodesic at the equator
  auto C = 0.0;

  constexpr auto epsilon = 2.220446049250313e-16;

  auto lambdaPrim = 0.0;
  auto iterations = 0;
  do {
    sinLambda = std::sin(lambda);
    cosLambda = std::cos(lambda);
    sinSqSigma = (cosU2 * sinLambda) * (cosU2 * sinLambda) +
                 (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) *
                     (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda);
    if (std::abs(sinSqSigma) < epsilon) {
      break;  // Co-incident/antipodal points (falls back on λ/σ = L)
    }
    sinSigma = std::sqrt(sinSqSigma);
    cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
    sigma = std::atan2(sinSigma, cosSigma);
    sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
    cosSqAlpha = 1 - sinAlpha * sinAlpha;
    cos2SigmaM = cosSqAlpha != 0.0
                     ? (cosSigma - 2.0 * sinU1 * sinU2 / cosSqAlpha)
                     : 0.0;  // on equatorial line cos²α = 0 (§6)
    C = f / 16.0 * cosSqAlpha * (4.0 + f * (4.0 - 3.0 * cosSqAlpha));
    lambdaPrim = lambda;
    lambda = L + (1.0 - C) * f * sinAlpha *
                     (sigma + C * sinAlpha *
                                  (cos2SigmaM +
                                   C * cosSigma *
                                       (-1.0 + 2.0 * cos2SigmaM * cos2SigmaM)));

    const auto iterationCheck =
        antipodal ? std::abs(lambda) - Coordinate::pi() : std::abs(lambda);
    if (iterationCheck > Coordinate::pi()) {
      throw std::domain_error("lambda > pi");
    }
  } while (std::abs(lambda - lambdaPrim) > 1e-12 && ++iterations < 1000);

  if (iterations >= 1000) {
    throw std::domain_error("Vincenty formula failed to converge");
  }

  const auto uSq = cosSqAlpha * (a * a - b * b) / (b * b);
  const auto A =
      1 +
      uSq / 16384.0 * (4096.0 + uSq * (-768.0 + uSq * (320.0 - 175.0 * uSq)));
  const auto B =
      uSq / 1024.0 * (256 + uSq * (-128.0 + uSq * (74.0 - 47.0 * uSq)));
  const auto deltaSigma =
      B * sinSigma *
      (cos2SigmaM +
       B / 4.0 *
           (cosSigma * (-1.0 + 2.0 * cos2SigmaM * cos2SigmaM) -
            B / 6.0 * cos2SigmaM * (-3.0 + 4.0 * sinSigma * sinSigma) *
                (-3.0 + 4.0 * cos2SigmaM * cos2SigmaM)));

  const auto distance =
      b * A * (sigma - deltaSigma);  // s = length of the geodesic

  // Note special handling of exactly antipodal points where sin²σ = 0 (due to
  // discontinuity atan2(0, 0) = 0 but atan2(ε, 0) = π/2 / 90°) - in which case
  // bearing is always meridional, due north (or due south!) α = azimuths of the
  // geodesic; α2 the direction P₁ P₂ produced
  const auto alpha1 =
      std::abs(sinSqSigma) < epsilon
          ? 0.0
          : std::atan2(cosU2 * sinLambda,
                       cosU1 * sinU2 - sinU1 * cosU2 * cosLambda);
  const auto alpha2 =
      std::abs(sinSqSigma) < epsilon
          ? Coordinate::pi()
          : std::atan2(cosU1 * sinLambda,
                       -sinU1 * cosU2 + cosU1 * sinU2 * cosLambda);

  auto initialBearing = std::numeric_limits<double>::quiet_NaN();
  auto finalBearing = std::numeric_limits<double>::quiet_NaN();

  if (!(std::abs(distance) < epsilon)) {
    initialBearing = Coordinate::wrap360(Coordinate::toDegrees(alpha1));
    finalBearing = Coordinate::wrap360(Coordinate::toDegrees(alpha2));
  }

  return std::make_tuple(distance, initialBearing, finalBearing);
}

bool Point::operator==(const Point &other) const {
  return datum() == other.datum() && erkir::Point::operator==(other);
}

bool Point::operator!=(const Point &other) const { return !(*this == other); }

}  // namespace ellipsoidal
}  // namespace erkir
