/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2018-2019 Vahan Aghajanyan <vahancho@gmail.com>                  *
*                                                                                 *
*  Latitude/longitude spherical geodesy tools         (c) Chris Veness 2005-2016  *
*  www.movable-type.co.uk/scripts/latlong-convert-coords.html                     *
*  www.movable-type.co.uk/scripts/geodesy/docs/module-latlon-ellipsoidal.html     *
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

#include <unordered_map>
#include "ellipsoidalpoint.h"

namespace erkir
{

namespace ellipsoidal
{

struct Ellipsoid
{
  enum class Type
  {
    WGS84,
    Airy1830,
    AiryModified,
    Bessel1841,
    Clarke1866,
    Clarke1880IGN,
    GRS80,
    Intl1924, // aka Hayford
    WGS72
  };

  Ellipsoid()
  {}

  Ellipsoid(double a, double b, double f)
    :
      m_a(a),
      m_b(b),
      m_f(f)
  {}

  /// Major axis (a).
  double m_a{ 0.0 };

  /// Minor axis (b).
  double m_b{ 0.0 };

  /// Flattening (f).
  double m_f{ 0.0 };
};

/// Ellipsoid parameters; major axis (a), minor axis (b), and flattening (f) for each ellipsoid.
static const std::unordered_map<Ellipsoid::Type, Ellipsoid> s_ellipsoids =
{
  { Ellipsoid::Type::WGS84,         { 6378137.0,   6356752.314245, 1.0 / 298.257223563 } },
  { Ellipsoid::Type::Airy1830,      { 6377563.396, 6356256.909,    1.0 / 299.3249646 } },
  { Ellipsoid::Type::AiryModified,  { 6377340.189, 6356034.448,    1.0 / 299.3249646 } },
  { Ellipsoid::Type::Bessel1841,    { 6377397.155, 6356078.962818, 1.0 / 299.1528128 } },
  { Ellipsoid::Type::Clarke1866,    { 6378206.4,   6356583.8,      1.0 / 294.978698214 } },
  { Ellipsoid::Type::Clarke1880IGN, { 6378249.2,   6356515.0,      1.0 / 293.466021294 } },
  { Ellipsoid::Type::GRS80,         { 6378137.0,   6356752.314140, 1.0 / 298.257222101 } },
  { Ellipsoid::Type::Intl1924,      { 6378388.0,   6356911.946,    1.0 / 297.0  } },
  { Ellipsoid::Type::WGS72,         { 6378135.0,   6356750.5,      1.0 / 298.26 } },
};

/// The transform.
/*!
  Transforms: t in metres, s in ppm, r in arcseconds
*/
struct Transform
{
  Transform()
  {}

  Transform(double tx, double ty, double tz, double s, double rx,
            double ry, double rz)
    :
      m_tx(tx),
      m_ty(ty),
      m_tz(tz),
      m_s(s),
      m_rx(rx),
      m_ry(ry),
      m_rz(rz)
  {}

  Transform &inverse()
  {
    m_tx = -m_tx;
    m_ty = -m_ty;
    m_tz = -m_tz;
    m_s  = -m_s;
    m_rx = -m_rx;
    m_ry = -m_ry;
    m_rz = -m_rz;

    return *this;
  }

  double m_tx{ 0.0 };
  double m_ty{ 0.0 };
  double m_tz{ 0.0 };
  double m_s{ 0.0 };
  double m_rx{ 0.0 };
  double m_ry{ 0.0 };
  double m_rz{ 0.0 };
};

/// Defines the datum.
struct Datum
{
  Datum(const Ellipsoid &ellipsoid, const Transform &transform)
    :
      m_ellipsoid(ellipsoid),
      m_transform(transform)
  {}

  Ellipsoid m_ellipsoid;
  Transform m_transform;
};

/// The definition of datums.
/*!
  Datums; with associated ellipsoid, and Helmert transform parameters to convert from WGS 84 into
  given datum.

  Note that precision of autoious datums will autoy, and WGS-84 (original) is not defined to be
  accurate to better than +/-1 metre. No transformation should be assumed to be accurate to better
  than a meter; for many datums somewhat less.

  Sources:
    - ED50:       www.gov.uk/guidance/oil-and-gas-petroleum-operations-notices#pon-4
    - Irl1975:    www.osi.ie/wp-content/uploads/2015/05/transformations_booklet.pdf
    - NAD27:      en.wikipedia.org/wiki/Helmert_transformation
    - NAD83:      www.uvm.edu/giv/resources/WGS84_NAD83.pdf [strictly, WGS84(G1150) -> NAD83(CORS96) @ epoch 1997.0]
                  (note NAD83(1986) = WGS84(Original); confluence.qps.nl/pages/viewpage.action?pageId=29855173)
    - NTF:        Nouvelle Triangulation Francaise geodesie.ign.fr/contenu/fichiers/Changement_systeme_geodesique.pdf
    - OSGB36:     www.ordnancesurvey.co.uk/docs/support/guide-coordinate-systems-great-britain.pdf
    - Potsdam:    kartoweb.itc.nl/geometrics/Coordinate%20transformations/coordtrans.html
    - TokyoJapan: www.geocachingtoolbox.com?page=datumEllipsoidDetails
    - WGS72:      www.icao.int/safety/pbn/documentation/eurocontrol/eurocontrol wgs 84 implementation manual.pdf

  More transform parameters are available from earth-info.nga.mil/GandG/coordsys/datums/NATO_DT.pdf,
  www.fieldenmaps.info/cconv/web/cconv_params.js

 note:
    - ETRS89 reference frames are coincident with WGS-84 at epoch 1989.0
      (ie null transform) at the one metre level.
*/
static const std::unordered_map<Point::Datum, Datum> s_datums =
{
  { Point::Datum::ED50,       { s_ellipsoids.at(Ellipsoid::Type::Intl1924),
                              { 89.5, 93.8, 123.1, -1.2, 0.0, 0.0, 0.156} } }, // epsg.io/1311
  { Point::Datum::Irl1975,    { s_ellipsoids.at(Ellipsoid::Type::AiryModified),
                              { -482.530, 130.596, -564.557, -8.150, 1.042, 0.214, 0.631} } }, // epsg.io/1954
  { Point::Datum::NAD27,      { s_ellipsoids.at(Ellipsoid::Type::Clarke1866),
                              { 8.0, -160.0, -176.0, 0.0, 0.0, 0.0, 0.0 } } },
  { Point::Datum::NAD83,      { s_ellipsoids.at(Ellipsoid::Type::GRS80),
                              { 0.9956, -1.9103, -0.5215, -0.00062, 0.025915, 0.009426, 0.011599} } },
  { Point::Datum::NTF,        { s_ellipsoids.at(Ellipsoid::Type::Clarke1880IGN),
                              { 168.0, 60.0, -320.0, 0.0, 0.0, 0.0, 0.0 } } },
  { Point::Datum::OSGB36,     { s_ellipsoids.at(Ellipsoid::Type::Airy1830),
                              { -446.448, 125.157, -542.060, 20.4894, -0.1502, -0.2470, -0.8421 } } }, // epsg.io/1314
  { Point::Datum::Potsdam,    { s_ellipsoids.at(Ellipsoid::Type::Bessel1841),
                              { -582.0, -105.0, -414.0, -8.3, 1.04, 0.35, -3.08 } } },
  { Point::Datum::TokyoJapan, { s_ellipsoids.at(Ellipsoid::Type::Bessel1841),
                              { 148.0, -507.0, -685.0, 0.0, 0.0, 0.0, 0.0 } } },
  { Point::Datum::WGS72,      { s_ellipsoids.at(Ellipsoid::Type::WGS72),
                              { 0.0, 0.0, -4.5, -0.22, 0.0, 0.0, 0.554 } } },
  { Point::Datum::WGS84,      { s_ellipsoids.at(Ellipsoid::Type::WGS84),
                              { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} } }
};

static Transform transform(Point::Datum datum)
{
  auto it = s_datums.find(datum);
  if (it != s_datums.cend())
  {
    return it->second.m_transform;
  }
  return Transform();
}

static Ellipsoid ellipsoid(Point::Datum datum)
{
  auto it = s_datums.find(datum);
  if (it != s_datums.cend())
  {
    return it->second.m_ellipsoid;
  }
  return Ellipsoid();
}

/// Applies Helmert transform to the point using transform parameters.
/*!
  \param   vector The vector to apply the transformation to.
  \param   transform Transform to apply to the point.
*/
static void applyTransform(Vector3d &vector, const Transform &transform)
{
  // This point
  auto x1 = vector.x();
  auto y1 = vector.y();
  auto z1 = vector.z();

  // Transform parameters
  auto tx = transform.m_tx;            // x-shift
  auto ty = transform.m_ty;            // y-shift
  auto tz = transform.m_tz;            // z-shift
  auto s1 = transform.m_s / 1e6 + 1.0; // scale: normalise parts-per-million to (s+1)
  auto rx = Coordinate::toRadians(transform.m_rx / 3600.0); // x-rotation: normalise arc-seconds to radians
  auto ry = Coordinate::toRadians(transform.m_ry / 3600.0); // y-rotation: normalise arc-seconds to radians
  auto rz = Coordinate::toRadians(transform.m_rz / 3600.0); // z-rotation: normalise arc-seconds to radians

  // Apply transform
  auto x2 = tx + x1 * s1 - y1 * rz + z1 * ry;
  auto y2 = ty + x1 * rz + y1 * s1 - z1 * rx;
  auto z2 = tz - x1 * ry + y1 * rx + z1 * s1;

  vector = Vector3d(x2, y2, z2);
}


////////////////////////////////////////////////////////////////////////////////

Point::Point(const Latitude &latitude, const Longitude &longitude, Point::Datum datum)
  :
    erkir::Point(latitude, longitude),
    m_datum(datum)
{}

Point::Datum Point::datum() const
{
  return m_datum;
}

Point &Point::convertToDatum(Datum toDatum)
{
  auto currentDatum = datum();
  if (currentDatum == toDatum) {
    return *this;
  }

  Transform trans;

  if (currentDatum == Datum::WGS84) {
    // Converting from WGS 84
    trans = transform(toDatum);
  } else if (toDatum == Datum::WGS84) {
    // Converting to WGS 84; use inverse transform (don't overwrite original!)
    auto currentTransform = transform(currentDatum);
    trans = currentTransform.inverse();
  } else {
    // Neither this datum nor toDatum are WGS84: convert this to WGS84 first
    convertToDatum(Datum::WGS84);
    trans = transform(toDatum);
  }

  auto oldCartesian = toCartesian();      // Convert polar to Cartesian...
  applyTransform(oldCartesian, trans);    // ...apply transform...
  *this = toPoint(oldCartesian, toDatum); // ...and convert Cartesian to polar

  return *this;
}

Vector3d Point::toCartesian()
{
  auto phi = latitude().radians();
  auto lambda = longitude().radians();
  auto h = 0; // Height above ellipsoid - not currently used

  auto currentDatum = datum();
  auto ellips = ellipsoid(currentDatum);
  auto a = ellips.m_a;
  auto f = ellips.m_f;

  auto sinPhi = std::sin(phi);
  auto cosPhi = std::cos(phi);
  auto sinLambda = std::sin(lambda);
  auto cosLambda = std::cos(lambda);

  auto eSq = 2.0 * f - f * f; // 1st eccentricity squared = (a² - b²)/a²
  auto nu = a / std::sqrt(1.0 - eSq * sinPhi * sinPhi); // Radius of curvature in prime vertical

  auto x = (nu + h) * cosPhi * cosLambda;
  auto y = (nu + h) * cosPhi * sinLambda;
  auto z = (nu * (1.0 - eSq) + h) * sinPhi;

  return Vector3d(x, y, z);
}

Point Point::toPoint(const Vector3d &vector, Datum pointDatum)
{
  auto x = vector.x();
  auto y = vector.y();
  auto z = vector.z();

  auto currentDatum = datum();
  auto ellips = ellipsoid(currentDatum);
  auto a = ellips.m_a;
  auto b = ellips.m_b;
  auto f = ellips.m_f;

  auto e2 = 2.0 * f - f * f;           // 1st eccentricity squared  (a² - b²) / a²
  auto epsilon2 = e2 / (1.0 - e2);     // 2nd eccentricity squared  (a² - b²) / b²
  auto p = std::sqrt(x * x + y * y); // distance from minor axis
  auto R = std::sqrt(p * p + z * z); // polar radius

  // parametric latitude (Bowring eqn 17, replacing tanBeta = z·a / p·b)
  auto tanBeta = (b * z) / (a * p) * (1.0 + epsilon2 * b / R);
  auto sinBeta = tanBeta / std::sqrt(1 + tanBeta * tanBeta);
  auto cosBeta = sinBeta / tanBeta;

  // geodetic latitude (Bowring eqn 18)
  auto phi = std::isnan(cosBeta) ? 0 : std::atan2(z + epsilon2 * b * sinBeta * sinBeta * sinBeta,
                                                  p - e2 * a * cosBeta * cosBeta * cosBeta);

  // longitude
  auto lambda = std::atan2(y, x);

  // height above ellipsoid (Bowring eqn 7) [not currently used]
  auto sinPhi = std::sin(phi), cosphi = std::cos(phi);
  auto nu = a / std::sqrt(1 - e2 * sinPhi * sinPhi); // Length of the normal terminated by the minor axis
  auto h = p * cosphi + z * sinPhi - (a * a / nu);

  return Point(Coordinate::toDegrees(phi),
               Coordinate::toDegrees(lambda),
               pointDatum);
};

} // ellipsoidal

} // erkir

