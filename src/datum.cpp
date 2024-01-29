/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2020-2024 Vahan Aghajanyan <vahancho@gmail.com>                       *
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

#include <unordered_map>
#include <cassert>

#include "datum.h"
#include "cartesianpoint.h"
#include "coordinate.h"

namespace erkir
{

namespace ellipsoidal
{

/// Functor object to calculate hash for scoped enum types.
struct EnumHash
{
  template <class T>
  std::size_t operator()(T type) const
  {
    return static_cast<int>(type);
  }
};

Datum::Ellipsoid::Ellipsoid(double a, double b, double f)
  :
    m_a(a),
    m_b(b),
    m_f(f)
 {}

/// Ellipsoid parameters; major axis (a), minor axis (b), and flattening (f) for each ellipsoid.
static const std::unordered_map<Datum::Ellipsoid::Type, Datum::Ellipsoid, EnumHash> s_ellipsoids =
{
  { Datum::Ellipsoid::Type::WGS84,         { 6378137.0,   6356752.314245, 1.0 / 298.257223563 } },
  { Datum::Ellipsoid::Type::Airy1830,      { 6377563.396, 6356256.909,    1.0 / 299.3249646   } },
  { Datum::Ellipsoid::Type::AiryModified,  { 6377340.189, 6356034.448,    1.0 / 299.3249646   } },
  { Datum::Ellipsoid::Type::Bessel1841,    { 6377397.155, 6356078.962822, 1.0 / 299.15281285  } },
  { Datum::Ellipsoid::Type::Clarke1866,    { 6378206.4,   6356583.8,      1.0 / 294.978698214 } },
  { Datum::Ellipsoid::Type::Clarke1880IGN, { 6378249.2,   6356515.0,      1.0 / 293.466021294 } },
  { Datum::Ellipsoid::Type::GRS80,         { 6378137.0,   6356752.314140, 1.0 / 298.257222101 } },
  { Datum::Ellipsoid::Type::Intl1924,      { 6378388.0,   6356911.946128, 1.0 / 297.0         } },
  { Datum::Ellipsoid::Type::WGS72,         { 6378135.0,   6356750.52,     1.0 / 298.26        } },
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
  double m_s { 0.0 };
  double m_rx{ 0.0 };
  double m_ry{ 0.0 };
  double m_rz{ 0.0 };
};

/// Defines the datum.
struct Datum_data
{
    Datum_data(const Datum::Ellipsoid &ellipsoid, const Transform &transform)
    :
      m_ellipsoid(ellipsoid),
      m_transform(transform)
  {}

  Datum::Ellipsoid m_ellipsoid;
  Transform m_transform;
};

/// The definition of datums.
/*!
  Datums; with associated ellipsoid, and Helmert transform parameters to convert from WGS-84 into
  given datum.

  Note that precision of various datums will vary, and WGS-84 (original) is not defined to be
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
      (i.e. null transform) at the one metre level.
*/
static const std::unordered_map<Datum::Type, Datum_data, EnumHash> s_datums =
{
  // Transforms: t in metres, s in ppm, r in arcseconds
  //   tx        ty        tz        s        rx        ry        rz
  { Datum::Type::ED50,       { s_ellipsoids.at(Datum::Ellipsoid::Type::Intl1924),
    { 89.5,     93.8,    123.1,    -1.2,     0.0,      0.0,      0.156   } } }, // epsg.io/1311
  { Datum::Type::Irl1975,    { s_ellipsoids.at(Datum::Ellipsoid::Type::AiryModified),
    { -482.530, 130.596, -564.557, -8.150,   1.042,    0.214,    0.631   } } }, // epsg.io/1954
  { Datum::Type::NAD27,      { s_ellipsoids.at(Datum::Ellipsoid::Type::Clarke1866),
    { 8.0,      -160.0,  -176.0,   0.0,      0.0,      0.0,      0.0     } } },
  { Datum::Type::NAD83,      { s_ellipsoids.at(Datum::Ellipsoid::Type::GRS80),
    { 0.9956,   -1.9103, -0.5215,  -0.00062, 0.025915, 0.009426, 0.011599} } },
  { Datum::Type::NTF,        { s_ellipsoids.at(Datum::Ellipsoid::Type::Clarke1880IGN),
    { 168.0,    60.0,    -320.0,   0.0,      0.0,      0.0,      0.0     } } },
  { Datum::Type::OSGB36,     { s_ellipsoids.at(Datum::Ellipsoid::Type::Airy1830),
    { -446.448, 125.157, -542.060, 20.4894,  -0.1502,  -0.2470,  -0.8421 } } }, // epsg.io/1314
  { Datum::Type::Potsdam,    { s_ellipsoids.at(Datum::Ellipsoid::Type::Bessel1841),
    { -582.0,   -105.0,  -414.0,   -8.3,     1.04,     0.35,     -3.08   } } },
  { Datum::Type::TokyoJapan, { s_ellipsoids.at(Datum::Ellipsoid::Type::Bessel1841),
    { 148.0,    -507.0,  -685.0,   0.0,      0.0,      0.0,      0.0     } } },
  { Datum::Type::WGS72,      { s_ellipsoids.at(Datum::Ellipsoid::Type::WGS72),
    { 0.0,      0.0,     -4.5,     -0.22,    0.0,      0.0,      0.554   } } },
  { Datum::Type::WGS84,      { s_ellipsoids.at(Datum::Ellipsoid::Type::WGS84),
    { 0.0,      0.0,     0.0,      0.0,      0.0,      0.0,      0.0     } } }
};

static const Transform &transform(Datum::Type datum)
{
  auto it = s_datums.find(datum);
  assert(it != s_datums.cend());
  return it->second.m_transform;
}

/// Applies Helmert transform to the point using transform parameters.
/*!
  \param vector The vector to apply the transformation to.
  \param transform Transform to apply to the point.
*/
static void applyTransform(cartesian::Point &point, const Transform &transform)
{
  // This point
  auto x1 = point.x();
  auto y1 = point.y();
  auto z1 = point.z();

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

  point = cartesian::Point(x2, y2, z2);
}

////////////////////////////////////////////////////////////////////////////////

Datum::Datum(Datum::Type type)
  :
    m_type(type)
{}

const Datum::Ellipsoid &Datum::ellipsoid() const
{
  auto it = s_datums.find(m_type);
  assert(it != s_datums.cend());
  return it->second.m_ellipsoid;
}

bool Datum::operator==( const Datum &other ) const
{
  return m_type == other.m_type;
}

Datum::Type Datum::type() const
{
  return m_type;
}

void Datum::toDatum(cartesian::Point &point, Type targetDatum) const
{
  auto trans = transform(targetDatum);
  if (targetDatum == Type::WGS84) {
    trans.inverse();
  }
  applyTransform(point, trans);
}

} // ellipsoidal

} // erkir

