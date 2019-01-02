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

  double m_tx;
  double m_ty;
  double m_tz;
  double m_s;
  double m_rx;
  double m_ry;
  double m_rz;
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

Point::Point(const Latitude &latitude, const Longitude &longitude, Point::Datum datum)
  :
    erkir::Point(latitude, longitude),
    m_datum(datum)
{}

Point::Datum Point::datum() const
{
  return m_datum;
}

} // ellipsoidal

} // erkir

