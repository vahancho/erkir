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

#ifndef DATUM_H
#define DATUM_H

#include "export.h"

namespace erkir
{

namespace cartesian
{
  class Point;
}

namespace ellipsoidal
{

/// Implements a geodetic datum.
/*!
  Note that precision of various datums will vary, and WGS-84 (original) is not defined to be
  accurate to better than ±1 metre. No transformation should be assumed to be accurate to better
  than a meter; for many datums somewhat less.
*/
class ERKIR_EXPORT Datum
{
public:
  enum class Type
  {
    ED50,       /*!< The older European Datum */
    Irl1975,
    NAD27,      /*!< The older North American Datum, of which NAD83 was basically a readjustment */
    NAD83,      /*!< The North American Datum which is very similar to WGS 84 */
    NTF,        /*!< Nouvelle Triangulation Francaise */
    OSGB36,     /*!< Of the Ordnance Survey of Great Britain */
    Potsdam,    /*!< The local datum of Germany with underlying Bessel ellipsoid */
    TokyoJapan,
    WGS72,      /*!< 72 of the World Geodetic System */
    WGS84       /*!< 84 of the World Geodetic System */
  };

  //! Constructs a datum with the given \p type. WGS84 is the default datum.
  Datum(Type type = Type::WGS84);

  //! Implements a reference ellipsoid.
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

    Ellipsoid(double a, double b, double f);

    /// Major axis (a).
    double m_a{0.0};

    /// Minor axis (b).
    double m_b{0.0};

    /// Flattening (f).
    double m_f{0.0};
  };

  /// Returns the reference ellipsoid for this datum.
  const Ellipsoid &ellipsoid() const;

  /// Returns the type of this datum.
  Type type() const;

  /// Converts the given cartesian \p point to the \p targetDatum.
  void toDatum(cartesian::Point &point, Type targetDatum) const;

  /// Compares two datums.
  bool operator==(const Datum &other) const;

private:
  Type m_type{Type::WGS84};
};

} // ellipsoidal

} // erkir

#endif // DATUM_H
