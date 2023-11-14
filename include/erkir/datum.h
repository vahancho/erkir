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

#ifndef ERKIR_DATUM_H_
#define ERKIR_DATUM_H_

#include "erkir/export.h"

namespace erkir {

namespace cartesian {
class Point;
}

namespace ellipsoidal {

/**
 * @brief Implements a geodetic datum.
 * Note that precision of various datums will vary, and WGS-84 (original) is not
 * defined to be accurate to better than ±1 metre. No transformation should be
 * assumed to be accurate to better than a meter; for many datums somewhat less.
 */
class ERKIR_EXPORT Datum {
 public:
  enum class Type {
    /**
     * @brief Older European Datum.
     */
    ED50,
    /**
     * @brief Ireland 1975.
     */
    Irl1975,
    /**
     * @brief Older North American Datum, of which NAD83 was basically a
     * readjustment.
     */
    NAD27,
    /**
     * @brief North American Datum which is very similar to WGS 84.
     */
    NAD83,
    /**
     * @brief Nouvelle Triangulation Francaise.
     */
    NTF,
    /**
     * @brief Of the Ordnance Survey of Great Britain.
     */
    OSGB36,
    /**
     * @brief Local datum of Germany with underlying Bessel ellipsoid.
     */
    Potsdam,
    /**
     * @brief Tokyo Japan.
     */
    TokyoJapan,
    /**
     * @brief 72 of the World Geodetic System.
     */
    WGS72,
    /**
     * @brief 84 of the World Geodetic System.
     */
    WGS84
  };

  /**
   * @brief Constructs a datum with the given @p type
   * WGS84 is the default datum.
   *
   * @param type Type.
   */
  Datum(Type type = Type::WGS84);

  /**
   * @brief Implements a reference ellipsoid.
   */
  struct Ellipsoid {
    enum class Type {
      WGS84,
      Airy1830,
      AiryModified,
      Bessel1841,
      Clarke1866,
      Clarke1880IGN,
      GRS80,
      /**
       * @brief a.k.a. Hayford.
       */
      Intl1924,
      WGS72
    };

    /**
     * @brief Construct a new Ellipsoid object.
     *
     * @param a Major axis.
     * @param b Minor axis.
     * @param f Flattening.
     */
    Ellipsoid(double a, double b, double f);

    /**
     * @brief Major axis (a).
     */
    double m_a{0.0};

    /**
     * @brief Minor axis (b).
     */
    double m_b{0.0};

    /**
     * @brief Flattening (f).
     */
    double m_f{0.0};
  };

  /**
   * @brief Returns the reference ellipsoid for this datum.
   *
   * @return Reference ellipsoid.
   */
  const Ellipsoid &ellipsoid() const;

  /**
   * @brief Returns the type of this datum.
   *
   * @return Datum type.
   */
  Type type() const;

  /**
   * @brief Converts the given cartesian @p point to the @p targetDatum.
   *
   * @param point Cartesian point.
   * @param targetDatum Target datum.
   */
  void toDatum(cartesian::Point &point, Type targetDatum) const;

  /**
   * @brief Check if this datum is equal to @p other.
   *
   * @param other Other datum.
   * @return True if equals, false otherwise.
   */
  bool operator==(const Datum &other) const;

 private:
  Type m_type{Type::WGS84};
};

}  // namespace ellipsoidal
}  // namespace erkir

#endif  // ERKIR_DATUM_H_
