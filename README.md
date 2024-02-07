# Երկիր (Erkir) - a C++ library for geodesic and trigonometric calculations

[Erkir (armenian: Երկիր, means Earth)](https://github.com/vahancho/erkir) - is inspired
by and based on the great work of [Chris Veness](https://github.com/chrisveness),
the owner of the [Geodesy functions](https://github.com/chrisveness/geodesy)
project - provides a set of comprehensive API for geodesic and trigonometric calculations.
I would call it a C++ port of JavaScript functions provided by the mentioned Chris Veness' project,
however I designed the library to be more object oriented. Thus the code is organized a
little bit differently, but the implementation itself is preserved.

[![Latest release](https://img.shields.io/github/v/release/vahancho/erkir?include_prereleases)](https://github.com/vahancho/erkir/releases)
[![Test (CMake)](https://github.com/vahancho/erkir/actions/workflows/cmake.yml/badge.svg)](https://github.com/vahancho/erkir/actions/workflows/cmake.yml)
[![codecov](https://codecov.io/gh/vahancho/erkir/branch/master/graph/badge.svg)](https://codecov.io/gh/vahancho/erkir)

### Prerequisites

There are no special requirements and dependencies except *C++11* compliant compiler.
The class is tested with *gcc 4.8.4* and *MSVC 15.x* (Visual Studio 2017).
The library is written with pure STL without any third party dependencies.
For more details see the CI badges (*GitHub Actions*) above.

### Installation

No installation required. Just incorporate header files from the *include/* and
source files from *src/* directories in your project and compile them. All library
classes are in *erkir* namespace.

#### Integration with `CMake` projects

However, if you use `CMake` and want to integrate the library into your project
you might want to install it first by invoking a `CMake` command from the build directory:

```
cmake --install . --prefix=<install_path> --config=Release
```

Once the library is installed you can use it from in your project by adjusting its
`CMake` script. For example:

```
[..]
find_package(erkir REQUIRED)

add_executable(example main.cpp)
target_link_libraries(example erkir)
[..]
```

### The API

The code is virtually split into three domains (namespaces) that represent spherical
and ellipsoidal geodetic coordinates and cartesian (x/y/z) for geocentric ones:
`erkir::spherical`, `erkir::ellipsoidal` and `erkir::cartesian` correspondingly.
Spherical Earth model based calculations are accurate enough for most cases, however
in order to gain more precise measurements use `erkir::ellipsoidal` classes.

`erkir::spherical::Point` class implements geodetic point on the basis of a spherical
earth (ignoring ellipsoidal effects). It uses formulae to calculate distances between
two points (using haversine formula), initial bearing from a point, final bearing to a point, etc.

`erkir::ellipsoidal::Point` class represents geodetic point based on ellipsoidal
earth model. It includes ellipsoid parameters and datums for different coordinate
systems, and methods for converting between them and to Cartesian coordinates.

`erkir::Vector3d` implements 3-d vector manipulation routines. With this class you
can perform basic operations with the vectors, such as calculate dot (scalar) product
of two vectors, multiply vectors, add and subtract them.

`erkir::cartesian::Point` implements ECEF (earth-centered earth-fixed) geocentric
cartesian (x/y/z) coordinates.

### Usage Examples:

```cpp
#include "sphericalpoint.h"
#include "ellipsoidalpoint.h"

int main(int argc, char **argv)
{
  // Calculate great-circle distance between two points.
  erkir::spherical::Point p1{ 52.205, 0.119 };
  erkir::spherical::Point p2{ 48.857, 2.351 };
  auto d = p1.distanceTo(p2); // 404.3 km

  // Get destination point by given distance (shortest) and bearing from start point.
  erkir::spherical::Point p3{ 51.4778, -0.0015 };
  auto dest = p3.destinationPoint(7794.0, 300.7); // 51.5135°N, 000.0983°W

  // Convert a point from one coordinates system to another.
  erkir::ellipsoidal::Point pWGS84(51.4778, -0.0016, ellipsoidal::Datum::Type::WGS84);
  auto pOSGB = pWGS84.toDatum(ellipsoidal::Datum::Type::OSGB36); // 51.4778°N, 000.0000°E

  // Convert to Cartesian coordinates.
  auto cartesian = pWGS84.toCartesianPoint();

  // Convert Cartesian point to a geodetic one.
  auto geoPoint = cartesian->toGeoPoint();

  return 0;
}
```

#### Conversion from/to strings

Library supports latitude/longitude coordinates conversion
in three formats: Degrees Minutes Seconds *(D° M' S")*,
Decimal Minutes *(D° M.M')*, and Decimal Degrees *(D.D°)*.

```cpp
auto lon = Longitude::fromString("45° 46’ 47.36” W");
auto lat = Latitude::fromString("45°46′ 45.36″ N");
```

```cpp
auto lon = Longitude{45.7790};
auto lonStr = lon.toString(Coordinate::Format::DMS); // 45° 46′ 45.36″ E
```

For more usage examples please refer to the unit tests at `/test/test.cpp` file.

### Building and Testing

There are unit tests. You can find them in the *test/* directory.
To run them you have to build and run the test application. For doing that you must invoke the following
commands from the terminal, assuming that compiler and environment are already configured:

#### Linux (gcc)

```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTING=True
cmake --build .
ctest
```

#### Windows (MSVC Toolchain)

```
mkdir build && cd build
cmake .. -DENABLE_TESTING=True -A x64
cmake --build . --config=Release
ctest -C Release
```

For x86 builds use `-A Win32` option instead.

### Performance Tests

I measured performance (on Intel Core i5 series processor) for some spherical geodesy
functions (`Point` class). I used similar approach as Chris Veness did in his tests,
i.e. called functions for 5000 random points or pairs of points. And here are my results:

| Function             | Avg. time/calculation (nanoseconds)|
| -------------------- |:----------------------------------:|
| Distance (haversine) | 162                                |
| Initial bearing      | 190                                |
| Destination point    | 227                                |

*of course timings are machine dependent*

## See Also

* [Movable Type Scripts Latitude/Longitude Calculations Reference](http://www.movable-type.co.uk/scripts/latlong.html)

