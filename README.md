# Երկիր (Erkir) - a C++ library for geodesic and trigonometric calculations

[Erkir (armenian: Երկիր, means Earth)](https://github.com/vahancho/erkir) - is inspired by and based on the great work of [Chris Veness](https://github.com/chrisveness), the owner of the [Geodesy functions](https://github.com/chrisveness/geodesy) project - provides a set of comprehensive API for geodesic and trigonometric calculations. I would call it a C++ port of JavaScript functions provided by the mentioned Chris Veness' project, however I designed the library to be more object oriented. Thus the code is organized a little bit differently, but the implementation itself is preserved.

[![Build Status](https://travis-ci.org/vahancho/erkir.svg?branch=master)](https://travis-ci.org/vahancho/erkir)
[![Build status](https://ci.appveyor.com/api/projects/status/gh9v3ynrm1dt1w7t/branch/master?svg=true)](https://ci.appveyor.com/project/vahancho/erkir/branch/master)
[![Coverage Status](https://coveralls.io/repos/github/vahancho/erkir/badge.svg)](https://coveralls.io/github/vahancho/erkir)

### Prerequisites

There are no special requirements and dependencies except *C++11* compliant compiler. The class is tested with *gcc 4.8.4* and *MSVC 15.x* (Visual Studio 2017). The library is written with pure STL without any third party dependencies.
For more details see the CI badges (*Travis CI & AppVeyor CI*) above.

### Installation

No installation required. Just incorporate header files from the include/ and source files from src/ directories in your project and compile them. All library classes are in *erkir* namespace.

### The API

The code is virtually split into two domains (namespaces) that represent spherical and ellipsoidal earth models: `erkir::spherical` and `erkir::ellipsoidal` correspondingly. Spherical Earth model based calculations are accurate enough for most cases, however in order to gain more precise measurements use `erkir::ellipsoidal` classes.

`erkir::spherical::Point` class implements geodetic point on the basis of a spherical earth (ignoring ellipsoidal effects). It uses formulae to calculate distances between two points (using haversine formula), initial bearing from a point, final bearing to a point, etc.

`erkir::ellipsoidal::Point` class represents geodetic point based on ellipsoidal earth model. It includes ellipsoid parameters and datums for different coordinate systems, and methods for converting between them and to Cartesian coordinates.

`erkir::Vector3d` implements 3-d vector manipulation routines. With this class you can perform basic operations with the vectors, such as calculate dot (scalar) product of two vectors, multiply vectors, add and subtract them.

### Usage Examples:

```cpp
#include "sphericalpoint.h"

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
  erkir::ellipsoidal::Point pWGS84(51.4778, -0.0016, ellipsoidal::Point::Datum::WGS84);
  auto pOSGB = pWGS84.convertToDatum(ellipsoidal::Point::Datum::OSGB36); // 51.4778°N, 000.0000°E

  return 0;
}
```

### Test

There are unit tests provided for `erkir::Point` class. You can find them in the *test/* directory.
To run them you have to build and run the test application. For doing that you must invoke the following
commands from the terminal, assuming that compiler and environment are already configured:

##### Linux (gcc)
```
cd test
g++ -std=c++11 -Isrc -Iinclude test.cpp -o test
./test
```

##### Windows
```
cd test
cl /I..\src /I..\include /W4 /EHsc test.cpp /link /out:test.exe
test
```

### Performance Tests

I measured performance (on Intel Core i5 series processor) for some spherical geodesy functions (`Point` class). I used similar approach as Chris Veness did in his tests, i.e. called functions for 5000 random points or pairs of points. And here are my results:

| Function             | Avg. time/calculation (nanoseconds)|
| -------------------- |:----------------------------------:|
| Distance (haversine) | 162                                |
| Initial bearing      | 190                                |
| Destination point    | 227                                |

*of course timings are machine dependent*

## See Also

* [Movable Type Scripts Latitude/Longitude Calculations Reference](http://www.movable-type.co.uk/scripts/latlong.html)

