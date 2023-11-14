# Erkir (Երկիր) - _C++ library for geodesic and trigonometric calculations_

[![version](https://img.shields.io/github/v/release/vahancho/erkir?label=version)](https://github.com/vahancho/erkir/releases/latest)
[![release](https://github.com/vahancho/erkir/actions/workflows/release.yaml/badge.svg)](https://github.com/vahancho/erkir/actions/workflows/release.yaml)
[![codecov](https://codecov.io/gh/vahancho/erkir/branch/master/graph/badge.svg)](https://codecov.io/gh/vahancho/erkir)

[`Erkir`](https://github.com/vahancho/erkir) (_Երկիր_) is the Armenian word for `Earth`.

It is inspired by and based on [Chris Veness](https://github.com/chrisveness)'s amazing work as the owner of the [Geodesy functions](https://github.com/chrisveness/geodesy) project, which provides a set of comprehensive API for geodesic and trigonometric calculations.
I'd call it a C++ port of the JavaScript functions offered by Chris Veness' project, but I designed the library to be more object oriented.
As a result, the code is arranged slightly differently, but the implementation remains unchanged.

## Example

> [!NOTE]
> Please see the unit tests file at [tests/test.cpp](./tests/test.cpp) for more examples

```cpp
#include <erkir/erkir.h>

int main(int argc, char **argv) {
  // Calculate great-circle distance between two points
  erkir::spherical::Point p1{ 52.205, 0.119 };
  erkir::spherical::Point p2{ 48.857, 2.351 };
  auto d = p1.distanceTo(p2); // 404.3 km

  // Get destination point by given distance (shortest) and bearing from start point
  erkir::spherical::Point p3{ 51.4778, -0.0015 };
  auto dest = p3.destinationPoint(7794.0, 300.7); // 51.5135°N, 000.0983°W

  // Convert a point from one coordinates system to another
  erkir::ellipsoidal::Point pWGS84(51.4778, -0.0016, ellipsoidal::Datum::Type::WGS84);
  auto pOSGB = pWGS84.toDatum(ellipsoidal::Datum::Type::OSGB36); // 51.4778°N, 000.0000°E

  // Convert to Cartesian coordinates
  auto cartesian = pWGS84.toCartesianPoint();

  // Convert Cartesian point to a Geodetic one
  auto geoPoint = cartesian->toGeoPoint();

  return 0;
}
```

## API

The code is virtually split into three domains (_namespaces_) that represent _spherical_ (`erkir::spherical`) and _ellipsoidal_ (`erkir::ellipsoidal`) geodetic coordinates and _cartesian_ (x/y/z) (`erkir::cartesian`) for geocentric ones.
Spherical Earth model based calculations are accurate enough for most cases, however in order to obtain more precise measurements use `erkir::ellipsoidal` classes.

### erkir::spherical::Point

`erkir::spherical::Point` class implements geodetic point on the basis of a spherical earth (ignoring ellipsoidal effects).
It uses formulae to calculate distances between two points (using _haversine_ formula), initial bearing from a point, final bearing to a point, etc.

### erkir::ellipsoidal::Point

`erkir::ellipsoidal::Point` class represents geodetic point based on ellipsoidal earth model.
It includes ellipsoid parameters and datums for different coordinate systems, and methods for converting between them and to Cartesian coordinates.

### erkir::cartesian::Point

`erkir::cartesian::Point` implements ECEF (earth-centered earth-fixed) geocentric cartesian (x/y/z) coordinates.

### erkir::Vector3d

`erkir::Vector3d` implements 3-d vector manipulation routines.
With this class you can perform basic operations with the vectors, such as calculate dot (scalar) product of two vectors, multiply vectors, add and subtract them.

## Contributing

I would love to see your contribution :heart:

See [CONTRIBUTING](./CONTRIBUTING.md) guidelines

## Development

### Requirements

| **Name**       | **Homepage**                                                                                                      |    **Required**    | **Notes**                     |
| -------------- | ----------------------------------------------------------------------------------------------------------------- | :----------------: | ----------------------------- |
| `CXX Compiler` | [`GCC`](https://gcc.gnu.org) \| [`Clang`](https://clang.llvm.org) \| [`MSVC`](https://visualstudio.microsoft.com) | :heavy_check_mark: | `>= C++11`                    |
| `CMake`        | <https://cmake.org>                                                                                               | :heavy_check_mark: | `>= 3.9`                      |
| `Clang Format` | <https://clang.llvm.org/docs/ClangFormat.html>                                                                    |        :x:         |
| `Clang Tidy`   | <https://clang.llvm.org/extra/clang-tidy>                                                                         |        :x:         |
| `Cppcheck`     | <https://github.com/danmar/cppcheck>                                                                              |        :x:         |
| `Cpplint`      | <https://github.com/cpplint/cpplint>                                                                              |        :x:         | `pip install cpplint`         |
| `cmake lang`   | <https://github.com/cheshirekow/cmake_format>                                                                     |        :x:         | `pip install cmakelang[YAML]` |
| `Doxygen`      | <https://www.doxygen.nl>                                                                                          |        :x:         | Documentation                 |

### Build Options

| **Name**                     | **Description**      | **Type** | **Default** | **Notes**                                                                                                             |
| ---------------------------- | -------------------- | -------- | ----------- | --------------------------------------------------------------------------------------------------------------------- |
| `BUILD_SHARED_LIBS`          | Build shared library | `BOOL`   | `TRUE`      | See [`CMake` documentation](https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html) for more information |
| `ERKIR_BUILD_DOCS`           | Build documentation  | `BOOL`   | `FALSE`     | [`Doxygen`](https://www.doxygen.nl) is required                                                                       |
| `ERKIR_BUILD_TESTS`          | Build tests          | `BOOL`   | `FALSE`     | `CMake` version `>= 3.24`                                                                                             |
| `ERKIR_BUILD_TESTS_COVERAGE` | Build tests coverage | `BOOL`   | `FALSE`     | Build option `ERKIR_BUILD_TESTS` must be `TRUE`. [`LCOV`](https://github.com/linux-test-project/lcov) is required     |

### Build

1. Generate project

    > [!NOTE]
    > Change build options' values as needed

    ```sh
    cmake \
        -S . \
        -B ./build \
        -DCMAKE_BUILD_TYPE:STRING=Release \
        -DBUILD_SHARED_LIBS:BOOL=TRUE \
        -DERKIR_BUILD_DOCS:BOOL=FALSE \
        -DERKIR_BUILD_TESTS:BOOL=FALSE \
        -DERKIR_BUILD_TESTS_COVERAGE:BOOL=FALSE
    ```

1. Build project

    > [!NOTE]
    > Change value of `--config` to match value of `CMAKE_BUILD_TYPE`

    ```sh
    cmake \
        --build ./build \
        --config Release
    ```

## Documentation

> [!IMPORTANT]
> `ERKIR_BUILD_DOCS:BOOL=TRUE` required

File `index.html` available under [`build/docs/html`](./build/docs/html) directory

More information can be found in [`docs/README.md`](./docs/README.md)

```sh
cmake \
    --build ./build \
    --target erkir_docs
```

## Tests

> [!IMPORTANT]
> `ERKIR_BUILD_TESTS:BOOL=TRUE` required

> [!NOTE]
> Change value of `--build-config` to match value of `CMAKE_BUILD_TYPE`

More information can be found in [`tests/README.md`](./tests/README.md)

```sh
ctest \
    --verbose \
    --test-dir ./build/tests \
    --build-config Release
```

### Coverage

> [!IMPORTANT]
> `ERKIR_BUILD_TESTS_COVERAGE:BOOL=TRUE` required

File `index.html` available under [`build/erkir_coverage`](./build/erkir_coverage) directory

```sh
cmake \
    --build ./build \
    --config Debug \
    --target erkir_coverage
```

## Scripts

More information can be found in [`scripts/README.md`](./scripts/README.md)

## License

This project is licensed under the [MIT](https://opensource.org/license/MIT) License \
See [LICENSE](./LICENSE) file for details
