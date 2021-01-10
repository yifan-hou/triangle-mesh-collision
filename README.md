# triangle-mesh-collision
This is a wrapper of CGAL using Eigen data type. Currently supporting:
- Triangle to triangle collision detection between two meshes.
- Ray-tracing on a mesh.

# Install
If installing to home folder, add
```
-DCMAKE_INSTALL_PREFIX=$HOME
```
to every cmake command.

## Dependencies
**Eigen** : Eigen should come with Ubuntu 18.04.

[**CGAL**](https://github.com/CGAL/cgal): The library that does the actual work.

The installation of CGAL can be nasty. The following procedure should work.

If you miss any dependency, [this document](https://docs.google.com/viewer?a=v&pid=sites&srcid=ZGVmYXVsdGRvbWFpbnxzaW5naHN1a2hyYWp8Z3g6MThiNjRkNjhhMWY0ZjgyYw) is a good reference(though it is for an older version)
Note you don't have to install QT. It is only useful for visualization with CGAL, which we don't need for now.

1. Download the CGAL release from [here](https://github.com/CGAL/cgal/releases), scroll down to `CGAL-4.1`, click to unfold `Assets`, download `CGAL-4.14.tar.xz`.
2. Unzip the file.
2. Build and install:
```
    cd CGAL-4.14
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
    sudo make install
```

## Build and install
Now you have all the dependencies, go back to this folder and:
```
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
    sudo make install
```

**Notes**
* Currently only supports .off file type.