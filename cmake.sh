#! /bin/sh

#
# Arguments given are passed to cmake. Examples:
#    -D CMAKE_INSTALL_PREFIX=/usr
# To install CRRCSim, run 
#   make install
# in ./out_cmake/ after successfull compilation

mkdir -p ./out_cmake/
cd out_cmake
echo "args given to cmake: $*"
cmake -D CMAKE_BUILD_TYPE=Release $* ../
make
