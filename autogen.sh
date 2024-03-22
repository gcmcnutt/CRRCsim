#!/bin/sh
#

# remove old aclocal.m4 just in case it was created
# by a different autotools version
if [ -e aclocal.m4 ]
then
  echo "Removing old aclocal.m4..."
  rm -f aclocal.m4
fi

echo "Running autoheader..."
autoheader
echo "Running aclocal..."
aclocal -I m4
echo "Running automake..."
automake --add-missing --foreign
echo "Running autoconf..."
autoconf

if [ -x configure ]
then
  echo
  echo "--------------------------------------------"
  echo "Success, now you're ready to run ./configure"
  echo "--------------------------------------------"
  echo
else
  echo
  echo "--------------------------------------------------------------"
  echo "No ./configure script found, something must have gone wrong..."
  echo "--------------------------------------------------------------"
  echo
fi
