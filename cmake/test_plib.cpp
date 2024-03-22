#include <plib/ul.h>
#include <plib/sg.h>
#include <plib/ssg.h>
#include <plib/ssgaSky.h>
#include <plib/pu.h>
#include <plib/puAux.h>

#define MIN_PLIB_VERSION 184

int main(void)
{
  if ( PLIB_VERSION < MIN_PLIB_VERSION ) 
    return 0;
  else
    return 42;
}
