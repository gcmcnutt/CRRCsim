/***
 * for internationnalisation  by gettext() invocation
 * to include in every  .c file containing string to translate
 ***/
#include <libintl.h>
#if 1
//validation of gettext invocation
#define _(String) gettext (String)
//#define _X(String) String
#else
//not use of gettext
#define _(String) String
//#define _X(String) String
#endif
