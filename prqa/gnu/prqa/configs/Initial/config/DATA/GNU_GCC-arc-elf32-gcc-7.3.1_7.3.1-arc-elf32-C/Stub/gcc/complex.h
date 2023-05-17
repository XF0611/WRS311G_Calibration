#ifndef QAC_COMPLEX
#define QAC_COMPLEX

/* The following macros are not defined in GCC's complex.h */
#define _Complex_I 0.0f
#define I _Complex_I

#if ((__QAC_MAJOR__ > 82) || ((__QAC_MAJOR__ == 82) && (__QAC_MINOR__ >= 2)))
#  define _Complex_I _ignore_modification
#  define I _ignore_modification
#endif


#include_next "complex.h"

#endif
