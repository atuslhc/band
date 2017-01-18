#ifndef DEBUG_H
#define DEBUG_H

//
#define ENABLE_DEBUG_LOG1

//
#if defined(ENABLE_DEBUG_LOG) && defined(DEBUG)

#include <stdio.h>
#define CCTRACE(X, ...) printf(X, ##__VA_ARGS__)

#include <assert.h>
#define CCASSERT(X)	assert(X)

#else

#define CCTRACE(X, ...)
#define CCASSERT(X)

#endif

#endif

