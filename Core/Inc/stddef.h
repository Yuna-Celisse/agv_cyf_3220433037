#ifndef _PROJECT_STDDEF_H
#define _PROJECT_STDDEF_H

#if defined(_WIN64) || defined(__x86_64__) || defined(__aarch64__)
typedef unsigned long long size_t;
typedef long long ptrdiff_t;
#else
typedef unsigned int size_t;
typedef int ptrdiff_t;
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

#define offsetof(type, member) ((size_t)&(((type *)0)->member))

#endif
