#ifndef UTIL_H
#define UTIL_H

#define BIT(x) (1<<(x))

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof(*arr))

#define ATOMIC_BEGIN do { cli();
#define ATOMIC_END   sei(); } while(0);

#ifndef NULL
#define NULL ((void*)0U)
#endif

#endif