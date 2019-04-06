#ifndef TIME_H
#define TIME_H

#define LT -1
#define EQ  0
#define GT  1

struct timeval_t {
	uint64_t secs;
	uint64_t nsecs;
};

void now(struct timeval_t* t);
void now_fast(struct timeval_t* t);
void timeval_add_nsec(struct timeval_t* t, uint64_t nsecs);
int8_t timecmp(struct timeval_t* a, struct timeval_t* b);
void timedelta(struct timeval_t* pre, struct timeval_t* post, struct timeval_t* delta);

#endif