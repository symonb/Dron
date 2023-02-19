#ifndef STATISTICS_H_
#define STATISTICS_H_

typedef struct
{
    float m_oldM;   // previous average of measurements
    float m_newM;   // current average of measurements
    float m_oldS;   // previous value of variance*n-1
    float m_newS;   // current value of varaince*n
    int m_n;        // counter of measurements
} stdev_t;

void dev_clear(stdev_t* dev);
void dev_push(stdev_t* dev, float x);
float dev_variance(stdev_t* dev);
float dev_standard_deviation(stdev_t* dev);


#endif /* STATISTICS_H_ */
