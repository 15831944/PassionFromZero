
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

long pow(const int & a, const int & b)
{
    long result=1;
    for (int i=0; i<b; ++i)
        result*=a;

    return result;
}


int main()
{
    printf("%ld\n",time(NULL));
    printf("2^30=%ld\n",pow(2,30));
    printf("2^31=%ld\n",pow(2,31));

    //RAND_MAX is 2147483647 (2^31-1)
    srand((unsigned int)time(NULL));

    for (int i=0;i<30;++i)
        printf("rand()[10,99]=%d\n",rand()%90+10); //[10,99]


    return 0;
}


