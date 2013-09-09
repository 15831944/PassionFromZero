//1. check whether the str has duplicate characters
//2. If so, delete it

#include <stdio.h>
#include <string.h>
#include <map>

using namespace std;
#define MAXLEN 30


int main()
{
    char *str_ori= new char[MAXLEN];
    memset(str_ori, 0, MAXLEN);
	int rt = scanf("%s", str_ori);

	printf("rt: %d,  str_ori is: %s\n", rt, str_ori);
	int length=strlen(str_ori);
    char dup[10]="No";

	map<char, int> stat;

    int i=0;
    for(; i< length; ++i)
    {
        stat[str_ori[i]]++;
        if (stat[str_ori[i]]>=2) snprintf(dup,4,"Yes");
    }

    printf("Duplicate: %s\n", dup);


    if ( !strcmp(dup, "Yes") )
    {
        char *str_rmdup= new char[MAXLEN];
        memset(str_rmdup, 0, MAXLEN);

        int j=0;
        for (i=0; i<length; ++i)
        {
            if (stat[str_ori[i]]<2)
            {
                str_rmdup[j]=str_ori[i];
                j++;
            }
        }
        str_rmdup[j]='\0';

        printf("non-dup: %s\n", str_rmdup);
        delete [] str_rmdup;
    }
    else
        printf("non-duplicated string is itself.\n");


    delete [] str_ori;

	return 0;
}
