// reverse a string

#include <stdio.h>
#include <string.h>
#include <string>

int main()
{
	char str_ori[]="magzine";
	
	int length=strlen(str_ori);
	char *str_rev= new char[length+1];
	int i=0;
	for (; i<length; ++i )
		str_rev[i]=str_ori[length-1-i];

	str_rev[i]='\0';

	printf("str_ori=%s\nstr_rev=%s\n", str_ori, str_rev);
	delete [] str_rev;
	
	int a=10,b=20;
	int* const p=new int(b);
	*p=a;
	printf("%d\n", *p);
	
	
	return 0;
}


