#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>

using namespace std;

int main()
{
	/*
	//int arrayA[] = new int[10];  // incorrect
	int *arrayA = new int[10];
	//int var= new int;  // incorrect
	int *ptr= new int;
	
	int a='a'*2;
	
	printf("a=%d\n",a);
	*/

	string str;
	str="HelloBruce";
	cout << str.length() << endl;
	char* chararray = new char[str.length()+1];
	unsigned int i;
	
	for (i=0; i<str.length(); ++i)
	{
		cout << "str[" << i << "]=" << str[i] << endl;
		chararray[i]=str[i];
		cout << "chararray[" << i << "]=" << chararray[i] << endl;	
	}
	cout<< "strlen=" << strlen(chararray) << endl;
	chararray[i]='\0';  //chararray[i]='Q';
	cout<< "strlen=" << strlen(chararray) << endl;
	cout << chararray << endl;

	cout << "complete output" << endl;
	for (i=0; i<(str.length()+1); ++i)
		cout << "chararray[" << i << "]=" << chararray[i] << endl;

	delete [] chararray;







return 0;
}
