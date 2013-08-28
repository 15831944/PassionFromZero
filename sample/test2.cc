#include <iostream>
#include <stdio.h>
#include <vector>

using namespace std;

int main()
{

/*
	int * a[2];//它里边放了2个int * 型变量
	a[0]= new int[3]; 
	a[1]=new int[3]; 
	delete [] a[0]; 
	delete a[1];
*/
    vector<int> v(5);
	
    //v.clear();
	for (int i=5; i>0; --i)
        v.push_back(i*i);

    vector<int>::const_iterator itr=v.begin();
    cout << *itr << endl;

    itr=itr+5;
    cout << *itr << endl;
/*	
	//unsigned short tar=-2;
	unsigned short tar=0xFFFE;
	
	printf("tar=%u\n", tar);
	short tar2= tar;
	printf("tar2=%d\n", tar2);
*/
	unsigned short i = 0xFFFF;		
	short j = 0xFFFF;
	short k = 0x0FFF;
	cout<<i<<"\t"<<j<<"\t"<<k<<endl;
	i>>=8;
	j>>=8;
	k>>=8;
	cout<<i<<"\t"<<j<<"\t"<<k<<endl;

	return 0;
}
