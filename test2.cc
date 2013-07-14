#include <iostream>

int main()
{
	int * a[2];//它里边放了2个int * 型变量
	a[0]= new int[3]; 
	a[1]=new int[3]; 
	delete [] a[0]; 
	delete a[1];



	return 0;
}
