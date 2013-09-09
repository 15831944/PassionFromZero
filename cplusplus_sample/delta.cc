#include <iostream>

#include <cmath>

using namespace std;

double computeSmallestDelta(const double & agl1, const double & agl2)
{
	double m;
	m=abs(agl1-agl2);
	// the request m should be between 0~180
	while (m>180)
	{
		m=abs(m-360);
	}
	return m;
}

int main()
{
    double agl1, agl2, delta;
	cout<< "input the two angles in degree:->" << endl;
	cin >> agl1 >> agl2 ;
	delta=computeSmallestDelta(agl1, agl2);
	cout << "The smallest delta is:" << delta<<endl;
	return 0;
}



