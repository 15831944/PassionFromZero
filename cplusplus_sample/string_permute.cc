#include <iostream>
#include <string.h>

using namespace std;

int permute(const string &str)
{
	int length = str.length();
	cout << "length=" << length << endl;
	
	bool * used = new bool[length];
	char * buffer = new char[length];
	if (!used || !buffer) return -1;
	memset(used, false, length);
	//memset(buffer, 0, length);
	
	////////////////////////////////
	void do_permute(const string &str, const int &length, bool * used, char * buffer, int level);
	do_permute(str, length, used, buffer, 0);
	////////////////////////////////
	
	delete [] used;
	delete [] buffer;
	
	return 0;
}

void do_permute(const string &str, const int &length, bool * used, char * buffer, int level)
{	
	if (level == length)
	{
		cout << buffer << endl;
		return;
	}
	
	int i;
	for (i=0; i<length; ++i)
	{
		if (used[i]) continue;
		
		buffer[level]=str[i];
		used[i]=true;
		do_permute(str, length, used, buffer, level+1);
		used[i]=false;
	}

}

int main()
{
	string str;
	cout << "Input the string->" << endl;
	cin >> str;
	cout << "------------------" << endl;

	if (permute(str)<0) cout<<"Failed"<<endl;

	return 0;
}














