#include <stdio.h>
#include <iostream>
#include <vector>
#include <typeinfo>

using namespace std;

int main()
{
/*
    // test '%'  
    int a = 8 % 4;
    printf("----->%d\n",a);
*/

/*
    // test Const Reference    
    int int1 = 5;
    const int& crint = int1;
    cout<< "int1="<< int1<< ","<< "crint="<< crint<< endl;

    int1 = 8;
    cout<< "After changing int1,"<< endl
        << "int1="<< int1<< ","<< "crint="<< crint<< endl;

    //crint =10; // will cause error
*/


    //test vector array
    //vector<int>* va = new vector<int>[10];
    //vector<int> va[10]; //效果与上面等价
    vector< vector<int> > va(10);//, vector<int>(1));  //这个用法太经典！！！用后vector可以用'='直接赋值了

    //va[0][0]=4;
    cout << "^^^^^^^^^^^^^^^^^^^^"<<endl;
    va[0].push_back(11111);   
    va[0].push_back(22222); 
    va[0].push_back(33333);
    va[5].push_back(123);
    va[4].push_back(-100);      
  
    cout << typeid(va).name() <<endl;
    cout << typeid(va[0]).name() <<endl;
    cout << typeid(va[1]).name() <<endl;
    cout << typeid(va[1][0]).name() <<endl;
    cout << typeid(va[0][0]).name() <<endl;


    cout << "va[0][0]=" << va[0][0] << endl; 
    cout << "va[0][1]=" << va[0][1] << endl;
    cout << "va[0][2]=" << va[0][2] << endl;
    cout << "va[0][3]=" << va[0][3] << endl;
    cout << "va[0][4]=" << va[0][4] << endl;
    cout << "va[0][5]=" << va[0][5] << endl;
    //cout << "va[1][0]=" << va[1][0] << endl;  // output: Segmentation fault
    cout << "va[5][0]=" << va[5][0] << endl; 
    //cout << "va[4][0]=" << va[4][0] << endl;
    cout << "va[0].size()=" << va[0].size() <<endl;
    cout << "va[1].size()=" << va[1].size() <<endl;
    //cout << "va[0].max_size()=" << va[0].max_size() <<endl;
    cout << "va[0].capacity()=" << va[0].capacity() <<endl;
    cout << "va[4].size()=" << va[4].size() <<endl;
    //cout << "va[4].max_size()=" << va[4].max_size() <<endl;
    cout << "va[4].capacity()=" << va[4].capacity() <<endl;
    cout << "va[3].size()=" << va[3].size() <<endl;
    cout << "va[3].capacity()=" << va[3].capacity() <<endl;

    //va[0].clear();      cout << "********after va[0].clear()********"<< endl;  
    //va[0].resize(0);    cout << "********after va[0].resize(0)********"<< endl;
    //va[0].pop_back();      cout << "********after va[0].pop_back()********"<< endl;
   
    cout << "va.size()=" << va.size() <<endl;
    cout << "va.capacity()=" << va.capacity() <<endl;
    cout << "va[0].size()=" << va[0].size() <<endl;
    cout << "va[0].capacity()=" << va[0].capacity() <<endl;
    va.clear();      cout << "********after va.clear()********"<< endl; 
    cout << "va.size()=" << va.size() <<endl;
    cout << "va.capacity()=" << va.capacity() <<endl;
    cout << "va[0].size()=" << va[0].size() <<endl;
    cout << "va[0].capacity()=" << va[0].capacity() <<endl;
    va[0].clear();      cout << "******** and after va[0].clear()********"<< endl;
    cout << "va.size()=" << va.size() <<endl;
    cout << "va.capacity()=" << va.capacity() <<endl;
    cout << "va[0].size()=" << va[0].size() <<endl;
    cout << "va[0].capacity()=" << va[0].capacity() <<endl;



    cout << "va[0][0]=" << va[0][0] << endl; 
    cout << "va[0][1]=" << va[0][1] << endl;    
    cout << "va[0][2]=" << va[0][2] << endl;
    cout << "va[0][3]=" << va[0][3] << endl;
    cout << "va[0][4]=" << va[0][4] << endl;
    cout << "va[0][5]=" << va[0][5] << endl;
    //cout << "va[4][0]=" << va[4][0] << endl;
    cout << "va[0].size()=" << va[0].size() <<endl;
    for (vector<int>::const_iterator itr=va[0].begin(); itr != va[0].end(); ++itr)
    {
        cout << *itr <<endl;
    } 
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" <<endl;




    //cout << "va[0].max_size()=" << va[0].max_size() <<endl;
    cout << "va[0].capacity()=" << va[0].capacity() <<endl;
    cout << "va[4].size()=" << va[4].size() <<endl;
    //cout << "va[4].max_size()=" << va[4].max_size() <<endl;
    cout << "va[4].capacity()=" << va[4].capacity() <<endl;
    cout << "va[3].size()=" << va[3].size() <<endl;
    cout << "va[3].capacity()=" << va[3].capacity() <<endl;



/*
    //test output 'unsigned char'
    char c,c2,c3;
    unsigned char uc, uc2, uc3;
    c=0; c2=97; c3=255;
    uc=0; uc2=97; uc3=255;

    printf("c=%c, c2=%c, c3=%c \n",c,c2,c3);
    printf("uc=%u, uc2=%u, uc3=%u \n",uc,uc2,uc3);
    cout << "c=" << c << ", c2=" << c2 << ", c3=" << c3 << endl;
    cout << "uc=" << uc << ", uc2=" << uc2 << ", uc3=" << uc3 << endl;

    cout << "c=" << (int)c << ", c2=" << (int)c2 << ", c3=" << (int)c3 << endl;
    cout << "uc=" << (int)uc << ", uc2=" << (int)uc2 << ", uc3=" << (int)uc3 << endl;
*/




    return 0;
}
