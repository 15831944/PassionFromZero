/**
 * Test of HP Vertica
 * ----------------------------------------------------------------
 * Given an input sequence of n integers,
 * each of which has a value between 1 and 100 (inclusive),
 * write a function that outputs the sum of the k largest integers.
 * ----------------------------------------------------------------
 * @version: C++
 * @author: Haiyue (Bruce) Li (jkzuoluo19@gmail.com)
 * @date: 10/08/2013
 *
 * Copyright (c) 2013, Haiyue Li. All rights reserved.
 */

#include <iostream>
#include <vector>
#include <limits>

using namespace std;


/**
 * @brief quicksort: Sort a vector in descending order.
 * @param src:  Source vector
 * @param srcLen: Size of the source vector
 */
void quicksort(vector<int> & src, int srcLen)
{
    if (srcLen<=1)
        return;

    vector<int> less(srcLen);
    vector<int> more(srcLen);
    int lessLen=0, moreLen=0;
    int pivot=src[srcLen/2];

    int i;
    for (i=0; i<srcLen; ++i)
    {
        if ( src[i]<pivot )
            less[lessLen++]=src[i];
        else
            more[moreLen++]=src[i];
    }

    if (lessLen==0)  // src[median] happens to be the lowest (moreLen==srcLen)
    {
        lessLen=0;
        moreLen=0;

        //less.clear();
        //more.clear();
        for (int i=0; i<srcLen; ++i)
        {
            if ( src[i]==pivot )
                less[lessLen++]=src[i];
            else
                more[moreLen++]=src[i];
        }
    }

    if (lessLen==0 || moreLen==0) // all the numbers are the same
        return;

    quicksort(less, lessLen);
    quicksort(more, moreLen);

    // descending
    for (i=0; i<moreLen; ++i)
        src[i]=more[i];

    for (i=0; i<lessLen; ++i)
        src[moreLen+i]=less[i];


    return;
}


/**
 * @brief prepareSource: Input 1. the source integer sequence
 *                             2. the k (the number of integers you want to sum)
 *                       <Note> those inappropriate numbers or k will be dumped.
 * @param srcData: Store the integer sequence
 * @return k
 */
int prepareSource(vector<int> & srcData)
{
    int value;
    cout<<"Input numbers between 1 and 100, end up with';' -->"<<endl;
    while (cin >> value)
    {
        if (value<1 || value>100)
            cout<<"Inappropriate number is dumped!"<<endl;
        else
            srcData.push_back(value);
    }

    cin.clear();                                            // set cin flag to 'true'
    cin.ignore( numeric_limits<streamsize>::max( ), '\n' ); // clear the buffer until '\n'

    int k;
    cout<<"Input k between 1 and data's size -->"<<endl;
    while (cin >> k)
    {
        if (k<1 || k>(int)srcData.size())
            cout<<"k is inappropriate, input again!"<<endl;
        else
            break;
    }

    return k;
}


/**
 * @brief sumOfkLargest: The target algorithm
 * @param srcData: Source integer sequence
 * @param k: The number of integers you want to sum
 * @return sum result
 */
int sumOfkLargest(vector<int> & srcData, const int & k)
{
    //--- sort
    quicksort(srcData, srcData.size());
    /*
    //DEBUG
    for(vector<int>::const_iterator itr=srcData.begin(); itr<srcData.end(); ++itr)
        cout<<*itr<<" ";

    cout << endl;
    */


    //--- sum
    int sum=0;
    for (int i=0; i<k; ++i)
        sum+=srcData[i];


    return sum;
}



int main()
{
    //--- create source
    vector<int> srcData;
    srcData.reserve(50);

    int k=prepareSource(srcData);


    //--- output srcData
    cout <<"~~~~~~~~~~~~~~~"<<endl<<"The data is: ";
    for(vector<int>::const_iterator itr=srcData.begin(); itr<srcData.end(); ++itr)
        cout<<*itr<<" ";

    cout <<endl;


    //--- output sum
    cout << "The sum of "<< k <<" largest numbers is: "<< sumOfkLargest(srcData, k) << endl;


    return 0;
}




