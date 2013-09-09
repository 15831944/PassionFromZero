
#include <iostream>

using namespace std;

#define LEN 12


void quicksort(int* src, const int & length, int pivot, int dst[])   // can we use int* XXX instead of int XXX[]?
{
    if (length==1)
    {
        dst[0]=src[0];
        return;
    }

	int* less = new int[length];
	int* more = new int[length];
	int lessLen=0, moreLen=0;
	int* lessResult = new int[length];
	int* moreResult = new int[length];

	int i;
	for (i=0; i<length; ++i)
	{
		if ( src[i]<src[pivot] )
            less[lessLen++]=src[i];
		else
            more[moreLen++]=src[i];
	}
	 
	if (moreLen==length)  // src[median] happens to be the lowest
	{
        if (pivot)
        {
            pivot--;
            quicksort(src, length, pivot, dst);
        }
        else // all the elements in more[] are the same!
        {
            for (int j=0; j<moreLen; ++j)
                dst[j]=more[j];
        }
	}
	else
	{
		quicksort(less, lessLen, lessLen-1, lessResult);
		quicksort(more, moreLen, moreLen-1, moreResult);
	
		int j;

        /*
        // ascending
        for (j=0; j<lessLen; ++j)
			dst[j]=lessResult[j];
	
		for (j=0; j<moreLen; ++j)
            dst[lessLen+j]=moreResult[j];
        */

        // descending
        for (j=0; j<moreLen; ++j)
            dst[j]=moreResult[j];

        for (j=0; j<lessLen; ++j)
            dst[moreLen+j]=lessResult[j];
	}

	delete [] less;
	delete [] more;
	delete [] lessResult;
	delete [] moreResult;

	return;
}


int main()
{
	//--- create source
	int* srcarray = new int[LEN];
    srcarray[0]=0;
    srcarray[1]=11;
    srcarray[2]=-2;
    srcarray[3]=3;
    srcarray[4]=0;
    srcarray[5]=1;
    srcarray[6]=4;
    srcarray[7]=0;
    srcarray[8]=3;
    srcarray[9]=4;
    srcarray[10]=3;
    srcarray[11]=10;


	//--- sort
	int* dstarray = new int[LEN];
	//memset(dstarray, 0, sizeof(int)*LEN);
	quicksort(srcarray, LEN, LEN-1, dstarray);


	//--- output
	int i;
	for (i=0; i<LEN; ++i)
		cout << srcarray[i] << " "; 

	cout << endl;

	for (i=0; i<LEN; ++i)
		cout << dstarray[i] << " "; 

	cout << endl;


	//--- memory release
	delete [] srcarray;
	delete [] dstarray;
	

	return 0;
}

