#include <stdio.h>

template <class T>
void SwapAnything(T &a,T &b)
{
	T c=a;
	a=b;
	b=c;
}

template <class T>
void SampleQuickSort(int nObj,T obj[])
{
	if(1>=nObj)
	{
		return;
	}
	else
	{
		T ref=obj[nObj/2];
		int nLess=0,nLarger=0;
		T *less=new T [nObj];
		T *larger=new T [nObj];

		int i;
		for(i=0; i<nObj; i++)
		{
			if(obj[i]<ref)
			{
				less[nLess++]=obj[i];
			}
			else
			{
				larger[nLarger++]=obj[i];
			}
		}

		if(0==nLess)
		{
			nLess=0;
			nLarger=0;
			for(i=0; i<nObj; i++)
			{
				if(obj[i]==ref)
				{
					less[nLess++]=obj[i];
				}
				else
				{
					larger[nLarger++]=obj[i];
				}
			}
		}

		if(0==nLess || 0==nLarger)
		{
			return;
		}

		SampleQuickSort(nLess,less);
		SampleQuickSort(nLarger,larger);

        /*
        // ascending
		for(i=0; i<nLess; i++)
			obj[i]=less[i];

		for(i=0; i<nLarger; i++)
			obj[nLess+i]=larger[i];
        */

        // descending
        for(i=0; i<nLarger; i++)
            obj[i]=larger[i];

        for(i=0; i<nLess; i++)
            obj[nLarger+i]=less[i];


		delete [] less;
		delete [] larger;
	}
}

int main(void)
{
    int numbers[]={8,1,3,7,6,6,9,3,0,4};

	SampleQuickSort(10,numbers);

	int i;
	for(i=0; i<10; i++)
	{
		printf("%d\n",numbers[i]);
	}

	return 0;
}
