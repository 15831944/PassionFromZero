# include <iostream>

#define DynMemAllocFailed   -10
#define NoNodetoRemove      -11
#define NoNodetoInsertAfter -12


using namespace std;

class linkNode
{
protected:
	int value;

public:
	linkNode* next;
	
	linkNode():value(0),next(NULL) {};
	linkNode(int vl):value(vl),next(NULL) {};
	
	int get() 			{ return value; };
	void set(int vl) 	{ value=vl; };
	void print() 		{ cout << value << "	"; };
};


class SingleList
{
protected:
	linkNode* head;
	linkNode* tail;
	
public:
	SingleList():head(NULL), tail(NULL) {};	
	
	linkNode* find(int data);
	bool remove(int data);
	bool insertAfter(linkNode* ptr, int data);
	void print();
	void deleteAll();
};


linkNode* SingleList::find(int data)
{
	if (!head) return NULL;

	linkNode* ptr=head;
	while (ptr && ptr->get() != data)
	{
		ptr=ptr->next;	
	}

	return ptr;
}


bool SingleList::remove(int data)
{
	linkNode* ptr=head;
	if (!ptr)                         // for 0-list
	{
		throw NoNodetoRemove;
		return false;
	}
	
	
	if (head && head==tail)           // for 1-list
	{
		if (ptr->get()==data)
		{
			head=NULL;
			tail=NULL;
			delete ptr;
			return true;
		}
		else
		{
			throw NoNodetoRemove;
			return false;
		}
	}
	
								      // for 2 or up -list
	if (data==head->get())
	{
		head=head->next;
		delete ptr;
		return true;
	}
	
	while(ptr)
	{
		if (ptr->next && ptr->next->get()==data)
		{
			linkNode* deleteMe=ptr->next;
			ptr->next=ptr->next->next;
			delete deleteMe;
			
			if (!ptr->next) tail=ptr;
			
			return true;
		}

		ptr=ptr->next;	
	}
	
	
	
	throw NoNodetoRemove;
	return false;
	
}


bool SingleList::insertAfter(linkNode* ptr, int data)
{
	linkNode* newNode = new linkNode(data);
	if (!newNode)
	{
		throw DynMemAllocFailed;
		return false;
	}
	
	if (!ptr)                  // for insert a new head
	{
		linkNode* temp=head;
		head=newNode;
		head->next=temp;
		
		if (!tail) tail=head;  // for 0-list
		
		return true;
	}
	
	linkNode* node=find(ptr->get());
	if (!node)				   // for node doesn't exist
	{
		delete newNode;
		throw NoNodetoInsertAfter;
		return false;
	}
	
			
	newNode->next=node->next;
	node->next=newNode;
	
	if (node==tail) tail=newNode;
	
	return true;
}


void SingleList::print()
{
	if (!head) 
	{
		cout << "Empty List~"<< endl;
		return;
	}

	linkNode* ptr=head;
	while(ptr)
	{
		ptr->print();
		ptr=ptr->next;
	}
	
	cout << endl;
}


void SingleList::deleteAll()
{
	linkNode* ptr=head;
	while(ptr)
	{
		head=head->next;
		delete ptr;
		ptr=head;
	}	

	tail=NULL;
}


int main()
{
try
{
	SingleList singlelist;
	
	
	singlelist.insertAfter(NULL,13);
	singlelist.print();
	
	singlelist.insertAfter(singlelist.find(13),198);
	singlelist.print();
	
	singlelist.insertAfter(singlelist.find(13),-25);
	singlelist.print();
	
	
	linkNode* node= new linkNode(44);
	singlelist.insertAfter(node,-20);
	singlelist.print();
	
	singlelist.remove(13);
	singlelist.print();
	
	//singlelist.remove(-20+1);
	//singlelist.print();
	
	//singlelist.remove(198);
	//singlelist.print();
	
	
	singlelist.deleteAll();
	singlelist.print();

}

catch (const int& exct)
{
	switch (exct)
	{
	case DynMemAllocFailed:   cout << "Error: Dynamic Memory Allocation Failed!" << endl;
							  break;
	case NoNodetoRemove: 	  cout << "Error: No Node to Remove!" << endl;
							  break;
	case NoNodetoInsertAfter: cout << "Error: No Node to Insert After!"<< endl;
							  break;

	} 
}	

	return 0;
	
}



