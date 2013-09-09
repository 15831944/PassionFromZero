#include <iostream>
#include <stdlib.h>     // srand, rand
#include <time.h>       // time 

using namespace std;


class BinarySearchTree;

class Nodes
{
protected:
	int value;

public:
	Nodes* left;
	Nodes* right;
	Nodes* parent;
	
	Nodes():value(0), left(NULL), right(NULL), parent(NULL) {};
	Nodes(int vl):value(vl), left(NULL), right(NULL), parent(NULL) {};

	void print() { cout << value << endl; };	
	
	friend class BinarySearchTree;
};


class BinarySearchTree
{
protected:
	Nodes* root;

public:
	BinarySearchTree():root(NULL) {};
	
	Nodes* getroot() { return root; };
	Nodes* addNodes(int data);
	void preorderTraverse(Nodes* seeker);
	void postorderTraverse(Nodes* seeker);
	
	void postorderDelete(Nodes* seeker);	
	void deleteAll();
};


Nodes* BinarySearchTree::addNodes(int data)
{
	Nodes* newNode=new Nodes(data);
	
	if (root==NULL) 
	{
		root=newNode;
		return root;
	}

	Nodes* seeker=root;
	while(1)
	{
		if (data <= seeker->value)
		{	
			if (!seeker->left)
			{
				seeker->left=newNode;
				newNode->parent=seeker;
				break;
			}
			else
			{
				seeker=seeker->left;
			}
		}
		else
		{
			if (!seeker->right)
			{
				seeker->right=newNode;
				newNode->parent=seeker;
				break;
			}
			else
			{
				seeker=seeker->right;
			}
		}
	
	}
	
	return root;
}


void BinarySearchTree::preorderTraverse(Nodes* seeker)
{	
	if (!root)
	{
		cout << "Empty Binary Search Tree~" << endl;
		return;
	}

	if (!seeker)
	{
		return;
	}
	seeker->print();
	preorderTraverse(seeker->left);
	preorderTraverse(seeker->right);
	
	return;
}


void BinarySearchTree::postorderTraverse(Nodes* seeker)
{
	if (!root)
	{
		cout << "Empty Binary Search Tree~" << endl;
		return;
	}

	if (!seeker)
	{
		return;
	}
	
	postorderTraverse(seeker->left);
	postorderTraverse(seeker->right);	
	seeker->print();
	
	return;
}


void BinarySearchTree::postorderDelete(Nodes* seeker)
{
	if (!root)
	{
		cout << "Empty Binary Search Tree~" << endl;
		return;
	}

	if (!seeker)
	{
		return;
	}
	
	postorderDelete(seeker->left);
	postorderDelete(seeker->right);	
	delete seeker;
	
	return;
}


void BinarySearchTree::deleteAll()
{
	postorderDelete(root);
	return ;
}


int main()
{
	srand(time(NULL));
	
	BinarySearchTree BSTree;
	
/*	
	int data;
	for (int i=0; i<10; ++i)
	{
		data=rand()%100+1;
		cout << data << endl;
		BSTree.addNodes(data);
	}	
*/

	BSTree.addNodes(37);
	BSTree.addNodes(10);
	BSTree.addNodes(20);
	BSTree.addNodes(40);
	BSTree.addNodes(66);
	BSTree.addNodes(6);
	BSTree.addNodes(5);
	BSTree.addNodes(38);
	BSTree.addNodes(9);	

	cout << "----------" << endl;

	cout << "preorder:"<< endl;
	BSTree.preorderTraverse(BSTree.getroot());
	cout << "postorder:"<< endl;
	BSTree.postorderTraverse(BSTree.getroot());

	BSTree.deleteAll();

	return 0;
}
