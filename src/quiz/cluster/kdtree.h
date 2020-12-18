/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
using namespace std; 

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertBST(Node **node, int depth, vector<float> point, int id) {
		// cout << "Entered helper function" << endl;

		if(*node == NULL) {
			// cout << "in null" << endl;
			*node = new Node(point, id);
		}

		else {
			int kd_depth = depth % 2;
			// cout << kd_depth << endl;

			if(point[kd_depth] < (*node)->point[kd_depth])
				insertBST(&((*node)->left), depth+1, point, id);
			else 
				insertBST(&((*node)->right), depth+1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertBST(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




