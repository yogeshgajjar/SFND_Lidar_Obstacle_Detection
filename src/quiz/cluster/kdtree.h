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

		if(*node == NULL) {
			*node = new Node(point, id);
		}

		else {
			
			if(point[depth % 2] < (*node)->point[depth % 2])
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

	void searchBST(vector<float> &target, vector<int> &ids, float distanceTol, Node *node, int depth) {

		// target(x1,y1), node->points(x2,y2) -> calculate eucledian distance
		if(node) {
			// if the point is within the distance tolerance

			if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))) {
				float dist = sqrt(pow((node->point[0] - target[0]),2) + pow((node->point[1] - target[1]),2));
				if(dist <= distanceTol)
					ids.push_back(node->id);
			}

			//else 
			if((target[depth % 2]-distanceTol) < node->point[depth % 2]) 
				searchBST(target, ids, distanceTol, node->left, depth+1);
			if((target[depth % 2]+distanceTol) > node->point[depth % 2]) 
				searchBST(target, ids, distanceTol, node->right, depth+1);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchBST(target, ids, distanceTol, root, 0);
		return ids;
	}
	

};




