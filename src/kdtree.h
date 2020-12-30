#include "render/render.h"

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
			
			if(point[depth % 3] < (*node)->point[depth % 3])
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
		if(node != NULL) {
			// if the point is within the distance tolerance

			if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) && (node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol))) {
				float dist = sqrt(pow((node->point[0] - target[0]),2) + pow((node->point[1] - target[1]),2) + pow((node->point[2] - target[2]),2));
				if(dist <= distanceTol)
					ids.push_back(node->id);
			}

			//else 
			if((target[depth % 3]-distanceTol) < node->point[depth % 3]) 
				searchBST(target, ids, distanceTol, node->left, depth+1);
			if((target[depth % 3]+distanceTol) > node->point[depth % 3]) 
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

// struct Node
// {
// 	pcl::PointXYZI point;
// 	int id;
// 	Node* left;
// 	Node* right;

// 	Node(pcl::PointXYZI arr, int setId)	
// 	: point(arr), id(setId), left(NULL), right(NULL) 
// 	{}
// };

// struct KdTree
// {
// 	Node* root;

// 	KdTree() 
// 	: root(NULL) 
// 	{}

// 	void insertBST(Node** node, int depth, pcl::PointXYZI point, int id)
// 	{
//         // std::cout << "enterted insert bst function" << std::endl;
// 		if (*node == NULL)
// 		{
// 			(*node) = new Node(point, id);
// 		}
// 		else
// 		{
// 			int cd = depth % 3;  // 3 dim kd-tree

// 			if (cd == 0) 
// 			{
// 				if (point.x < (*node)->point.x) 
// 					insertBST(&(*node)->left, depth + 1, point, id);
// 				else 
// 					insertBST(&(*node)->right, depth + 1, point, id);
// 			}
// 			else 
// 			{
// 				if (point.y < (*node)->point.y) 
// 					insertBST(&(*node)->left, depth + 1, point, id);
// 				else 
// 					insertBST(&(*node)->right, depth + 1, point, id);
// 			}
// 		}
// 	}

// 	void insert(pcl::PointXYZI point, int id)
// 	{
//         // std::cout << "entered insert function" << std::endl;
// 		// TODO: Fill in this function to insert a new point into the tree
// 		// the function should create a new node and place correctly with in the root 
// 		insertBST(&root, 0, point, id);
// 	}

// 	void searchBST(pcl::PointXYZI pivot, Node* node, int depth, float distanceTol, std::vector<int>& ids)
// 	{
//         // std::cout << "entered seatchbst ffunction " << std::endl;

// 		if (node != NULL)
// 		{
// 			if ((node->point.x >= (pivot.x - distanceTol) && (node->point.x <= (pivot.x + distanceTol))) && (node->point.y >= (pivot.y - distanceTol) && (node->point.y <= (pivot.y + distanceTol))))
// 			{
// 				float distance = sqrt((node->point.x - pivot.x) * (node->point.x - pivot.x) + (node->point.y - pivot.y) * (node->point.y - pivot.y));

// 				if (distance <= distanceTol) 
// 					ids.push_back(node->id);
// 			}
// 			if (depth % 3 == 0) // 3 dim kd-tree
// 			{
// 				if ((pivot.x - distanceTol) < node->point.x) 
// 					searchBST(pivot, node->left, depth + 1, distanceTol, ids);

// 				if ((pivot.x + distanceTol) > node->point.x) 
// 					searchBST(pivot, node->right, depth + 1, distanceTol, ids);
// 			}
// 			else 
// 			{
// 				if ((pivot.y - distanceTol) < node->point.y) 
// 					searchBST(pivot, node->left, depth + 1, distanceTol, ids);
// 				if ((pivot.y + distanceTol) > node->point.y) 
// 					searchBST(pivot, node->right, depth + 1, distanceTol, ids);
// 			}

// 		}
// 	}

// 	// return a list of point ids in the tree that are within distance of pivot
// 	std::vector<int> search(pcl::PointXYZI pivot, float distanceTol)
// 	{
//         // std::cout << "enterted search function" << std::endl;
// 		std::vector<int> ids;
// 		searchBST(pivot, root, 0, distanceTol, ids);

// 		return ids;
// 	}
// };
