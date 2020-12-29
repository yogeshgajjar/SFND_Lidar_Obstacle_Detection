// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "render/render.h"
// #include "quiz/cluster/kdtree.h"

using namespace std;

struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)	
	: point(arr), id(setId), left(NULL), right(NULL) 
	{}
};

struct KdTree
{
	Node* root;

	KdTree() 
	: root(NULL) 
	{}

	void insertBST(Node** node, int depth, pcl::PointXYZI point, int id)
	{
		if (*node == NULL)
		{
			(*node) = new Node(point, id);
		}
		else
		{
			int cd = depth % 3;  // 3 dim kd-tree

			if (cd == 0) 
			{
				if (point.x < (*node)->point.x) 
					insertBST(&(*node)->left, depth + 1, point, id);
				else 
					insertBST(&(*node)->right, depth + 1, point, id);
			}
			else 
			{
				if (point.y < (*node)->point.y) 
					insertBST(&(*node)->left, depth + 1, point, id);
				else 
					insertBST(&(*node)->right, depth + 1, point, id);
			}
		}
	}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertBST(&root, 0, point, id);
	}

	void searchBST(pcl::PointXYZI pivot, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if ((node->point.x >= (pivot.x - distanceTol) && (node->point.x <= (pivot.x + distanceTol))) && (node->point.y >= (pivot.y - distanceTol) && (node->point.y <= (pivot.y + distanceTol))))
			{
				float distance = sqrt((node->point.x - pivot.x) * (node->point.x - pivot.x) + (node->point.y - pivot.y) * (node->point.y - pivot.y));

				if (distance <= distanceTol) 
					ids.push_back(node->id);
			}
			if (depth % 3 == 0) // 3 dim kd-tree
			{
				if ((pivot.x - distanceTol) < node->point.x) 
					searchBST(pivot, node->left, depth + 1, distanceTol, ids);

				if ((pivot.x + distanceTol) > node->point.x) 
					searchBST(pivot, node->right, depth + 1, distanceTol, ids);
			}
			else 
			{
				if ((pivot.y - distanceTol) < node->point.y) 
					searchBST(pivot, node->left, depth + 1, distanceTol, ids);
				if ((pivot.y + distanceTol) > node->point.y) 
					searchBST(pivot, node->right, depth + 1, distanceTol, ids);
			}

		}
	}

	// return a list of point ids in the tree that are within distance of pivot
	std::vector<int> search(pcl::PointXYZI pivot, float distanceTol)
	{
		std::vector<int> ids;
		searchBST(pivot, root, 0, distanceTol, ids);

		return ids;
	}
};


// struct Node
// {
// 	std::vector<float> point;
// 	int id;
// 	Node* left;
// 	Node* right;

// 	Node(std::vector<float> arr, int setId)
// 	:	point(arr), id(setId), left(NULL), right(NULL)
// 	{}
// };

// struct KdTree
// {
// 	Node* root;

// 	KdTree()
// 	: root(NULL)
// 	{}

// 	void insertBST(Node **node, int depth, vector<float> point, int id) {

// 		if(*node == NULL) {
// 			*node = new Node(point, id);
// 		}

// 		else {
			
// 			if(point[depth % 2] < (*node)->point[depth % 2])
// 				insertBST(&((*node)->left), depth+1, point, id);
// 			else 
// 				insertBST(&((*node)->right), depth+1, point, id);
// 		}
// 	}

// 	void insert(std::vector<float> point, int id)
// 	{
// 		// TODO: Fill in this function to insert a new point into the tree
// 		// the function should create a new node and place correctly with in the root 
// 		insertBST(&root, 0, point, id);
// 	}

// 	void searchBST(vector<float> &target, vector<int> &ids, float distanceTol, Node *node, int depth) {

// 		// target(x1,y1), node->points(x2,y2) -> calculate eucledian distance
// 		if(node) {
// 			// if the point is within the distance tolerance

// 			if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))) {
// 				float dist = sqrt(pow((node->point[0] - target[0]),2) + pow((node->point[1] - target[1]),2));
// 				if(dist <= distanceTol)
// 					ids.push_back(node->id);
// 			}

// 			//else 
// 			if((target[depth % 2]-distanceTol) < node->point[depth % 2]) 
// 				searchBST(target, ids, distanceTol, node->left, depth+1);
// 			if((target[depth % 2]+distanceTol) > node->point[depth % 2]) 
// 				searchBST(target, ids, distanceTol, node->right, depth+1);
// 		}
// 	}
// 	// return a list of point ids in the tree that are within distance of target
// 	std::vector<int> search(std::vector<float> target, float distanceTol)
// 	{
// 		std::vector<int> ids;
// 		searchBST(target, ids, distanceTol, root, 0);
// 		return ids;
// 	}
	

// };

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> myRansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    void myclusterTemp(int id, std::vector<int> &cluster, typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, KdTree* tree, std::vector<bool> points_processed);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> myeuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */