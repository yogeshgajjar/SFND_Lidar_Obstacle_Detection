
#ifndef CLUSTER_H
#define CLUSTER_H

#include <iostream>
#include <vector>
// #include "kdtree.h"


inline void clusterTemp(int id, std::vector<int> &cluster, const std::vector<std::vector<float>>& points, float distanceTol, KdTree* tree, std::vector<bool> points_processed) {

	points_processed[id] = true;

	cluster.push_back(id);
	std::vector<int> clust_id = tree->search(points[id], distanceTol);

	for(const auto &in:clust_id) {
		if(!points_processed[in]) {
			clusterTemp(in, cluster, points, distanceTol, tree, points_processed);
		}
	}
}

inline std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> points_processed(points.size(), false);

	for(int i=0; i < points.size(); i++) {
		if(!points_processed[i]) {
            std::vector<int> cluster;
            clusterTemp(i, cluster, points, distanceTol, tree, points_processed);
            clusters.push_back(cluster);
        }
	}
 
	return clusters;

}

#endif