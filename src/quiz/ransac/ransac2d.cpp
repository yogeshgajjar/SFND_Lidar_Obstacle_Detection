/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <vector>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations-- >= 0) {
		unordered_set<int> inliers_set;

		while(inliers_set.size() < 2) 
			inliers_set.insert(rand() % (cloud->points.size()));

		float x1{}, x2{}, y1{}, y2{}, a{}, b{}, c{};

		auto itr = inliers_set.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		a = (y1-y2);
		b = (x2-x1);
		c = (x1*y2 - x2*y1);

		for(int i=0; i < cloud->points.size(); i++) {
			if(inliers_set.count(i))
				continue;

			float x0{}, y0{}, dist{};
			x0 = cloud->points[i].x;
			y0 = cloud->points[i].y;

			dist = fabs(((a*x0)+(b*y0)+c)) / sqrt((a*a) + (b*b));

			if(dist <= distanceTol)
				inliers_set.insert(i);
		}

		if(inliers_set.size() > inliersResult.size())
			inliersResult = inliers_set;

	}
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations-- >= 0) {
		unordered_set<int> inliers_set;

		while(inliers_set.size() < 3) 
			inliers_set.insert(rand() % (cloud->points.size()));

		float x1{}, x2{}, x3{}, y1{}, y2{}, y3{}, z1{}, z2{}, z3{}, a{}, b{}, c{}, d{};

		auto itr = inliers_set.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		vector<float> v1, v2, v3;
		// v1.push_back(x2-x1);
		// v1.push_back(y2-y1);
		// v1.push_back(z2-z1);
		// v2.push_back(x3-x1);
		// v2.push_back(y3-y1);
		// v2.push_back(x3-x1);
		
		
		// v3.push_back(((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1)));
		// v3.push_back(((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1)));
		// v3.push_back(((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1)));

		a = ((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1));
		b = ((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1));
		c = ((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1));
		d = -(a*x1 + b*y1 + c*z1);

		// a = v3[0];
		// b = v3[1];
		// c = v3[2];
		// d = -(a*x1 + b*y1 + c*z1);


		// a = (y1-y2);
		// b = (x2-x1);
		// c = (x1*y2 - x2*y1);

		for(int i=0; i < cloud->points.size(); i++) {
			if(inliers_set.count(i))
				continue;

			float x0{}, y0{}, z0{}, dist{};
			x0 = cloud->points[i].x;
			y0 = cloud->points[i].y;
			z0 = cloud->points[i].z;

			dist = fabs((a*x0)+(b*y0)+(c*z0)+d) / sqrt((a*a) + (b*b) + (c*c));

			if(dist <= distanceTol)
				inliers_set.insert(i);
		}

		if(inliers_set.size() > inliersResult.size())
			inliersResult = inliers_set;

	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
