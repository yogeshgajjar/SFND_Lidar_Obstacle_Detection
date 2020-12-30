/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    Color c{255,0,0};

    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar{cars, 0};  // created this LIDAR object
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = lidar->scan();

    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "point_clouds");
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *point_processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> pair_cloud = point_processor->SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer, pair_cloud.first, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, pair_cloud.second, "obstacle_road", Color(0,1,0));

    // std::vector<pcl::PointXYZ> clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor->Clustering(pair_cloud.first, 1.0, 3, 30);
    
    int clusterID{};
    std::vector<Color> colors{Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters) {
        std::cout << "cluster size";
        point_processor->numPoints(cluster);
        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterID);
        renderPointCloud(viewer, cluster, "obstacle_cloud"+std::to_string(clusterID), colors[clusterID]);
        ++clusterID;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, Eigen::Vector4f (-10.0, -5, -5.0, 1), Eigen::Vector4f (30.0, 5.0, -0.50, 1));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-20, -6, -3, 1), Eigen::Vector4f (30, 7, 2, 1));

    renderPointCloud(viewer,filterCloud,"filterCloud");

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> pair_cloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> pair_cloud = pointProcessorI->myRansacPlane(filterCloud, 100, 0.2);
    
    KdTree* tree = new KdTree;
  
    for (int i=0; i<pair_cloud.first->points.size(); i++) 
    	tree->insert(pair_cloud.first->points[i],i);

    // std::vector<pcl::PointXYZI> clusters;
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(pair_cloud.first, 0.5, 50, 500);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->myeuclideanCluster(pair_cloud.first, tree, 0.5, 50, 500);

    renderPointCloud(viewer, pair_cloud.first, "obstacle_cloud", Color(1,0,0));
    renderPointCloud(viewer, pair_cloud.second, "obstacle_road", Color(0,1,0));

    int clusterID{};
    std::vector<Color> colors{Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters) {
        std::cout << "cluster size";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacle_cloud"+std::to_string(clusterID), colors[clusterID]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterID);
        ++clusterID;
    }
}

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display City Block     -----
//     // ----------------------------------------------------

//     ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
//     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//     //   renderPointCloud(viewer,inputCloud,"inputCloud");

//     pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, Eigen::Vector4f (-10.0, -5, -5.0, 1), Eigen::Vector4f (30.0, 5.0, -0.50, 1));
//     renderPointCloud(viewer,filterCloud,"filterCloud");


//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> pair_cloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
//     renderPointCloud(viewer, pair_cloud.first, "obstacle_cloud", Color(1,0,0));
//     renderPointCloud(viewer, pair_cloud.second, "obstacle_road", Color(0,1,0));

//     // std::vector<pcl::PointXYZI> clusters;
//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(pair_cloud.first, 0.5, 50, 500);
    
//     int clusterID{};
//     std::vector<Color> colors{Color(1,0,0), Color(0,1,0), Color(0,0,1)};
//     for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters) {
//         std::cout << "cluster size";
//         pointProcessorI->numPoints(cluster);
//         Box box = pointProcessorI->BoundingBox(cluster);
//         renderBox(viewer, box, clusterID);
//         renderPointCloud(viewer, cluster, "obstacle_cloud"+std::to_string(clusterID), colors[clusterID]);
//         ++clusterID;
//     }

  
// }

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {   
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}