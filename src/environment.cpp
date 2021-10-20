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

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, float tol, int min_points, int max_points)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan = lidar->scan(); 
    auto position = lidar->position;

    renderPointCloud(viewer, scan, "test"); 
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> ppc;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ppc.SegmentPlane(scan, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));


    std::cout << "Hyper params: " << std::to_string(tol) << ", " << std::to_string(min_points) << ", " << std::to_string(max_points) << std::endl; 
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    // // 3 2 50    - Best params for pcl clustering
    // auto cloudClusters = ppc.Clustering(segmentCloud.first, tol, min_points, max_points);

    auto cloudClusters = ppc.OwnClustering(segmentCloud.first, tol, min_points, max_points);

    int name_id{0}; 
    for (auto cluster : cloudClusters){
        Box box = ppc.BoundingBox(cluster);
        std::string name_cloud = std::to_string(name_id) + "_cluster";
        renderBox(viewer, box, name_id);
        renderPointCloud(viewer, cluster, name_cloud, colors[name_id%colors.size()]);
        name_id++;
    }
}

/**
 * \brief load, render and process pcd data
 * \param [in] viewer a PCLVisualizer
 * \param [in] tol distance tolerance for clustering 
 * \param [in] min_points min points in cluster
 * \param [in] max_points min points in cluster
 */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, float tol, int min_points, int max_points) { 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Experiment with the ? values and find what works best
    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, .25 , Eigen::Vector4f (-10, -5.5, -2, 1), Eigen::Vector4f ( 15, 6, 2, 1));

    auto segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, .2);
    renderPointCloud(viewer, segmentCloud.first, "obs", Color(1, 1, 1));
    // renderPointCloud(viewer, segmentCloud.second, "plane", Color(0,1,0));
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    // auto cloudClusters = pointProcessorI->Clustering(segmentCloud.first, tol, min_points, max_points); 
    // 2.2 60 400 - Best params for ownclustering method
    auto cloudClusters = pointProcessorI->OwnClustering(segmentCloud.first, tol, min_points, max_points);

    int name_id{0}; 
    for (auto cluster : cloudClusters){
        Box box = pointProcessorI->BoundingBox(cluster);
        std::string name_cloud = std::to_string(name_id) + "_cluster";
        renderBox(viewer, box, name_id);
        name_id++;
    }
}

/**
 * \brief load, render and process pcd data
 * \param [in] viewer a PCLVisualizer
 * \param [in] pointProcessorI a ProcessPointCloud handler
 * \param [in] inputCloud  pcd 
 * \param [in] tol distance tolerance for clustering 
 * \param [in] min_points min points in cluster
 * \param [in] max_points min points in cluster
 */
void cityBlockLoop(pcl::visualization::PCLVisualizer::Ptr &viewer,
                   ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud,
                   float tol, int min_points, int max_points,
                   float tol1, int min_p1, int max_p1) {

  // Experiment with the ? values and find what works best
  auto filterCloud = pointProcessorI->FilterCloud(inputCloud, .18, Eigen::Vector4f(-10, -5.5, -2, 1), Eigen::Vector4f(15, 6.5, 2, 1));

  auto segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 120, .15);
  renderPointCloud(viewer, segmentCloud.first, "obs", Color(1, 1, 1));
  renderPointCloud(viewer, segmentCloud.second, "plane", Color(0, 1, 0));
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

  // auto cloudClusters = pointProcessorI->Clustering(segmentCloud.first, tol, min_points, max_points); 
  // 2.2 60 400 - Best params for ownclustering method
  // 2.27 30 2000
  // float tol_1{0.45};
  // int min_points_1{3};
  // int max_points_1{1000}; ./environment .35 5 500 3 3 80
  auto cloudClusters = pointProcessorI->OwnClustering(segmentCloud.first, tol, min_points, max_points);

  // Get mean coordinate point of box and cluster these new points
  auto clusters = pointProcessorI->ReCluster(cloudClusters); 
  auto subClusters = pointProcessorI->OwnClustering(clusters, tol1, min_p1, max_p1);

  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>> finalClusters; 

  // From the box clustering, get back the original points from cloudClusters
  // Here intensity is used to get back the points from the original clusters
  for (auto cluster: subClusters) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZI>());
    for (pcl::PointXYZI p : cluster->points){
      for (auto point :cloudClusters[(int)p.intensity]->points){
        temp_cluster->points.push_back(point);
      }
    }
    // Put them into a pointxyzi vector
    finalClusters.push_back(temp_cluster); 
  }

  // std::cout << "Final cluster count: " << finalClusters.size() << std::endl; 
  int name_id = 0;
  for (auto cluster : finalClusters) {
    Box box = pointProcessorI->BoundingBox(cluster);
    std::string name_cloud = std::to_string(name_id) + "_cluster";
    renderBox(viewer, box, name_id);
    name_id++;
  }

}

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

    // Arguments to test clustering
    double tol, tol1;
    int min_p, max_p, min_p1, max_p1;
    if (argc == 1) {
      tol = 0.35;
      min_p = 5;
      max_p = 500;
      tol1 = 3.5;
      min_p1 = 5;
      max_p1 = 100;
    } else {
      tol = atof(argv[1]);
      min_p = atoi(argv[2]);
      max_p = atoi(argv[3]);
      tol1 = atof(argv[4]);
      min_p1 = atoi(argv[5]);
      max_p1 = atoi(argv[6]);
    }

    // 2.2 20 400
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer, tol, min_p, max_p);
    // cityBlock(viewer, tol, min_p, max_p); 

    // Loop through pcd/data_1
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlockLoop(viewer, pointProcessorI, inputCloudI, tol, min_p, max_p, tol1, min_p1, max_p1);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}