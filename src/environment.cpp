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

    // 3 2 50    - Best params for pcl clustering
    auto cloudClusters = ppc.Clustering(segmentCloud.first, tol, min_points, max_points); 

    int name_id{0}; 
    for (auto cluster : cloudClusters){
        Box box = ppc.BoundingBox(cluster);
        std::string name_cloud = std::to_string(name_id) + "_cluster";
        renderBox(viewer, box, name_id);
        renderPointCloud(viewer, cluster, name_cloud, colors[name_id%colors.size()]);
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
    double tol;
    int min_p, max_p;
    if (argc == 1) {
      tol = 0.65;
      min_p = 6;
      max_p = 30;
    } else {
      tol = atof(argv[1]);
      min_p = atoi(argv[2]);
      max_p = atoi(argv[3]);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer, tol, min_p, max_p);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}