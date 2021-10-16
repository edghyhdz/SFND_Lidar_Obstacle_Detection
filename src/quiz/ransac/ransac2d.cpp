/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

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

/** 
 * \brief Helper function to \b Ransac
 * \param[in] cloud a PCD
 * \param[in] distanceTol tolerated distance between points
 * \param[in] points a points vector to get point_1 and point_2
 */
std::unordered_set<int> get_inliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float distanceTol, std::vector<int> const points) {

  pcl::PointXYZ point_1 = cloud->at((points)[0]);
  pcl::PointXYZ point_2 = cloud->at((points)[1]);

  std::unordered_set<int> inliers; 

  for (int k = 0; k < cloud->size() - 1; k++){
	pcl::PointXYZ sample_point = cloud->at(k);
	float line = (point_1.y - point_2.y) * sample_point.x +
                 (point_2.x - point_1.x) * sample_point.y +
                 (point_1.x * point_2.y - point_2.x * point_1.y);

	// Measure distance between every point and fitted line
	float distance = abs(line) / sqrt(pow((point_1.y - point_2.y), 2.0) + pow((point_2.x - point_1.x), 2.0)); 
	
	// If distance is smaller than threshold count it as inlier
	if (distance <= distanceTol){
		inliers.insert(k);
	}
  }

  std::cout << "Inliers: " << std::to_string(inliers.size()) << std::endl; 

  return inliers; 
}

/** 
 * \brief Ransac implementation
 * \param[in] cloud a PCD
 * \param[in] maxIterations max iterations before stopping
 * \param[in] distanceTol tolerated distance between points
 */
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	// TODO: Fill in this function
	std::vector<std::vector<int>> sampled_points;
	int point_1, point_2;
	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_int_distribution<> distr(0, cloud->size() - 1);

	int max_value{0};
	int max_index;  

	// Randomly sample subset and fit line
	// For max iterations 
	for (int i = 0; i < maxIterations; i++) {
		point_1 = distr(eng); 
		point_2 = distr(eng); 

		// two points and third item is how many outliers are closer to line
		std::vector<int> temp_vector{point_1, point_2}; 
		std::unordered_set<int> inliers = get_inliers(cloud, distanceTol, temp_vector);
		sampled_points.push_back(temp_vector); 

		if (inliers.size() > max_value){
			max_value = inliers.size();
			max_index = sampled_points.size() - 1;  
			inliersResult = inliers; 
		}
	}

	std::cout << "Max value: " << std::to_string(max_value) << std::endl; 
	std::vector<int> result = sampled_points.at(max_index); 

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 1);

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
