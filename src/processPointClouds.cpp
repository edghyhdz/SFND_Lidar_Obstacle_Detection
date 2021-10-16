// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(

  pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());
  // Copy inliers point cloud as plane
  for (int index : inliers->indices) {
      plane_cloud->points.push_back(cloud->points[index]);
  }
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers so that we can get obstacles point cloud
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
  return segResult;
}

/**
 * \brief Segmentation using own \b Ransac3D implementation
 * \param[in] cloud a PointT pcl::PointCloud
 * \param[in] maxIterations max iterations before stopping
 * \param[in] distanceThreshold x or y split index
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  auto startTime = std::chrono::steady_clock::now();
  auto inliers = Ransac3D(cloud, maxIterations, distanceThreshold); 
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "Ransac3D segmentation took " << elapsedTime.count() << " microseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

/**
 * \brief Ransac3D implementation as per quiz ransac's solution
 * \param[in] cloud a PointT pcl::PointCloud
 * \param[in] maxIterations max iterations before stopping
 * \param[in] distanceTol distance threshold
 */
template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud,
                                int maxIterations, float distanceTol){

  pcl::PointIndices::Ptr inliersR(new pcl::PointIndices());
  std::unordered_set<int> inlierResults;
  srand(time(NULL));
  while (maxIterations--) {
    std::unordered_set<int> inliers;
    while (inliers.size() < 3)
      inliers.insert(rand() % (cloud->points.size()));

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    auto itr = inliers.begin();
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

    // Cross product from v1 x v2
    float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

    for (int index = 0; index < cloud->points.size(); index++) {

      if (inliers.count(index) > 0)
        continue;

      PointT point = cloud->points[index];
      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;

      i *(x4 - x1) + j *(y4 - y1) + k *(z4 - z1);
      float d = -(i * x1 + j * y1 + k * z1);

      float dist =
          fabs(i * x4 + j * y4 + k * z4 + d) / sqrt(i * i + j * j + k * k);

      if (dist <= distanceTol) {
        inliers.insert(index);
      }
    }
	if (inliers.size() > inlierResults.size()){
		inlierResults = inliers; 
	}
  }

  for (int idx : inlierResults){
    inliersR->indices.push_back(idx);
  }

  return inliersR; 
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    for (int i = 0; i < cluster_indices.size() - 1; i++) {
    typename pcl::PointCloud<PointT>::Ptr temp_cluster(new pcl::PointCloud<PointT>());

      for (int idx : cluster_indices[i].indices) {
        auto temp = cloud->points[idx]; 
        temp_cluster->points.push_back(temp); 
      }
      temp_cluster->width = temp_cluster->points.size(); 
      temp_cluster->height = 1; 
      temp_cluster->is_dense = true;

      clusters.push_back(temp_cluster);  
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " microseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}