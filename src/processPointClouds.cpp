// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "cluster.h"

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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>()); 
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setNegative(true);
    extract.setIndices(inliers);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
	pcl::ExtractIndices<PointT> extract;
	for (int index:inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstCloud);
	
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	auto startTime =std::chrono::steady_clock::now();
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));
		auto itr = inliers.begin();
		float x1,y1,z1,x2,y2,z2,x3,y3,z3,i,j,k,a,b,c,d;
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		++itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		++itr;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		a = i;
		b = j;
		c = k;
		d = -(i*x1+j*y1+k*z1);

		for(int index = 0; index < cloud->points.size(); ++index)
		{
			if(inliers.count(index)>0)
				continue;
			float x4 = cloud->points[index].x;
			float y4 = cloud->points[index].y;
			float z4 = cloud->points[index].z;
			float distance = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a+b*b+c*c);
			if(distance <= distanceTol)
			{
				inliers.insert(index);
			}
		}
		if(inliers.size()>inliersResult.size())
			inliersResult = inliers;
	} 
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	// Randomly sample subset and fit line
	std::cout << "RANSAC3D took " << elapsedTime.count() << " milliseconds" << std::endl;
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
  	pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
	pcl::SACSegmentation<PointT> seg;
	
    std::unordered_set<int> inliersResult = Ransac3D(cloud, maxIterations, distanceThreshold);
    for(auto itr = inliersResult.begin(); itr!=inliersResult.end(); ++itr)
    {
        inliers->indices.push_back(*itr);
    }
	// seg.setOptimizeCoefficients (true);
	// seg.setModelType (pcl::SACMODEL_PLANE);
	// seg.setMethodType (pcl::SAC_RANSAC);
	// seg.setMaxIterations (maxIterations);
	// seg.setDistanceThreshold (distanceThreshold);
	// // Segment the largest planar component from the remaining cloud
	// seg.setInputCloud (cloud);
	// seg.segment (*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset.";
	}
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersCloud;
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> points;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    KdTree* tree = new KdTree;

    // tree->setInputCloud(cloud);
    for (int i=0; i<cloud->points.size(); i++) 
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point, i);
        points.push_back(point);
    }

    indices = euclideanCluster(points, tree, clusterTolerance);
    // std::cout<< "Found " << indices.size() << " clusters" << std::endl;
    for(std::vector<int> cluster : indices)
  	{
        if (cluster.size() < minSize || cluster.size() > maxSize)
            continue;
        typename pcl::PointCloud<PointT>::Ptr cloud1 (new pcl::PointCloud<PointT>);
  		for(int indice : cluster)
          {
            PointT point;
            point.x = points[indice][0];
            point.y = points[indice][1];
            point.z = points[indice][2];
            cloud1->points.push_back(point);
          }
        clustersCloud.push_back(cloud1);
  	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << indices.size() << " clusters" << std::endl;

    return clustersCloud;
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