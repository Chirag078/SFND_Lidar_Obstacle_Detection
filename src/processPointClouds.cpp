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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    	// Time segmentation process
    	auto startTime = std::chrono::steady_clock::now();
    
	//create segmentaion object
    	pcl::SACSegmentation<PointT> seg;
    	pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    
    	seg.setOptimizeCoefficients (true);
    	seg.setModelType(pcl::SACMODEL_PLANE);
    	seg.setMethodType(pcl::SAC_RANSAC);
    	seg.setMaxIterations(maxIterations);
    	seg.setDistanceThreshold(distanceThreshold);
    	seg.setInputCloud(cloud);
    	seg.segment(*inliers,*coefficients);

    	if (inliers->indices.size() == 0)
	{
        	std::cout << ("Could not estimate a planar model..") << std::endl;
    	}

    	auto endTime = std::chrono::steady_clock::now();
    	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    	return segResult;
}




template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_Using_RANSEC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
        std::unordered_set<int> inliersResult;
        srand(time(NULL));
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
        // For max iterations 
        while(maxIterations--)
        {
        // Randomly sample subset and fit line
                std::unordered_set<int> inliers;
                while(inliers.size()<3)
                        inliers.insert(rand()%(cloud->points.size()));

                float x1,x2,x3,y1,y2,y3,z1,z2,z3;

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

                std::vector<int> v1v2;
                v1v2.push_back((y2-y1)*(z3-z1) - (z2-z1)*(y3-y1));
                v1v2.push_back((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));
                v1v2.push_back((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));

                float a = v1v2[0];
                float b = v1v2[1];
                float c = v1v2[2];
                float d = -(v1v2[0]*x1 + v1v2[1]*y1 + v1v2[2]*z1);

                for(int index=0;index<cloud->points.size();index++)
                {
                        if(inliers.count(index)>0)
                                continue;

                        PointT point = cloud->points[index];

                        float x = point.x;
                        float y = point.y;
                        float z = point.z;

                        // Measure distance between every point and fitted line
                        float dis = fabs(a*x+b*y+c*z+d)/sqrt(a*a + b*b + c*c);

                        // If distance is smaller than threshold count it as inlier
                        if (dis <= distanceThreshold)
                        {
                                inliers.insert(index);
                        }
                }


                if(inliers.size()>inliersResult.size())
                {
                        inliersResult = inliers;
                }

        }

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    	for(int index = 0; index < cloud->points.size(); index++)
    	{
        	PointT point = cloud->points[index];
        	if(inliersResult.count(index))
            		cloudInliers->points.push_back(point);
        	else
            		cloudOutliers->points.push_back(point);
    	}

    	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
	
	return segResult;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	typename pcl::PointCloud<PointT>::Ptr  cloud_filtered (new pcl::PointCloud<PointT>);

 	pcl::VoxelGrid<PointT> vg;
  	vg.setInputCloud (cloud);
  	vg.setLeafSize (filterRes, filterRes, filterRes);
  	vg.filter (*cloud_filtered);

	typename pcl::PointCloud<PointT>::Ptr cloudROI (new pcl::PointCloud<PointT>);

    	pcl::CropBox<PointT> ROI(true);
	ROI.setMin(minPoint);
	ROI.setMax(maxPoint);
	ROI.setInputCloud(cloud_filtered);
	ROI.filter (*cloudROI);
	
	std::vector<int> indices;
    	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
	roof.setMax(Eigen::Vector4f (2.6,1.7,-.4,1));
	roof.setInputCloud(cloudROI);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
	for(int point : indices)
		inliers->indices.push_back(point);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloudROI);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloudROI);
		
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudROI;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT> ());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ());

	for(int index : inliers->indices)
		planeCloud->points.push_back(cloud->points[index]);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    	// Time clustering process
    	auto startTime = std::chrono::steady_clock::now();

    	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    	// kd tree
    	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    	tree->setInputCloud (cloud);

    	// euclidean cluster
    	std::vector<pcl::PointIndices> cluster_indices;
    	pcl::EuclideanClusterExtraction<PointT> ec;
    	ec.setClusterTolerance (clusterTolerance); 
    	ec.setMinClusterSize (minSize);
    	ec.setMaxClusterSize (maxSize);
    	ec.setSearchMethod (tree);
    	ec.setInputCloud (cloud);
    	ec.extract (cluster_indices);

    	// get cluster cloud
    	for (pcl::PointIndices get_indices: cluster_indices)
    	{
       		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        	for (int i: get_indices.indices)
            		cloud_cluster->points.push_back(cloud->points[i]); 

        	cloud_cluster->width = cloud_cluster->points.size ();
        	cloud_cluster->height = 1;
        	cloud_cluster->is_dense = true;

        	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        	clusters.push_back(cloud_cluster);
    	}

    	auto endTime = std::chrono::steady_clock::now();
    	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    	return clusters;
}




template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_Using_ED(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    	// Time clustering process
    	auto startTime = std::chrono::steady_clock::now();
  
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  	std::vector<pcl::PointIndices> clusterIndices;
  	std::vector<std::vector<int>> clusterResults;
  	std::vector<std::vector<float>> pointsAsVector;
  	KdTree* tree = new KdTree;

  	for(int i = 0; i < cloud->points.size(); i++)
  	{
    		PointT point = cloud->points[i];
    		std::vector<float> vectorPoint = {point.x, point.y, point.z};
    		pointsAsVector.push_back(vectorPoint);
    		tree->insert(vectorPoint, i);
  	}

  	clusterResults = euclideanCluster(pointsAsVector, tree, clusterTolerance);
  	for(auto indexSet : clusterResults)
  	{
    		if(indexSet.size() < minSize || indexSet.size() > maxSize)
    		{
        		continue; 
    		}

    		typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
    
    		for(auto index : indexSet)
    		{
      			cluster->points.push_back(cloud->points[index]);
    		}
  
    		cluster->width = cluster->points.size();
    		cluster->height = 1;
    		cluster->is_dense = true;
    		clusters.push_back(cluster);
  	}
    	auto endTime = std::chrono::steady_clock::now();
    	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
