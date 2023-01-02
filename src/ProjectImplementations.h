// Project implementations based on the course work

#include <vector>
#include <string>
#include <unordered_set>
#include <pcl/common/common.h>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insert(PointT point, int id)
	{
	    // Completed during the course according to helps and instructions
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);

	}

	void insertHelper(Node<PointT> *&node, uint depth, PointT point, int id)
	{
		if(node == NULL)
		{
			node = new Node<PointT>(point, id);
		}
		else
		{
			std::vector<float> p {point.x, point.y, point.z};
			std::vector<float> n {(*node).point.x, (*node).point.y, (*node).point.z};
			uint cd = depth % 3;
			if (p[cd] < n[cd])
				insertHelper((*node).left, depth+1, point, id);
			else
				insertHelper((*node).right, depth+1, point, id);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

	void searchHelper(PointT target, Node<PointT>* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			// Third diention added:
			if ((node->point.x>=(target.x-distanceTol))&&(node->point.x<=(target.x+distanceTol))&&
				(node->point.y>=(target.y-distanceTol))&&(node->point.y<=(target.y+distanceTol))&&
				(node->point.z>=(target.z-distanceTol))&&(node->point.z<=(target.z+distanceTol)))
			{
				float distance = std::sqrt((node->point.x-target.x)*(node->point.x-target.x)+
										   (node->point.y-target.y)*(node->point.y-target.y)+
										   (node->point.z-target.z)*(node->point.z-target.z));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}
			std::vector<float> t {target.x, target.y, target.z};
			std::vector<float> n {(*node).point.x, (*node).point.y, (*node).point.z};
			if ((t[depth % 3]-distanceTol) < n[depth % 3])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if ((t[depth % 3]+distanceTol) > n[depth % 3])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}
};

template<typename PointT>
struct EuclideanClustering
{
	typename pcl::PointCloud<PointT>::Ptr inputCloud;
	KdTree<PointT> tree;
	std::vector<bool> processed;

	EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud)
	: inputCloud(cloud), processed(std::vector<bool>(cloud->size(), false))
	{
		for (int i = 0; i < cloud->size(); i++)
			tree.insert(cloud->points[i],i);
	}

	// Euclidean Cluster - Implementation from the course
	void clusterHelper(int i, std::vector<int>& cluster, float distanceTol)
	{
		processed[i] = true;
		cluster.push_back(i);

		std::vector<int> nearest = tree.search(inputCloud->points[i], distanceTol);
		for (int id : nearest)
		{
			if (!processed[id])
				clusterHelper(id, cluster, distanceTol);
		}
	}

	std::vector<std::vector<int>> euclideanCluster(float distanceTol, int minSize, int maxSize)
	{
		std::vector<std::vector<int>> clusters;

		for (int i = 0; i < inputCloud->size(); i++)
		{
			if (processed[i])
				continue;

			std::vector<int> cluster;
			clusterHelper  (i, cluster, distanceTol);
			if ((cluster.size() > minSize) && (cluster.size()<maxSize))
				clusters.push_back(cluster);
		}

		return clusters;
	}
};

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	if (cloud->points.size() == 0) 
		return inliersResult;

	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line

		while (inliers.size() < 3)
			inliers.insert(rand()%cloud->points.size());

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto iter = inliers.begin();
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
		iter++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;
		iter++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		float ux, vx, wx, uy, vy, wy, uz, vz, wz;

		ux = x2 - x1;
		uy = y2 - y1;
		uz = z2 - z1;

		vx = x3 - x1;
		vy = y3 - y1;
		vz = z3 - z1;

		wx = uy*vz - vy*uz;
		wy = uz*vx - vz*ux;
		wz = ux*vy - vx*uy;

		float A, B, C, D;
		A = wx;
		B = wy;
		C = wz;
		D = - (A*x1 + B*y1 + C*z1);

		float x, y, z, d, sq;
		sq = std::sqrt(A*A+B*B+C*C);

		// Measure distance between every point and the plane
		// If distance is smaller than threshold count it as inlier
		for (int j = 0; j < cloud->points.size(); j++)
		{
			if (inliers.count(j))
				continue;

			x = cloud->points[j].x;
			y = cloud->points[j].y;
			z = cloud->points[j].z;
			d = fabs(A*x + B*y + C*z +D)/sq;

			if (d < distanceTol)
				inliers.insert(j);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
};

