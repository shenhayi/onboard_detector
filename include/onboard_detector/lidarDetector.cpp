/*
    FILE: lidarDetector.cpp
    ------------------
    class function definitions for lidar-based obstacle detector
*/
#include <onboard_detector/lidarDetector.h>
namespace onboardDetector{
    lidarDetector::lidarDetector(){
        this->eps_ = 0.5;
        this->minPts_ = 10;
        this->cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void lidarDetector::setParams(double eps, int minPts, double groundHeight, double roofHeight){
        this->eps_ = eps;
        this->minPts_ = minPts;
        this->groundHeight_ = groundHeight;
        this->roofHeight_ = roofHeight;
    }

    void lidarDetector::getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        this->cloud_ = cloud;
    }

    void lidarDetector::lidarDBSCAN(){
        if(!cloud_ || cloud_->empty()){
            ROS_WARN("Empty pointcloud");
            return;
        }
        this->preFilter();
        std::vector<Point> points;
        for(size_t i=0; i<cloud_->size(); ++i){
            Point p;
            p.x = cloud_->points[i].x;
            p.y = cloud_->points[i].y;
            p.z = cloud_->points[i].z;
            p.clusterID = UNCLASSIFIED;
            points.push_back(p);
        }

        DBSCAN dbscan(minPts_, eps_, points);
        dbscan.run();


        // find how many clusters
        int clusterNum = 0;
        for (size_t i=0; i<dbscan.m_points.size(); ++i){
            onboardDetector::Point pDB = dbscan.m_points[i];
            if (pDB.clusterID > clusterNum){
                clusterNum = pDB.clusterID;
            }
        }

        this->clusters_.clear();
        this->clusters_.resize(clusterNum);
        this->bboxes_.clear();
        
        for(size_t i=0; i<dbscan.m_points.size(); ++i){
            if (dbscan.m_points[i].clusterID > 0){
                pcl::PointXYZ point;
                point.x = dbscan.m_points[i].x;
                point.y = dbscan.m_points[i].y;
                point.z = dbscan.m_points[i].z;
                this->clusters_[dbscan.m_points[i].clusterID-1].points->push_back(point);
                this->clusters_[dbscan.m_points[i].clusterID-1].cluster_id = dbscan.m_points[i].clusterID;
            }
        }

        for(auto& cluster : this->clusters_){
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster.points, centroid);
            cluster.centroid = centroid;
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cluster.points, minPt, maxPt);
            cluster.dimensions = Eigen::Vector3f(maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z); 

            onboardDetector::box3D bbox;
            bbox.x = centroid(0);
            bbox.y = centroid(1);
            bbox.z = centroid(2);
            bbox.x_width = maxPt.x - minPt.x;
            bbox.y_width = maxPt.y - minPt.y;
            bbox.z_width = maxPt.z - minPt.z;
            this->bboxes_.push_back(bbox);
        }
    }

    void lidarDetector::preFilter() {
        // Filter out points that are too close to the ground or too high
        auto& points = this->cloud_->points;
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [this](const pcl::PointXYZ& point) {
                    return point.z < this->groundHeight_ || point.z > this->roofHeight_;
                }),
            points.end());
    }

    
    std::vector<onboardDetector::Cluster>& lidarDetector::getClusters(){
        return this->clusters_;
    }

    std::vector<onboardDetector::box3D>& lidarDetector::getBBoxes(){
        return this->bboxes_;
    }
}