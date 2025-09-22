/*
    FILE: dbscan.h
    ------------------
    helper class header for dbscan
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;
namespace onboardDetector{
    struct Point
    {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
        
        // Constructor
        Point() : x(0), y(0), z(0), clusterID(UNCLASSIFIED) {}
        Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_), clusterID(UNCLASSIFIED) {}
        
        // Comparison operator for unordered_set
        bool operator==(const Point& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    
    // Hash function for Point
    struct PointHash {
        std::size_t operator()(const Point& p) const {
            return std::hash<float>()(p.x) ^ (std::hash<float>()(p.y) << 1) ^ (std::hash<float>()(p.z) << 2);
        }
    };

    class DBSCAN {
    public:    
        DBSCAN(unsigned int minPts, float eps, vector<Point> points){
            m_minPoints = minPts;
            m_epsilon = eps;
            m_epsilonSquared = eps * eps;  // Precompute squared value
            m_epsilonSquaredDouble = static_cast<double>(m_epsilonSquared);  // Double precision version
            m_points = points;
            m_pointSize = points.size();
            m_cacheBuilt = false;
            m_neighborCache.resize(m_pointSize);
            
            // Initialize grid index
            m_cellSize = eps;  // Set grid cell size to epsilon
            m_invCellSize = 1.0f / eps;  // Precompute inverse for faster division
            buildSpatialIndex();
        }
        ~DBSCAN(){}

        int run();
        vector<int> calculateCluster(const Point& point);
        int expandCluster(const Point& point, int clusterID);
        inline double calculateDistanceSquared(const Point& pointCore, const Point& pointTarget) const;
        
        // Optimized neighbor search method
        vector<int> calculateClusterOptimized(const Point& point);
        
        // Build spatial index to improve neighbor search efficiency
        void buildSpatialIndex();
        vector<int> findNeighborsInGrid(const Point& point);
        
        // Point lookup optimization
        int findPointIndex(const Point& point) const;

        int getTotalPointSize() const {return m_pointSize;}
        int getMinimumClusterSize() const {return m_minPoints;}
        int getEpsilonSize() const {return m_epsilon;}
        
    public:
        vector<Point> m_points;
        
    private:    
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;
        float m_epsilonSquared;  // Precomputed epsilon squared value
        double m_epsilonSquaredDouble;  // Double precision version for distance comparison
        
        // Data structures for optimization
        std::vector<std::vector<int>> m_neighborCache;  // Neighbor cache
        bool m_cacheBuilt;  // Whether cache is built
        
        // Grid index related
        struct GridCell {
            std::vector<int> pointIndices;
        };
        std::vector<std::vector<std::vector<GridCell>>> m_grid;  // 3D grid
        float m_cellSize;  // Grid cell size
        float m_invCellSize;  // Precomputed 1/cellSize for faster division
        int m_gridSizeX, m_gridSizeY, m_gridSizeZ;  // Grid dimensions
        Point m_minBounds, m_maxBounds;  // Point cloud bounds
        
        // Precomputed grid bounds for faster access
        float m_gridMinX, m_gridMinY, m_gridMinZ;
        
        // Point lookup optimization
        std::unordered_map<Point, int, PointHash> m_pointToIndex;  // Fast point lookup
    };
}
#endif // DBSCAN_H
