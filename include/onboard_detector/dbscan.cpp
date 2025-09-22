/*
    FILE: dbscan.h
    ------------------
    helper class function definitions for dbscan
*/
#include <onboard_detector/dbscan.h>
#include <iostream>

namespace onboardDetector{
    int DBSCAN::run()
    {
        int clusterID = 1;
        
        // Pre-allocate memory for better performance
        vector<int> clusterSeeds;
        clusterSeeds.reserve(std::min(m_pointSize, 1024u));  // Reasonable upper bound
        
        // Use indices instead of iterators for better cache locality
        for(size_t i = 0; i < m_pointSize; ++i)
        {
            if ( m_points[i].clusterID == UNCLASSIFIED )
            {
                if ( expandCluster(m_points[i], clusterID) != FAILURE )
                {
                    clusterID += 1;
                }
            }
        }

        return 0;
    }

    int DBSCAN::expandCluster(const Point& point, int clusterID)
    {    
        vector<int> clusterSeeds = calculateClusterOptimized(point);

        if ( clusterSeeds.size() < m_minPoints )
        {
            // Use fast point lookup instead of linear search
            int pointIdx = findPointIndex(point);
            if (pointIdx >= 0) {
                m_points[pointIdx].clusterID = NOISE;
            }
            return FAILURE;
        }
        else
        {
            // Mark all seed points with current cluster ID
            for(int seedIdx : clusterSeeds)
            {
                m_points[seedIdx].clusterID = clusterID;
            }
            
            // Use unordered_set to avoid duplicates and enable fast lookup
            std::unordered_set<int> seedSet(clusterSeeds.begin(), clusterSeeds.end());
            
            // Process seed point queue
            for(size_t i = 0; i < clusterSeeds.size(); ++i)
            {
                vector<int> clusterNeighbors = calculateClusterOptimized(m_points[clusterSeeds[i]]);

                if ( clusterNeighbors.size() >= m_minPoints )
                {
                    for (int neighborIdx : clusterNeighbors)
                    {
                        if ( m_points[neighborIdx].clusterID == UNCLASSIFIED || m_points[neighborIdx].clusterID == NOISE )
                        {
                            if ( m_points[neighborIdx].clusterID == UNCLASSIFIED )
                            {
                                // Avoid duplicate additions
                                if (seedSet.find(neighborIdx) == seedSet.end())
                                {
                                    clusterSeeds.push_back(neighborIdx);
                                    seedSet.insert(neighborIdx);
                                }
                            }
                            m_points[neighborIdx].clusterID = clusterID;
                        }
                    }
                }
            }

            return SUCCESS;
        }
    }

    vector<int> DBSCAN::calculateCluster(const Point& point)
    {
        vector<int> clusterIndex;
        clusterIndex.reserve(std::min(m_pointSize, 256u));  // More reasonable initial size
        
        for (size_t i = 0; i < m_pointSize; ++i)
        {
            if ( calculateDistanceSquared(point, m_points[i]) <= m_epsilonSquaredDouble )
            {
                clusterIndex.push_back(i);
            }
        }
        return clusterIndex;
    }
    
    vector<int> DBSCAN::calculateClusterOptimized(const Point& point)
    {
        // Use grid-based neighbor search for better performance
        return findNeighborsInGrid(point);
    }

    inline double DBSCAN::calculateDistanceSquared(const Point& pointCore, const Point& pointTarget) const
    {
        // Calculate squared distance directly, avoid using pow function
        // Use float arithmetic for better performance
        float dx = pointCore.x - pointTarget.x;
        float dy = pointCore.y - pointTarget.y;
        float dz = pointCore.z - pointTarget.z;
        return static_cast<double>(dx*dx + dy*dy + dz*dz);
    }
    
    void DBSCAN::buildSpatialIndex()
    {
        if (m_pointSize == 0) return;
        
        // Find bounding box
        m_minBounds = m_maxBounds = m_points[0];
        for (size_t i = 1; i < m_pointSize; ++i) {
            if (m_points[i].x < m_minBounds.x) m_minBounds.x = m_points[i].x;
            if (m_points[i].y < m_minBounds.y) m_minBounds.y = m_points[i].y;
            if (m_points[i].z < m_minBounds.z) m_minBounds.z = m_points[i].z;
            if (m_points[i].x > m_maxBounds.x) m_maxBounds.x = m_points[i].x;
            if (m_points[i].y > m_maxBounds.y) m_maxBounds.y = m_points[i].y;
            if (m_points[i].z > m_maxBounds.z) m_maxBounds.z = m_points[i].z;
        }
        
        // Calculate grid dimensions using precomputed inverse
        m_gridSizeX = static_cast<int>((m_maxBounds.x - m_minBounds.x) * m_invCellSize) + 1;
        m_gridSizeY = static_cast<int>((m_maxBounds.y - m_minBounds.y) * m_invCellSize) + 1;
        m_gridSizeZ = static_cast<int>((m_maxBounds.z - m_minBounds.z) * m_invCellSize) + 1;
        
        // Precompute grid bounds for faster access
        m_gridMinX = m_minBounds.x;
        m_gridMinY = m_minBounds.y;
        m_gridMinZ = m_minBounds.z;
        
        // Initialize grid
        m_grid.resize(m_gridSizeX);
        for (int x = 0; x < m_gridSizeX; ++x) {
            m_grid[x].resize(m_gridSizeY);
            for (int y = 0; y < m_gridSizeY; ++y) {
                m_grid[x][y].resize(m_gridSizeZ);
            }
        }
        
        // Assign points to grid cells and build point lookup map
        for (size_t i = 0; i < m_pointSize; ++i) {
            // Use precomputed inverse and bounds for faster division
            int gridX = static_cast<int>((m_points[i].x - m_gridMinX) * m_invCellSize);
            int gridY = static_cast<int>((m_points[i].y - m_gridMinY) * m_invCellSize);
            int gridZ = static_cast<int>((m_points[i].z - m_gridMinZ) * m_invCellSize);
            
            // Clamp to valid range
            gridX = std::max(0, std::min(gridX, m_gridSizeX - 1));
            gridY = std::max(0, std::min(gridY, m_gridSizeY - 1));
            gridZ = std::max(0, std::min(gridZ, m_gridSizeZ - 1));
            
            m_grid[gridX][gridY][gridZ].pointIndices.push_back(i);
            
            // Build point lookup map for fast point index retrieval
            m_pointToIndex[m_points[i]] = i;
        }
    }
    
    vector<int> DBSCAN::findNeighborsInGrid(const Point& point)
    {
        vector<int> neighbors;
        neighbors.reserve(64);  // More reasonable initial size based on typical neighborhood
        
        // Calculate grid coordinates using precomputed inverse and bounds
        int gridX = static_cast<int>((point.x - m_gridMinX) * m_invCellSize);
        int gridY = static_cast<int>((point.y - m_gridMinY) * m_invCellSize);
        int gridZ = static_cast<int>((point.z - m_gridMinZ) * m_invCellSize);
        
        // Search in 3x3x3 neighborhood around the point's grid cell
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    int checkX = gridX + dx;
                    int checkY = gridY + dy;
                    int checkZ = gridZ + dz;
                    
                    // Check bounds
                    if (checkX < 0 || checkX >= m_gridSizeX ||
                        checkY < 0 || checkY >= m_gridSizeY ||
                        checkZ < 0 || checkZ >= m_gridSizeZ) {
                        continue;
                    }
                    
                    // Check all points in this grid cell
                    const auto& cellPoints = m_grid[checkX][checkY][checkZ].pointIndices;
                    for (int pointIdx : cellPoints) {
                        // Early termination: if we already have enough neighbors, stop checking
                        if (neighbors.size() >= m_minPoints * 2) {
                            break;
                        }
                        if (calculateDistanceSquared(point, m_points[pointIdx]) <= m_epsilonSquaredDouble) {
                            neighbors.push_back(pointIdx);
                        }
                    }
                }
            }
        }
        
        return neighbors;
    }
    
    int DBSCAN::findPointIndex(const Point& point) const
    {
        auto it = m_pointToIndex.find(point);
        return (it != m_pointToIndex.end()) ? it->second : -1;
    }
}


