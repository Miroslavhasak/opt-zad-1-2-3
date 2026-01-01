#pragma once

#include <octomap\octomap.h>
#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <vector>

using Point = Eigen::Vector3d;


struct PointCloud {
    std::vector<Point> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx][dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

class Map {
    private:
    octomap::OcTree oc_tree;
    PointCloud cloud;

    using KDTreeType = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud>, 
        PointCloud, 3>;

    std::unique_ptr<KDTreeType> kd_tree;

    public:
    Map(double resolution,
                 double prob_hit   = 0.6,
                 double prob_miss  = 0.45,
                 double clamping_thres_min = 0.12,
                 double clamping_thres_max = 0.97)
        : oc_tree(resolution)  
        {
            oc_tree.setOccupancyThres(0.5);
            oc_tree.setProbHit(prob_hit);
            oc_tree.setProbMiss(prob_miss);
            oc_tree.setClampingThresMin(clamping_thres_min);
            oc_tree.setClampingThresMax(clamping_thres_max);
        }

    void addPoints(const Point & origin,const std::vector<Point>& points, bool rebuildKD = true, bool doPrune = true) {
        for (const auto& pt : points) {
            oc_tree.insertRay(octomap::point3d(origin.x(),origin.y(),origin.z()), octomap::point3d(pt.x(), pt.y(), pt.z()));
        }

        if (doPrune) {
            oc_tree.prune();  // merge uniform subtrees to save RAM
        }

        if (rebuildKD) {
            buildKD();
        }
    }


     void buildKD() {
       
        cloud.pts.clear();

        for (auto it = oc_tree.begin_leafs(), end = oc_tree.end_leafs(); it != end; ++it) {
            if (oc_tree.isNodeOccupied(*it)) {
                octomap::point3d p = it.getCoordinate();
                cloud.pts.emplace_back(p.x(), p.y(), p.z());
            }
        }

        kd_tree = std::make_unique<KDTreeType>(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kd_tree->buildIndex();

     }

    
    // Check if map is empty
    [[nodiscard]] bool isEmpty() const noexcept {
        return oc_tree.size() == 0;
    }

    int size()const noexcept {
        return oc_tree.getNumLeafNodes();
    }

    int numOccupiedPoints() const {
        int count = 0;
        for (auto it = oc_tree.begin_leafs(), end = oc_tree.end_leafs(); it != end; ++it) {
            if (oc_tree.isNodeOccupied(*it)) {
                ++count;
            }
        }
        return count;
    }


      // Find nearest occupied nodes (no probabilities)
    bool findCorrespondence(const Point& query, std::pair<Point, Point>& correspondence, double max_distance)
    {
    
          Point nearest_pt;
          if(nearest(query,nearest_pt))
          {     
           if ((nearest_pt - query).norm() < max_distance) {
                correspondence={query, nearest_pt};
                return true;    
           }
          }
       
        return false;
    }

    
    // Find nearest occupied nodes (no probabilities)
    std::vector<std::pair<Point, Point>> findCorrespondences(const std::vector<Point>& queries, double max_distance) 
    {
        std::vector<std::pair<Point, Point>> correspondences;
        correspondences.reserve(queries.size());

        for (size_t i = 0; i < queries.size(); ++i) {
              std::pair<Point, Point> correspondence={Point(0,0,0),Point(0,0,0)};
              if (findCorrespondence(queries[i], correspondence, max_distance))
                  correspondences.emplace_back(correspondence);
        }
        return correspondences;
    }

    bool findCorrespondenceWithProb(const Point& query, std::tuple<Point, Point, double>& correspondence, double max_distance) 
    {
          Point nearest_pt;
          double prob;

          if(nearestWithProb(query,nearest_pt,prob))
          {    
            if((nearest_pt - query).norm() < max_distance) {
                correspondence={query, nearest_pt, prob};
                return true;
          }
          }
        return false;
    }

    // Find nearest occupied nodes (with probabilities)
    std::vector<std::tuple<Point, Point, double>> findCorrespondencesWithProb(const std::vector<Point>& queries, double max_distance) 
    {
        std::vector<std::tuple<Point, Point, double>> correspondences;
        correspondences.reserve(queries.size());

        for (size_t i = 0; i < queries.size(); ++i) {
           std::tuple<Point, Point, double> correspondence={Point(0,0,0),Point(0,0,0),0.0};
           if(findCorrespondenceWithProb(queries[i], correspondence, max_distance))
            correspondences.emplace_back(correspondence);
        }
        return correspondences;
    }

    bool nearest(const Point& query, Point& pt)  {

        if (!kd_tree || cloud.pts.empty()) return false;

        double query_pt[3] = {query.x(), query.y(), query.z()};
        size_t num_results = 1;
        size_t ret_index;
        double out_dist_sqr;

        nanoflann::KNNResultSet<double> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr);        
        kd_tree->findNeighbors(resultSet, query_pt);

        pt = cloud.pts[ret_index];
        return true;

    }

    bool nearestWithProb(const Point& query, Point& pt, double& prob) {

        if (!kd_tree || cloud.pts.empty()) return false;

        double query_pt[3] = {query.x(), query.y(), query.z()};
        size_t num_results = 1;
        size_t ret_index;
        double out_dist_sqr;

        nanoflann::KNNResultSet<double> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr);      
        kd_tree->findNeighbors(resultSet, query_pt);
       
        pt = cloud.pts[ret_index];
        octomap::OcTreeNode* node = oc_tree.search(octomap::point3d(pt.x(), pt.y(), pt.z()));
        prob = node ? node->getOccupancy() : 0.0;
        return true;
    }

    // Get all occupied points
    std::vector<Point> getPoints() const {
        std::vector<Point> points;
        points.reserve(oc_tree.size());
        for (auto it = oc_tree.begin_leafs(), end = oc_tree.end_leafs(); it != end; ++it) {
            if (oc_tree.isNodeOccupied(*it)) {
                points.emplace_back(Point(it.getX(), it.getY(), it.getZ()));
            }
        }
        return points;
    }

    // Get occupied points with probabilities
    std::vector<std::pair<Point, double>> getPointsWithProb() const {
        std::vector<std::pair<Point, double>> points;
        points.reserve(oc_tree.size());

        for (auto it = oc_tree.begin_leafs(), end = oc_tree.end_leafs(); it != end; ++it) {
            if (oc_tree.isNodeOccupied(*it)) {
                points.emplace_back(Point(it.getX(), it.getY(), it.getZ()), it->getOccupancy());
            }
        }
        return points;
    }

    // Erase nodes below probability threshold
    void eraseBelowProb(double threshold) {
        std::vector<octomap::point3d> to_delete;

        for (auto it = oc_tree.begin_leafs(), end = oc_tree.end_leafs(); it != end; ++it) {
            if (it->getOccupancy() < threshold) {
                to_delete.emplace_back(it.getCoordinate());
            }
        }
        for (const auto& pt : to_delete) {
            oc_tree.deleteNode(pt);
        }
        buildKD();
    }


    void clear() {
        oc_tree.clear();
        buildKD();
    }
 

    // Iterator over occupied points
    class iterator {
        public:
        iterator(octomap::OcTree::leaf_iterator it,
                 octomap::OcTree::leaf_iterator end,
                 const octomap::OcTree* tree)
            : it_(it), end_(end), oc_tree(tree)
            {
                skipUnoccupied();
            }

        Point operator*() const {
            return Point(it_.getX(), it_.getY(), it_.getZ());
        }

        float occupancy() const {
            return  it_->getOccupancy() ;
        }

        iterator& operator++() {
            ++it_;
            skipUnoccupied();
            return *this;
        }

        bool operator!=(const iterator& other) const {
            return it_ != other.it_;
        }

        private:
        void skipUnoccupied() {
            while (it_ != end_ && !oc_tree->isNodeOccupied(*it_)) {
                ++it_;
            }
        }

        octomap::OcTree::leaf_iterator it_;
        octomap::OcTree::leaf_iterator end_;
        const octomap::OcTree* oc_tree;
    };

    iterator begin() const {
        return iterator(oc_tree.begin_leafs(), oc_tree.end_leafs(), &oc_tree);
    }

    iterator end() const {
        return iterator(oc_tree.end_leafs(), oc_tree.end_leafs(), &oc_tree);
    }


};