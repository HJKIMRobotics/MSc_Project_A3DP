#ifndef NORMAL_DISTRIBUTION_OCTREE_H
#define NORMAL_DISTRIBUTION_OCTREE_H

#include <iostream>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigenvalues> 

#include <pcl/point_types.h>

namespace nd_map
{

typedef std::list<Eigen::RowVector3f> PointList;
static const float meanThreshold = 0.001;
static const float covThreshold = 0.001;


using namespace octomap;

class NDOcTreeNode : public OcTreeNode {
public:

    friend class NDOcTree;

    NDOcTreeNode(): 
        OcTreeNode(),
        ptList(),
        category("empty"),
        mu(Eigen::RowVector3f(0,0,0)),
        sigma(Eigen::Matrix3f::Zero())
        {}

    NDOcTreeNode(const NDOcTreeNode& rhs)
        : OcTreeNode(rhs),
          ptList(rhs.ptList),
          mu(rhs.mu),
          sigma(rhs.sigma),
          category(rhs.category)
    {}


    bool operator == ( const NDOcTreeNode& other ) const {
        return ( other.value == value );
    }

    bool operator != ( const NDOcTreeNode& other ) const {
        return ( other.value != value );
    }

    void copyData( const NDOcTreeNode &other ) {
        OcTreeNode::copyData(other);
        this->ptList = other.getPointList();

    }

    PointList getPointList() const {return ptList;}
    int listsize() const { return ptList.size();}
    void clear_ndPtList() { ptList.clear(); }
    void addPoint(float x, float y, float z ) 
    { 
        ptList.push_back(Eigen::RowVector3f(x, y, z)); 
    }
    void removePoint() {    
        ptList.pop_front(); 
    }

    Eigen::RowVector3f getMean() const {return mu; }
    void setMean(Eigen::RowVector3f m) { mu = m; }
    Eigen::RowVector3f calMean();
    bool compareMean(Eigen::RowVector3f m)
    {
        float d = (mu - m).norm();
        if (d > meanThreshold)    { return true; }
        return false;

    }

    float compareMean2(Eigen::RowVector3f m)
    {
        float d = (mu - m).norm();
        return d;

    }

    Eigen::Matrix3f getSigma() const {return sigma;}
    void setSigma(Eigen::Matrix3f s) { sigma = s; } 
    Eigen::Matrix3f calSigma();
    bool compareSigma(Eigen::Matrix3f s)
    {
        float d = (sigma - s).norm();
        if (d > covThreshold)    { return true; }
        return false;

    }

    float compareSigma2(Eigen::Matrix3f s)
    {
        float d = (sigma - s).norm();
        return d;

    }

    std::string classify(Eigen::Matrix3f matrix);
    void setcategory(std::string c) { category = c; }
    std::string getcategory() const {return category;}
    bool compareSigma(std::string c){ return category == c; }

    bool approx_equal(float a, float b) const
    {
        return std::abs(a - b) < 1.0e-3;
    }

    std::istream& readData( std::istream &ins ) {
        ins.read((char*) &value, sizeof(value)); // Occupancy.
        return ins;
    }

    std::ostream& writeData( std::ostream &out ) const {
        out.write((const char*) &value, sizeof(value)); // Occupancy.
        return out;
    }

public:

    PointList ptList;
    Eigen::RowVector3f mu;
    Eigen::Matrix3f sigma;
    std::string category;
};

class NDOcTree : public OccupancyOcTreeBase<NDOcTreeNode> {
public:
    NDOcTree( double resolution );
    NDOcTree* create() const { return new NDOcTree(resolution); }

    std::string getTreeType() const { return "NDOcTree"; }

    /**
     * Add the point to list as a Eigen::Vector3f data type. 
     * If list size == 10, do list.pop_front() and add new point
     * **/
    NDOcTreeNode* addPoint( const OcTreeKey &key, float x, float y, float z );

    NDOcTreeNode* addPoint( float x, float y, float z ) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return NULL;
        }
        return addPoint(key, x, y, z);
    }

    NDOcTreeNode* clearPoint( const OcTreeKey &key);

    NDOcTreeNode* clearPoint( float x, float y, float z ) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return NULL;
        }
        return clearPoint(key);
    }

    int getptlistsize( const OcTreeKey &key) ;

    int getptlistsize( float x, float y, float z) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return NULL;
        }

        return getptlistsize(key);
    } 

    // Calculate and set the mean & sigma
    NDOcTreeNode* updateNodeMeanSigma( const OcTreeKey &key) ;

    NDOcTreeNode* updateNodeMeanSigma( float x, float y, float z) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return NULL;
        }

        return updateNodeMeanSigma(key);
    }

    NDOcTreeNode* nodeClassify( const OcTreeKey &key) ;

    NDOcTreeNode* nodeClassify( float x, float y, float z) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return NULL;
        }

        return nodeClassify(key);
    }

    std::string nodeLabel( const OcTreeKey &key) ;

    std::string nodeLabel( float x, float y, float z) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return NULL;
        }

        return nodeLabel(key);
    }           

    NDOcTreeNode* nodeChangeDetect( const OcTreeKey &key) ;

    NDOcTreeNode* nodeChangeDetect( float x, float y, float z) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return NULL;
        }

        return nodeChangeDetect(key);
    }    




protected:
    class StaticMemberInitializer{
    public:
        StaticMemberInitializer() {
            NDOcTree *tree = new NDOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

        void ensureLinking() {}
    };

    static StaticMemberInitializer ndOcTreeMemberInit;
};

}


#endif