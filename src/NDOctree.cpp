#include "NDOctree.h"
 
namespace nd_map
{

    // node implementation -------------------------------------
    Eigen::RowVector3f NDOcTreeNode::calMean()
    {
        Eigen::RowVector3f mean(0,0,0);
        for(auto const& point : ptList)
        {
            mean += point;
            
        }
        mean = mean / ptList.size();
        return mean;
    }

    Eigen::Matrix3f NDOcTreeNode::calSigma()
    {
        Eigen::Matrix3f sig = Eigen::Matrix3f::Zero();
        
        for(auto const& point : ptList)
        {
            Eigen::RowVector3f d = point - mu;
            sig += d.transpose()*d;
        }
        sig /= (ptList.size()-1);
        return sig;

    }

    std::string NDOcTreeNode::classify(Eigen::Matrix3f matrix)
    {
        std::string shape;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(matrix, false);
        Eigen::Vector3f value = es.eigenvalues().real();

        if(approx_equal(value[0], value[1]) && approx_equal(value[2], value[1]))
        {
            shape = "sphere";
        }
        else if(approx_equal(value[2], value[1]) && value[2]>value[0])
        {
            shape = "plane";
        }
        else if(approx_equal(value[0], value[1]) && value[2]>value[0])
        {
            shape = "line";
        }
        else 
        {
            shape = "empty";
        }
        return shape;
    }

    // Tree implementation -------------------------------------
    NDOcTree::NDOcTree( double resolution )
        : OccupancyOcTreeBase<NDOcTreeNode>(resolution) 
    {
        ndOcTreeMemberInit.ensureLinking();
    }


    NDOcTreeNode* NDOcTree::addPoint( const OcTreeKey &key, float x, float y, float z  ) {
        NDOcTreeNode *node = search(key);
        if ( node ) {
            if (node -> listsize() > 9 )
            {
              node->removePoint();  
            }
            node->addPoint(x,y,z);
        }
        return node;
    }

    NDOcTreeNode* NDOcTree::clearPoint( const OcTreeKey &key) {
        NDOcTreeNode *node = search(key);
        if ( node ) {
            node->clear_ndPtList();
        }
        return node;
    }



    NDOcTreeNode* NDOcTree::updateNodeMeanSigma( const OcTreeKey &key) {
        NDOcTreeNode *node = search(key);
        if ( node ) {
            // cov computed with 3 or less points will always be singular
            // so calculate for node contains >5 points 
            if(node -> listsize() < 5)
            {
                return node;
            }
            else
            {
                node->setMean( node->calMean() );
                node->setSigma( node->calSigma() );
            }

        }
        return node;
    }

    int NDOcTree::getptlistsize( const OcTreeKey &key) {
        NDOcTreeNode *node = search(key);
        if ( node ) {
            return node->listsize();
        }
    }

    NDOcTreeNode* NDOcTree::nodeClassify( const OcTreeKey &key) {
        NDOcTreeNode *node = search(key);
        if ( node ) {
            // cov computed with 3 or less points will always be singular
            // so calculate for node contains >5 points 
            if(node -> listsize() < 5)
            {
                return node;
            }
            else
            {
                node->setcategory( node->classify(node->getSigma()) );
            }
        }
        return node;
    }

    std::string NDOcTree::nodeLabel( const OcTreeKey &key){
        NDOcTreeNode *node = search(key);
        if ( node ) {

            return node->getcategory();
        }
    }

    NDOcTreeNode* NDOcTree::nodeChangeDetect( const OcTreeKey &key) {
        NDOcTreeNode *node = search(key);
        if ( node ) {
            // Eigen::RowVector3f new_m =  node->calMean() ;
            // Eigen::Matrix3f new_cov = node->calSigma() ;
            // if(node->compareMean(new_m) || node -> compareSigma(new_cov))
            // {

            // }
        }
        return node;
    }


    NDOcTree::StaticMemberInitializer NDOcTree::ndOcTreeMemberInit;

}