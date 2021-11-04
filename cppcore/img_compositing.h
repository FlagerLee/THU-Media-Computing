//
// Created by flagerlee on 2021/11/1.
//

#ifndef MEDIAPROC_IMG_COMPOSITING_H
#define MEDIAPROC_IMG_COMPOSITING_H

#include "library.h"

namespace composite {

    typedef std::pair<int, int> coordinate;

    class QuadTreeNode;

    enum kErrorReason {
        SHOULD_SPLIT_ERR1,
        SHOULD_SPLIT_ERR2
    };

    struct Result {
        // result of CheckConstraint
        bool success;
        std::vector<std::pair<QuadTreeNode *, QuadTreeNode *>> err_list;
        std::vector<kErrorReason> err_reason;
    };

    class QuadTreeNode {
    public:
        // method
        QuadTreeNode();

        QuadTreeNode(coordinate center, int block_size);

        void Split();

        Result CheckConstraint(QuadTreeNode *);

        // data
        QuadTreeNode *lt, *lb, *rt, *rb; // left-top, left-bottom, right-top, right-bottom
        coordinate center;
        bool is_final;
        int block_size;
    };

    class Solver {
    public:
        // method
        Solver(cv::Mat);

        void CreateQuadTree(std::vector<coordinate>, cv::Mat);

        cv::Mat VisualizeQuadTree();

        // data
        QuadTreeNode *root;
        cv::Mat sTaTas, sTaTb;
        std::vector<double> x_delta, y_delta;
        int x_dim, y_dim;
    };
}

#endif //MEDIAPROC_IMG_COMPOSITING_H
