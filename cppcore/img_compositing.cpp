//
// Created by flagerlee on 2021/11/1.
//

#include "img_compositing.h"
#include <set>
#include <map>
#include <ctime>
#include <queue>
#include <random>

namespace composite {

    typedef enum {
        LT,
        LB,
        RT,
        RB
    } kPosition;

    QuadTreeNode::QuadTreeNode(point ltp, int blk_size) {
        this->ltp = ltp;
        this->block_size = blk_size;
        this->lt = this->lb = this->rt = this->rb = nullptr;
    }

    QuadTreeNode::~QuadTreeNode() {
        delete lt;
        delete lb;
        delete rt;
        delete rb;
    }

    Solver::Solver(const cv::Mat& img, const std::vector<cv::Point2i>& seam) {
        src_img = img.clone();
        CreateQuadTree(seam);
    }

    const int kNumQuadTreeSize = 16;
    constexpr int kQuadTreeSizes[kNumQuadTreeSize] = {
            (1 << 0) + 1,
            (1 << 1) + 1,
            (1 << 2) + 1,
            (1 << 3) + 1,
            (1 << 4) + 1,
            (1 << 5) + 1,
            (1 << 6) + 1,
            (1 << 7) + 1,
            (1 << 8) + 1,
            (1 << 9) + 1,
            (1 << 10) + 1,
            (1 << 11) + 1,
            (1 << 12) + 1,
            (1 << 13) + 1,
            (1 << 14) + 1,
            (1 << 15) + 1
    };

    void Solver::CreateQuadTree(const std::vector<cv::Point2i>& seam) {
        // build quadtree from leaf to root
        // create root node
        int max_size = std::max(this->src_img.rows, this->src_img.cols);
        assert(max_size > 1);
        int root_size = -1;
        for(int idx = 0; idx < kNumQuadTreeSize; idx ++)
            if(max_size > kQuadTreeSizes[idx] && max_size <= kQuadTreeSizes[idx + 1])
                root_size = kQuadTreeSizes[idx + 1] - 1;
        assert(root_size != -1);
        this->root = new QuadTreeNode(std::make_pair(0, 0), root_size);
        std::set<point> leaf_set;
        std::map<std::pair<point, int>, QuadTreeNode*> node_map;
        std::queue<QuadTreeNode*> parent, son;
        // creat leaf nodes
        for(auto & p : seam) {
            if(leaf_set.count(std::make_pair(p.x, p.y))) continue;
            leaf_set.insert(std::make_pair(p.x, p.y));
            if(p.x > 0) {
                leaf_set.insert(std::make_pair(p.x - 1, p.y));
                if(p.y > 0) {
                    point ltp = std::make_pair(p.x - 1, p.y - 1);
                    auto lt = new QuadTreeNode(ltp, 1);
                    parent.push(lt);
                    node_map.insert(std::make_pair(std::make_pair(ltp, 1), lt));
                    leaf_set.insert(ltp);
                    leaf_set.insert(std::make_pair(p.x, p.y - 1));
                }
                if(p.y < root_size) {
                    point ltp = std::make_pair(p.x - 1, p.y);
                    auto lb = new QuadTreeNode(ltp, 1);
                    parent.push(lb);
                    node_map.insert(std::make_pair(std::make_pair(ltp, 1), lb));
                    leaf_set.insert(std::make_pair(p.x - 1, p.y + 1));
                    leaf_set.insert(std::make_pair(p.x, p.y + 1));
                }
            }
            if(p.x < root_size) {
                leaf_set.insert(std::make_pair(p.x + 1, p.y));
                if(p.y > 0) {
                    point ltp = std::make_pair(p.x, p.y - 1);
                    auto rt = new QuadTreeNode(ltp, 1);
                    parent.push(rt);
                    node_map.insert(std::make_pair(std::make_pair(ltp, 1), rt));
                    leaf_set.insert(std::make_pair(p.x + 1, p.y - 1));
                }
                if(p.y < root_size) {
                    point ltp = std::make_pair(p.x, p.y);
                    auto rb = new QuadTreeNode(ltp, 1);
                    node_map.insert(std::make_pair(std::make_pair(ltp, 1), rb));
                    leaf_set.insert(std::make_pair(p.x + 1, p.y + 1));
                }
            }
        }
        // generate QuadTreeNode
        auto GenNode = [&](QuadTreeNode* snode, point ltp, int blk_size, kPosition pos)mutable {
            auto iter = node_map.find(std::make_pair(ltp, blk_size));
            if(iter != node_map.end()) {
                switch (pos) {
                    case LT:
                        iter->second->lt = snode;
                        break;
                    case LB:
                        iter->second->lb = snode;
                        break;
                    case RT:
                        iter->second->rt = snode;
                        break;
                    case RB:
                        iter->second->rb = snode;
                        break;
                }
                return;
            }
            auto node = new QuadTreeNode(ltp, blk_size);
            switch (pos) {
                case LT:
                    node->lt = snode;
                    break;
                case LB:
                    node->lb = snode;
                    break;
                case RT:
                    node->rt = snode;
                    break;
                case RB:
                    node->rb = snode;
                    break;
            }
            node_map.insert(std::make_pair(std::make_pair(ltp, blk_size), node));
            parent.push(node);
        };
        // extend four-connect region
        auto ExtendFCR = [&](point ltp, int blk_size)mutable {
            if(ltp.first >= blk_size) {
                // extend left-side region
                point new_ltp = std::make_pair(ltp.first - blk_size, ltp.second);
                auto iter = node_map.find(std::make_pair(new_ltp, blk_size));
                if(iter == node_map.end()) {
                    auto node = new QuadTreeNode(new_ltp, blk_size);
                    node_map.insert(std::make_pair(std::make_pair(new_ltp, blk_size), node));
                    parent.push(node);
                }
            }
            if(ltp.first <= root_size - 2 * blk_size) {
                // extend right-side region
                point new_ltp = std::make_pair(ltp.first + blk_size, ltp.second);
                auto iter = node_map.find(std::make_pair(new_ltp, blk_size));
                if(iter == node_map.end()) {
                    auto node = new QuadTreeNode(new_ltp, blk_size);
                    node_map.insert(std::make_pair(std::make_pair(new_ltp, blk_size), node));
                    parent.push(node);
                }
            }
            if(ltp.second >= blk_size) {
                // extend top region
                point new_ltp = std::make_pair(ltp.first, ltp.second - blk_size);
                auto iter = node_map.find(std::make_pair(new_ltp, blk_size));
                if(iter == node_map.end()) {
                    auto node = new QuadTreeNode(new_ltp, blk_size);
                    node_map.insert(std::make_pair(std::make_pair(new_ltp, blk_size), node));
                    parent.push(node);
                }
            }
            if(ltp.second <= root_size - 2 * blk_size) {
                // extend bottom region
                point new_ltp = std::make_pair(ltp.first, ltp.second + blk_size);
                auto iter = node_map.find(std::make_pair(new_ltp, blk_size));
                if(iter == node_map.end()) {
                    auto node = new QuadTreeNode(new_ltp, blk_size);
                    node_map.insert(std::make_pair(std::make_pair(new_ltp, blk_size), node));
                    parent.push(node);
                }
            }
        };
        for(int blk_size = 2; blk_size < root_size; blk_size *= 2) {
            std::swap(parent, son);
            assert(parent.empty());
            node_map.clear();
            while(!son.empty()) {
                QuadTreeNode* snode = son.front();
                son.pop();
                // complete snode whose son-pointer might be NULL
                if(snode->lt != nullptr || snode->lb != nullptr || snode->rt != nullptr || snode->rb != nullptr) {
                    if(snode->lt == nullptr) {
                        point ltp = std::make_pair(snode->ltp.first, snode->ltp.second);
                        snode->lt = new QuadTreeNode(ltp, snode->block_size / 2);
                    }
                    if(snode->lb == nullptr) {
                        point ltp = std::make_pair(snode->ltp.first, snode->ltp.second + snode->block_size / 2);
                        snode->lb = new QuadTreeNode(ltp, snode->block_size / 2);
                    }
                    if(snode->rt == nullptr) {
                        point ltp = std::make_pair(snode->ltp.first + snode->block_size / 2, snode->ltp.second);
                        snode->rt = new QuadTreeNode(ltp, snode->block_size / 2);
                    }
                    if(snode->rb == nullptr) {
                        point ltp = std::make_pair(snode->ltp.first + snode->block_size / 2, snode->ltp.second + snode->block_size / 2);
                        snode->rb = new QuadTreeNode(ltp, snode->block_size / 2);
                    }
                }
                // create bigger quadtree node
                if(snode->ltp.first % blk_size == 0) {
                    if(snode->ltp.second % blk_size == 0) {
                        // generate node and set this node as its lt
                        point ltp = snode->ltp;
                        GenNode(snode, ltp, blk_size, LT);
                        // extent fcr
                        ExtendFCR(ltp, blk_size);
                    }
                    else {
                        // generate node and set this node as its lb
                        point ltp = std::make_pair(snode->ltp.first, snode->ltp.second - blk_size / 2);
                        GenNode(snode, ltp, blk_size, LB);
                        // extent fcr
                        ExtendFCR(ltp, blk_size);
                    }
                }
                else {
                    if(snode->ltp.second % blk_size == 0) {
                        // generate node and set this node as its rt
                        point ltp = std::make_pair(snode->ltp.first - blk_size / 2, snode->ltp.second);
                        GenNode(snode, ltp, blk_size, RT);
                        // extent fcr
                        ExtendFCR(ltp, blk_size);
                    }
                    else {
                        // generate node and set this node as its rb
                        point ltp = std::make_pair(snode->ltp.first - blk_size / 2, snode->ltp.second - blk_size / 2);
                        GenNode(snode, ltp, blk_size, RB);
                        // extent fcr
                        ExtendFCR(ltp, blk_size);
                    }
                }
            }
        }
        std::swap(parent, son);
        assert(parent.empty());
        assert(son.size() <= 4);
        while(!son.empty()) {
            QuadTreeNode* snode = son.front();
            son.pop();
            const point& ltp = snode->ltp;
            if(ltp.first == 0) {
                if(ltp.second == 0) this->root->lt = snode;
                else this->root->lb = snode;
            }
            else {
                if(ltp.second == 0) this->root->rt = snode;
                else this->root->rb = snode;
            }
        }
    }

    template<typename T>
    void TraverseQuadTree(QuadTreeNode* node, point bound, T* data, std::function<void (QuadTreeNode*, T*)> func) {
        if(node == nullptr) return;
        if(node->ltp.first >= bound.first || node->ltp.second >= bound.second) return;
        func(node, data);
        if(node->lt != nullptr) TraverseQuadTree(node->lt, bound, data, func);
        if(node->lb != nullptr) TraverseQuadTree(node->lb, bound, data, func);
        if(node->rt != nullptr) TraverseQuadTree(node->rt, bound, data, func);
        if(node->rb != nullptr) TraverseQuadTree(node->rb, bound, data, func);
    }

    cv::Mat Solver::VisualizeQuadTree() {
        point bound = std::make_pair(src_img.rows, src_img.cols);
        class data {
        public:
            std::default_random_engine e;
            std::uniform_int_distribution<short> dis;
            cv::Mat img;

            data(int rows, int cols) {
                img = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));
                dis = std::uniform_int_distribution<short>(0, 255);
                std::random_device rd;
                e = std::default_random_engine(rd());
            }
        };

        data* D = new data(this->src_img.rows, this->src_img.cols);

        std::function<void (QuadTreeNode*, data*)> dyeing = [](QuadTreeNode* node, data* d) {
            if(node->isLeaf()) {
                auto color = (uchar)d->dis(d->e);
                if(color == 0) color = 1;
                const point& p = node->ltp;
                for(int i = 0; i < node->block_size; i ++) {
                    uchar *data = d->img.ptr<uchar>(i + p.first) + p.second;
                    if(*data != 0) printf("error");
                    memset(data, color, node->block_size * sizeof(uchar));
                }
            }
        };

        TraverseQuadTree<data>(this->root, std::make_pair(this->src_img.rows, this->src_img.cols) ,D, dyeing);

        cv::Mat image_color;
        cv::applyColorMap(D->img, image_color, cv::COLORMAP_JET);
        return image_color;
    }
}