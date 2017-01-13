#ifndef KDTREE_HH
#define KDTREE_HH

// for drawing routine
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <vector>
#include <algorithm>
#include <glm/glm.hpp>
#include <limits>
#include "neighbordata.h"
typedef glm::vec3 Vector;

class KdNode {
public:
  bool leaf;
  union {
    struct {
      KdNode *left;
      KdNode *right;
      float separator;
      char dimension;
    } nodedata;
    struct {
      std::pair<int, Vector> const *begin;
      std::pair<int, Vector> const *end;
    } leafdata;
  };
  
  KdNode(std::vector<std::pair<int, Vector> >::const_iterator const &b, 
         std::vector<std::pair<int, Vector> >::const_iterator const &e):
    leaf(true)
  {
    leafdata.begin = &(*b);
    leafdata.end = leafdata.begin + (e-b);
  }

  KdNode(float sep, char dim, KdNode *c1, KdNode *c2):
    leaf(false)
  {
    nodedata.separator = sep;
    nodedata.dimension = dim;
    nodedata.left = c1;
    nodedata.right = c2;
  }
  
  ~KdNode() {
    if (!leaf) {
      delete nodedata.left;
      delete nodedata.right;
    }
  }
};

struct point_comp {
  char dim;
  point_comp(char d): dim(d) {};
  
  inline bool operator() (std::pair<int, Vector> const &x, std::pair<int, Vector> const &y) const {
    return x.second[dim] < y.second[dim];
  }
};

class KdTree {
  float query_radius, query_radius2;
  int bucket_size;
  Vector bbox_min, bbox_max;
  
  // storage for points
  std::vector<std::pair<int, Vector> > positions;
  
  // index tree
  KdNode *root;
  
  KdNode *splitCell(std::vector<std::pair<int, Vector> >::iterator begin, 
                    std::vector<std::pair<int, Vector> >::iterator end, 
                    Vector const &bmin, Vector const &bmax) {
    int n = end-begin;

    if (n <= bucket_size) {
      return new KdNode(begin, end);
    }
    
    char dim;
    // get longest axis
    Vector diff = bmax - bmin;
    if (diff.x > diff.y) {
      if (diff.x > diff.z) 
        dim = 0;
      else
        dim = 2;
    } else {
      if (diff.y > diff.z)
        dim = 1;
      else
        dim = 2;
    }

    std::vector<std::pair<int, Vector> >::iterator mid = begin + n/2;
    
    // partially sort the range we're operating on
    std::nth_element(begin, mid, end, point_comp(dim));
    
    float separator = mid->second[dim];
    
    Vector divmin = bmin, divmax = bmax;
    divmin[dim] = separator;
    divmax[dim] = separator;
    
    return new KdNode(separator, dim, 
                      splitCell(begin, mid, bmin, divmax), 
                      splitCell(mid, end, divmin, bmax));
  }
  
  // this works only in 2d
  inline bool containsSphere(Vector const &x, Vector const &bmin, Vector const &bmax) const {
    float minx = x.x - query_radius;
    float maxx = x.x + query_radius;
    float miny = x.y - query_radius;
    float maxy = x.y + query_radius;
    
    return 
      minx >= bmin.x && maxx < bmax.x &&
      miny >= bmin.y && maxy < bmax.y;    
  }

  // 2D only!
  inline bool intersectsSphere(Vector const &x, Vector const &bmin, Vector const &bmax) const {
    float dist = 0;    
    for(int i = 0; i < 2; ++i) {
      if(x[i] < bmin[i]) {
        float d = bmin[i] - x[i];
        dist += d * d;
      } else if(x[i] > bmax[i]) {
        float d = x[i] - bmax[i];
        dist += d * d;
      }
    }
    
    return dist <= query_radius2;
  }
  
  inline void gatherSamples(KdNode const *node, Vector const &bmin, Vector const &bmax, Vector const &x, 
                            std::vector<NeighborData> &nbs) const {
    if (node->leaf) {
      // check all in this bucket and add those within range
      for (std::pair<int, Vector> const * it = node->leafdata.begin; it != node->leafdata.end; ++it) {
        Vector dir = it->second - x;
        float d_squared = glm::dot(dir, dir);
        if (d_squared <= query_radius2) {
          NeighborData data;
          data.idx = it->first;
          data.d_squared = d_squared;
          data.d = sqrt(d_squared);
          if (data.d != 0) 
            data.d_normalized = dir/data.d;
          else
            data.d_normalized = Vector(0,0,0);
          nbs.push_back(data);
        }        
      }
    } else {
      // check if children intersect
      Vector divmin = bmin, divmax = bmax;
      divmin[node->nodedata.dimension] = node->nodedata.separator;
      divmax[node->nodedata.dimension] = node->nodedata.separator;
      if (intersectsSphere(x, bmin, divmax))
        gatherSamples(node->nodedata.left, bmin, divmax, x, nbs);
      if (intersectsSphere(x, divmin, bmax))
        gatherSamples(node->nodedata.right, divmin, bmax, x, nbs);
    }
  }
  
  void drawKdNode(KdNode const *node, Vector const &bmin, Vector const &bmax, Vector const &x, bool containing = true) const {
    if (node->leaf) {
      // mark all intersecting cells red
      if (intersectsSphere(x, bmin, bmax)) {
        glColor3f(1, 0, 0);
        glVertex2f(bmin.x, bmin.y);
        glVertex2f(bmin.x, bmax.y);
        
        glVertex2f(bmin.x, bmax.y);
        glVertex2f(bmax.x, bmax.y);
        
        glVertex2f(bmax.x, bmax.y);
        glVertex2f(bmax.x, bmin.y);
        
        glVertex2f(bmax.x, bmin.y);
        glVertex2f(bmin.x, bmin.y);
        glColor3f(0.6f, 0.6f, 0);
      }
      
      return;
    }
    
    if (node->nodedata.dimension == 0) {
      glVertex2f(node->nodedata.separator, bmin.y);
      glVertex2f(node->nodedata.separator, bmax.y);
    } else {
      glVertex2f(bmin.x, node->nodedata.separator);
      glVertex2f(bmax.x, node->nodedata.separator);
    }
    
    Vector divmin = bmin, divmax = bmax;
    divmin[node->nodedata.dimension] = node->nodedata.separator;
    divmax[node->nodedata.dimension] = node->nodedata.separator;  
    
    // mark smallest enclosing cell bright green
    bool containsleft = containsSphere(x, bmin, divmax);
    bool containsright = containsSphere(x, divmin, bmax);
    
    if (containing && !containsleft && !containsright) {
      glColor3f(0, 1, 0);
      glVertex2f(bmin.x, bmin.y);
      glVertex2f(bmin.x, bmax.y);
      
      glVertex2f(bmin.x, bmax.y);
      glVertex2f(bmax.x, bmax.y);
      
      glVertex2f(bmax.x, bmax.y);
      glVertex2f(bmax.x, bmin.y);
      
      glVertex2f(bmax.x, bmin.y);
      glVertex2f(bmin.x, bmin.y);
      glColor3f(0.6f, 0.6f, 0);      
    } 
    
    drawKdNode(node->nodedata.left, bmin, divmax, x, containsleft);
    drawKdNode(node->nodedata.right, divmin, bmax, x, containsright);
  };
  
public:
  KdTree(int bucketsize = 10): bucket_size(bucketsize), root(NULL) {
  }
  
  ~KdTree() {
    delete root;
  }
  
  inline KdNode const *getRoot() const {
    return root;
  }
  
  inline void getBBox(Vector &bmin, Vector &bmax) const {
    bmin = bbox_min;
    bmax = bbox_max;
  }

  inline float queryRadius() const {
    return query_radius;
  }
  
  // sets the query radius. This is cheap, and can be done before each query
  inline float queryRadius(float r) {
    query_radius = r;
    query_radius2 = r*r;
	return query_radius;
  }
    
  inline void clear() {
    if (root)
      delete root;
    root = NULL;
    positions.clear();
    bbox_min = Vector(0,0,0);
    bbox_max = Vector(-1,-1,-1);
  }

  inline void insert(int idx, Vector const &x) {
    positions.push_back(std::make_pair(idx, x));
  }
  
  // actually create the tree
  void init() {
    if (positions.empty())
      return;
    
    // compute bounding box
    bbox_min = bbox_max = positions[0].second;
    // only 2D!
    for (unsigned int i = 0; i < positions.size(); ++i) {
      if (positions[i].second.x < bbox_min.x) {
        bbox_min.x = positions[i].second.x;
      } else if (positions[i].second.x > bbox_max.x) {
        bbox_max.x = positions[i].second.x;
      }
        
      if (positions[i].second.y < bbox_min.y) {
        bbox_min.y = positions[i].second.y;
      } else if (positions[i].second.y > bbox_max.y) {
        bbox_max.y = positions[i].second.y;
      }
    }
    
    root = splitCell(positions.begin(), positions.end(), bbox_min, bbox_max);    
  }
  
  void neighbors(Vector const &x, std::vector<NeighborData> &nbs) const {
    nbs.clear();
    if (!root)
      return;

    // descend down tree to smallest cell that fully contains query 
    KdNode const *node = root;
    // node has bbox bmin-bmax
    Vector bmin = bbox_min, bmax = bbox_max;
    while (!node->leaf) {
      Vector divmin = bmin, divmax = bmax;
      divmin[node->nodedata.dimension] = node->nodedata.separator;
      divmax[node->nodedata.dimension] = node->nodedata.separator;
      if (containsSphere(x, bmin, divmax)) {
        node = node->nodedata.left;
        bmax = divmax;
      } else if (containsSphere(x, divmin, bmax)) {
        node = node->nodedata.right;
        bmin = divmin;
      } else
        break;
    }
  
    gatherSamples(node, bmin, bmax, x, nbs);
  }
  
  void draw(Vector const &p = Vector(std::numeric_limits<double>::infinity(), 
                                     std::numeric_limits<double>::infinity(), 
                                     std::numeric_limits<double>::infinity())) const {
    glLineWidth(2);
    
    bool containing = containsSphere(p, bbox_min, bbox_max);
    if (containing)
      glColor3f(0.6f, 0.6f, 0);
    else
      glColor3f(0, 1, 0);      
    glBegin(GL_LINES);
    
    // draw bbox
    glVertex2f(bbox_min.x, bbox_min.y);
    glVertex2f(bbox_min.x, bbox_max.y);
    
    glVertex2f(bbox_min.x, bbox_max.y);
    glVertex2f(bbox_max.x, bbox_max.y);
    
    glVertex2f(bbox_max.x, bbox_max.y);
    glVertex2f(bbox_max.x, bbox_min.y);
    
    glVertex2f(bbox_max.x, bbox_min.y);
    glVertex2f(bbox_min.x, bbox_min.y);
    glColor3f(0.6f, 0.6f, 0);
    
    // draw dividers
    drawKdNode(root, bbox_min, bbox_max, p, containing);
    glEnd();    
  }
};

#endif

