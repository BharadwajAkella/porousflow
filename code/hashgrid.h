#ifndef HASHGRID_HH
#define HASHGRID_HH

// for drawing routine
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <vector>
#include "neighbordata.h"
#include <glm/glm.hpp>

#if defined(__linux__)
#include <ext/hash_map>
#include <ext/hash_set>
using namespace __gnu_cxx;
#elif defined(__APPLE__)
#include <ext/hash_map>
#include <ext/hash_set>
using namespace __gnu_cxx;
#else // windows, maybe?
#include <hash_map>
#include <hash_set>
using namespace stdext;
using stdext::hash_map;
using stdext::hash_set;
using stdext::hash_multimap;
#endif
#include <limits>

typedef glm::vec3 Vector;
struct hash_cell 
{
  // spatial cell index of particle
  int x, y, z;
  
  inline bool operator==(hash_cell const &c) const
  {
    return x == c.x && y == c.y && z == c.z;
  }
};

class hash_comp
{
public:
  enum {bucket_size = 4, min_buckets = 8};
  
  //static const size_t bucket_size;
  //static const size_t min_buckets;
  // (semi-)large primes
  const size_t p1, p2, p3;
  static size_t modulo;
  
  // p1, p2, p3 are the Mersenne Primes for 17, 19, 13
  hash_comp(): 
  p1(131071), p2(524287), p3(8191) {}
  
  // the hash function
  size_t operator()(hash_cell const &key) const
  {
    return (key.x*p1 + key.y*p2 + key.z*p3);
  }
  
  // total ordering
  bool operator() (hash_cell const &key1, 
                   hash_cell const &key2) const
  {
    if (key1.x < key2.x)
      return true;
    else if (key1.x > key2.x)
      return false;
    else if (key1.y < key2.y) // x1 == x2
      return true;
    else if (key1.y > key2.y)
      return false;
    else if (key1.z < key2.z) // y1 == y2
      return true;
    else 
      return false;
  }
  
  static size_t get_modulo() {return modulo; };
  static void set_modulo(size_t _modulo) { modulo = _modulo; };
};

class HashGrid {
  typedef hash_map<hash_cell, std::vector<std::pair<int, Vector> >, hash_comp> HashTable;
  HashTable hashtable;
  
  float query_radius;
  float query_radius2;
  float cellsize;
  
  // compute the cell we're in
  inline hash_cell cell(Vector const &v) const {
    hash_cell c;
    
    c.x = int(v.x/cellsize);
    c.y = int(v.y/cellsize);
    c.z = int(v.z/cellsize);
    
    if (v.x < 0)
      c.x--;
    if (v.y < 0)
      c.y--;
    if (v.z < 0)
      c.z--;
    
    return c;
  }
  
public:
  inline float cellSize() const {
    return cellsize;
  }
  
  inline float queryRadius() const {
    return query_radius;
  }
  
  inline float queryRadius(float r) {
    query_radius = r;
    query_radius2 = r*r;
    cellsize = 2*query_radius;
    
    // put all positions back into the hash
    //HashTable tmp = hashtable;
    
	for (HashTable::iterator it = hashtable.begin(); it != hashtable.end(); ++it) {
      std::vector<std::pair<int, Vector> > const &entries = it->second;
      for (std::vector<std::pair<int, Vector> >::const_iterator it2 = entries.begin(); 
           it2 != entries.end(); ++it2)
        insert(it2->first, it2->second);
    }
	hashtable.clear();
	return query_radius;
  }
  
  inline void insert(int idx, Vector const &x) {
    hashtable[cell(x)].push_back(std::make_pair(idx, x));
  }
  
  inline void clear() {
    hashtable.clear();
  }
  
  void init() {
    // nothing do to after insertion
  }
  
  // this works only in 2D
  void neighbors(Vector const &x, std::vector<NeighborData> &nbs) const {
    nbs.clear();
    
    // find the right neighbors to look at
    std::vector<hash_cell> cells(4, cell(x));
    hash_cell &c = cells[0];
    hash_cell &c01 = cells[1];
    hash_cell &c11 = cells[2];
    hash_cell &c10 = cells[3];
    if ((c.x+0.5) * cellsize < x.x) {
      c10.x += 1;
      c11.x += 1;
    } else {
      c10.x -= 1;
      c11.x -= 1;
    }
    if ((c.y+0.5) * cellsize < x.y) {
      c01.y += 1;
      c11.y += 1;
    } else {
      c01.y -= 1;
      c11.y -= 1;
    }
    
    for (int i = 0; i < 4; ++i) {
      HashTable::const_iterator it = hashtable.find(cells[i]);
      
      if (it == hashtable.end())
        continue;
            
      std::vector<std::pair<int, Vector> > const &cand = it->second;
      for (int j = 0; j < cand.size(); ++j) {
        // check all points in this cell
        Vector dir = cand[j].second - x;
        float d_squared = glm::dot(dir, glm::normalize(dir));
        
        if (d_squared <= query_radius2) {
          NeighborData data;
          data.idx = cand[j].first;
          data.d_squared = d_squared;
          data.d = sqrt(d_squared);
          if (data.d != 0) 
            data.d_normalized = dir/data.d;
          else
            data.d_normalized = Vector(0,0,0);
          nbs.push_back(data);
        }
      }
    }
  }
  
  void draw(Vector const &p = Vector(std::numeric_limits<double>::infinity(), 
                                     std::numeric_limits<double>::infinity(), 
                                     std::numeric_limits<double>::infinity())) const {
    glLineWidth(2);
    glColor3f(0, 0.5, 0);
    glBegin(GL_LINES);
    for (float x = -cellsize; x <= 1+cellsize; x += cellsize) {
      glVertex2f(x, -cellsize);
      glVertex2f(x, 1+cellsize);
    }
    for (float y = -cellsize; y <= 1+cellsize; y += cellsize) {
      glVertex2f(-cellsize, y);
      glVertex2f(1+cellsize, y);
    }
    glEnd();
    
    // mark background of cells that are actually stored yellow
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_QUADS);
    glColor4f(0.6, 0.6, 0, 0.4);
    
    for (HashTable::const_iterator it = hashtable.begin(); it != hashtable.end(); ++it) {
      hash_cell const &c = it->first;

      Vector bmin, bmax;
      bmin = Vector(c.x * cellsize, c.y * cellsize, c.z * cellsize);
      bmax = bmin + Vector(cellsize, cellsize, cellsize);

      glVertex2f(bmin.x, bmin.y);
      glVertex2f(bmin.x, bmax.y);
      glVertex2f(bmax.x, bmax.y);
      glVertex2f(bmax.x, bmin.y);
    }
    glEnd();

    // mark cells that are searched red
    std::vector<hash_cell> cells(4, cell(p));
    hash_cell &c = cells[0];
    hash_cell &c01 = cells[1];
    hash_cell &c11 = cells[2];
    hash_cell &c10 = cells[3];
    if ((c.x+0.5) * cellsize < p.x) {
      c10.x += 1;
      c11.x += 1;
    } else {
      c10.x -= 1;
      c11.x -= 1;
    }
    if ((c.y+0.5) * cellsize < p.y) {
      c01.y += 1;
      c11.y += 1;
    } else {
      c01.y -= 1;
      c11.y -= 1;
    }
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUADS);
    glColor3f(1, 0, 0);
    for (int i = 0; i < 4; ++i) {
      hash_cell const &c = cells[i];
      Vector bmin, bmax;
      bmin = Vector(c.x * cellsize, c.y * cellsize, c.z * cellsize);
      bmax = bmin + Vector(cellsize, cellsize, cellsize);
      
      glVertex2f(bmin.x, bmin.y);
      glVertex2f(bmin.x, bmax.y);
      glVertex2f(bmax.x, bmax.y);
      glVertex2f(bmax.x, bmin.y);      
    }
    
    glEnd();
  }
};

#endif

