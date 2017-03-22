/*
 * rrt_connect.h
 *
 *  Created on: Mar 10, 2017
 *      Author: charvak
 */

#ifndef RRT_CONNECT_H_
#define RRT_CONNECT_H_
#include <boost/bind.hpp>
#include <openrave/plugin.h>
#include <openrave/openrave.h>
#include <tr1/unordered_map>

using namespace std;
using namespace OpenRAVE;

/* Helper class for a Node. */
class RRTNode {
  typedef boost::shared_ptr<RRTNode> NodePtr;
private:
  static int _nodeCounter;
  /* ID of the node. */
  int _id;

  /* Configuration of the robot at the node. */
  vector<dReal> _configuration;

  /* Pointer to the parent node. */
  NodePtr _parentNode;

public:
  RRTNode(vector<dReal> configuration, NodePtr parent);

  RRTNode();

  const vector<dReal>& getConfiguration() const;

  void setConfiguration(const vector<dReal>& configuration);

  const NodePtr getParentNode() const;

  void setParentNode(const NodePtr parentNode);

  int getId() const;

//  dReal distanceFrom(NodePtr otherNode);

//  NodePtr newNodeTowards(NodePtr otherNode);

  /* Returns true if the robot is colliding in this node configuration. */
//  bool checkCollision(EnvironmentBasePtr penv);

  /* Equals override based on configuration equality. */
  bool operator==(const RRTNode& other);

  bool operator!=(const RRTNode& other);


};


/*--------------------------------------------------------------------------------------------------------*/


/* Tree of nodes explored. */
class NodeTree {
  typedef boost::shared_ptr<RRTNode> NodePtr;
private:
  /* Pointers to all the nodes explored. */
  vector<NodePtr> _nodes;
//  tr1::unordered_map<int, NodePtr> _nodes;

public:
  NodePtr getMostRecentNode() const;

  /* Adds a node pointer to the list of nodes explored. */
  bool addNode(NodePtr p_node);

  /* Deletes a node from the list. */
  void deleteNode(NodePtr node);

  /* Returns a node from the list of the given ID.
   * Throws 0 if no node with given ID exists. */
  NodePtr getNode(int id);

  /* Returns the path from the root to the node of given ID, including the node. */
  vector<NodePtr> getPathTo(int id);

  vector<NodePtr>& getAllNodes();

  size_t getSize();
  /* Returns the nearest neighbor to the given node. */
//  NodePtr getClosestTo(NodePtr node);

//  string extendTo(NodePtr node);
//
//  string connectTo(NodePtr node);

};

/* Returns a node from the node tree closest to the given node. */
//NodePtr GetClosestTo(RRTNode &node);

#endif /* RRT_CONNECT_H_ */
