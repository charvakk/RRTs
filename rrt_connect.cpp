#include "rrt_connect.h"
#include <algorithm>
#include <limits>
#include <math.h>
#include <openrave/utils.h>
#include <openrave/planningutils.h>

#define COMPUTATION_TIME 35000
//#define STEP_SIZE 0.18
//#define STEP_SIZE 0.15
#define STEP_SIZE 0.3
#define GOAL_BIAS 5
#define INPUT_SIZE 52
#define CLOSE_ENOUGH 0.2

typedef boost::shared_ptr<RRTNode> NodePtr;
typedef boost::shared_ptr<NodeTree> TreePtr;

/* RRTNode method implementations.*/
int RRTNode::_nodeCounter = 0;

RRTNode::RRTNode(vector<dReal> configuration, NodePtr parent){
  _id = ++_nodeCounter;
  _configuration = configuration;
  _parentNode = parent;
}

RRTNode::RRTNode(){
  _id = ++_nodeCounter;
  _parentNode = nullptr;
}

const vector<dReal>& RRTNode::getConfiguration() const{
  return _configuration;
}

void RRTNode::setConfiguration(const vector<dReal>& configuration){
  _configuration = configuration;
}

int RRTNode::getId() const{
  return _id;
}

const NodePtr RRTNode::getParentNode() const{
  return _parentNode;
}

void RRTNode::setParentNode(const NodePtr parentNode){
  _parentNode = parentNode;
}

/*dReal RRTNode::distanceFrom(NodePtr otherNode){
  dReal distanceSquared = 0;
  vector<dReal> otherNodeConfig = otherNode->getConfiguration();
  for(size_t i = 0; i < _configuration.size(); ++i){
    if(i == 3 || i == 6)
      distanceSquared += pow(_configuration[i]/10000 - otherNodeConfig[i]/10000, 2);
    else
      distanceSquared += pow(_configuration[i] - otherNodeConfig[i], 2);
  }
  return sqrt(distanceSquared);
}*/

/*dReal RRTNode::distanceFrom(NodePtr otherNode){
  dReal distanceSquared = 0;
  vector<dReal> otherNodeConfig = otherNode->getConfiguration();
  for(size_t i = 0; i < _configuration.size(); ++i){
    distanceSquared += pow(_configuration[i] - otherNodeConfig[i], 2);
  }
  return sqrt(distanceSquared);
}*/


/*NodePtr RRTNode::newNodeTowards(NodePtr otherNode){
  vector<dReal> vectorBetweenNodes;
  for(size_t i=0; i < _configuration.size(); ++i)
    vectorBetweenNodes.push_back(otherNode->getConfiguration()[i] - _configuration[i]);
  dReal magnitude = distanceFrom(otherNode);

  NodePtr newNode(new RRTNode());
  vector<dReal> newConfiguration;
  for(size_t i=0; i < _configuration.size(); ++i)
    newConfiguration.push_back(_configuration[i] + (vectorBetweenNodes[i]*STEP_SIZE/magnitude));

  newNode->setConfiguration(newConfiguration);
  return newNode;
}*/

/*bool RRTNode::operator==(const RRTNode& other){
  bool result = true;
  for(size_t i = 0; i < _configuration.size(); ++i){
    result = result && (_configuration[i] == other.getConfiguration()[i]);
  }
  return result;
}*/

bool RRTNode::operator==(const RRTNode& other){
  bool result = true;
  for(size_t i = 0; i < _configuration.size(); ++i){
    result = result && ((_configuration[i] - other.getConfiguration()[i]) < CLOSE_ENOUGH);
  }
  return result;
}

bool RRTNode::operator!=(const RRTNode& other){
  return !(*this == other);
}

/*---------------------------------------------------------------------------------------------------------------------------*/

/* NodeTree method implementations.*/
NodePtr NodeTree::getMostRecentNode() const {
  return _nodes.back();
}

bool NodeTree::addNode(NodePtr p_node){
  try{
    _nodes.push_back(p_node);
    return true;
  }catch(exception &e){
    cout << e.what() << endl;
    return false;
  }
}

void NodeTree::deleteNode(NodePtr node){
  _nodes.erase(std::remove(_nodes.begin(), _nodes.end(), node), _nodes.end());
  }

NodePtr NodeTree::getNode(int id){
  for(NodePtr p_node : _nodes){
    if(p_node->getId() == id)
      return p_node;
  }
  throw 0;
}

vector<NodePtr> NodeTree::getPathTo(int id){
  // Find the node
  NodePtr final;
  for(NodePtr p_node : _nodes){
    if(p_node->getId() == id)
      final = p_node;
  }

  // Find the path from the parents
  vector<NodePtr> path;
  path.push_back(final);
  while(final->getParentNode() != nullptr){
    final = final->getParentNode();
    path.push_back(final);
  }
//  std::reverse(path.begin(), path.end());   //Reverse the path since its been added end to start.
  return path;
}

vector<NodePtr> NodeTree::getAllNodes(){
  return _nodes;
}

size_t NodeTree::getSize(){
  return _nodes.size();
}

/*NodePtr NodeTree::getClosestTo(NodePtr node){
  dReal lowestDistance = numeric_limits<double>::max();
  NodePtr closestNode;
  for(NodePtr n : _nodes){
    dReal distance = n->distanceFrom(node);
    if(distance <= lowestDistance){
      lowestDistance = distance;
      closestNode = n;
    }
  }
  return closestNode;
}*/

/*--------------------------------------------------------------------------------------------------------------------*/

class rrt_module : public ModuleBase {
public:
  rrt_module(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
    _penv = penv;
    __description = "Implementation of RRT-Connect for RBE 550.";
    RegisterCommand("birrt",boost::bind(&rrt_module::BiRRT,this,_1,_2),
                    "Plans and executes path to given goal configuration using Bidirectional RRTs.");
    RegisterCommand("rrtconnect",boost::bind(&rrt_module::RRTConnect,this,_1,_2),
                    "Plans and executes path to given goal configuration using RRTConnect.");
  }
  virtual ~rrt_module() {}

  bool BiRRT(ostream& sout, istream& sin){

    // Initialize private members from the input
    Init(sout, sin);

    // Initialize two trees with the start and goal nodes at the respective roots.
    TreePtr treeA(new NodeTree());
    treeA->addNode(startNode);

    TreePtr treeB(new NodeTree());
    treeB->addNode(goalNode);


    for(int k = 0; k < COMPUTATION_TIME; ++k){
      NodePtr randomNode = CreateRandomNode();

      if(Extend(treeA, randomNode) != "Trapped"){
        if(Connect(treeB, treeA->getMostRecentNode()) == "Reached"){
          vector<NodePtr> pathA = treeA->getPathTo(treeA->getMostRecentNode()->getId());
//          vector<NodePtr> pathB = treeB->getPathTo(treeB->getMostRecentNode()->getId());
//          vector<NodePtr> fullPath;

//          vector<NodePtr> pathA = treeB->getPathTo(goalNode->getId());

//          std::reverse(pathA.begin(), pathA.end());
//          fullPath.reserve(pathA.size() + pathB.size()); // preallocate memory
//          fullPath.insert(fullPath.end(), pathA.begin(), pathA.end()); //TODO check if this can be done with end, begin to avoid reversing
//          fullPath.insert(fullPath.end(), pathB.begin(), pathB.end());

//          vector<NodePtr> pathA = GetPath(goalNode);

          vector< vector<dReal> > configPath;
//          configPath.reserve(fullPath.size());
//          for(NodePtr pnode : fullPath)
//            configPath.push_back(pnode->getConfiguration());

          configPath.reserve(pathA.size());
          for(NodePtr pnode : pathA)
            configPath.push_back(pnode->getConfiguration());

          std::reverse(configPath.begin(), configPath.end());
          cout << "Found a path!!!" << endl;
          cout << "Executing the path." << endl;

          cout << "limits " << endl;
          for(size_t i = 0; i < _activeLowerLimits.size(); ++i){
            cout << _activeLowerLimits[i]  << " ";
          }
          cout << endl;
          for(size_t i = 0; i < _activeLowerLimits.size(); ++i){
            cout << _activeUpperLimits[i] << " ";
          }
          cout << endl;

          cout << "path :" << endl;
          for(auto c : configPath){
            for(size_t i = 0; i < c.size(); ++i){
              cout << c[i] << " ";
            }
            cout << endl;
          }
          cout << "Number of nodes explored :" << endl;
          cout << treeA->getSize() + treeB->getSize() << endl;
          ExecuteTrajectory(configPath);
          return true;
        }
      }
      swap(treeA, treeB);
      if(k % 5000 == 0)
        cout << k << ". Searching..." << endl;
    }
    cout << "Time up :(" << endl;
    return false;
  }

  bool RRTConnect(ostream& sout, istream& sin){
    // Initialize private members from the input
    Init(sout, sin);

    // Initialize the tree with the start node at the root.
    TreePtr tree(new NodeTree());
    tree->addNode(startNode);

    for(int k = 0; k < COMPUTATION_TIME; ++k){
      NodePtr randomNode = CreateRandomNodeWithBias();

      if(Connect(tree, randomNode) == "GoalReached"){
        cout << "5 HERE" << endl; //TODO
        vector<NodePtr> path = tree->getPathTo(goalNode->getId());
//        std::reverse(path.begin(), path.end());

        vector< vector<dReal> > configPath;
        configPath.reserve(path.size());
        for(NodePtr pnode : path)
          configPath.push_back(pnode->getConfiguration());

//        std::reverse(configPath.begin(), configPath.end());

        cout << "Found a path!!!" << endl;
        cout << "Executing the path." << endl;

        cout << "path :" << endl;
        for(auto c : configPath){
          for(size_t i = 0; i < c.size(); ++i){
            cout << c[i] << " ";
          }
          cout << endl;
        }
        cout << "Number of nodes explored :" << endl;
        cout << tree->getSize() << endl;

        ExecuteTrajectory(configPath);
        return true;
      }
      if(k % 5000 == 0)
        cout << k << ". Searching..." << endl;
    }
    cout << "Time up :(" << endl;
    return false;
  }

  /* Initializes the members by calling the input parser. */
  void Init(ostream& so, istream& si){
    _penv->GetRobots(_robots);
    _robot = _robots.at(0);

    _robot->GetActiveDOFValues(_startConfig);
    _goalConfig = GetInputAsVector(so, si);
    _robot->GetActiveDOFLimits(_activeLowerLimits, _activeUpperLimits);
    assert(_goalConfig.size() == 7 && "goalConfig should be of size 7!");
    assert(_startConfig.size() == 7 && "startConfig wasn't size 7 :(");

    // Root and Goal nodes
    startNode = NodePtr(new RRTNode(_startConfig, nullptr));
    goalNode = NodePtr(new RRTNode(_goalConfig, nullptr));
  }

  NodePtr CreateRandomNode(){
    vector<dReal> randomConfig(_activeLowerLimits.size());
    NodePtr randomNode(new RRTNode());

    do{
      for(size_t i = 0; i < _activeLowerLimits.size(); ++i)
        randomConfig[i] = static_cast<dReal>((RandomNumberGenerator()/100 * (_activeUpperLimits[i] - _activeLowerLimits[i])) + _activeLowerLimits[i]);
      randomNode->setConfiguration(randomConfig);
    }while(CheckCollision(randomNode));

    return randomNode;
  }

  NodePtr CreateRandomNodeWithBias(){
    /*This method was referenced from the Internet.*/
    vector<dReal> randomConfig(_activeLowerLimits.size());
    NodePtr randomNode(new RRTNode());

    if(RandomNumberGenerator() <= GOAL_BIAS){
//      cout << "GOAL SELECTED" << endl;
      return goalNode;
    }else{
      do{
        for(size_t i = 0; i < _activeLowerLimits.size(); ++i)
          randomConfig[i] = static_cast<dReal>((RandomNumberGenerator()/100 * (_activeUpperLimits[i] - _activeLowerLimits[i])) + _activeLowerLimits[i]);
        randomNode->setConfiguration(randomConfig);
      }while(CheckCollision(randomNode));

      return randomNode;
    }
  }

  /* Returns a random number between, and including, 0 and 99.*/
  float RandomNumberGenerator(){
    return rand() % 100;
  }

  bool CheckCollision(NodePtr node){
//    EnvironmentMutex& lock = _penv->GetMutex();
//    lock.lock();
    _robot->SetActiveDOFValues(node->getConfiguration());
    bool check = _penv->CheckCollision(_robot);
    _robot->SetActiveDOFValues(_startConfig);
    cout << check << endl;
//    lock.unlock();
    return check;
  }

  string Extend(TreePtr tree, NodePtr node){
    NodePtr nearestNode = NearestNode(tree, node);
    NodePtr newNode = NewNodeStep(nearestNode, node);
    if(!CheckCollision(newNode)){
      newNode->setParentNode(nearestNode);
      tree->addNode(newNode);
      if(DistanceBetween(newNode, node) <= STEP_SIZE){
        node->setParentNode(newNode);
        tree->addNode(node);
        if(node == goalNode)
          return "GoalReached";
        else
          return "Reached";
      }else
        return "Advanced";
    }else
      return "Trapped";
  }

  /*string Connect(TreePtr tree, NodePtr node){
    string status;
//    cout << "4.1 HERE" << endl; //TODO
    do{
      status = Extend(tree, node);
//      cout << "4.2 HERE" << endl; //TODO
    }while(status == "Advanced");

    return status;
  }*/

  string Connect(TreePtr tree, NodePtr node){
    string status;
    NodePtr nearestNode = NearestNode(tree, node);
    int i = 0;
    do{

      i++;
      NodePtr newNode = NewNodeStep(nearestNode, node);
      if(!CheckCollision(newNode)){
        newNode->setParentNode(nearestNode);
        tree->addNode(newNode);
        nearestNode = newNode;
        if(DistanceBetween(newNode, node) <= STEP_SIZE){
          node->setParentNode(newNode);
          tree->addNode(node);
          if(node == goalNode)
            status = "GoalReached";
          else
            status = "Reached";
        }else
          status = "Advanced";
      }else
        status = "Trapped";
//      cout << i << endl;
    }while(status == "Advanced" && i < 20);

    return status;
  }

  vector<dReal> GetInputAsVector(ostream& sout, istream& sinput){
    char input[INPUT_SIZE];
    vector<dReal> goalConfig;
    try{
      vector<string> temp;
      sinput.getline(input, INPUT_SIZE);
      utils::TokenizeString(input, "[ ,]", temp);
      for(string s : temp)
        goalConfig.push_back(atof(s.c_str()));
    }catch(exception &e){
      cout << e.what() << endl;
    }
    return goalConfig;
  }

  void ExecuteTrajectory(vector< vector<dReal> > configPath){
    EnvironmentMutex& lock = _penv->GetMutex();
    lock.lock();
    TrajectoryBasePtr traj = RaveCreateTrajectory(_penv);
    traj->Init(_robot->GetActiveConfigurationSpecification());


    for(vector<dReal> config : configPath)
      traj->Insert(0, config);
    traj->Insert(0, _startConfig);

//    planningutils::SmoothActiveDOFTrajectory(traj, _robot);

    planningutils::RetimeActiveDOFTrajectory(traj, _robot);

    _robot->GetController()->SetPath(traj);

    lock.unlock();
  }

  NodePtr NewNodeStep(NodePtr fromNode, NodePtr toNode){
    vector<dReal> vectorBetweenNodes = toNode->getConfiguration();
    for(size_t i=0; i < fromNode->getConfiguration().size(); ++i)
      vectorBetweenNodes.push_back(toNode->getConfiguration()[i] - fromNode->getConfiguration()[i]);

//    _robot->SubtractActiveDOFValues(vectorBetweenNodes, fromNode->getConfiguration());

//    dReal magnitude = DistanceBetween(fromNode, toNode);

    dReal magnitudeSq = 0;
    for(dReal e : vectorBetweenNodes)
      magnitudeSq += pow(e, 2);
    dReal magnitude = sqrt(magnitudeSq);

    NodePtr newNode(new RRTNode());
    vector<dReal> newConfiguration;
    for(size_t i=0; i < fromNode->getConfiguration().size(); ++i)
      newConfiguration.push_back(fromNode->getConfiguration()[i] + ((vectorBetweenNodes[i]/magnitude)*STEP_SIZE));

    newNode->setConfiguration(newConfiguration);
    return newNode;
  }

  dReal DistanceBetween(NodePtr node1, NodePtr node2){
      dReal distanceSquared = 0;
      vector<dReal> node1Config = node1->getConfiguration();
      vector<dReal> node2Config = node2->getConfiguration();

      //Normalized distance
      for(size_t i = 0; i < node1Config.size(); ++i)
          distanceSquared += pow((node1Config[i] - node2Config[i])/(_activeUpperLimits[i]-_activeLowerLimits[i]), 2);
      return sqrt(distanceSquared);
  }

  NodePtr NearestNode(TreePtr tree, NodePtr node){
      dReal lowestDistance = numeric_limits<double>::max();
      NodePtr closestNode;
      for(NodePtr n : tree->getAllNodes()){
        dReal distance = DistanceBetween(n, node);
        if(distance <= lowestDistance){
          lowestDistance = distance;
          closestNode = n;
        }
      }
      return closestNode;
  }

  /*vector<NodePtr> GetPath(NodePtr node){
    vector<NodePtr> path;
    path.push_back(node);
    while(node->getParentNode() != nullptr){
      node = node->getParentNode();
      path.push_back(node);
    }
    //  std::reverse(path.begin(), path.end());   //Reverse the path since its been added end to start.
    return path;
  }*/


private:
  EnvironmentBasePtr _penv;
  vector<dReal> _startConfig;
  vector<dReal> _goalConfig;
  vector<dReal> _activeLowerLimits;
  vector<dReal> _activeUpperLimits;
  vector<RobotBasePtr> _robots;
  RobotBasePtr _robot;
  NodePtr startNode;
  NodePtr goalNode;
  };


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
  if( type == PT_Module && interfacename == "rrt_module" ) {
    return InterfaceBasePtr(new rrt_module(penv,sinput));
  }

  return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
  info.interfacenames[PT_Module].push_back("rrt_module");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

