#include "rrt_connect.h"
#include <algorithm>
#include <limits>
#include <math.h>
#include <openrave/utils.h>
#include <openrave/planningutils.h>

#define COMPUTATION_TIME 35000
#define STEP_SIZE 0.4
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
  for(NodePtr n : _nodes){
    if(n->getId() == id)
      return n;
  }
  throw 0;
}

vector<NodePtr> NodeTree::getPathTo(int id){
  // Find the node
  NodePtr final = getNode(id);

  // Find the path from the parents
  vector<NodePtr> path;
  path.push_back(final);
  while(final->getParentNode() != nullptr){
    final = final->getParentNode();
    path.push_back(final);
  }
  return path;
}

vector<NodePtr>& NodeTree::getAllNodes(){
  return _nodes;
}

size_t NodeTree::getSize(){
  return _nodes.size();
}


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
          treeA->getAllNodes().pop_back();
          vector<NodePtr> pathA = treeA->getPathTo(treeA->getMostRecentNode()->getId());
          vector<NodePtr> pathB = treeB->getPathTo(treeB->getMostRecentNode()->getId());
          vector<NodePtr> fullPath;

          std::reverse(pathA.begin(), pathA.end());
          fullPath.reserve(pathA.size() + pathB.size()); // preallocate memory
          fullPath.insert(fullPath.end(), pathA.begin(), pathA.end()); //TODO check if this can be done with end, begin to avoid reversing
          fullPath.insert(fullPath.end(), pathB.begin(), pathB.end());

          vector< vector<dReal> > configPath;
          configPath.reserve(fullPath.size());
          for(NodePtr pnode : fullPath)
            configPath.push_back(pnode->getConfiguration());

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

      string status = Connect(tree, randomNode);
      if(status == "GoalReached"){
        vector<NodePtr> path = tree->getPathTo(goalNode->getId());

        vector< vector<dReal> > configPath;
        configPath.reserve(path.size());
        for(NodePtr pnode : path)
          configPath.push_back(pnode->getConfiguration());


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

    _activeDOFRanges.reserve(_activeLowerLimits.size());
    for(size_t i = 0; i < _activeLowerLimits.size(); ++i){
      if(i == 4 || i == 6){
        _activeUpperLimits[i] = M_PI;
        _activeLowerLimits[i] = -M_PI;
      }
      _activeDOFRanges[i] = _activeUpperLimits[i]-_activeLowerLimits[i];
    }

    _dofWeights = {3.17104, 2.75674, 2.2325, 1.78948, 0, 0.809013, 0};

    // Root and Goal nodes
    startNode = NodePtr(new RRTNode(_startConfig, nullptr));
    goalNode = NodePtr(new RRTNode(_goalConfig, nullptr));
  }

  /* Returns a random node without any goal bias. */
  NodePtr CreateRandomNode(){
    vector<dReal> randomConfig(_activeLowerLimits.size());
    NodePtr randomNode(new RRTNode());

    do{
      for(size_t i = 0; i < _activeLowerLimits.size(); ++i)
        randomConfig[i] = static_cast<dReal>((RandomNumberGenerator()/100 * (_activeDOFRanges[i])) + _activeLowerLimits[i]);
      randomNode->setConfiguration(randomConfig);
    }while(CheckCollision(randomNode));

    return randomNode;
  }

  NodePtr CreateRandomNodeWithBias(){
    /*The idea for goal biasing was referenced from the Internet.*/
    vector<dReal> randomConfig(_activeLowerLimits.size());
    NodePtr randomNode(new RRTNode());

    if(RandomNumberGenerator() <= GOAL_BIAS){
      return goalNode;
    }else{
      do{
        for(size_t i = 0; i < _activeLowerLimits.size(); ++i){
            randomConfig[i] = static_cast<dReal>((RandomNumberGenerator()/100 * (_activeDOFRanges[i])) + _activeLowerLimits[i]);
        }
        randomNode->setConfiguration(randomConfig);
      }while(CheckCollision(randomNode));

      return randomNode;
    }
  }

  /* Returns a random number between, and including, 0 and 99.*/
  float RandomNumberGenerator(){
    return rand() % 100;
  }

  /* Checks collision of the robot with the environment and itself and returns true if there is any collision detected. */
  bool CheckCollision(NodePtr node){
    _robot->SetActiveDOFValues(node->getConfiguration());
    bool check1 = _penv->CheckCollision(_robot);
    bool check2 = _penv->CheckSelfCollision(_robot);
    _robot->SetActiveDOFValues(_startConfig);
    return check1 || check2;
  }

  /* Extends one step towards the given node. */
  string Extend(TreePtr tree, NodePtr node){
    NodePtr nearestNode = NearestNode(tree, node);
    NodePtr newNode = NewStep(nearestNode, node);
    if(InLimits(newNode) && !CheckCollision(newNode)){
      newNode->setParentNode(nearestNode);
      tree->addNode(newNode);
      if(UnweightedDistance(newNode, node) <= STEP_SIZE){
        node->setParentNode(newNode);
        tree->addNode(node);
        return "Reached";
      }else
        return "Advanced";
    }else
      return "Trapped";
  }

  /* Tries to connect the tree to the given node. */
  string Connect(TreePtr tree, NodePtr node){
    string status;
    NodePtr nearestNode = NearestNode(tree, node);
    NodePtr start = nearestNode;
    do{
      NodePtr newNode = NewStep(nearestNode, node);

      if(InLimits(newNode) && !CheckCollision(newNode)){
        newNode->setParentNode(nearestNode);
        tree->addNode(newNode);
        nearestNode = newNode;
        if(UnweightedDistance(newNode, node) <= STEP_SIZE){
          node->setParentNode(newNode);
          tree->addNode(node);
          if(UnweightedDistance(node, goalNode) <= STEP_SIZE)
            return "GoalReached";
          else
            return "Reached";
        }else
          status = "Advanced";
      }else{
        return "Trapped";}
    }while(status == "Advanced");
    return status;
  }

  /* Checks if the configuration of the node is in DOF limits. */
  bool InLimits(NodePtr node){
    vector<dReal> config = node->getConfiguration();
    for(size_t i = 0; i < config.size(); ++i){
      if(config[i] < _activeLowerLimits[i] || config[i] > _activeUpperLimits[i])
        return false;
    }
    return true;
  }

  /*Parses the input from the python script and returns the goal config. */
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

  /* Generates and executes a given configuration path. */
  void ExecuteTrajectory(vector< vector<dReal> > &configPath){
    EnvironmentMutex& lock = _penv->GetMutex();
    lock.lock();
    TrajectoryBasePtr traj = RaveCreateTrajectory(_penv);
    traj->Init(_robot->GetActiveConfigurationSpecification());


    for(vector<dReal> config : configPath)
      traj->Insert(0, config);
    traj->Insert(0, _startConfig);

    planningutils::RetimeActiveDOFTrajectory(traj, _robot);

    _robot->GetController()->SetPath(traj);

    lock.unlock();
  }

  /* Returns a new node in the direction of the second node at a step_size distance from the first node. */
  NodePtr NewStep(NodePtr from, NodePtr to){
    vector<dReal> unitVector;
    vector<dReal> fromConfig = from->getConfiguration();
    vector<dReal> toConfig = to->getConfiguration();
    dReal mag = UnweightedDistance(from, to);

    for(size_t i=0; i < fromConfig.size(); ++i){
      unitVector.push_back((toConfig[i] - fromConfig[i])/mag);
    }
    vector<dReal> newConfig;
    for(size_t i=0; i < fromConfig.size(); ++i){
      newConfig.push_back(fromConfig[i] + (unitVector[i]*STEP_SIZE));
    }
    NodePtr newNode(new RRTNode());
    newNode->setConfiguration(newConfig);
    return newNode;
  }

  /* Calculates and returns the distance between two nodes taking into consideration the DOF weights. */
  dReal WeightedDistance(NodePtr node1, NodePtr node2){
      dReal distanceSquared = 0;
      vector<dReal> node1Config = node1->getConfiguration();
      vector<dReal> node2Config = node2->getConfiguration();

      // Weighted distance
      for(size_t i = 0; i < node1Config.size(); ++i){
        distanceSquared += pow((node1Config[i] - node2Config[i])*_dofWeights[i], 2);
      }

      return sqrt(distanceSquared);
  }

  /* Calculates and returns the distance between two nodes. */
  dReal UnweightedDistance(NodePtr node1, NodePtr node2){
        dReal distanceSquared = 0;
        vector<dReal> node1Config = node1->getConfiguration();
        vector<dReal> node2Config = node2->getConfiguration();

        for(size_t i = 0; i < node1Config.size(); ++i){
          distanceSquared += pow((node1Config[i] - node2Config[i]), 2);
        }

        return sqrt(distanceSquared);
    }

  /* Returns the nearest node in the tree to the node. */
  NodePtr NearestNode(TreePtr tree, NodePtr node){
      dReal lowestDistance = numeric_limits<double>::max();
      NodePtr closestNode;
      for(NodePtr n : tree->getAllNodes()){
        dReal distance = WeightedDistance(n, node);
        if(distance <= lowestDistance){
          lowestDistance = distance;
          closestNode = n;
        }
      }
      return closestNode;
  }


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
  vector<dReal> _activeDOFRanges;
  vector<dReal> _dofWeights;
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

