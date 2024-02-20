#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  setStart(150, 150, 0);
  setGoal(250,150,0);
  // ___________________
  // TOPICS TO SUBSCRIBE
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);

};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
  //  ros::Time t1 = ros::Time::now();
  //  ros::Duration d(t1 - t0);
  //  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available

  if (grid->info.height >= start.pose.pose.position.y &&
      start.pose.pose.position.y >= 0 &&
      grid->info.width >= start.pose.pose.position.x &&
      start.pose.pose.position.x >= 0) {
    // set the start as valid and plan
    validStart = true;
  } else {
    validStart = false;
  }
  plan();
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(int start_x, int start_y, double start_t) {
  geometry_msgs::PoseWithCovarianceStamped initial;
  initial.pose.pose.position.x = start_x;

  initial.pose.pose.position.y = start_y;
  initial.pose.pose.orientation = tf::createQuaternionMsgFromYaw(start_t);
  float x = initial.pose.pose.position.x / Constants::cellSize;
  float y = initial.pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial.pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial.pose.pose.position;
  startN.pose.orientation = initial.pose.pose.orientation;
  startN.header.frame_id = "camera_init";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y
            << " t:" << Helper::toDeg(t) << std::endl;

  validStart = true;
  start = initial;

  // publish start for RViz
  pubStart.publish(startN);
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(int end_x, int end_y, double end_t) {
  geometry_msgs::PoseStamped end;
  end.pose.position.x = end_x;
  end.pose.position.y = end_y;
  end.pose.orientation = tf::createQuaternionMsgFromYaw(end_t);
  // retrieving goal position
  float x = end.pose.position.x / Constants::cellSize;
  float y = end.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end.pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y
            << " t:" << Helper::toDeg(t) << std::endl;

  validGoal = true;
  goal = end;

}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    std::cout << "TIME in ms: " << (t1-t0).toSec() << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);



    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
