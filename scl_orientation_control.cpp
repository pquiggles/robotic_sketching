/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
 */
/* \file scl_tutorial4_control_gc_op.cpp
 *
 *  Created on: Jul 30, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/util/DatabaseUtils.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include <float.h>

// #include <zmqpp/zmqpp.hpp>

 void jointSpaceControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
 void opSpacePositionControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
 void opSpaceOrientationControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
 void opSpacePositionOrientationControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
 void readGraph();
 std::vector<std::string> split(const std::string &s, char delim, std::vector<std::string> &elems);
 std::vector<std::string> split(const std::string &s, char delim);


const double X_OFFS_MAX = 0.05;
const double X_OFFS_MIN = -.25;

const double Y_OFFS_MIN = -.1;
const double Y_OFFS_MAX = .1;


const double ON_CANVAS_Z_OFFS = -.018;
const double OFF_CANVAS_Z_OFFS = 0.05;

double theta = M_PI/3;
class Edge;
// bool zmqInitialized = false;



 class Vertex
 {
 public:
  int id;
  double x;
  double y;
  bool visited;
  std::vector<Edge *> neighbors;
  bool isChild;

  Vertex()
  {
    visited = false;
    id = 0;
    x = 0;
    y = 0;
    isChild = false;
  }

  bool operator < (const Vertex & v) const
  {
    return y < v.y;
  }

};

class Edge
{
public:
  Vertex * start;
  Vertex * end;
  bool visited;
  Edge(Vertex * v1, Vertex * v2)
  {
    start = v1;
    end = v2;
    visited = false;
  }
};

class PointGraph
{
public:
  std::vector<Vertex *> vertices;
  bool pulled_away;
  bool returning;
  Edge * returningEdge;
  Vertex * target_vertex;
  std::stack <Edge *> edges_to_visit;


  PointGraph()
  {
    pulled_away = true;
    returning = false;
    returningEdge = NULL;
    target_vertex= NULL;
  }

  bool isPulledAway()
  {
    return pulled_away;
  }

  Vertex * getNextTarget()
  {

    if(pulled_away)
    {
      pulled_away = false;
      return target_vertex;
    }
    else if (returning)
    {
      returning = false;
      target_vertex = returningEdge->end;
    }
    else if ( target_vertex->neighbors.size() > 0 )
    {
      if ( target_vertex->neighbors.size() > 1 )
      {
        for (uint i = 1; i < target_vertex->neighbors.size(); i++)
        {
          if (!target_vertex->neighbors[i]->visited)
          {
            edges_to_visit.push(target_vertex->neighbors[i]);
          }         
        }
      }
      target_vertex->neighbors[0]->visited = true;
      target_vertex = target_vertex->neighbors[0]->end;
    }
    else if ( !edges_to_visit.empty() )
    {
      returning = true;
      pulled_away = true;
      returningEdge =  edges_to_visit.top();
      edges_to_visit.pop();
      returningEdge->visited = true;
      target_vertex = returningEdge->start;
    }
    else
    {
      target_vertex = NULL;
      for (unsigned int i = 0; i < vertices.size(); i++)
      {
        if (!vertices[i]->visited)
        {
          std::cout << "FOUND VERTEX: " << i << std::endl;
          target_vertex = vertices[i];
          target_vertex->visited = true;
          if(target_vertex->neighbors.size() == 0 ||
            target_vertex->isChild)
            continue;
          pulled_away = true;
          break;
        }
      }
    }
    if (target_vertex != NULL)
      target_vertex->visited = true;
    return target_vertex;
  }

  Vertex * getFirtVertex()
  {
    if(vertices.size() > 0)
    {
      target_vertex = vertices[0];
      target_vertex->visited = true;
      return vertices[0];
    }
    return NULL;
  }

  void renormalize(){
    double x_min, y_min, x_max, y_max;
    x_min = DBL_MAX;
    y_min = DBL_MAX;
    x_max = -DBL_MAX;
    y_max = -DBL_MAX;
   
    for(uint i = 0; i < vertices.size(); i++)
    {
      if( vertices[i]->x < x_min ){
        x_min = vertices[i]->x;
      }
      if( vertices[i]->x > x_max ){
        x_max = vertices[i]->x;
      }
      if (vertices[i]->y < y_min){
        y_min = vertices[i]->y;
      }
      if (vertices[i]->y > y_max){
        y_max = vertices[i]->y;
      }
    }
    double x_range = x_max - x_min;
    double y_range = y_max - y_min;
    double scaling_factor;
    if (x_range > y_range)
    {
      scaling_factor = (X_OFFS_MAX - X_OFFS_MIN) / x_range;
    }
    else
    {
      scaling_factor = (Y_OFFS_MAX - Y_OFFS_MIN) / y_range;
    }
    // rescale
    for (uint i = 0; i < vertices.size(); i++)
    {
      vertices[i]->x = X_OFFS_MIN + scaling_factor * ( vertices[i]->x - x_min);
      vertices[i]->y = Y_OFFS_MIN + scaling_factor * ( vertices[i]->y - y_min);
    }
  }
};

PointGraph pg;

std::vector<std::string> split(const std::string &s, char delim, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim))
  {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}



void readGraph()
{
  std::string line;
  std::ifstream infile;
  infile.open("graph_representation.txt");

  std::getline(infile, line);
  std::string::size_type sz;
  int num_vertices = std::stoi (line ,&sz);

  pg.vertices.resize(num_vertices);
  for (int i = 0; i < num_vertices; i++)
  {

    Vertex * v = new Vertex();
    pg.vertices[i] = v;
  }

  for(int i = 0; i < num_vertices; i++)
  {
    std::getline(infile, line);
    std::vector<std::string> tokens = split(line, ' ');

    pg.vertices[i]->x = atof(tokens[1].c_str());
    pg.vertices[i]->y = atof(tokens[2].c_str());
    int num_neighbors= std::stoi(tokens[3] ,&sz);
    for(int j = 0; j < num_neighbors; j++)
    {
      int neighborIdx = std::stoi(tokens[4 + j] ,&sz);
      Vertex * neighbor = pg.vertices[neighborIdx];
      neighbor->isChild = true;
      Edge * e =  new Edge(pg.vertices[i], neighbor);
      pg.vertices[i]->neighbors.push_back(e);
    }

  }
  infile.close();
  pg.renormalize();
  //std::sort(pg.vertices.begin(), pg.vertices.end());
}


/** A sample application to demonstrate a physics simulation in scl.
 *
 * Moving forward from tutorial 3, we will now control the 6 DOF
 * demo robot (r6bot) with the physics engine running.
 *
 * SCL Modules used:
 * 1. data_structs
 * 2. dynamics (physics)
 * 4. dynamics (control matrices)
 * 4. graphics (chai)
 * */
 int main(int argc, char** argv)
 {
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Control Types";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...
  sutil::CSystemClock::start();

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  bool flag = p.readRobotFromFile("iiwa/iiwaCfg.xml","./","iiwaBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Error check.

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("iiwa/iiwaCfg.xml","iiwaBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the r6bot's physics. \nWill test different controllers.\n Press (x) to exit at anytime.";

  omp_set_num_threads(2);
  int thread_id;





#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();
    if(thread_id==1) //Simulate physics and update the rio data structure..
    {

      readGraph();
      opSpacePositionOrientationControl(rio,rgcm,dyn_scl,dyn_tao);
     
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }
    else  //Read the rio data structure and updated rendererd robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }
    }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  return 0;
}


// void sendToRobot(scl::SRobotIO &robot_)
//   {
//       // Initialize a 0MQ publisher socket
//       static zmqpp::context context;
//       static zmqpp::socket pub(context, zmqpp::socket_type::publish);

//       if (!zmqInitialized)
//       {
//           // Need to pair this with the endpoint port in ROS code
//           pub.bind("tcp://*:3883");
//           zmqInitialized = true;
//       }
//       // TODO: Is data lock needed here?
//       else
//       {
//           zmqpp::message msg;
//           Eigen::VectorXd q;
//           q = robot_.sensors_.q_;
//           msg << std::to_string(q[0]) + " " +
//                      std::to_string(q[1]) + " " +
//                      std::to_string(q[2]) + " " +
//                      std::to_string(-q[3]) + " " +  // joint 3 is inverted
//                      std::to_string(q[4]) + " " +
//                      std::to_string(q[5]) + " " +
//                      std::to_string(q[6]);

         
//           std::cout << "SENDING POSITIONS: " << q << std::endl;
//           pub.send(msg);
//       }
//   }


// static bool initialized = false;
// bool receiveFromRobot(Eigen::VectorXd &q)
//   {
//     bool flag=true;
//     // Initialize a 0MQ subscriber socket
//     static zmqpp::context context;
//     static zmqpp::socket sub(context, zmqpp::socket_type::subscribe);
   

//     if (!initialized)
//     {
//       sub.subscribe("");
//       sub.connect("tcp://localhost:3884");
//       initialized = true;
//       return false;
//     }
//     else
//     {
//       zmqpp::message message;
//       if (sub.receive(message, true))
//       {
//         std::string msg;
//         message >> msg;
//         std::stringstream msg_stream(msg);

//         // Read the current joint positions from the robot
//         Eigen::VectorXd q_received, dq_received;
//         q_received.resize(7);
//         msg_stream >> q_received[0] >> q_received[1] >> q_received[2] >> q_received[3] >> q_received[4] >> q_received[5] >> q_received[6];
//         //>> dq_received[0] >> dq_received[1] >> dq_received[2] >> dq_received[3] >> dq_received[4] >> dq_received[5] >> dq_received[6]

//         q = q_received;
//       }
//       else
//       { return false; }
//     }
//     return flag;
//   }


void opSpacePositionOrientationControl(scl::SRobotIO &rio, scl::SGcModel& rgcm, scl::CDynamicsScl& dyn_scl, scl::CDynamicsTao &dyn_tao)
{
  double tstart, tcurr;
  double dt=0.0001;
  long iter = 0;
  bool flag = false;

  std::cout<<"\n\n***************************************************************"
  <<"\n Starting op space position + orientation controller..."
  <<"\n***************************************************************\n";
  tstart = sutil::CSystemClock::getSysTime(); iter = 0;

  scl::SRigidBodyDyn *rhand = rgcm.rbdyn_tree_.at("end-effector");
  const Eigen::Vector3d hpos(0,0,0.05); //control position of op-point wrt. hand

  /*********************************************************/
  /*********************************************************/
  /*************     YOUR CODE HERE     ********************/
  /*********************************************************/
  /*********************************************************/


  // Define the matrices and vectors you will need
  Eigen::VectorXd Gamma;
  Eigen::MatrixXd J, A, A_inv, Lambda, R, R_des, Rx, R_init, x_init;
  Eigen::Vector3d w;
  Eigen::VectorXd g, F;
  Eigen::VectorXd Fg(6);
  Eigen::VectorXd q_init(7);

  R_des.resize(3,3);
  Rx.resize(3,3);

  Vertex * target_vertex = pg.getFirtVertex();
  // bool firstRun = true;
  // bool robotEnabled = false;


  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    Eigen::VectorXd q_recv_;
    // if (firstRun && !robotEnabled && receiveFromRobot(q_recv_))
    // {
    //   firstRun = false;
    //   std::cout << "Q Received! " << q_recv_ << std::endl;
    //   rio.sensors_.q_ = q_recv_;
    //   robotEnabled = true;
    //   for(int i = 0; i < 7; i++)
    //     rio.sensors_.dq_[i] = 0;     
    // }



    //get current time and compute the model
    tcurr = sutil::CSystemClock::getSysTime();
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    // initial position
    if(false == flag)
    {
      x_init = rhand->T_o_lnk_ * hpos;
      rhand->T_o_lnk_.rotation();
      flag = true;
      q_init = rio.sensors_.q_;
    }
   
    // Compute your Jacobians
    dyn_scl.computeJacobianWithTransforms(J,*rhand,rio.sensors_.q_,hpos);

    // Mass Matrix and its inverse
    A = rgcm.M_gc_;
    A_inv = rgcm.M_gc_inv_;

    // gains   
    double kp = 500;
    double ko = 400;
    double kv = 75;

    double x_offs = target_vertex->x;
    double y_offs =  target_vertex->y;
    double z_offs =  pg.isPulledAway() ? OFF_CANVAS_Z_OFFS : ON_CANVAS_Z_OFFS;

    // current and desired position
    Eigen::VectorXd x = rhand->T_o_lnk_ * hpos;
    Eigen::VectorXd x_des(3);
    x_des << x_offs, y_offs, z_offs;
    x_des += x_init;
    Eigen::VectorXd dx = (x - x_des);
    if(dx.norm() < .01)
    {
      target_vertex = pg.getNextTarget();
      if(target_vertex == NULL)
        return;
      double x_offs = target_vertex->x ;
      double y_offs = target_vertex->y;
      double z_offs = pg.isPulledAway() ? OFF_CANVAS_Z_OFFS : ON_CANVAS_Z_OFFS;

      std::cout << "Going to target " << target_vertex->x  << " , " << target_vertex->y << std::endl;

      x_des << x_offs, y_offs, z_offs;
      x_des += x_init;
      dx = (x - x_des);
    } 

    // current and desired orientations
    R = rhand->T_o_lnk_.rotation();
    R_des << -1, 0, 0,
              0, 1, 0,
              0, 0, -1;  



    // angular error vector
    Eigen::Vector3d d_phi;
    d_phi << 0, 0, 0;
    for (int i = 0; i < 3; i++)
    {
      Eigen::Vector3d R3;
      Eigen::Vector3d Rd3;
      R3 << R(0,i), R(1,i), R(2,i);
      Rd3 << R_des(0,i), R_des(1,i), R_des(2,i);
      d_phi += R3.cross(Rd3);
    }

    d_phi *= -.5;
    // current velocity
    Eigen::VectorXd dv = J * rio.sensors_.dq_;

    // Operational space Inertia matrix
    Lambda = J * A_inv * J.transpose();
    Lambda = Lambda.inverse();
   
    // Operational space controller force
    Eigen::VectorXd dp(6);
    dp << kp * dx, ko * d_phi;   
    F = Lambda * - ( dp + kv * dv);
   
    // joint space gravity
    Fg << 0, 0, -9.81, 0, 0, 0;
    g = J.transpose() * Fg;

    // joint torques
    Gamma = J.transpose() * F - g;



    Eigen::VectorXd ns_damping(6);
    
    Eigen::VectorXd joint_drift(7);
    Eigen::VectorXd ns_task(6);
    Eigen::MatrixXd J_bar = A_inv*J.transpose()*Lambda;
    
    Eigen::VectorXd damping_vec(7);
    damping_vec = rio.sensors_.dq_;


    joint_drift = (q_init - rio.sensors_.q_);

    ns_damping = (Eigen::MatrixXd::Identity(6,6)-J.transpose()*J_bar.transpose())*rio.sensors_.dq_;
    

    ns_task = (Eigen::MatrixXd::Identity(6,6)-J.transpose()*J_bar.transpose())*(joint_drift);


    rio.actuators_.force_gc_commanded_ = Gamma  - 13.0*ns_damping + 13.0 * ns_task;


    // Integrate the dynamics
    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, 5000};/*.05ms*/ nanosleep(&ts,NULL);
    rio.sensors_.q_(6) = theta;


    // if(!firstRun && iter % 8 == 0)
    // {
    //   //rio.sensors_.q_ = q_recv_;
    //   sendToRobot(rio);
    // }

    // robotEnabled = false;




    // print output
    if(iter % 1000 == 0)
    {
      std::cout <<"\nDx norm:"<< dx.norm() << std::endl;
      std::cout<<"\n" << tcurr << " " << x.transpose() << " " << x_des.transpose() << " " <<d_phi.transpose();
     std::cout.flush();
   }

    /*********************************************************/
    /*********************************************************/
    /*************     END OF YOUR CODE     ******************/
    /*********************************************************/
    /*********************************************************/
 }
}