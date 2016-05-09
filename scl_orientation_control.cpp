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



void jointSpaceControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
void opSpacePositionControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
void opSpaceOrientationControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
void opSpacePositionOrientationControl(scl::SRobotIO& , scl::SGcModel& , scl::CDynamicsScl& , scl::CDynamicsTao&);
void readGraph();
std::vector<std::string> split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);


class Edge;

class Vertex
{
 public:
  int id;
  double x;
  double y;
  bool visited;
  std::vector<Edge *> neighbors;
  Vertex()
  {
    visited = false;
    id = 0;
    x = 0;
    y = 0;
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

std::stack <Edge *> edges_to_visit;
class PointGraph
{
public:
  std::vector<Vertex *> vertices;
  PointGraph(){
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
      Edge * e =  new Edge(pg.vertices[i], neighbor);
      pg.vertices[i]->neighbors.push_back(e);
    }

  }
  infile.close();
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
  Eigen::MatrixXd J, A, A_inv, Lambda, R, R_des, R_init, x_init;
  Eigen::Vector3d w;
  Eigen::VectorXd g, F;
  Eigen::VectorXd Fg(6);

  R_des.resize(3,3);

  Vertex * target_vertex = pg.vertices[0];
  target_vertex->visited = true;
  bool returning = false;
  bool pulled_away = false;
  Edge * returningEdge = NULL;
  while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
  {
    //get current time and compute the model
    tcurr = sutil::CSystemClock::getSysTime();
    dyn_scl.computeGCModel(&rio.sensors_,&rgcm);

    // initial position
    if(false == flag) 
    { 
      x_init = rhand->T_o_lnk_ * hpos;
      rhand->T_o_lnk_.rotation();
      flag = true; 
    }
    
    // Compute your Jacobians
    dyn_scl.computeJacobianWithTransforms(J,*rhand,rio.sensors_.q_,hpos);

    // Mass Matrix and its inverse
    A = rgcm.M_gc_;
    A_inv = rgcm.M_gc_inv_;

    // gains    
    double kp = 500;
    double ko = 200;
    double kv = 70;

    double x_offs = pulled_away ? 0.0 : 0.3 ;


    double y_offs = .3 + target_vertex->x;
    double z_offs = -.2 + target_vertex->y;

    // current and desired position 
    Eigen::VectorXd x = rhand->T_o_lnk_ * hpos;
    Eigen::VectorXd x_des(3);
    x_des << x_offs, y_offs, z_offs;
    x_des += x_init;
    Eigen::VectorXd dx = (x - x_des);
    if(dx.norm() < .01)
      {
        if(pulled_away)
        {
            pulled_away = false;
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
          if(!target_vertex->neighbors[0]->visited)
          {
            target_vertex->neighbors[0]->visited = true;
            target_vertex = target_vertex->neighbors[0]->end;
          }
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
          bool nextVertexFound = false;
          for (unsigned int i = 0; i < pg.vertices.size(); i++)
          {
            if (!pg.vertices[i]->visited)
            {
              target_vertex = pg.vertices[i];
              target_vertex->visited = true;
              nextVertexFound = true;
              pulled_away = true;
              break;
            }
          }
          if ( !nextVertexFound )
          {
            break;
          }
        }
        double x_offs = pulled_away ? 0.0 : 0.3 ;
        double y_offs = .3 + target_vertex->x;
        double z_offs = -.2 + target_vertex->y;
        target_vertex->visited = true;

        x_des << x_offs, y_offs, z_offs;
        x_des += x_init;
        dx = (x - x_des); 
      }  

    // current and desired orientations
    R = rhand->T_o_lnk_.rotation();
    R_des << cos(M_PI / 3), 0, sin(M_PI / 3),
                        0,  1, 0,
                -sin(M_PI / 3), 0, cos(M_PI / 3);   
    
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
    Fg << 0, 0, -9.8, 0, 0, 0;
    g = J.transpose() * Fg;

    // joint torques
    Gamma = J.transpose() * F - g;



    Eigen::VectorXd ns_damping(6);
    Eigen::MatrixXd J_bar = A_inv*J.transpose()*Lambda;
    ns_damping = (Eigen::MatrixXd::Identity(6,6)-J.transpose()*J_bar.transpose())*rio.sensors_.dq_;

    rio.actuators_.force_gc_commanded_ = Gamma  - 15.0*ns_damping;


    // Integrate the dynamics
    dyn_tao.integrate(rio,dt); iter++; const timespec ts = {0, 5000};/*.05ms*/ nanosleep(&ts,NULL);

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


