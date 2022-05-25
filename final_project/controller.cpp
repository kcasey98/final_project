/**
* @file controller.cpp
* @brief Controller file
*
*/
 
#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
// #include "readEigen.h"
 
#include <iostream>
#include <string>
 
using namespace std;
using namespace Eigen;
 
#include <fstream>
#include <Eigen/Dense>
 
#define MAXBUFSIZE  ((int) 1e5)
 
#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}
 
#define RAD(deg) ((double)(deg) * M_PI / 180.0)
 
#include "redis_keys.h"
 
// Location of URDF files specifying world and robot information
const string robot_file = "./resources/panda_arm.urdf";
 
enum State
{
   POSTURE = 0,
   MOTION = 1,
   LAUNCH1 = 2,
   PREGRAB = 3,
   BEGINNING = 4,
   GRAB = 5,
   POSTGRAB = 6,
   PRELAUNCH = 7,
   ORIENTATE = 8,
};
 
int main() {
 
   // initial state
   int state = PREGRAB;
   // MOTION or PREGRAB
   string controller_status = "1";
  
   // start redis client
   auto redis_client = RedisClient();
   redis_client.connect();
 
   Vector3d X_plan, x, dx, X_der, X_der2;
 
   MatrixXd X_traj(10,1);
 
   double dt = 0.001;
 
   // set up signal handler
   signal(SIGABRT, &sighandler);
   signal(SIGTERM, &sighandler);
   signal(SIGINT, &sighandler);
 
   // read trajectory **************** William
   // const string pos_traj_fname = "pos_traj.txt";
   // const string vel_traj_fname = "vel_traj.txt";
   // auto pos_traj = readMatrix(pos_traj_fname.c_str());
   // auto vel_traj = readMatrix(vel_traj_fname.c_str());
 
   // load robots, read current state and update the model
   auto robot = new Sai2Model::Sai2Model(robot_file, false);
   robot->_q.head(7) = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
   robot->_dq.head(7) = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
   robot->updateModel();
 
   // prepare controller
   int dof = robot->dof();
   VectorXd command_torques = VectorXd::Zero(dof + 2);  // panda + gripper torques
   MatrixXd N_prec = MatrixXd::Identity(dof, dof);
 
   // pose task
   const string control_link = "link7";
   const Vector3d control_point = Vector3d(0, 0, 0.22);
   auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
   posori_task->_use_interpolation_flag = true;
   posori_task->_use_velocity_saturation_flag = true;

   posori_task->setDynamicDecouplingFull(); //Mikael added this in OH, dynamic decoupling
   
   const Vector3d pos_in_ee_link = Vector3d(0, 0, 0.22);
   // containers
   Vector3d ee_pos;
   Matrix3d ee_rot;
   Matrix3d R_init;
   
   //OTG
   robot->position(x, "link7", pos_in_ee_link);
   //robot->rotation(R_init,control_link);
   VectorXd initial_pos = VectorXd::Zero(3); 
//    VectorXd goal_position = VectorXd::Zero(3); 
//    VectorXd goal_velocity = VectorXd::Zero(3); 
//    VectorXd next_position = VectorXd::Zero(3); 
//    VectorXd next_velocity = VectorXd::Zero(3); 
//    VectorXd next_acceleration = VectorXd::Zero(3); 

    double angle_throw = 0;

   initial_pos << x(0), x(1), x(2);
   //cout << initial_pos << endl;
   //auto OTG = new Sai2Primitives::OTG_posori(initial_pos,R_init,0.001);
   //auto OTG = new Sai2Primitives::OTG(initial_pos,0.001);

   //OTG_posori(const Eigen::VectorXd& initial_position, const Eigen::Matrix3d& initial_orientation, const double loop_time);

   VectorXd posori_task_torques = VectorXd::Zero(dof);
   posori_task->_kp_pos = 200.0;
   posori_task->_kv_pos = 20.0;
   posori_task->_kp_ori = 200.0;
   posori_task->_kv_ori = 20.0;
 
   // joint task
   auto joint_task = new Sai2Primitives::JointTask(robot);
   joint_task->_use_interpolation_flag = true;
   joint_task->_use_velocity_saturation_flag = true;
 
   VectorXd joint_task_torques = VectorXd::Zero(dof);
   joint_task->_kp = 200.0;
   joint_task->_kv = 40.0;
 
   VectorXd q_init_desired(dof);
   VectorXd init_desired(dof);
   VectorXd q_zero(dof);
   q_zero << 0,0,0,0,0,0,0;
   init_desired = robot->_q;
   q_init_desired << 0, -10, 0, -100.0, 0.0, 180, 45.0;
   q_init_desired *= M_PI/180.0;
   

   // q_init_desired << 0, -0.6, 0, 0.8, 0, M_PI, M_PI_4;
   //q_init_desired << 0, 0.2, 0, -1.5, 0, M_PI, M_PI_4;
   
//    q_init_desired << 0, -0.6, 0, 0.8, 0, M_PI, M_PI_4;
   joint_task->_desired_position = q_init_desired;
 
   // gripper task containers
   VectorXd gripper_command_torques(2);
   VectorXd q_gripper(2), dq_gripper(2);
   VectorXd q_gripper_desired(2);
   //q_gripper_desired.setZero();
   q_gripper_desired << -0.1, 0.1;
   double kp_gripper = 400;
   double kv_gripper = 40;
 
    VectorXd coriolis = VectorXd::Zero(dof);

   // setup redis callback
   redis_client.createReadCallback(0);
   redis_client.createWriteCallback(0);
 
   // add to read callback
   redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
   redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
   redis_client.addEigenToReadCallback(0, GRIPPER_JOINT_ANGLES_KEY, q_gripper);
   redis_client.addEigenToReadCallback(0, GRIPPER_JOINT_VELOCITIES_KEY, dq_gripper);
 
   // add to write callback
   redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
   redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
 
   // create a timer
   LoopTimer timer;
   timer.initializeTimer();
   timer.setLoopFrequency(1000);
   double start_time = timer.elapsedTime(); //secs
   bool fTimerDidSleep = true;
   //LoopTimer timer2;
   //double start_time2;
 
   unsigned long long counter = 0;
   unsigned long long counter2 = 0;
 
   runloop = true;

    // getting plots pt 1
    // read terminal input, i.e. ./hw3 51
	int controller_number = 1;

   string filename;
	if(controller_number == 1)
		filename = "../../panda_gripper_example/data_files/question_1.txt";

	ofstream data_file;
	data_file.open(filename);

   while (runloop) {

       // wait for next scheduled loop
       timer.waitForNextLoop();
       double time = timer.elapsedTime() - start_time;
 
       // execute redis read callback
       redis_client.executeReadCallback(0);
 
       robot->position(x, "link7", pos_in_ee_link);
       robot->linearVelocity(dx, "link7", pos_in_ee_link);
 
       // update model
       robot->updateModel();
       robot->coriolisForce(coriolis);
 
       if (state == PREGRAB) {
           // update task model and set hierarchy
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
 
           if (x(0) > 0.52 && x(2) < 0.84 && dx(2) < 0.001){
 
            //cout << "lol" << endl;
			state = GRAB;
			joint_task->reInitializeTask();
            posori_task->reInitializeTask();
            robot->rotation(R_init,control_link);
			// q_gripper_desired << -0.02, 0.02;
           }
           // 0.6 0.0 0.8 **** position
       }
 
       if (state == BEGINNING) {
           double time2 = counter2*dt;

           posori_task->_use_interpolation_flag = false;
           posori_task->_use_velocity_saturation_flag = false;
 
           if (counter2 == 0){
			   MatrixXd A(10, 10);
			   A << 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,1,2,4,8,16,0,0,0,0,0,0,0,0,0,0,1,2,4,8,16,0,1,4,12,32,0,0,0,0,0,0,0,0,0,0,0,1,4,12,32;
            //    A << 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,1,2,3,4,0,0,0,0,0,0,0,0,0,0,0,1,2,3,4;
               MatrixXd B(10,1);
               B << x(1),x(2),0,0,0,0,0,0.9,2,0.5; //last 4 y,z,dy,dz as of 5/21 5:19pm, ex1
            //    B << x(1),x(2),0,0,0,0,-0.1,0.9,1.11,0.35; //last 4 y,z,dy,dz as of 5/21 5:20pm, ex2 current
               //0;1.13;1.11;0.35];
               X_traj = A.inverse()*B;
               cout << x << endl;
           }

           if (counter2 <= 2000){
                X_plan(0) = 0;
                X_plan(1) = X_traj(0) + X_traj(1)*time2 + X_traj(2)*(time2*time2) + X_traj(3)*time2*time2*time2 + X_traj(4)*time2*time2*time2*time2; 
                X_plan(2) = X_traj(5) + X_traj(6)*time2 + X_traj(7)*(time2*time2) + X_traj(8)*time2*time2*time2 + X_traj(9)*time2*time2*time2*time2;	

                X_der(0) = 0;
                X_der(1) = X_traj(1) + 2*X_traj(2)*(time2) + 3*X_traj(3)*time2*time2 + 4*X_traj(4)*time2*time2*time2; 
                X_der(2) = X_traj(6) + 2*X_traj(7)*(time2) + 3*X_traj(8)*time2*time2 + 4*X_traj(9)*time2*time2*time2; 

                X_der2(0) = 0;
                X_der2(1) = 2*X_traj(2) + 6*X_traj(3)*time2 + 12*X_traj(4)*time2*time2; 
                X_der2(2) = 2*X_traj(7) + 6*X_traj(8)*time2 + 12*X_traj(9)*time2*time2; 
           } else {
               X_der2 << 0,0,0;
               X_plan = X_der*dt + X_plan;
            //    cout << "hello" << endl;
           }

            // double angle  = M_PI/6;
            if (X_der.norm() > 1){
                angle_throw = atan2(X_der(2),X_der(1));
            }

            //final orientation
            //get orientation at t = 2 seconds, and arctan , make that angle the orientation at middle or start??

            // cout << "time" << endl;
            // cout << time2 << endl;
            // cout << "angle_throw" << endl;
            // cout << angle_throw*(180/M_PI) << endl;
            // cout << "Xder" << endl;
            // cout << X_der << endl;
            // // cout << "Xder1" << endl;
            // // cout << X_der(1) << endl;
            // cout << "x" << endl;
            // cout << x << endl;
            // cout << "norm" << endl;
            // cout << X_der.norm() << endl;
           
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

            posori_task->_desired_position = Vector3d(X_plan(0), X_plan(1), X_plan(2));
            posori_task->_desired_velocity = Vector3d(X_der(0), X_der(1), X_der(2));
            posori_task->_desired_acceleration = Vector3d(X_der2(0), X_der2(1), X_der2(2));
            joint_task->_desired_position = q_zero; //joint task to all zeros

            posori_task->_desired_orientation = AngleAxisd(angle_throw, Vector3d(1, 0, 0)).toRotationMatrix()*AngleAxisd(-M_PI_4, Vector3d(0, 0, 1)).toRotationMatrix();
            //posori_task->_desired_orientation = AngleAxisd(angle, Vector3d(0, 1, 0)).toRotationMatrix();
            //posori_task->_desired_orientation = R_init * AngleAxisd(angle, Vector3d(0, -1, 0)).toRotationMatrix(); 

			posori_task->computeTorques(posori_task_torques);
           	joint_task->computeTorques(joint_task_torques);
           	command_torques.head(7) = posori_task_torques + joint_task_torques + coriolis.head(7);
 
           	gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           	command_torques.tail(2) = gripper_command_torques + coriolis.tail(2); 
            if (counter2 > 1900){
                q_gripper_desired << -0.1,0.1;
            }

			if (counter2 > 2100) {
                // cout << "time2" << endl;
                // cout << time2 << endl;
                // cout << "x" << endl;
                // cout << x << endl;
                // cout << "X_plan" << endl;
                // cout << X_plan << endl;
				joint_task->_desired_position.setZero();
				state = POSTURE;
			}

            if(counter2 % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << X_plan(0) << '\t' << X_plan(1) << '\t' << X_plan(2) << '\t';
                data_file << dx(0) << '\t' << dx(1) << '\t' << dx(2) << '\t';
				data_file << X_der(0) << '\t' << X_der(1) << '\t' << X_der(2) << '\n';
			}
			counter2++;
       }
       if (state == POSTURE) {
           // update task model and set hierarchy
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;

       } else if (state == MOTION) {
           q_init_desired << 0, 0, 0, 0, 0, 0, 0;
           joint_task->_desired_position = q_init_desired;
		   
		   // update task model and set hierarchy
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
        //    cout << x << endl;
		   if (robot->_q(3) == 0.8 && robot->_dq(3) == 0){
				joint_task->reInitializeTask();
				posori_task->reInitializeTask();
            	robot->rotation(R_init,control_link);

				state = LAUNCH1;
			}
       } else if (state == GRAB){
           //call the following 4 lines every time you switch tasks
           N_prec.setIdentity();
           posori_task->updateTaskModel(N_prec);
           N_prec = posori_task->_N;
           joint_task->updateTaskModel(N_prec);
           //joint_task->_desired_position = q_init_desired;
 
        //    posori_task->_desired_position = Vector3d(0.6, 0, 0.73); //gets us to dart
            posori_task->_desired_position = Vector3d(0.6, 0, 0.73); //gets us to dart
           posori_task->_desired_orientation = R_init* AngleAxisd(0, Vector3d(0, 1, 0)).toRotationMatrix(); //perpendicular to dart
           

        //    cout << "norm" << endl;
        //     cout << (x - Vector3d(0.45, 0, 0.73)).norm() << endl;

            // now grab
            //Mikael said to create new task for grabbing
		   if ((x - Vector3d(0.6, 0, 0.73)).norm() <= 0.08){
			   q_gripper_desired << -0.019, 0.019;
               //cout << (q_gripper-q_gripper_desired).norm() << endl;
			   if ((q_gripper - q_gripper_desired).norm() < 0.004){
				   state = POSTGRAB;
				   joint_task->reInitializeTask();
				   posori_task->reInitializeTask();
			   }
		   }

		   posori_task->computeTorques(posori_task_torques);
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = posori_task_torques + joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
          
       } else if (state == POSTGRAB){
           //is postgrab needed? unsure
		   N_prec.setIdentity();
           posori_task->updateTaskModel(N_prec);
           N_prec = posori_task->_N;
           joint_task->updateTaskModel(N_prec);
           //joint_task->_desired_position = q_init_desired;
 
           posori_task->_desired_position = Vector3d(0.3, 0, 1);
        //    q_init_desired << -M_PI_2, M_PI_2, 0, 0, 0, M_PI, M_PI_4;
        //    joint_task->_desired_position = q_init_desired;
           // posori_task->_desired_orientation = R_init * AngleAxisd(angle, Vector3d(0, 1, 0)).toRotationMatrix();  //william
 
           // posori_task->_desired_orientation = AngleAxisd(angle, Vector3d(0, 1, 0)).toRotationMatrix();
		   posori_task->computeTorques(posori_task_torques);
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = posori_task_torques + joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
		   if (x(2) > 0.99){
			   cout << robot->_q << endl;
			   //state = PRELAUNCH;
               state = ORIENTATE;
			   joint_task->reInitializeTask();
			   posori_task->reInitializeTask();
               init_desired = robot->_q; //save joint orientation
		   }
	   } else if (state == PRELAUNCH){
           //moves robot into prelaunch position right before trajectory
            // don't know if this is optimal, have been playing around based pn TA consulting

           //q_init_desired << -M_PI_2, M_PI_2, 0, M_PI_2, 0, M_PI, M_PI_4; //-1.5,1.5,0,1.5,0,3,0.75 
           //q_init_desired << -M_PI_2, M_PI_2, 0, M_PI_2, 0, M_PI, M_PI_4; //trying midrange
    
            // q_init_desired << 90, -30, 0, 45, 0, 180, 45;
            q_init_desired << 90, -90, 0, 0, 0, 180, 45;
           q_init_desired *= (M_PI/180);
		   joint_task->_desired_position = q_init_desired;
		   N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
		   joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
		   // 0.000184859 -0.866935 0.360262
		   if ((q_init_desired-robot->_q).norm() < 0.01){
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();
               state = LAUNCH1;

                // //mikael added today 5/24
                // posori_task->_use_velocity_saturation_flag = true; //this does nothing
                // posori_task->_linear_saturation_velocity = 1000.0;
                // posori_task->_angular_saturation_velocity = M_PI_2;

               robot->rotation(R_init,control_link);

                angle_throw = acos(R_init.col(2).dot(Vector3d::UnitZ())); //current angle with x axis of world
		   }
	   } else if (state == LAUNCH1){
           robot->position(x, "link7", pos_in_ee_link);
            robot->linearVelocity(dx, "link7", pos_in_ee_link);
           joint_task->_use_interpolation_flag = false;
            joint_task->_use_velocity_saturation_flag = false;
           
           N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);

           q_init_desired << 90, 0, 0, 0, 0, 180, 45;
           q_init_desired *= (M_PI/180);
		   joint_task->_desired_position = q_init_desired;
 
           // compute torques
           joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;
           cout << (q_init_desired-robot->_q).norm() << endl;

           cout << "time" << endl;
            cout << time << endl;
            cout << "x" << endl;
            cout << x << endl;
            cout << "dx" << endl;
            cout << dx << endl;

           if (time > 14.7){
            cout << "lol" << endl;
            q_gripper_desired << -0.1,0.1;
			// state = POSTURE;
			// joint_task->reInitializeTask();
            // posori_task->reInitializeTask();
           }
           // 0.6 0.0 0.8 **** position
       } else if (state == ORIENTATE){
           q_init_desired = init_desired;
           q_init_desired(0) = M_PI_2; 
           
		   joint_task->_desired_position = q_init_desired;
		   N_prec.setIdentity();
           joint_task->updateTaskModel(N_prec);
 
           // compute torques
		   joint_task->computeTorques(joint_task_torques);
           command_torques.head(7) = joint_task_torques;
 
           gripper_command_torques = - kp_gripper * (q_gripper - q_gripper_desired) - kv_gripper * dq_gripper;
           command_torques.tail(2) = gripper_command_torques;

        //    cout << (q_init_desired-robot->_q).norm() << endl;
		   
		   if ((q_init_desired-robot->_q).norm() < 0.0048){
               joint_task->_saturation_velocity = (M_PI/10)*VectorXd::Ones(9); //slow down joint task velocity to keep dart still/perpendicular
               state = PRELAUNCH;
               joint_task->reInitializeTask();
               posori_task->reInitializeTask();   
               robot->rotation(R_init,control_link);
		   }
       }
 
 
 
       // execute redis write callback
       redis_client.executeWriteCallback(0);  
 
       counter++;
   }

   data_file.close();
 
   double end_time = timer.elapsedTime();
   std::cout << "\n";
   std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
   std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
   std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
 
   redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating
 
   return 0;
}