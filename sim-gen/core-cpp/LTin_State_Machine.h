#ifndef ROBOCALC_STATEMACHINES_LTIN_STATE_MACHINE_H_
#define ROBOCALC_STATEMACHINES_LTIN_STATE_MACHINE_H_

#include "RoboCalcAPI/State.h"
#include <cstdlib>
#include "LTin_Step.h"
#include "LTin_Controller.h"
#include "Timer.h"
#include "Functions.h"
#include "DataTypes.h"
#include <assert.h>

#define SM_DEBUG

class LTin_State_Machine: public robochart::StateMachine
{
	public:
		std::shared_ptr<robochart::Timer> T;
		std::shared_ptr<LTin_Step> R_LTin_Step;
		std::shared_ptr<LTin_Controller> C_LTin_Controller;
	public:
		bool move_finished;
		bool adhered;
		int contactedCell_ID;
		double probabilityOfAdhesion;
		double distanceToMove;
		double distanceMoved;
		double movement_interval;
	public:
		double pJunctionValue;
	public:
		LTin_State_Machine(
				std::shared_ptr<LTin_Step> R_LTin_Step, 
				std::shared_ptr<LTin_Controller> C_LTin_Controller);
		~LTin_State_Machine();
		int Initial();
		virtual void Execute();

	public:
		class Moving : public robochart::State 
		{
			public:
				Moving(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("Moving"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
				{
						// instantiate states && add substates of machine
						std::shared_ptr<i0> Moving_i0 = std::make_shared<i0>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Moving_i0);
						std::shared_ptr<Motile> Moving_Motile = std::make_shared<Motile>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Moving_Motile);
						std::shared_ptr<Disassociated> Moving_Disassociated = std::make_shared<Disassociated>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Moving_Disassociated);
						std::shared_ptr<Associated> Moving_Associated = std::make_shared<Associated>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Moving_Associated);
						std::shared_ptr<p0> Moving_p0 = std::make_shared<p0>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Moving_p0);
					
						std::shared_ptr<t0> Moving_t0 = std::make_shared<t0>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Moving_i0, Moving_Motile);
						Moving_i0->transitions.push_back(Moving_t0);
						std::shared_ptr<t1> Moving_t1 = std::make_shared<t1>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Moving_p0, Moving_Disassociated);
						Moving_p0->transitions.push_back(Moving_t1);
						std::shared_ptr<t2> Moving_t2 = std::make_shared<t2>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Moving_Motile, Moving_p0);
						Moving_Motile->transitions.push_back(Moving_t2);
						std::shared_ptr<t3> Moving_t3 = std::make_shared<t3>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Moving_p0, Moving_Associated);
						Moving_p0->transitions.push_back(Moving_t3);
						std::shared_ptr<t4> Moving_t4 = std::make_shared<t4>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Moving_Motile, Moving_Motile);
						Moving_Motile->transitions.push_back(Moving_t4);
				}
				void Entry()
				{
					S_LTin_State_Machine->distanceToMove = R_LTin_Step->cellSpeed;
					S_LTin_State_Machine->distanceMoved = 0;
					S_LTin_State_Machine->movement_interval = 0.1;
					R_LTin_Step->angle_to_move = calculate_angle_from_direction(99);
					S_LTin_State_Machine->move_finished = false;
				}
				void Exit() 
				{
					R_LTin_Step->update_tracking_stats(R_LTin_Step->tracking, R_LTin_Step->cell_id, R_LTin_Step->LTin_loc, S_LTin_State_Machine->distanceMoved);
				}
			private:
				std::shared_ptr<LTin_Step> R_LTin_Step;
				std::shared_ptr<LTin_Controller> C_LTin_Controller;
				std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
			public:
				class i0 : public robochart::State 
				{
					public:
						i0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("i0"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
				~Moving()
				{
				}
				int Initial()
				{
					return 0;
				}
				void Execute()
				{
					State::Execute();
				}
			public:
				class Motile : public robochart::State 
				{
					public:
						Motile(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("Motile"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
						void Entry()
						{
							if (S_LTin_State_Machine->move_finished == false) 
							{
								if (S_LTin_State_Machine->movement_interval > S_LTin_State_Machine->distanceToMove) 
								{
									S_LTin_State_Machine->movement_interval = S_LTin_State_Machine->distanceToMove;
								}
								R_LTin_Step->LTin_loc = calculateNewPosition(R_LTin_Step->LTin_loc, R_LTin_Step->angle_to_move, S_LTin_State_Machine->movement_interval);
								S_LTin_State_Machine->distanceToMove = S_LTin_State_Machine->distanceToMove - S_LTin_State_Machine->movement_interval;
								S_LTin_State_Machine->distanceMoved = S_LTin_State_Machine->distanceMoved + S_LTin_State_Machine->movement_interval;
								R_LTin_Step->setPositionOnTract(R_LTin_Step->cell_id, R_LTin_Step->LTin_loc);
								if (S_LTin_State_Machine->distanceToMove == 0) 
								{
									S_LTin_State_Machine->move_finished = true;
								}
								if (check_valid_location_on_grid(R_LTin_Step->LTin_loc) == true) 
								{
									S_LTin_State_Machine->contactedCell_ID = collision_check(R_LTin_Step->LTin_loc, R_LTin_Step->tracking, R_LTin_Step->cell_id, std::string("LTin"));
									S_LTin_State_Machine->probabilityOfAdhesion = calculate_probability_adhesion(S_LTin_State_Machine->contactedCell_ID);
								}
								else 
								{
									R_LTin_Step->stop_cell_on_schedule(R_LTin_Step->cell_id);
									S_LTin_State_Machine->move_finished = true;
									S_LTin_State_Machine->probabilityOfAdhesion = 0;
								}
							}
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
			public:
				class Disassociated : public robochart::State 
				{
					public:
						Disassociated(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("Disassociated"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
						void Entry()
						{
							R_LTin_Step->angle_to_move = avoidCellCollision(R_LTin_Step->LTin_loc, S_LTin_State_Machine->distanceToMove, S_LTin_State_Machine->contactedCell_ID);
							R_LTin_Step->LTin_loc = calculateNewPosition(R_LTin_Step->LTin_loc, R_LTin_Step->angle_to_move, S_LTin_State_Machine->distanceToMove);
							if (check_valid_location_on_grid(R_LTin_Step->LTin_loc) == true) 
							{
								R_LTin_Step->setPositionOnTract(R_LTin_Step->cell_id, R_LTin_Step->LTin_loc);
							}
							else 
							{
								R_LTin_Step->stop_cell_on_schedule(R_LTin_Step->cell_id);
							}
							S_LTin_State_Machine->distanceMoved = S_LTin_State_Machine->distanceMoved + S_LTin_State_Machine->distanceToMove;
							S_LTin_State_Machine->distanceToMove = 0;
							S_LTin_State_Machine->move_finished = true;
							S_LTin_State_Machine->contactedCell_ID = -1;
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
			public:
				class Associated : public robochart::State 
				{
					public:
						Associated(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("Associated"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
						void Entry()
						{
							S_LTin_State_Machine->adhered = true;
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
			public:
				class p0 : public robochart::State 
				{
					public:
						p0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("p0"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
						void Entry() 
						{
							S_LTin_State_Machine->pJunctionValue = (double)rand() / RAND_MAX;
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
			public:
				class t0 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Moving_t0", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
				};
			public:
				class t2 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t2(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Moving_t2", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
						bool Condition() {
							if (S_LTin_State_Machine->contactedCell_ID != -1) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t2 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t2 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t4 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t4(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Moving_t4", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
						bool Condition() {
							if (S_LTin_State_Machine->contactedCell_ID == -1 && S_LTin_State_Machine->move_finished == false) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t4 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t4 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t1 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t1(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Moving_t1", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
						bool Condition() {
							if (S_LTin_State_Machine->pJunctionValue > (0) && S_LTin_State_Machine->pJunctionValue <= (0 + (1 - S_LTin_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t1 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t1 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion
			public:
				class t3 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t3(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Moving_t3", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
						bool Condition() {
							if (S_LTin_State_Machine->pJunctionValue > (0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion) && S_LTin_State_Machine->pJunctionValue <= (0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion + (S_LTin_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t3 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Moving_t3 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion + S_LTin_State_Machine->probabilityOfAdhesion
			//0
		};
		class i0 : public robochart::State 
		{
			public:
				i0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("i0"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
				{
				}
			private:
				std::shared_ptr<LTin_Step> R_LTin_Step;
				std::shared_ptr<LTin_Controller> C_LTin_Controller;
				std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
		};
		class Adhesion_Response : public robochart::State 
		{
			public:
				Adhesion_Response(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("Adhesion_Response"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
				{
						// instantiate states && add substates of machine
						std::shared_ptr<i0> Adhesion_Response_i0 = std::make_shared<i0>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Adhesion_Response_i0);
						std::shared_ptr<Adhered> Adhesion_Response_Adhered = std::make_shared<Adhered>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Adhesion_Response_Adhered);
						std::shared_ptr<Disassociated> Adhesion_Response_Disassociated = std::make_shared<Disassociated>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Adhesion_Response_Disassociated);
						std::shared_ptr<p0> Adhesion_Response_p0 = std::make_shared<p0>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine);
						states.push_back(Adhesion_Response_p0);
					
						std::shared_ptr<t0> Adhesion_Response_t0 = std::make_shared<t0>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Adhesion_Response_Adhered, Adhesion_Response_p0);
						Adhesion_Response_Adhered->transitions.push_back(Adhesion_Response_t0);
						std::shared_ptr<t1> Adhesion_Response_t1 = std::make_shared<t1>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Adhesion_Response_p0, Adhesion_Response_Disassociated);
						Adhesion_Response_p0->transitions.push_back(Adhesion_Response_t1);
						std::shared_ptr<t2> Adhesion_Response_t2 = std::make_shared<t2>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Adhesion_Response_p0, Adhesion_Response_Adhered);
						Adhesion_Response_p0->transitions.push_back(Adhesion_Response_t2);
						std::shared_ptr<t3> Adhesion_Response_t3 = std::make_shared<t3>(R_LTin_Step, C_LTin_Controller, S_LTin_State_Machine, Adhesion_Response_i0, Adhesion_Response_Adhered);
						Adhesion_Response_i0->transitions.push_back(Adhesion_Response_t3);
				}
				void Entry()
				{
					S_LTin_State_Machine->distanceToMove = R_LTin_Step->cellSpeed;
					S_LTin_State_Machine->distanceMoved = 0;
					R_LTin_Step->angle_to_move = calculate_angle_from_direction(99);
					S_LTin_State_Machine->move_finished = false;
				}
				void Exit() 
				{
					R_LTin_Step->update_tracking_stats(R_LTin_Step->tracking, R_LTin_Step->cell_id, R_LTin_Step->LTin_loc, S_LTin_State_Machine->distanceMoved);
				}
			private:
				std::shared_ptr<LTin_Step> R_LTin_Step;
				std::shared_ptr<LTin_Controller> C_LTin_Controller;
				std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
			public:
				class Adhered : public robochart::State 
				{
					public:
						Adhered(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("Adhered"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
						void Entry()
						{
							if (S_LTin_State_Machine->move_finished == false) 
							{
								S_LTin_State_Machine->probabilityOfAdhesion = calculate_probability_adhesion(S_LTin_State_Machine->contactedCell_ID);
							}
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
			public:
				class Disassociated : public robochart::State 
				{
					public:
						Disassociated(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("Disassociated"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
						void Entry()
						{
							R_LTin_Step->angle_to_move = avoidCellCollision(R_LTin_Step->LTin_loc, S_LTin_State_Machine->distanceToMove, S_LTin_State_Machine->contactedCell_ID);
							R_LTin_Step->LTin_loc = calculateNewPosition(R_LTin_Step->LTin_loc, R_LTin_Step->angle_to_move, S_LTin_State_Machine->distanceToMove);
							if (check_valid_location_on_grid(R_LTin_Step->LTin_loc) == true) 
							{
								R_LTin_Step->setPositionOnTract(R_LTin_Step->cell_id, R_LTin_Step->LTin_loc);
							}
							else 
							{
								R_LTin_Step->stop_cell_on_schedule(R_LTin_Step->cell_id);
							}
							S_LTin_State_Machine->distanceMoved = S_LTin_State_Machine->distanceMoved + S_LTin_State_Machine->distanceToMove;
							S_LTin_State_Machine->distanceToMove = 0;
							S_LTin_State_Machine->move_finished = true;
							S_LTin_State_Machine->contactedCell_ID = -1;
							S_LTin_State_Machine->adhered = false;
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
			public:
				class p0 : public robochart::State 
				{
					public:
						p0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("p0"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
						void Entry() 
						{
							S_LTin_State_Machine->pJunctionValue = (double)rand() / RAND_MAX;
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
			public:
				class i0 : public robochart::State 
				{
					public:
						i0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine) : State("i0"), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine) 
						{
						}
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				};
				~Adhesion_Response()
				{
				}
				int Initial()
				{
					return 0;
				}
				void Execute()
				{
					State::Execute();
				}
			public:
				class t0 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Adhesion_Response_t0", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
						bool Condition() {
							if (S_LTin_State_Machine->move_finished == false) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Adhesion_Response_t0 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Adhesion_Response_t0 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t1 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t1(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Adhesion_Response_t1", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
						bool Condition() {
							if (S_LTin_State_Machine->pJunctionValue > (0) && S_LTin_State_Machine->pJunctionValue <= (0 + (1 - S_LTin_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Adhesion_Response_t1 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Adhesion_Response_t1 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion
			public:
				class t2 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t2(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Adhesion_Response_t2", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
						bool Condition() {
							if (S_LTin_State_Machine->pJunctionValue > (0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion) && S_LTin_State_Machine->pJunctionValue <= (0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion + (S_LTin_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Adhesion_Response_t2 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTin_State_Machine_Adhesion_Response_t2 is true\n");
								#endif
								return false;
							}
						}
						void Action() {
							S_LTin_State_Machine->move_finished = true;
						}
				};
			
			//0 + 1 - S_LTin_State_Machine->probabilityOfAdhesion + S_LTin_State_Machine->probabilityOfAdhesion
			//0
			public:
				class t3 : public robochart::Transition {
					private:
						std::shared_ptr<LTin_Step> R_LTin_Step;
						std::shared_ptr<LTin_Controller> C_LTin_Controller;
						std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
					public:
						t3(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTin_State_Machine_Adhesion_Response_t3", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
						{}
				};
		};

		public:
			class t1 : public robochart::Transition {
				private:
					std::shared_ptr<LTin_Step> R_LTin_Step;
					std::shared_ptr<LTin_Controller> C_LTin_Controller;
					std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				public:
					t1(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTin_State_Machine_t1", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
					{}
					bool Condition() {
						if (S_LTin_State_Machine->contactedCell_ID == -1 && S_LTin_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t1 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t1 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTin_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t3 : public robochart::Transition {
				private:
					std::shared_ptr<LTin_Step> R_LTin_Step;
					std::shared_ptr<LTin_Controller> C_LTin_Controller;
					std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				public:
					t3(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTin_State_Machine_t3", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
					{}
					bool Condition() {
						if (S_LTin_State_Machine->contactedCell_ID != -1 && S_LTin_State_Machine->adhered == true && S_LTin_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t3 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t3 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTin_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t0 : public robochart::Transition {
				private:
					std::shared_ptr<LTin_Step> R_LTin_Step;
					std::shared_ptr<LTin_Controller> C_LTin_Controller;
					std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				public:
					t0(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTin_State_Machine_t0", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
					{}
					void Action() {
						S_LTin_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t2 : public robochart::Transition {
				private:
					std::shared_ptr<LTin_Step> R_LTin_Step;
					std::shared_ptr<LTin_Controller> C_LTin_Controller;
					std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				public:
					t2(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTin_State_Machine_t2", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
					{}
					bool Condition() {
						if (S_LTin_State_Machine->contactedCell_ID != -1 && S_LTin_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t2 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t2 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTin_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t4 : public robochart::Transition {
				private:
					std::shared_ptr<LTin_Step> R_LTin_Step;
					std::shared_ptr<LTin_Controller> C_LTin_Controller;
					std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine;
				public:
					t4(std::shared_ptr<LTin_Step> R_LTin_Step, std::shared_ptr<LTin_Controller> C_LTin_Controller, std::shared_ptr<LTin_State_Machine> S_LTin_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTin_State_Machine_t4", src, tgt), R_LTin_Step(R_LTin_Step), C_LTin_Controller(C_LTin_Controller), S_LTin_State_Machine(S_LTin_State_Machine)
					{}
					bool Condition() {
						if (S_LTin_State_Machine->contactedCell_ID == -1 && S_LTin_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t4 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTin_State_Machine_t4 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTin_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
};

#endif
