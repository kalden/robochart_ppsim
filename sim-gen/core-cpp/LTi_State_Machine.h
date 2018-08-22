#ifndef ROBOCALC_STATEMACHINES_LTI_STATE_MACHINE_H_
#define ROBOCALC_STATEMACHINES_LTI_STATE_MACHINE_H_

#include "RoboCalcAPI/State.h"
#include <cstdlib>
#include "LTiStep.h"
#include "LTiController.h"
#include "Timer.h"
#include "Functions.h"
#include "DataTypes.h"
#include <assert.h>

#define SM_DEBUG

class LTi_State_Machine: public robochart::StateMachine
{
	public:
		std::shared_ptr<robochart::Timer> T;
		std::shared_ptr<LTiStep> R_LTiStep;
		std::shared_ptr<LTiController> C_LTiController;
	public:
		bool move_finished;
		bool adhered;
		int contactedCell_ID;
		double probabilityOfAdhesion;
		double chemokine_response_probability;
		double distanceToMove;
		double distanceMoved;
		double movement_interval;
	public:
		double pJunctionValue;
	public:
		LTi_State_Machine(
				std::shared_ptr<LTiStep> R_LTiStep, 
				std::shared_ptr<LTiController> C_LTiController);
		~LTi_State_Machine();
		int Initial();
		virtual void Execute();

	public:
		class i0 : public robochart::State 
		{
			public:
				i0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("i0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
				{
				}
			private:
				std::shared_ptr<LTiStep> R_LTiStep;
				std::shared_ptr<LTiController> C_LTiController;
				std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
		};
		class Chemotactic : public robochart::State 
		{
			public:
				Chemotactic(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Chemotactic"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
				{
						// instantiate states && add substates of machine
						std::shared_ptr<i0> Chemotactic_i0 = std::make_shared<i0>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Chemotactic_i0);
						std::shared_ptr<Motile> Chemotactic_Motile = std::make_shared<Motile>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Chemotactic_Motile);
						std::shared_ptr<Disassociated> Chemotactic_Disassociated = std::make_shared<Disassociated>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Chemotactic_Disassociated);
						std::shared_ptr<Associated> Chemotactic_Associated = std::make_shared<Associated>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Chemotactic_Associated);
						std::shared_ptr<p0> Chemotactic_p0 = std::make_shared<p0>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Chemotactic_p0);
					
						std::shared_ptr<t0> Chemotactic_t0 = std::make_shared<t0>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Chemotactic_Motile, Chemotactic_p0);
						Chemotactic_Motile->transitions.push_back(Chemotactic_t0);
						std::shared_ptr<t1> Chemotactic_t1 = std::make_shared<t1>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Chemotactic_p0, Chemotactic_Disassociated);
						Chemotactic_p0->transitions.push_back(Chemotactic_t1);
						std::shared_ptr<t2> Chemotactic_t2 = std::make_shared<t2>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Chemotactic_p0, Chemotactic_Associated);
						Chemotactic_p0->transitions.push_back(Chemotactic_t2);
						std::shared_ptr<t7> Chemotactic_t7 = std::make_shared<t7>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Chemotactic_Motile, Chemotactic_Motile);
						Chemotactic_Motile->transitions.push_back(Chemotactic_t7);
						std::shared_ptr<t3> Chemotactic_t3 = std::make_shared<t3>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Chemotactic_i0, Chemotactic_Motile);
						Chemotactic_i0->transitions.push_back(Chemotactic_t3);
				}
				void Entry()
				{
					S_LTi_State_Machine->distanceToMove = R_LTiStep->cellSpeed;
					S_LTi_State_Machine->distanceMoved = 0;
					S_LTi_State_Machine->movement_interval = 0.1;
					R_LTiStep->angle_to_move = calculate_angle_from_direction(R_LTiStep->high_chemokine_grid_square);
					S_LTi_State_Machine->move_finished = false;
				}
				void Exit() 
				{
					R_LTiStep->update_tracking_stats(R_LTiStep->tracking, R_LTiStep->cell_id, R_LTiStep->LTi_loc, S_LTi_State_Machine->distanceMoved);
				}
			private:
				std::shared_ptr<LTiStep> R_LTiStep;
				std::shared_ptr<LTiController> C_LTiController;
				std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
			public:
				class Motile : public robochart::State 
				{
					public:
						Motile(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Motile"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							if (S_LTi_State_Machine->move_finished == false) 
							{
								if (S_LTi_State_Machine->movement_interval > S_LTi_State_Machine->distanceToMove) 
								{
									S_LTi_State_Machine->movement_interval = S_LTi_State_Machine->distanceToMove;
								}
								R_LTiStep->LTi_loc = calculateNewPosition(R_LTiStep->LTi_loc, R_LTiStep->angle_to_move, S_LTi_State_Machine->movement_interval);
								S_LTi_State_Machine->distanceToMove = S_LTi_State_Machine->distanceToMove - S_LTi_State_Machine->movement_interval;
								S_LTi_State_Machine->distanceMoved = S_LTi_State_Machine->distanceMoved + S_LTi_State_Machine->movement_interval;
								R_LTiStep->setPositionOnTract(R_LTiStep->cell_id, R_LTiStep->LTi_loc);
								if (S_LTi_State_Machine->distanceToMove == 0) 
								{
									S_LTi_State_Machine->move_finished = true;
								}
								if (check_valid_location_on_grid(R_LTiStep->LTi_loc) == true) 
								{
									S_LTi_State_Machine->contactedCell_ID = collision_check(R_LTiStep->LTi_loc, R_LTiStep->tracking, R_LTiStep->cell_id, std::string("LTi"));
									S_LTi_State_Machine->probabilityOfAdhesion = calculate_probability_adhesion(S_LTi_State_Machine->contactedCell_ID);
								}
								else 
								{
									R_LTiStep->stop_cell_on_schedule(R_LTiStep->cell_id);
									S_LTi_State_Machine->move_finished = true;
									S_LTi_State_Machine->probabilityOfAdhesion = 0;
								}
							}
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class Disassociated : public robochart::State 
				{
					public:
						Disassociated(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Disassociated"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							R_LTiStep->angle_to_move = avoidCellCollision(R_LTiStep->LTi_loc, S_LTi_State_Machine->distanceToMove, S_LTi_State_Machine->contactedCell_ID);
							R_LTiStep->LTi_loc = calculateNewPosition(R_LTiStep->LTi_loc, R_LTiStep->angle_to_move, S_LTi_State_Machine->distanceToMove);
							if (check_valid_location_on_grid(R_LTiStep->LTi_loc) == true) 
							{
								R_LTiStep->setPositionOnTract(R_LTiStep->cell_id, R_LTiStep->LTi_loc);
							}
							else 
							{
								R_LTiStep->stop_cell_on_schedule(R_LTiStep->cell_id);
							}
							S_LTi_State_Machine->distanceMoved = S_LTi_State_Machine->distanceMoved + S_LTi_State_Machine->distanceToMove;
							S_LTi_State_Machine->distanceToMove = 0;
							S_LTi_State_Machine->move_finished = true;
							S_LTi_State_Machine->contactedCell_ID = -1;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class Associated : public robochart::State 
				{
					public:
						Associated(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Associated"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							S_LTi_State_Machine->adhered = true;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class p0 : public robochart::State 
				{
					public:
						p0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("p0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry() 
						{
							S_LTi_State_Machine->pJunctionValue = (double)rand() / RAND_MAX;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class i0 : public robochart::State 
				{
					public:
						i0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("i0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
				~Chemotactic()
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
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Chemotactic_t0", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->contactedCell_ID != -1) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t0 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t0 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t7 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t7(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Chemotactic_t7", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->contactedCell_ID == -1 && S_LTi_State_Machine->move_finished == false) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t7 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t7 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t1 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t1(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Chemotactic_t1", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->pJunctionValue > (0) && S_LTi_State_Machine->pJunctionValue <= (0 + (1 - S_LTi_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t1 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t1 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion
			public:
				class t2 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t2(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Chemotactic_t2", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->pJunctionValue > (0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion) && S_LTi_State_Machine->pJunctionValue <= (0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion + (S_LTi_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t2 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Chemotactic_t2 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion + S_LTi_State_Machine->probabilityOfAdhesion
			//0
			public:
				class t3 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t3(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Chemotactic_t3", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
				};
		};
		class Non_Chemotactic : public robochart::State 
		{
			public:
				Non_Chemotactic(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Non_Chemotactic"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
				{
						// instantiate states && add substates of machine
						std::shared_ptr<i0> Non_Chemotactic_i0 = std::make_shared<i0>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Non_Chemotactic_i0);
						std::shared_ptr<Motile> Non_Chemotactic_Motile = std::make_shared<Motile>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Non_Chemotactic_Motile);
						std::shared_ptr<Disassociated> Non_Chemotactic_Disassociated = std::make_shared<Disassociated>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Non_Chemotactic_Disassociated);
						std::shared_ptr<Associated> Non_Chemotactic_Associated = std::make_shared<Associated>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Non_Chemotactic_Associated);
						std::shared_ptr<p0> Non_Chemotactic_p0 = std::make_shared<p0>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Non_Chemotactic_p0);
					
						std::shared_ptr<t0> Non_Chemotactic_t0 = std::make_shared<t0>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Non_Chemotactic_Motile, Non_Chemotactic_p0);
						Non_Chemotactic_Motile->transitions.push_back(Non_Chemotactic_t0);
						std::shared_ptr<t1> Non_Chemotactic_t1 = std::make_shared<t1>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Non_Chemotactic_p0, Non_Chemotactic_Disassociated);
						Non_Chemotactic_p0->transitions.push_back(Non_Chemotactic_t1);
						std::shared_ptr<t2> Non_Chemotactic_t2 = std::make_shared<t2>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Non_Chemotactic_p0, Non_Chemotactic_Associated);
						Non_Chemotactic_p0->transitions.push_back(Non_Chemotactic_t2);
						std::shared_ptr<t3> Non_Chemotactic_t3 = std::make_shared<t3>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Non_Chemotactic_Motile, Non_Chemotactic_Motile);
						Non_Chemotactic_Motile->transitions.push_back(Non_Chemotactic_t3);
						std::shared_ptr<t4> Non_Chemotactic_t4 = std::make_shared<t4>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Non_Chemotactic_i0, Non_Chemotactic_Motile);
						Non_Chemotactic_i0->transitions.push_back(Non_Chemotactic_t4);
				}
				void Entry()
				{
					S_LTi_State_Machine->distanceToMove = R_LTiStep->cellSpeed;
					S_LTi_State_Machine->distanceMoved = 0;
					R_LTiStep->angle_to_move = calculate_angle_from_direction(99);
					S_LTi_State_Machine->move_finished = false;
					S_LTi_State_Machine->movement_interval = 0.1;
				}
				void Exit() 
				{
					R_LTiStep->update_tracking_stats(R_LTiStep->tracking, R_LTiStep->cell_id, R_LTiStep->LTi_loc, S_LTi_State_Machine->distanceMoved);
				}
			private:
				std::shared_ptr<LTiStep> R_LTiStep;
				std::shared_ptr<LTiController> C_LTiController;
				std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
			public:
				class i0 : public robochart::State 
				{
					public:
						i0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("i0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
				~Non_Chemotactic()
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
						Motile(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Motile"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							if (S_LTi_State_Machine->move_finished == false) 
							{
								if (S_LTi_State_Machine->movement_interval > S_LTi_State_Machine->distanceToMove) 
								{
									S_LTi_State_Machine->movement_interval = S_LTi_State_Machine->distanceToMove;
								}
								R_LTiStep->LTi_loc = calculateNewPosition(R_LTiStep->LTi_loc, R_LTiStep->angle_to_move, S_LTi_State_Machine->movement_interval);
								S_LTi_State_Machine->distanceToMove = S_LTi_State_Machine->distanceToMove - S_LTi_State_Machine->movement_interval;
								S_LTi_State_Machine->distanceMoved = S_LTi_State_Machine->distanceMoved + S_LTi_State_Machine->movement_interval;
								R_LTiStep->setPositionOnTract(R_LTiStep->cell_id, R_LTiStep->LTi_loc);
								if (S_LTi_State_Machine->distanceToMove == 0) 
								{
									S_LTi_State_Machine->move_finished = true;
								}
								if (check_valid_location_on_grid(R_LTiStep->LTi_loc) == true) 
								{
									S_LTi_State_Machine->contactedCell_ID = collision_check(R_LTiStep->LTi_loc, R_LTiStep->tracking, R_LTiStep->cell_id, std::string("LTi"));
									S_LTi_State_Machine->probabilityOfAdhesion = calculate_probability_adhesion(S_LTi_State_Machine->contactedCell_ID);
								}
								else 
								{
									R_LTiStep->stop_cell_on_schedule(R_LTiStep->cell_id);
									S_LTi_State_Machine->move_finished = true;
									S_LTi_State_Machine->probabilityOfAdhesion = 0;
								}
							}
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class Disassociated : public robochart::State 
				{
					public:
						Disassociated(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Disassociated"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							R_LTiStep->angle_to_move = avoidCellCollision(R_LTiStep->LTi_loc, S_LTi_State_Machine->distanceToMove, S_LTi_State_Machine->contactedCell_ID);
							R_LTiStep->LTi_loc = calculateNewPosition(R_LTiStep->LTi_loc, R_LTiStep->angle_to_move, S_LTi_State_Machine->distanceToMove);
							if (check_valid_location_on_grid(R_LTiStep->LTi_loc) == true) 
							{
								R_LTiStep->setPositionOnTract(R_LTiStep->cell_id, R_LTiStep->LTi_loc);
							}
							else 
							{
								R_LTiStep->stop_cell_on_schedule(R_LTiStep->cell_id);
							}
							S_LTi_State_Machine->distanceMoved = S_LTi_State_Machine->distanceMoved + S_LTi_State_Machine->distanceToMove;
							S_LTi_State_Machine->distanceToMove = 0;
							S_LTi_State_Machine->move_finished = true;
							S_LTi_State_Machine->contactedCell_ID = -1;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class Associated : public robochart::State 
				{
					public:
						Associated(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Associated"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							S_LTi_State_Machine->adhered = true;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class p0 : public robochart::State 
				{
					public:
						p0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("p0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry() 
						{
							S_LTi_State_Machine->pJunctionValue = (double)rand() / RAND_MAX;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class t4 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t4(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Non_Chemotactic_t4", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
				};
			public:
				class t0 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Non_Chemotactic_t0", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->contactedCell_ID != -1) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t0 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t0 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t3 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t3(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Non_Chemotactic_t3", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->contactedCell_ID == -1 && S_LTi_State_Machine->move_finished == false) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t3 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t3 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t1 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t1(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Non_Chemotactic_t1", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->pJunctionValue > (0) && S_LTi_State_Machine->pJunctionValue <= (0 + (1 - S_LTi_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t1 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t1 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion
			public:
				class t2 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t2(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Non_Chemotactic_t2", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->pJunctionValue > (0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion) && S_LTi_State_Machine->pJunctionValue <= (0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion + (S_LTi_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t2 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Non_Chemotactic_t2 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion + S_LTi_State_Machine->probabilityOfAdhesion
			//0
		};
		class p0 : public robochart::State 
		{
			public:
				p0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("p0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
				{
				}
				void Entry() 
				{
					S_LTi_State_Machine->pJunctionValue = (double)rand() / RAND_MAX;
				}
			private:
				std::shared_ptr<LTiStep> R_LTiStep;
				std::shared_ptr<LTiController> C_LTiController;
				std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
		};
		class Adhesion_Response : public robochart::State 
		{
			public:
				Adhesion_Response(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Adhesion_Response"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
				{
						// instantiate states && add substates of machine
						std::shared_ptr<i0> Adhesion_Response_i0 = std::make_shared<i0>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Adhesion_Response_i0);
						std::shared_ptr<Adhered> Adhesion_Response_Adhered = std::make_shared<Adhered>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Adhesion_Response_Adhered);
						std::shared_ptr<Disassociated> Adhesion_Response_Disassociated = std::make_shared<Disassociated>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Adhesion_Response_Disassociated);
						std::shared_ptr<p0> Adhesion_Response_p0 = std::make_shared<p0>(R_LTiStep, C_LTiController, S_LTi_State_Machine);
						states.push_back(Adhesion_Response_p0);
					
						std::shared_ptr<t0> Adhesion_Response_t0 = std::make_shared<t0>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Adhesion_Response_Adhered, Adhesion_Response_p0);
						Adhesion_Response_Adhered->transitions.push_back(Adhesion_Response_t0);
						std::shared_ptr<t1> Adhesion_Response_t1 = std::make_shared<t1>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Adhesion_Response_p0, Adhesion_Response_Disassociated);
						Adhesion_Response_p0->transitions.push_back(Adhesion_Response_t1);
						std::shared_ptr<t2> Adhesion_Response_t2 = std::make_shared<t2>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Adhesion_Response_p0, Adhesion_Response_Adhered);
						Adhesion_Response_p0->transitions.push_back(Adhesion_Response_t2);
						std::shared_ptr<t3> Adhesion_Response_t3 = std::make_shared<t3>(R_LTiStep, C_LTiController, S_LTi_State_Machine, Adhesion_Response_i0, Adhesion_Response_Adhered);
						Adhesion_Response_i0->transitions.push_back(Adhesion_Response_t3);
				}
				void Entry()
				{
					S_LTi_State_Machine->distanceToMove = R_LTiStep->cellSpeed;
					S_LTi_State_Machine->distanceMoved = 0;
					R_LTiStep->angle_to_move = calculate_angle_from_direction(R_LTiStep->high_chemokine_grid_square);
					S_LTi_State_Machine->move_finished = false;
				}
				void Exit() 
				{
					R_LTiStep->update_tracking_stats(R_LTiStep->tracking, R_LTiStep->cell_id, R_LTiStep->LTi_loc, S_LTi_State_Machine->distanceMoved);
				}
			private:
				std::shared_ptr<LTiStep> R_LTiStep;
				std::shared_ptr<LTiController> C_LTiController;
				std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
			public:
				class Adhered : public robochart::State 
				{
					public:
						Adhered(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Adhered"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							if (S_LTi_State_Machine->move_finished == false) 
							{
								S_LTi_State_Machine->probabilityOfAdhesion = calculate_probability_adhesion(S_LTi_State_Machine->contactedCell_ID);
							}
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class Disassociated : public robochart::State 
				{
					public:
						Disassociated(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("Disassociated"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry()
						{
							R_LTiStep->angle_to_move = avoidCellCollision(R_LTiStep->LTi_loc, S_LTi_State_Machine->distanceToMove, S_LTi_State_Machine->contactedCell_ID);
							R_LTiStep->LTi_loc = calculateNewPosition(R_LTiStep->LTi_loc, R_LTiStep->angle_to_move, S_LTi_State_Machine->distanceToMove);
							if (check_valid_location_on_grid(R_LTiStep->LTi_loc) == true) 
							{
								R_LTiStep->setPositionOnTract(R_LTiStep->cell_id, R_LTiStep->LTi_loc);
							}
							else 
							{
								R_LTiStep->stop_cell_on_schedule(R_LTiStep->cell_id);
							}
							S_LTi_State_Machine->distanceMoved = S_LTi_State_Machine->distanceMoved + S_LTi_State_Machine->distanceToMove;
							S_LTi_State_Machine->distanceToMove = 0;
							S_LTi_State_Machine->move_finished = true;
							S_LTi_State_Machine->contactedCell_ID = -1;
							S_LTi_State_Machine->adhered = false;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class p0 : public robochart::State 
				{
					public:
						p0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("p0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
						void Entry() 
						{
							S_LTi_State_Machine->pJunctionValue = (double)rand() / RAND_MAX;
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				};
			public:
				class i0 : public robochart::State 
				{
					public:
						i0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine) : State("i0"), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine) 
						{
						}
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
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
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Adhesion_Response_t0", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->move_finished == false) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Adhesion_Response_t0 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Adhesion_Response_t0 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t1 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t1(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Adhesion_Response_t1", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->pJunctionValue > (0) && S_LTi_State_Machine->pJunctionValue <= (0 + (1 - S_LTi_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Adhesion_Response_t1 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Adhesion_Response_t1 is true\n");
								#endif
								return false;
							}
						}
				};
			
			//0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion
			public:
				class t2 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t2(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Adhesion_Response_t2", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
						bool Condition() {
							if (S_LTi_State_Machine->pJunctionValue > (0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion) && S_LTi_State_Machine->pJunctionValue <= (0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion + (S_LTi_State_Machine->probabilityOfAdhesion))) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Adhesion_Response_t2 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTi_State_Machine_Adhesion_Response_t2 is true\n");
								#endif
								return false;
							}
						}
						void Action() {
							S_LTi_State_Machine->move_finished = true;
						}
				};
			
			//0 + 1 - S_LTi_State_Machine->probabilityOfAdhesion + S_LTi_State_Machine->probabilityOfAdhesion
			//0
			public:
				class t3 : public robochart::Transition {
					private:
						std::shared_ptr<LTiStep> R_LTiStep;
						std::shared_ptr<LTiController> C_LTiController;
						std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
					public:
						t3(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTi_State_Machine_Adhesion_Response_t3", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
						{}
				};
		};

		public:
			class t0 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t0(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t0", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					void Action() {
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t1 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t1(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t1", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->contactedCell_ID == -1 && S_LTi_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t1 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t1 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						R_LTiStep->totalchemoLevels = measure_chemokine_expression(R_LTiStep->LTi_loc, R_LTiStep->chemomap);
						if (R_LTiStep->totalchemoLevels > 0) 
						{
							R_LTiStep->high_chemokine_grid_square = calculate_high_grid_square(R_LTiStep->chemomap);
						}
						else 
						{
							R_LTiStep->high_chemokine_grid_square = -1;
						}
						S_LTi_State_Machine->chemokine_response_probability = probability_responds_to_chemokine(R_LTiStep->LTi_loc, R_LTiStep->totalchemoLevels, R_LTiStep->chemomap);
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t6 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t6(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t6", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->contactedCell_ID != 1 && S_LTi_State_Machine->adhered == true && S_LTi_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t6 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t6 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t4 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t4(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t4", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->contactedCell_ID == -1 && S_LTi_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t4 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t4 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						R_LTiStep->totalchemoLevels = measure_chemokine_expression(R_LTiStep->LTi_loc, R_LTiStep->chemomap);
						if (R_LTiStep->totalchemoLevels > 0) 
						{
							R_LTiStep->high_chemokine_grid_square = calculate_high_grid_square(R_LTiStep->chemomap);
						}
						else 
						{
							R_LTiStep->high_chemokine_grid_square = -1;
						}
						S_LTi_State_Machine->chemokine_response_probability = probability_responds_to_chemokine(R_LTiStep->LTi_loc, R_LTiStep->totalchemoLevels, R_LTiStep->chemomap);
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t5 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t5(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t5", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->contactedCell_ID != -1 && S_LTi_State_Machine->adhered == true && S_LTi_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t5 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t5 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t2 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t2(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t2", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->pJunctionValue > (0) && S_LTi_State_Machine->pJunctionValue <= (0 + (1 - S_LTi_State_Machine->chemokine_response_probability))) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t2 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t2 is true\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		
		//0 + 1 - S_LTi_State_Machine->chemokine_response_probability
		public:
			class t3 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t3(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t3", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->pJunctionValue > (0 + 1 - S_LTi_State_Machine->chemokine_response_probability) && S_LTi_State_Machine->pJunctionValue <= (0 + 1 - S_LTi_State_Machine->chemokine_response_probability + (S_LTi_State_Machine->chemokine_response_probability))) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t3 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t3 is true\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		
		//0 + 1 - S_LTi_State_Machine->chemokine_response_probability + S_LTi_State_Machine->chemokine_response_probability
		//0
		public:
			class t11 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t11(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t11", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->contactedCell_ID != -1 && S_LTi_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t11 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t11 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t7 : public robochart::Transition {
				private:
					std::shared_ptr<LTiStep> R_LTiStep;
					std::shared_ptr<LTiController> C_LTiController;
					std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine;
				public:
					t7(std::shared_ptr<LTiStep> R_LTiStep, std::shared_ptr<LTiController> C_LTiController, std::shared_ptr<LTi_State_Machine> S_LTi_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTi_State_Machine_t7", src, tgt), R_LTiStep(R_LTiStep), C_LTiController(C_LTiController), S_LTi_State_Machine(S_LTi_State_Machine)
					{}
					bool Condition() {
						if (S_LTi_State_Machine->contactedCell_ID == -1 && S_LTi_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t7 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTi_State_Machine_t7 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						R_LTiStep->totalchemoLevels = measure_chemokine_expression(R_LTiStep->LTi_loc, R_LTiStep->chemomap);
						if (R_LTiStep->totalchemoLevels > 0) 
						{
							R_LTiStep->high_chemokine_grid_square = calculate_high_grid_square(R_LTiStep->chemomap);
						}
						else 
						{
							R_LTiStep->high_chemokine_grid_square = -1;
						}
						S_LTi_State_Machine->chemokine_response_probability = probability_responds_to_chemokine(R_LTiStep->LTi_loc, R_LTiStep->totalchemoLevels, R_LTiStep->chemomap);
						S_LTi_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
};

#endif
