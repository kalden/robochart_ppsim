#ifndef ROBOCALC_STATEMACHINES_LTO_STATE_MACHINE_H_
#define ROBOCALC_STATEMACHINES_LTO_STATE_MACHINE_H_

#include "RoboCalcAPI/State.h"
#include <cstdlib>
#include "LTo_Step.h"
#include "LTo_Controller.h"
#include "Timer.h"
#include "Functions.h"
#include "DataTypes.h"
#include <assert.h>

#define SM_DEBUG

class LTo_State_Machine: public robochart::StateMachine
{
	public:
		std::shared_ptr<robochart::Timer> T;
		std::shared_ptr<LTo_Step> R_LTo_Step;
		std::shared_ptr<LTo_Controller> C_LTo_Controller;
	public:
		const double retLigandProbability;
		int cell_division_clock;
		bool maxExpressionReached;
	public:
		double pJunctionValue;
	public:
		LTo_State_Machine(
				std::shared_ptr<LTo_Step> R_LTo_Step, 
				std::shared_ptr<LTo_Controller> C_LTo_Controller);
		~LTo_State_Machine();
		int Initial();
		virtual void Execute();

	public:
		class i0 : public robochart::State 
		{
			public:
				i0(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("i0"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
				{
				}
			private:
				std::shared_ptr<LTo_Step> R_LTo_Step;
				std::shared_ptr<LTo_Controller> C_LTo_Controller;
				std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
		};
		class Not_Expressing_RET_Ligand : public robochart::State 
		{
			public:
				Not_Expressing_RET_Ligand(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Not_Expressing_RET_Ligand"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
				{
				}
				void Entry()
				{
					R_LTo_Step->expressingRET = false;
				}
			private:
				std::shared_ptr<LTo_Step> R_LTo_Step;
				std::shared_ptr<LTo_Controller> C_LTo_Controller;
				std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
		};
		class Expressing_RET_Ligand : public robochart::State 
		{
			public:
				Expressing_RET_Ligand(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Expressing_RET_Ligand"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
				{
				}
				void Entry()
				{
					R_LTo_Step->expressingRET = true;
				}
			private:
				std::shared_ptr<LTo_Step> R_LTo_Step;
				std::shared_ptr<LTo_Controller> C_LTo_Controller;
				std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
		};
		class p0 : public robochart::State 
		{
			public:
				p0(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("p0"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
				{
				}
				void Entry() 
				{
					S_LTo_State_Machine->pJunctionValue = (double)rand() / RAND_MAX;
				}
			private:
				std::shared_ptr<LTo_Step> R_LTo_Step;
				std::shared_ptr<LTo_Controller> C_LTo_Controller;
				std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
		};
		class Upregulate_Adhesion_Molecules : public robochart::State 
		{
			public:
				Upregulate_Adhesion_Molecules(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Upregulate_Adhesion_Molecules"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
				{
				}
				void Entry()
				{
					R_LTo_Step->new_LTin_binding = false;
					if (R_LTo_Step->lTinContactStateChangeTimePoint == -1) 
					{
						R_LTo_Step->lTinContactStateChangeTimePoint = set_ltin_contact_time();
					}
					if (R_LTo_Step->adhesionExpressed < R_LTo_Step->maxVCAMProbability) 
					{
						R_LTo_Step->adhesionExpressed = R_LTo_Step->adhesionExpressed + R_LTo_Step->adhesionIncrement;
					}
				}
			private:
				std::shared_ptr<LTo_Step> R_LTo_Step;
				std::shared_ptr<LTo_Controller> C_LTo_Controller;
				std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
		};
		class Expressing_Chemokines : public robochart::State 
		{
			public:
				Expressing_Chemokines(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Expressing_Chemokines"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
				{
						// instantiate states && add substates of machine
						std::shared_ptr<i0> Expressing_Chemokines_i0 = std::make_shared<i0>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine);
						states.push_back(Expressing_Chemokines_i0);
						std::shared_ptr<Dividing> Expressing_Chemokines_Dividing = std::make_shared<Dividing>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine);
						states.push_back(Expressing_Chemokines_Dividing);
						std::shared_ptr<Interphase> Expressing_Chemokines_Interphase = std::make_shared<Interphase>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine);
						states.push_back(Expressing_Chemokines_Interphase);
					
						std::shared_ptr<t0> Expressing_Chemokines_t0 = std::make_shared<t0>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine, Expressing_Chemokines_i0, Expressing_Chemokines_Interphase);
						Expressing_Chemokines_i0->transitions.push_back(Expressing_Chemokines_t0);
						std::shared_ptr<t1> Expressing_Chemokines_t1 = std::make_shared<t1>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine, Expressing_Chemokines_Interphase, Expressing_Chemokines_Dividing);
						Expressing_Chemokines_Interphase->transitions.push_back(Expressing_Chemokines_t1);
				}
				void Entry()
				{
					R_LTo_Step->new_LTin_binding = false;
					R_LTo_Step->new_LTi_binding = false;
					if (R_LTo_Step->lTiContactStateChangeTimePoint == -1) 
					{
						R_LTo_Step->lTiContactStateChangeTimePoint = set_ltin_contact_time();
					}
					if (R_LTo_Step->chemokineExpressed > R_LTo_Step->chemokineExpressionLimit) 
					{
						R_LTo_Step->chemokineExpressed = R_LTo_Step->chemokineExpressed - R_LTo_Step->chemokineDecrement;
					}
					if (R_LTo_Step->adhesionExpressed > R_LTo_Step->maxVCAMProbability && R_LTo_Step->chemokineExpressed <= R_LTo_Step->chemokineExpressionLimit) 
					{
						R_LTo_Step->maxExpressionReached = true;
					}
				}
			private:
				std::shared_ptr<LTo_Step> R_LTo_Step;
				std::shared_ptr<LTo_Controller> C_LTo_Controller;
				std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
			public:
				class Dividing : public robochart::State 
				{
					public:
						Dividing(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Dividing"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
						{
						}
						void Entry()
						{
							R_LTo_Step->divide_cells(R_LTo_Step->gridLoc, R_LTo_Step->adhesionExpressed, R_LTo_Step->chemokineExpressed, R_LTo_Step->maxExpressionReached, 1, std::string("chemokine"));
						}
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				};
			public:
				class Interphase : public robochart::State 
				{
					public:
						Interphase(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Interphase"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
						{
						}
						void Entry()
						{
							S_LTo_State_Machine->cell_division_clock = S_LTo_State_Machine->cell_division_clock + 1;
						}
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				};
			public:
				class i0 : public robochart::State 
				{
					public:
						i0(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("i0"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
						{
						}
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				};
				~Expressing_Chemokines()
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
				class t1 : public robochart::Transition {
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
					public:
						t1(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTo_State_Machine_Expressing_Chemokines_t1", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
						{}
						bool Condition() {
							if (S_LTo_State_Machine->cell_division_clock == R_LTo_Step->cellDivisionTime) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTo_State_Machine_Expressing_Chemokines_t1 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTo_State_Machine_Expressing_Chemokines_t1 is false\n");
								#endif
								return false;
							}
						}
				};
			public:
				class t0 : public robochart::Transition {
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
					public:
						t0(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTo_State_Machine_Expressing_Chemokines_t0", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
						{}
				};
		};
		class Mature_LTo : public robochart::State 
		{
			public:
				Mature_LTo(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Mature_LTo"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
				{
						// instantiate states && add substates of machine
						std::shared_ptr<i0> Mature_LTo_i0 = std::make_shared<i0>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine);
						states.push_back(Mature_LTo_i0);
						std::shared_ptr<Interphase> Mature_LTo_Interphase = std::make_shared<Interphase>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine);
						states.push_back(Mature_LTo_Interphase);
						std::shared_ptr<Dividing> Mature_LTo_Dividing = std::make_shared<Dividing>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine);
						states.push_back(Mature_LTo_Dividing);
					
						std::shared_ptr<t0> Mature_LTo_t0 = std::make_shared<t0>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine, Mature_LTo_i0, Mature_LTo_Interphase);
						Mature_LTo_i0->transitions.push_back(Mature_LTo_t0);
						std::shared_ptr<t1> Mature_LTo_t1 = std::make_shared<t1>(R_LTo_Step, C_LTo_Controller, S_LTo_State_Machine, Mature_LTo_Interphase, Mature_LTo_Dividing);
						Mature_LTo_Interphase->transitions.push_back(Mature_LTo_t1);
				}
			private:
				std::shared_ptr<LTo_Step> R_LTo_Step;
				std::shared_ptr<LTo_Controller> C_LTo_Controller;
				std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
			public:
				class i0 : public robochart::State 
				{
					public:
						i0(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("i0"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
						{
						}
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				};
				~Mature_LTo()
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
				class Interphase : public robochart::State 
				{
					public:
						Interphase(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Interphase"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
						{
						}
						void Entry()
						{
							S_LTo_State_Machine->cell_division_clock = S_LTo_State_Machine->cell_division_clock + 1;
						}
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				};
			public:
				class Dividing : public robochart::State 
				{
					public:
						Dividing(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine) : State("Dividing"), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine) 
						{
						}
						void Entry()
						{
							R_LTo_Step->divide_cells(R_LTo_Step->gridLoc, R_LTo_Step->adhesionExpressed, R_LTo_Step->chemokineExpressed, R_LTo_Step->maxExpressionReached, 1, std::string("mature"));
						}
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				};
			public:
				class t0 : public robochart::Transition {
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
					public:
						t0(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTo_State_Machine_Mature_LTo_t0", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
						{}
				};
			public:
				class t1 : public robochart::Transition {
					private:
						std::shared_ptr<LTo_Step> R_LTo_Step;
						std::shared_ptr<LTo_Controller> C_LTo_Controller;
						std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
					public:
						t1(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
						   robochart::Transition("S_LTo_State_Machine_Mature_LTo_t1", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
						{}
						bool Condition() {
							if (S_LTo_State_Machine->cell_division_clock == R_LTo_Step->cellDivisionTime) {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTo_State_Machine_Mature_LTo_t1 is true\n");
								#endif
								return true;
							}
							else {
								#ifdef SM_DEBUG
									printf("Condition of transition S_LTo_State_Machine_Mature_LTo_t1 is false\n");
								#endif
								return false;
							}
						}
				};
		};

		public:
			class t0 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t0(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t0", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->entry_state == std::string("start")) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t0 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t0 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
						R_LTo_Step->new_LTin_binding = false;
						R_LTo_Step->new_LTi_binding = false;
						S_LTo_State_Machine->cell_division_clock = 0;
					}
			};
		public:
			class t14 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t14(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t14", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->entry_state == std::string("chemokine")) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t14 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t14 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
						R_LTo_Step->new_LTin_binding = false;
						R_LTo_Step->new_LTi_binding = false;
						S_LTo_State_Machine->cell_division_clock = 0;
					}
			};
		public:
			class t15 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t15(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t15", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->entry_state == std::string("mature")) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t15 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t15 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
						R_LTo_Step->new_LTin_binding = false;
						R_LTo_Step->new_LTi_binding = false;
						S_LTo_State_Machine->cell_division_clock = 0;
					}
			};
		public:
			class t4 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t4(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t4", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->new_LTin_binding == true && S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t4 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t4 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t1 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t1(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t1", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (S_LTo_State_Machine->pJunctionValue > (0) && S_LTo_State_Machine->pJunctionValue <= (0 + (1 - S_LTo_State_Machine->retLigandProbability))) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t1 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t1 is true\n");
							#endif
							return false;
						}
					}
			};
		
		//0 + 1 - S_LTo_State_Machine->retLigandProbability
		public:
			class t2 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t2(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t2", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (S_LTo_State_Machine->pJunctionValue > (0 + 1 - S_LTo_State_Machine->retLigandProbability) && S_LTo_State_Machine->pJunctionValue <= (0 + 1 - S_LTo_State_Machine->retLigandProbability + (S_LTo_State_Machine->retLigandProbability))) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t2 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t2 is true\n");
							#endif
							return false;
						}
					}
			};
		
		//0 + 1 - S_LTo_State_Machine->retLigandProbability + S_LTo_State_Machine->retLigandProbability
		//0
		public:
			class t3 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t3(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t3", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (S_LTo_State_Machine->T->GetCounter() > R_LTo_Step->retLigandTime) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t3 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t3 is false\n");
							#endif
							return false;
						}
					}
			};
		public:
			class t5 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t5(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t5", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->new_LTin_binding == true && S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t5 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t5 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t6 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t6(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t6", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->new_LTi_binding == true && S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t6 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t6 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t7 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t7(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t7", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->maxExpressionReached == true && S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t7 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t7 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t8 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t8(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t8", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (S_LTo_State_Machine->T->GetCounter() > R_LTo_Step->retLigandTime) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t8 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t8 is false\n");
							#endif
							return false;
						}
					}
			};
		public:
			class t10 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t10(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t10", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->new_LTi_binding == true && S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t10 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t10 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t11 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t11(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t11", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->new_LTin_binding == true && S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t11 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t11 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t12 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t12(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t12", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (R_LTo_Step->new_LTin_binding == false && R_LTo_Step->new_LTi_binding == false && S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t12 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t12 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
		public:
			class t9 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t9(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t9", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (S_LTo_State_Machine->T->GetCounter() > R_LTo_Step->retLigandTime) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t9 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t9 is false\n");
							#endif
							return false;
						}
					}
			};
		public:
			class t13 : public robochart::Transition {
				private:
					std::shared_ptr<LTo_Step> R_LTo_Step;
					std::shared_ptr<LTo_Controller> C_LTo_Controller;
					std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine;
				public:
					t13(std::shared_ptr<LTo_Step> R_LTo_Step, std::shared_ptr<LTo_Controller> C_LTo_Controller, std::shared_ptr<LTo_State_Machine> S_LTo_State_Machine, std::weak_ptr<robochart::State> src, std::weak_ptr<robochart::State> tgt):
					   robochart::Transition("S_LTo_State_Machine_t13", src, tgt), R_LTo_Step(R_LTo_Step), C_LTo_Controller(C_LTo_Controller), S_LTo_State_Machine(S_LTo_State_Machine)
					{}
					bool Condition() {
						if (S_LTo_State_Machine->T->GetCounter() > 0) {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t13 is true\n");
							#endif
							return true;
						}
						else {
							#ifdef SM_DEBUG
								printf("Condition of transition S_LTo_State_Machine_t13 is false\n");
							#endif
							return false;
						}
					}
					void Action() {
						S_LTo_State_Machine->T->SetCounter(0);
						#ifdef SM_DEBUG
							printf("Resetting Clock T\n");
						#endif
					}
			};
};

#endif
