

#include "State.h"

namespace robochart {

bool Transition::Execute() {
	if (Condition() & Check()) {  //check condition() first if it is false no need to perform check(); condition() && check()
		auto src = source.lock();
		src->stage = State::s_Exit;
		src->Execute();
		Action();
		auto tgt = target.lock();
		tgt->stage = State::s_Enter;
		tgt->Execute();
		return true;
	}
	return false;
}

void State::Execute() {
	switch (stage) {
	case s_Enter:
#ifdef STATE_DEBUG
		printf("Entering State %s\n", this->name.c_str());
#endif
		Entry();
		if (Initial() >= 0) {
			states[Initial()]->stage = s_Enter;  //this has already makes sure that every time the state machine is entered, it starts executing from initial state?
			states[Initial()]->Execute();
		}
		stage = s_Execute;
		break;
	case s_Execute:
#ifdef STATE_DEBUG
		printf("Executing a state %s\n", this->name.c_str());
#endif
		while(TryExecuteSubstates(states));      //this makes sure more than one transition can happen at one cycle; execute the state from bottom to up
		if (TryTransitions() == false) {
#ifdef STATE_DEBUG
			printf("Executing during action of %s!\n", this->name.c_str());
#endif
			During();                              //if no transition is enabled, execute during action in every time step
		}
		else {
#ifdef STATE_DEBUG
			printf("Not Executing during action of %s!\n", this->name.c_str());
#endif
		}
		break;
	case s_Exit:
		Exit();
		stage = s_Inactive;
		break;
	}
}

bool State::TryTransitions() {
#ifdef STATE_DEBUG
	printf("trying %ld transitions\n", transitions.size());
#endif
	for (int i = 0; i < transitions.size(); i++) {
#ifdef STATE_DEBUG
		printf("trying transition: %s\n", transitions[i]->name.c_str());
#endif
		bool b = transitions[i]->Execute();
		if (b) {
			this->mark = true;
			CancelTransitions(i);  //erase OTHER events (in the channel) already registered by the transitions of this state, as the state tried its every possible transitions
#ifdef STATE_DEBUG
			printf("transition %s true\n", transitions[i]->name.c_str());
#endif
			return true;
		}
		else {
#ifdef STATE_DEBUG
			printf("transition %s false\n", transitions[i]->name.c_str());
#endif
		}
	}
	this->mark = false;
	return false;
}

void State::CancelTransitions(int i) {
	for (int j = 0; j < transitions.size(); j++) {
		if (j != i) {
#ifdef STATE_DEBUG
			printf("CANCEL transition: %s\n",transitions[j]->name.c_str());
#endif
			transitions[j]->Cancel();
		}
	}
}

//return false either no sub states or no transitions are enabled in the sub states
bool State::TryExecuteSubstates(std::vector<std::shared_ptr<State>> s) {
	for (int i = 0; i < s.size(); i++) {
		// printf("state index : %d; stage: %d\n", i, states[i]->stage);
		// there should be only one active state in a single state machine
		if (s[i]->stage == s_Inactive) continue;
		else {
			s[i]->Execute();
			return s[i]->mark;      //keep trying the transitions at the same level if there is transition from one state to another
		}
	}
	return false;
}

}

