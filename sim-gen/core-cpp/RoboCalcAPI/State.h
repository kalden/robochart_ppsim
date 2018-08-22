

#ifndef ROBOCALC_STATE_H_
#define ROBOCALC_STATE_H_

#include <vector>
#include <memory>

#define STATE_DEBUG

namespace robochart {
class Transition;
class State {

public:
	std::string name;
	bool mark;
	State(std::string n) : name(n), stage(s_Inactive), mark(false) {}
	virtual ~State() { printf("Deleting state %s\n", name.c_str()); }
	virtual void Entry() {}
	virtual void During() {}
	virtual void Exit() {}
	virtual int Initial() { return -1; }

	enum Stages {
		s_Enter, s_Execute, s_Exit, s_Inactive
	};
	Stages stage;
	std::vector<std::shared_ptr<State>> states;
	std::vector<std::shared_ptr<Transition>> transitions;
	virtual void Execute();

	bool TryTransitions();

	bool TryExecuteSubstates(std::vector<std::shared_ptr<State>> s);

	void CancelTransitions(int i);
};

class StateMachine: public State {
public:
	StateMachine(std::string n): State(n) {}
	virtual ~StateMachine() {}
};

class Transition {

private:
	std::weak_ptr<State> source, target;
public:
	std::string name;  //added
public:
	Transition(std::string n, std::weak_ptr<State> src, std::weak_ptr<State> tgt) :
			name(n), source(src), target(tgt) {
	}
	virtual void Reg() {}
	virtual bool Check() { return true; }
	virtual void Cancel() {}
	virtual bool Condition() { return true; }
	virtual void Action() {}
	virtual void ClearEvent() {};
	virtual ~Transition() { source.reset(); target.reset(); printf("Deleting transition\n");}
	bool Execute();
};

}

#endif

