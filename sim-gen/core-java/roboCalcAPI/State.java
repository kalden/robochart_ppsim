package roboCalcAPI;
	
import java.util.ArrayList;

public class State {
	public String name;
	public Stages stage;
	public boolean mark;
		
	public State(String name, Stages stage)
	{
		this.name = name;
		this.stage = stage;
		this.mark = false;
	}
	
	public void Entry() {}
	public void During() {}
	public void Exit() {}
	
	public int Initial()
	{
		return -1;
	}
	
	public ArrayList<State> states = new ArrayList<State>();
	public ArrayList<Transition> transitions = new ArrayList<Transition>();

	public void Execute() 
	{
		switch (this.stage) {
			case s_Enter:
				//System.out.println("Entering State " + this.name);
				resetSubstates(states);
				Entry();
				if (Initial() >= 0) {
					states.get(Initial()).stage = Stages.s_Enter;  //this has already makes sure that every time the state machine is entered, it starts executing from initial state?
					states.get(Initial()).Execute();
				}
				this.stage = Stages.s_Execute;
				break;
				
			case s_Execute:
				while(TryExecuteSubstates(states));      //this makes sure more than one transition can happen at one cycle; execute the state from bottom to up
				if (!TryTransitions()) {
					During();                              //if no transition is enabled, execute during action in every time step
				}
				else {
					//System.out.println("Not Executing during action of %s!\n", this->name.c_str());
				}
				break;
			case s_Exit:
				Exit();
				stage = Stages.s_Inactive;
				break;
			case s_Inactive:
				break;
		}
	}

	public boolean TryTransitions()
	{
		for (int i = 0; i < transitions.size(); i++) {
			boolean b = transitions.get(i).Execute();
			if (b) 
			{
				this.mark = true;
				CancelTransitions(i);  //erase OTHER events (in the channel) already registered by the transitions of this state, as the state tried its every possible transitions
				return true;
			}
			else {
				//System.out.println("transition %s false\n", transitions[i]->name.c_str());
			}
		}
		this.mark = false;
		return false;
	}
	
	public void CancelTransitions(int i) 
	{
		for (int j = 0; j < transitions.size(); j++) 
		{
			if (j != i) 
			{
				transitions.get(j).Cancel();
			}
		}
	}
	
	//return false either no sub states or no transitions are enabled in the sub states
	public void resetSubstates(ArrayList<State> states) {
		for(int i=0;i<states.size();i++)
		{
			State a = states.get(i);
			if(a.states.size() > 0)
			{
				resetSubstates(a.states);
			}
			else
			{
				if(a.stage != Stages.s_Inactive)
				{
					a.stage = Stages.s_Inactive;
				}
			}
			
		}
	}
	
	//return false either no sub states or no transitions are enabled in the sub states
	public boolean TryExecuteSubstates(ArrayList<State> states) 
	{
		for (int i = 0; i < states.size(); i++) 
		{
			// printf("state index : %d; stage: %d\n", i, states[i]->stage);
			// there should be only one active state in a single state machine
			if (states.get(i).stage == Stages.s_Inactive) continue;
			else {
				states.get(i).Execute();
				//keep trying the transitions at the same level if there is transition from one state to another
				return states.get(i).mark;      
			}
		}
		return false;
	}
		
}
