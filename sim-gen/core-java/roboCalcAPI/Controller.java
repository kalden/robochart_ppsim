
package roboCalcAPI;

public abstract class Controller {
	public StateMachine stm;
	
	public void Execute()
	{
		if (stm != null) 
			stm.Execute();
	}

	public void Initialise() {}
}

