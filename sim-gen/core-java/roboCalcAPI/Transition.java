package roboCalcAPI;

public class Transition {
	
	private State source;
	private State target;
	public String name;
	
	public Transition(String name, State source, State target)
	{
		this.name = name;
		this.source = source;
		this.target = target;
	}
	
	public boolean Execute()
	{
		if(Condition() && Check())
		{
			// Condition is the guard, check is the trigger
			source.stage = Stages.s_Exit;
			//System.out.println(source.name+" Source Execute Called");
			source.Execute();
			Action();
			target.stage = Stages.s_Enter;
			//System.out.println(target.name+" Target Execute Called");
			target.Execute();
			return(true);
		}
		return(false);
	}
	
	
	public void Reg() {}
	public boolean Check() { return true; }
	public void Cancel() {}
	public boolean Condition() { return true; }
	public void Action() {}
	public void ClearEvent() {};
	
}
