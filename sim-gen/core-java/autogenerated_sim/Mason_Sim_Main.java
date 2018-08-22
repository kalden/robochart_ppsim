package autogenerated_sim;

import ec.util.MersenneTwisterFast;
import sim.engine.SimState;


public class Mason_Sim_Main extends SimState {
	
	public static MersenneTwisterFast rng;
	// Set this to the number of steps for which simulation should execute 
	public int number_of_steps = 0;
	
	public Mason_Sim_Main(long seed)
	{
		super(seed);
		rng = new MersenneTwisterFast(seed);
	}
	
	public void finish()
	{
		super.finish();
	}
	
	public void start()
	{
		super.start();
	}
	
	public static void main(String[] args)
	{
		// Set the seed here
		long seed = System.currentTimeMillis();
		
		Mason_Sim_Main state = new Mason_Sim_Main(seed);
				
		state.start();
				
		do
		{
			if(!state.schedule.step(state)) break;
		}
		while(state.schedule.getSteps() < state.number_of_steps);
				
		state.finish();
			
			      
		System.exit(0);	
		
	}
}
