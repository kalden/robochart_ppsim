package autogenerated_sim;

public class LTi_Attributes implements HardwareComponent {
	
	
	public Double2D LTi_loc;
	public double angle_to_move;
	public int high_chemokine_grid_square;
	public double cellSpeed;
	public Tracking_Stats tracking;
	public int cell_id;
	public double totalchemoLevels;
	public TreeMap chemomap;

	
	
	public LTi_Attributes()
	{
		this.LTi_loc = new Double2D();
		this.angle_to_move = 0;
		this.high_chemokine_grid_square = 99;
		this.cellSpeed = 0;
		this.tracking = new Tracking_Stats();
		this.cell_id = 0;
		this.totalchemoLevels = 0;
		this.chemomap = new TreeMap();
	
	}
	
	

	public void Sensors()
	{
		
	}
	public void Actuators()
	{
		
	}



}
