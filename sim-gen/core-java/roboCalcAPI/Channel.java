package roboCalcAPI;
import java.util.ArrayList;

public class Channel {

	private String name;
	private ArrayList<Event> events = new ArrayList<>();  //store the number of shared event pointer in the channel

	public Channel(String n)
	{
		this.name = n; 
	}
	
	/* return the size of the channel
	 *
	 */
	public int Size() {
		return events.size();
	}
	
	/* clear the channel
	 *
	 */
	public void Clear() {
		events.clear();
	}
	
	//This is the overloaded function
	public Event Reg(Event ci) {
		events.add(ci);
		return ci;
	}
	
	public boolean Check(Event e) 
	{
		for (int i=0; i<events.size();i++)
		{
			if(e.compatible(events.get(i))){
							e.match(events.get(i));
							events.get(i).setOther(e);
							//System.err.println("checking is true\n");
							return true;
			}
		}
		return false;
	}
	
	
	public void Cancel(Event e) 
	{
		if(e.getOther() == null){
					events.remove(e);
		}
	}
	
	public void Accept(Event e) 
	{  
		if (e.getOther()!=null) {
				e.accept();            //The first component will only accept the event (but not delete the event in the channel), because e->getOther().value().lock()->isAccepted() is false;
										//the second component will accept the event as well; but it will also delete both events in the channel, because e->getOther().value().lock()->isAccepted() becomes true.
				if (e.getOther().isAccepted()) {
	
					// The other has already been accepted so I can remove and reset both
					Event other = e.getOther();
					events.remove(e);
					events.remove(other);
				}
		}
	}
	
	//this is used for asynchronous communication
	public void acceptAndDelete(Event e){
		if(e.getOther() != null){
			Event other = e.getOther();
			events.remove(e);
			events.remove(other);
		}
	}
		
	public String GetName() {
			return name;
	}
}

