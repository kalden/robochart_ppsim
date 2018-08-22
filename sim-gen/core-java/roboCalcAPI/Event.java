package roboCalcAPI;

import java.util.ArrayList;

public class Event 
{
	private String channel;
	private String source;
	public ArrayList parameters;
	public Event otherEvent = null;
	
	public boolean accepted;
	
	public Event(String channel, String source, boolean accepted, Object eventValue, ArrayList params)
	{
		this.channel = channel;
		this.source = source;
		this.accepted = accepted;
		this.parameters = params; 
	}
	
	public ArrayList getParameters() {
		return(parameters);
	}
	
	public void setParameter(int index, String s) {
		this.parameters.set(index, s);
	}
	
	
	public String getSource() {
		return source;
	}
	
	public Event getOther() {
		return otherEvent;
	}
	
	public void setOther(Event other)
	{
		otherEvent = other;
	}
	
	public void accept() {
		accepted = true;
	}
	
	public boolean isAccepted() {
		return accepted;
	}
	
	//if two events come from different sources and their types are the same, these two events are compatible
	public boolean compatible(Event other) {
		String e1,e2;
		e1 = this.getSource();
		e2 = other.getSource();
			
		if (e1 != null && e2 != null && !e1.equals(e2))
		{
			//System.err.println("Event Are Compatible");
			return(true);
		}
		else
		{
			return(false);
		}
		
	}
		
		
	public void match(Event other){
			
		if(! compatible(other)){
			return;
		}
		else{
			
			//System.err.println("### EVENT MATCH EVENTS\n");
			
			setOther(other);
		}
	}	
		
		
	public String get_channel() {
		return channel;
	}
}
	
