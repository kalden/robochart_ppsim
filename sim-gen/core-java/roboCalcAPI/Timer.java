
package roboCalcAPI;

public class Timer {
	
private int counter;
private int waitPeriod;
private int startingCounter;
private boolean waitFlag;
private String name;
	
public Timer(String name)
{
	this.name = name;
	counter = 0;
	waitFlag = false;
	waitPeriod = 0;
	startingCounter = 65535;
}

public int GetCounter() {
		return counter;
	}

public void SetCounter(int i) 
{
	counter = i;
}

public void IncCounter() 
{
		counter++;
		if (counter - startingCounter >= waitPeriod) {
			waitFlag = false;
			startingCounter = 65535;
		}
	}

public void Wait(int i) 
{
		waitFlag = true;
		waitPeriod = i;
		startingCounter = counter;
	}

public boolean CheckWaitStatus() {
		return waitFlag;
	}

public String GetName() {
		return name;
	}
	

}

