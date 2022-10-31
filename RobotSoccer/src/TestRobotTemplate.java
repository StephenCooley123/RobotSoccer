
public class TestRobotTemplate extends Robot {

	// START BY COPYING THIS DOCUMENT.
	public TestRobotTemplate(boolean team, Game game) {
		super(team, game);
	}

	// This is the main loop that controls the robot code. When the game is running,
	// the loop method repeats indefinitely. When a goal is scored or the game
	// reset, the method restarts.
	//You can write your own code in here in place of the if statement or use the existing offense, defense, and neutral methods.
	public void loop() {
		if (game.inPossession(myTeam)) {
			doOffense();
		} else if (game.inPossession(otherTeam)) {
			doDefense();
		} else {
			doNeutral();
		}
	}

	
	//these are the three default methods for what your robot can do.
	//They control when your robot is on offense, on defense, and when no one has the ball, which is neutral.
	private void doOffense() {
		//write your offense code here.
	}

	private void doDefense() {
		//write your defense code here.
	}

	private void doNeutral() {
		//write your neutral code here.
	}

}
