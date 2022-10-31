
public class ShooterRobot extends Robot {

	// This robot is intended to test defense. It just picks up the ball and shoots
	// it from near the coordinates specified below.
	private final double SHOOT_FROM_X = distanceInFrontOfOtherGoal(300);
	private final double SHOOT_FROM_Y = Game.HEIGHT - 200;

	// AT LEAST ONE OF THESE MUST BE TRUE OR THE PROGRAM WILL FAIL. THESE CONTROL
	// WHICH SHOTS (ie. bouncing) THE ROBOT TAKES.
	private final boolean BOUNCE_OFF_TOP = true;
	private final boolean SHOOT_DIRECT = true;
	private final boolean BOUNCE_OFF_BOTTOM = true;

	public ShooterRobot(boolean team, Game game) {
		super(team, game);
	}

	// write your code for the robot in the loop.
	public void loop() {
		if (!BOUNCE_OFF_TOP && !BOUNCE_OFF_BOTTOM && !SHOOT_DIRECT) {
			System.err.println("Make sure one of the targeting options in ShooterRobot is set to true.");
			System.exit(0);
		} else {
			travelToBall();
			
			aimWithBallAt(SHOOT_FROM_X, SHOOT_FROM_Y);
			
			//System.out.println("Moving to shooting location");
			moveTo(SHOOT_FROM_X, SHOOT_FROM_Y);
			//System.out.println("Moved to shooting location");
			

			double random = Math.random();
			boolean validSelection = true;
			do {
				random = Math.random();
				validSelection = true;
				if (random < 0.33 && BOUNCE_OFF_TOP == false) {
					validSelection = false;
				} else if (random > 0.33 && random < 0.66 && SHOOT_DIRECT == false) {
					validSelection = false;
				} else if (random > 0.66 && BOUNCE_OFF_BOTTOM == false) {
					validSelection = false;
				}
			} while (!validSelection);

			//System.out.println("Random: " + random);
			if (random < 0.33) { // demonstrate two possible shots
				// bounce off top
				aimWithBallAt(otherGoalX, Game.HEIGHT + (Game.LOWER_GOAL_Y + 50));

			} else if (random < 0.66) {
				// shoot direct
				aimWithBallAt(otherGoalX, (Game.UPPER_GOAL_Y + Game.LOWER_GOAL_Y) / 2);
			} else {
				// bounce off bottom
				aimWithBallAt(otherGoalX, -1 * (Game.LOWER_GOAL_Y + 50));
			}
			kick(500);

			
		}
	}

}
