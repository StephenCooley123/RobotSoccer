
public abstract class Robot {
	private double xVel = 0;
	private double yVel = 0;
	double angle;

	// check which team by calling team==Game.RED or team==Game.BLUE
	final boolean myTeam;
	final boolean otherTeam;
	final Game game;
	private boolean halt = false;
	final double myGoalX;
	final double otherGoalX;
	final double goalTopY = Game.UPPER_GOAL_Y;
	final double goalBottomY = Game.LOWER_GOAL_Y;

	private Thread thread;

	public Robot(boolean team, Game game) {
		this.game = game;
		this.myTeam = team;
		this.otherTeam = !myTeam;
		if (team == Game.BLUE) {
			myGoalX = Game.WIDTH;
			otherGoalX = 0;
		} else {
			myGoalX = 0;
			otherGoalX = Game.WIDTH;
		}
	}

	public void run() {

	}

	public abstract void loop();

	public void halt() {
		halt = true;
	}

	public void startThread() {
		thread = new Thread() {
			public void run() {
				System.out.println("Start of thread");
				while (true) {

					loop();
					try {
						Thread.sleep(1);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
					}

				}

			}
		};
		thread.start();
	}

	public void haltThread() {
		thread.stop();
		System.out.println("Thread Ending");
	}

	public boolean getTeam() {
		return myTeam;
	}

	public synchronized double getX() {
		return game.getX(myTeam);
	}

	public synchronized double getY() {
		return game.getY(myTeam);
	}

	public synchronized double getXVel() {
		double theta = Game.angleBetweenTwoPoints(xVel, yVel, 0, 0);
		double absoluteVelocity = Math.max(0, Math.min(Game.MAX_ROBOT_VELOCITY, Math.sqrt(xVel * xVel + yVel * yVel)));

		xVel = (Math.cos(Math.toRadians(theta)) * absoluteVelocity);
		yVel = (-1 * Math.sin(Math.toRadians(theta)) * absoluteVelocity);
		return (Math.cos(Math.toRadians(theta)) * absoluteVelocity);
	}

	public synchronized double getYVel() {
		double theta = Game.angleBetweenTwoPoints(xVel, yVel, 0, 0);
		double absoluteVelocity = Math.max(0, Math.min(Game.MAX_ROBOT_VELOCITY, Math.sqrt(xVel * xVel + yVel * yVel)));

		xVel = (Math.cos(Math.toRadians(theta)) * absoluteVelocity);
		yVel = (-1 * Math.sin(Math.toRadians(theta)) * absoluteVelocity);
		return (-1 * Math.sin(Math.toRadians(theta)) * absoluteVelocity);
	}

	public void setAngle(double angle) {
		this.angle = angle % 360;
	}

	public double getAngle() {
		return angle;
	}

	public double getOtherX() {
		return game.getRobotX(!myTeam);
	}

	public double getOtherY() {
		return game.getRobotY(!myTeam);
	}

	public double getOtherVelX() {
		return game.getRobotVelX(!myTeam);
	}

	public double getOtherVelY() {
		return game.getRobotVelY(!myTeam);
	}

	public double getOtherAngle() {
		return game.getRobotAngle(!myTeam);
	}

	// setters
	public synchronized void setXVel(double xVel) {

		this.xVel = xVel;
	}

	public synchronized void setYVel(double yVel) {

		this.yVel = yVel;
	}

	public boolean inPossession(boolean team) {
		if (team == Game.BLUE) {
			return (Game.distance(game.getX(Game.BLUE), game.getY(Game.BLUE), game.getBallX(),
					game.getBallY()) < game.getBallDiameter() / 2.0 + game.getRobotDiameter() / 2.0
							+ Game.BALL_KICK_DISTANCE
					&& game.isFacing(Game.BLUE, game.getBallX(), game.getBallY(), Game.FACING_PRECISION));
		} else {
			return (Game.distance(game.getX(Game.RED), game.getY(Game.RED), game.getBallX(),
					game.getBallY()) < game.getBallDiameter() / 2.0 + game.getRobotDiameter() / 2.0
							+ Game.BALL_KICK_DISTANCE
					&& game.isFacing(Game.RED, game.getBallX(), game.getBallY(), Game.FACING_PRECISION));
		}
	}

	public void pointTowards(double x, double y) {
		angle = Game.angleBetweenTwoPoints(x, y, getX(), getY());
	}

	public void moveTo(double x, double y) {
		final double DEFAULT_DISTANCE = 5;
		moveTo(x, y, DEFAULT_DISTANCE);
	}

	public void moveTo(double x, double y, double distance) {
		x = Math.max(Game.GOAL_BUFFER_ZONE, x);
		x = Math.min(Game.WIDTH - Game.GOAL_BUFFER_ZONE, x);
		y = Math.max(0, y);
		y = Math.min(Game.HEIGHT, y);

		final double DEFAULT_DISTANCE = 5;
		if (distance < 1) {
			distance = DEFAULT_DISTANCE;
		}

		while (distance(game.getX(myTeam), game.getY(myTeam), x, y) > distance) {
			double dx = x - game.getX(myTeam);
			double dy = y - game.getY(myTeam);
			// System.out.println("MOVING BY : " + dx + " " + dy);
			double initialDist = distance(game.getX(myTeam), game.getY(myTeam), x, y);

			if (initialDist == 0) { // prevent div0 error
				initialDist = 1;
			}
			dx /= initialDist;
			dy /= initialDist;

			setXVel(dx * Game.MAX_ROBOT_VELOCITY);
			setYVel(dy * Game.MAX_ROBOT_VELOCITY);
		}

		setXVel(0);
		setYVel(0);
	}

	public static double distance(double x1, double y1, double x2, double y2) {
		return Game.distance(x1, y1, x2, y2);
	}

	public void rotateWithBallTo(double degrees) {
		game.rotateWithBall(this, degrees);
	}

	public void aimWithBallAt(double x, double y) {
		game.rotateWithBall(this, Game.angleBetweenTwoPoints(x, y, game.getRobotX(myTeam), game.getRobotY(myTeam)));
	}

	public void kick(double velocity) {
		game.kick(this, velocity);
	}

	public void travelToBall() {
		while (!inPossession(myTeam)) {
			// move towards the ball
			moveTowardsBall();
		}
		stopRobot();
	}

	public void aimAndShoot() {
		aimWithBallAt(otherGoalX, (Game.UPPER_GOAL_Y + Game.LOWER_GOAL_Y) / 2);
		kick(500);
	}

	// points towards the position it is traveling to and moves. Rotates with the
	// ball if in possession.
	public void travelTo(double x, double y) {
		if (inPossession(myTeam)) {
			aimWithBallAt(x, y);
		} else {
			pointTowards(x, y);
		}
		moveTo(x, y);
	}

	public void moveTowardsBall() {
		pointTowards(game.getBallX(), game.getBallY());
		if (Math.abs(game.getX(myTeam) - game.getBallX()) < 5) {
			setXVel(0);
		} else if (game.getX(myTeam) < game.getBallX()) {
			setXVel(500);
		} else {
			setXVel(-500);
		}

		if (Math.abs(game.getY(myTeam) - game.getBallY()) < 5) {
			setYVel(0);
		} else if (game.getY(myTeam) < game.getBallY()) {
			setYVel(500);
		} else {
			setYVel(-500);
		}
	}

	public void stopRobot() {
		setXVel(0);
		setYVel(0);
	}

	public double distanceInFrontOfMyGoal(double distance) {
		return Math.abs(myGoalX - distance);
	}

	public double distanceInFrontOfOtherGoal(double distance) {
		return Math.abs(otherGoalX - distance);
	}

	public void wait(double seconds) {
		try {
			Thread.sleep((int) (seconds / 1000 / Game.PLAYBACK_SPEED));
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
		}
	}

	public void moveTowards(double x, double y) {
		if (Math.abs(game.getX(myTeam) - x) < 5) {
			setXVel(0);
		} else if (game.getX(myTeam) < x) {
			setXVel(500);
		} else {
			setXVel(-500);
		}

		if (Math.abs(game.getY(myTeam) - y) < 5) {
			setYVel(0);
		} else if (game.getY(myTeam) < y) {
			setYVel(500);
		} else {
			setYVel(-500);
		}
	}
	
	public synchronized double getBallX() {
		return game.getBallX();
	}

	public synchronized double getBallY() {
		return game.getBallY();
	}

	public synchronized double getBallVelX() {
		return game.getBallVelX();
	}

	public synchronized double getBallVelY() {
		return game.getBallVelY();
	}
	
	public boolean isFacing(boolean team, double x, double y, double anglePrecision) {
		return game.isFacing(team, x, y, anglePrecision);
	}
	
	public int getScore(boolean team) {
		return game.getScore(team);
	}

}
