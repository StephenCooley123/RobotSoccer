import java.awt.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.*;

public class Game extends Canvas {

	// AIs :
	// In the place of TestRobot, write the name of the ai for blue team and red
	// team.
	//
	// If you just want to test your robot against a stationary red player, set blue
	// to your AI and make the SINGLE_PLAYER variable true. For an actual match,
	// make it false.
	private Robot blue = new TestRobotTemplate(BLUE, this);
	private Robot red = new ShooterRobot(RED, this);

	public static final boolean SINGLE_PLAYER = false;

	// timing
	public static final int FPS = 960;
	private static int totalGameTime = 300 * 1000;
	private long startTime = System.currentTimeMillis();
	public static final double PLAYBACK_SPEED = 1;

	public static final int WIDTH = 1545;
	public static final int HEIGHT = 1000;

	public static final boolean BLUE = true;
	public static final boolean RED = false;

	private boolean running = false;

	// starting positions
	public static final double GOAL_BUFFER_ZONE = 150;
	public static final double GOAL_PERCENTAGE = 0.25;
	public static final int RED_START_X = (int) (GOAL_BUFFER_ZONE);
	public static final int RED_START_Y = HEIGHT / 2;
	public static final int BLUE_START_X = (int) (WIDTH - GOAL_BUFFER_ZONE);
	public static final int BLUE_START_Y = HEIGHT / 2;
	public static final int BALL_START_X = WIDTH / 2;
	public static final int BALL_START_Y = HEIGHT / 2;

	// image scalars
	public static final double ROBOT_SCALE_FACTOR = 0.25;
	public static final double BALL_SCALE_FACTOR = 0.03;

	// scoreboard
	private int scoreRed = 0;
	private int scoreBlue = 0;

	// graphics
	private BufferedImage background;
	private BufferedImage redImage;
	private BufferedImage blueImage;
	private BufferedImage ballImage;

	private double ROBOT_DIAMETER;
	private double BALL_DIAMETER;

	// robots
	private double blueX = BLUE_START_X;
	private double blueY = BLUE_START_Y;
	private double redX = RED_START_X;
	private double redY = RED_START_Y;
	private Ball ball = new Ball();

	// constants
	public static final double MAX_ROBOT_VELOCITY = 200;
	public static final double MAX_BALL_VELOCITY = 475;
	public static final double BALL_KICK_DISTANCE = 10;
	public static final double FACING_PRECISION = 45;
	public static final double BALL_FRICTION_ACCELERATION = 87;

	// game board dimensions
	public static final int UPPER_GOAL_Y = (int) (HEIGHT / 2 + HEIGHT * GOAL_PERCENTAGE / 2);
	public static final int LOWER_GOAL_Y = (int) (HEIGHT / 2 - HEIGHT * GOAL_PERCENTAGE / 2);

	public double getX(boolean team) {
		if (team == BLUE) {
			return blueX;
		} else {
			return redX;
		}
	}

	public double getY(boolean team) {
		if (team == BLUE) {
			return blueY;
		} else {
			return redY;
		}
	}

	static JPanel p = new JPanel();

	@Override
	public void paint(Graphics g) {
	}

	public static void main(String[] args) {

		Game game = new Game();

		JFrame f = new JFrame("Game");

		p.setFocusable(true);

		f.add(game);
		// f.add(p);
		f.setSize(WIDTH + 18, HEIGHT + 40);
		f.setVisible(true);

		game.play();

	}

	private void play() {

		// initialization
		try {
			background = ImageIO.read(new File("SoccerField.png"));
			redImage = ImageIO.read(new File("RedRobot.png"));
			blueImage = ImageIO.read(new File("BlueRobot.png"));
			ballImage = ImageIO.read(new File("SoccerBall.png"));
			BALL_DIAMETER = ballImage.getHeight() * BALL_SCALE_FACTOR;
			ROBOT_DIAMETER = blueImage.getHeight() * ROBOT_SCALE_FACTOR;
			// p.add(picLabel);
			// f.setVisible(true);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		blue.startThread();
		blue.setAngle(180);
		if (!SINGLE_PLAYER) {
			red.startThread();
		}
		System.out.println("Finished Starting Threads");
		startTime = System.currentTimeMillis();
		reset();
		repaint();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		startTime = System.currentTimeMillis();
		running = true;

		// Handles Frames at the specified framerate.
		while (true) {

			long frameStartTime = System.currentTimeMillis();
			frame();
			// System.out.println("Frame Calculation Time: " + (System.currentTimeMillis() -
			// frameTime));
			int calculationTime = (int) (frameStartTime - System.currentTimeMillis());
			// System.out.println("Calculation Time: " + calculationTime);
			try {
				Thread.sleep(1000 / FPS - calculationTime);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			int totalFrameTime = (int) ((System.currentTimeMillis() - frameStartTime));

			// System.out.println("Total Frame Time: " + totalFrameTime);

		}

	}

	// updates the game on a time frame based system.
	private void frame() {
		long frameStartNanos = System.nanoTime();

		if (Keyboard.isKeyPressed(KeyEvent.VK_SPACE)) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			reset();
		}
		if (!isShowing()) {
			System.out.println("exiting");
			red.halt();
			blue.halt();
			System.exit(0);
		}

		// make sure velocities don't change over the course of the frame by pulling all
		// the values once
		double blueXVel = blue.getXVel();
		double blueYVel = blue.getYVel();
		double redXVel = red.getXVel();
		double redYVel = red.getYVel();
		double ballXVel = ball.getXVel();
		double ballYVel = ball.getYVel();

		// System.out.println("Blue Facing Ball: " + isFacing(BLUE, ball.getX(),
		// ball.getY(), 10));
		// allow for time at the end before the program closes
		if (System.currentTimeMillis() - startTime < totalGameTime / PLAYBACK_SPEED) {
			boolean ballOnWall = false;

			// decelerate ball and cap speed
			double ballTheta = angleBetweenTwoPoints(ball.getXVel(), ball.getYVel(), 0, 0);
			double absoluteBallVelocity = Math.min(MAX_BALL_VELOCITY,
					Math.sqrt(ball.getXVel() * ball.getXVel() + ball.getYVel() * ball.getYVel()));
			// double absoluteBallVelocity = Math.sqrt(ball.getXVel() * ball.getXVel() +
			// ball.getYVel() * ball.getYVel());

			absoluteBallVelocity -= BALL_FRICTION_ACCELERATION * PLAYBACK_SPEED / (FPS);
			absoluteBallVelocity = Math.max(0, absoluteBallVelocity);

			ball.setXVel((Math.cos(Math.toRadians(ballTheta)) * absoluteBallVelocity));
			ball.setYVel((-1 * Math.sin(Math.toRadians(ballTheta)) * absoluteBallVelocity));
			ballXVel = ball.getXVel();
			ballYVel = ball.getYVel();
			// System.out.println(absoluteBallVelocity + " x: " + ball.getXVel() + " y: " +
			// ball.getYVel());
			doRobotMovement(BLUE);
			doRobotMovement(RED);

			ball.setX(ball.getX() + ballXVel * PLAYBACK_SPEED / FPS);
			ball.setY(ball.getY() + ballYVel * PLAYBACK_SPEED / FPS);

			// cap speed of robots
			// blue

			// check for ball on wall
			if (ball.getX() - BALL_DIAMETER / 2 <= 0) {
				ballOnWall = true;
				if (ball.getY() <= UPPER_GOAL_Y && ball.getY() >= LOWER_GOAL_Y) {
					System.out.println("Score Blue!");
					scoreBlue();
					return;
				} else {
					ball.setXVel(Math.abs(ballXVel));
					// System.out.println("Hit Left Wall");
				}

			}
			// right wall
			if (ball.getX() + BALL_DIAMETER / 2 >= WIDTH) {
				ballOnWall = true;
				if (ball.getY() <= UPPER_GOAL_Y && ball.getY() >= LOWER_GOAL_Y) {
					System.out.println("Score Red!");
					scoreRed();
					return;
				} else {
					ball.setXVel(-1 * Math.abs(ballXVel));
					// System.out.println("Hit Right Wall");
				}

			}

			// top wall
			if (ball.getY() - BALL_DIAMETER / 2 <= 0) {
				ballOnWall = true;
				ball.setYVel(Math.abs(ballYVel));
				// System.out.println("Hit Bottom Wall");

			}

			// bottom wall
			if (ball.getY() + BALL_DIAMETER / 2 >= HEIGHT) {
				ballOnWall = true;
				ball.setYVel(-1 * Math.abs(ballYVel));
				// System.out.println("Hit Top Wall");

			}

			double oldBlueX = blueX;
			double oldBlueY = blueY;
			blueX += blueXVel * PLAYBACK_SPEED / FPS;
			blueY += blueYVel * PLAYBACK_SPEED / FPS;

			double oldRedX = redX;
			double oldRedY = redY;
			redX += redXVel * PLAYBACK_SPEED / FPS;
			redY += redYVel * PLAYBACK_SPEED / FPS;

			// check for invalid coordinates, ie. collision with other robot or wall, and if
			// so, undo the movement
			if (blueX <= GOAL_BUFFER_ZONE || blueX >= WIDTH - GOAL_BUFFER_ZONE
					|| (distance(blueX, blueY, redX, redY) <= ROBOT_DIAMETER + BALL_DIAMETER
							&& distance(blueX, blueY, redX, redY) < distance(oldBlueX, oldBlueY, oldRedX, oldRedY))
					|| (distance(ball.getX(), ball.getY(), blueX, blueY) <= ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0
							&& distance(ball.getX(), ball.getY(), blueX, blueY) < distance(ball.getX(), ball.getY(),
									oldBlueX, oldBlueY)
							&& ballOnWall)) {
				blueX = oldBlueX;

			}

			if (blueY - ROBOT_DIAMETER / 2.0 <= 0 || blueY + ROBOT_DIAMETER / 2.0 >= HEIGHT
					|| (distance(blueX, blueY, redX, redY) <= ROBOT_DIAMETER + BALL_DIAMETER
							&& distance(blueX, blueY, redX, redY) < distance(oldBlueX, oldBlueY, oldRedX, oldRedY))
					|| (distance(ball.getX(), ball.getY(), blueX, blueY) <= ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0
							&& distance(ball.getX(), ball.getY(), blueX, blueY) < distance(ball.getX(), ball.getY(),
									oldBlueX, oldBlueY)
							&& ballOnWall)) {
				blueY = oldBlueY;

			}

			// check for invalid coordinates, ie. collision with other robot or wall, and if
			// so, undo the movement
			// System.out.println("RED X VEL : " + redXVel + " RED Y VEL: " + redYVel);

			// check for invalid coordinates, ie. collision with other robot or wall, and if
			// so, undo the movement
			if (redX <= GOAL_BUFFER_ZONE || redX >= WIDTH - GOAL_BUFFER_ZONE
					|| (distance(blueX, blueY, redX, redY) <= ROBOT_DIAMETER + BALL_DIAMETER
							&& distance(blueX, blueY, redX, redY) < distance(oldBlueX, oldBlueY, oldRedX, oldRedY))
					|| (distance(ball.getX(), ball.getY(), redX, redY) <= ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0
							&& distance(ball.getX(), ball.getY(), redX, redY) < distance(ball.getX(), ball.getY(),
									oldRedX, oldRedY)
							&& ballOnWall)) {
				redX = oldRedX;

			}

			if (redY - ROBOT_DIAMETER / 2.0 <= 0 || redY + ROBOT_DIAMETER / 2.0 >= HEIGHT
					|| (distance(blueX, blueY, redX, redY) <= ROBOT_DIAMETER + BALL_DIAMETER
							&& distance(blueX, blueY, redX, redY) < distance(oldBlueX, oldBlueY, oldRedX, oldRedY))
					|| (distance(ball.getX(), ball.getY(), redX, redY) <= ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0
							&& distance(ball.getX(), ball.getY(), redX, redY) < distance(ball.getX(), ball.getY(),
									oldRedX, oldRedY)
							&& ballOnWall)) {
				redY = oldRedY;

			}

			// COLLISIONS CHECKING
			// left wall

			repaint();

		}
		// System.out.println("Frametime: " + ((System.nanoTime() - frameStartNanos) /
		// 1000000));
	}

	// adds core to blue.
	private void scoreBlue() {
		// TODO Auto-generated method stub
		// System.out.println("SCORE BLUE");
		scoreBlue++;
		reset();

	}

	// adds score to red
	private void scoreRed() {
		// System.out.println("SCORE RED");
		scoreRed++;
		reset();
	}

	// resets the board.
	private void reset() {
		blueX = BLUE_START_X;
		blueY = BLUE_START_Y;
		blue.setXVel(0);
		blue.setYVel(0);
		blue.haltThread();
		blue.startThread();

		redX = RED_START_X;
		redY = RED_START_Y;
		red.setXVel(0);
		red.setYVel(0);
		if (!SINGLE_PLAYER) {
			red.haltThread();
			red.startThread();
		}

		ball.setX(BALL_START_X);
		ball.setY(BALL_START_Y);
		ball.setXVel((Math.random() * 2 - 1) * 300);
		ball.setYVel((Math.random() * 2 - 1) * 300);
	}

	// updates the graphics. Method called automatically.
	public void update(Graphics g) {
		long frameTime = System.currentTimeMillis();

		Graphics2D offgc;
		Image offscreen = null;

		// create the offscreen buffer and associated Graphics
		offscreen = createImage(WIDTH, HEIGHT);
		offgc = (Graphics2D) offscreen.getGraphics();
		// clear the exposed area
		offgc.drawImage(background, 0, 0, this);
		BufferedImage rotatedRed = rotateImageByDegrees((BufferedImage) (redImage), red.angle);
		BufferedImage rotatedBlue = rotateImageByDegrees((BufferedImage) (blueImage), blue.angle);

		offgc.drawImage(
				rotatedRed.getScaledInstance((int) (rotatedRed.getWidth() * ROBOT_SCALE_FACTOR),
						(int) (rotatedRed.getHeight() * ROBOT_SCALE_FACTOR), BufferedImage.SCALE_SMOOTH),
				(int) (redX - rotatedRed.getWidth() * ROBOT_SCALE_FACTOR / 2.0),
				HEIGHT - (int) (redY + rotatedRed.getHeight() * ROBOT_SCALE_FACTOR / 2.0), this);
		offgc.drawImage(
				rotatedBlue.getScaledInstance((int) (rotatedBlue.getWidth() * ROBOT_SCALE_FACTOR),
						(int) (rotatedBlue.getHeight() * ROBOT_SCALE_FACTOR), BufferedImage.SCALE_SMOOTH),
				(int) (blueX - rotatedBlue.getWidth() * ROBOT_SCALE_FACTOR / 2.0),
				HEIGHT - (int) (blueY + rotatedBlue.getHeight() * ROBOT_SCALE_FACTOR / 2.0), this);
		offgc.drawImage(
				ballImage.getScaledInstance((int) (BALL_DIAMETER), (int) (BALL_DIAMETER), BufferedImage.SCALE_SMOOTH),
				(int) (ball.getX() - BALL_DIAMETER / 2.0), HEIGHT - (int) (ball.getY() + BALL_DIAMETER / 2.0), this);

		offgc.setFont(new Font("Arial Black", Font.BOLD, 20));
		offgc.drawString("Red: " + scoreRed, WIDTH / 4 - 50, 50);
		offgc.drawString("Blue: " + scoreBlue, WIDTH * 3 / 4 + 50, 50);
		offgc.drawString("Time: "
				+ ((int) ((totalGameTime - (System.currentTimeMillis() - startTime) * PLAYBACK_SPEED) / (1000) + 1)),
				WIDTH / 2 - 60, 50);
		// Score and Timer

		// offgc.fillOval((int) (System.currentTimeMillis() - startTime) / 10, 100, 50,
		// 50);

		// do normal redraw
		RenderingHints hints = new RenderingHints(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		offgc.addRenderingHints(hints);

		paint(offgc);
		// transfer offscreen to window
		g.drawImage(offscreen, 0, 0, this);
		// System.out.println("Update Calculation Time: " + (System.currentTimeMillis()
		// - frameTime));

	}

	// rotates an image by degrees
	private BufferedImage rotateImageByDegrees(BufferedImage img, double angle) {

		double rads = Math.toRadians(angle);
		double sin = Math.abs(Math.sin(rads)), cos = Math.abs(Math.cos(rads));
		int w = img.getHeight();
		int h = img.getHeight();
		int newWidth = (int) Math.floor(w * cos + h * sin);
		int newHeight = (int) Math.floor(h * cos + w * sin);

		BufferedImage rotated = new BufferedImage(newWidth, newHeight, BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2d = rotated.createGraphics();
		AffineTransform at = new AffineTransform();
		at.translate((newWidth - w) / 2, (newHeight - h) / 2);

		int x = w / 2;
		int y = h / 2;

		at.rotate(rads, x, y);
		g2d.setTransform(at);
		g2d.drawImage(img, 0, 0, this);
		g2d.dispose();

		return rotated;
	}

	// API for interacting with other robot
	public synchronized double getRobotX(boolean color) {
		if (color == BLUE) {
			return blueX;
		} else {
			return redX;
		}
	}

	public synchronized double getRobotY(boolean color) {
		if (color == BLUE) {
			return blueY;
		} else {
			return redY;
		}
	}

	public synchronized double getRobotVelX(boolean team) {
		if (team == BLUE) {
			return blue.getXVel();
		} else {
			return red.getXVel();
		}
	}

	public synchronized double getRobotVelY(boolean team) {
		if (team == BLUE) {
			return blue.getYVel();
		} else {
			return red.getYVel();
		}
	}

	public synchronized double getRobotAngle(boolean team) {
		if (team == BLUE) {
			return blue.getAngle() % 360;
		} else {
			return red.getAngle() % 360;
		}
	}

	public synchronized double getBallX() {
		return ball.getX();
	}

	public synchronized double getBallY() {
		return ball.getY();
	}

	public synchronized double getBallVelX() {
		return ball.getXVel();
	}

	public synchronized double getBallVelY() {
		return ball.getYVel();
	}

	public synchronized double getRobotDiameter() {
		return ROBOT_DIAMETER;
	}

	public synchronized double getBallDiameter() {
		return BALL_DIAMETER;
	}
	
	public int getScore(boolean team) {
		if(team == RED) {
			return scoreRed;
		} else {
			return scoreBlue;
		}
	}

	// returns whether the robot on team "team" is facing coordinates x and y within
	// anglePrecision degrees to either side (ie. the total angle coming out is
	// twice anglePrecision).
	public synchronized boolean isFacing(boolean team, double x, double y, double anglePrecision) {

		if (team == BLUE) {
			double angleToTarget = angleBetweenTwoPoints(x, y, blueX, blueY) % 360;

			double anglediff = (blue.getAngle() - angleToTarget + 180 + 360) % 360 - 180;

			return (anglediff <= FACING_PRECISION && anglediff >= -FACING_PRECISION);
		} else {
			// System.out.println("BLUE ANGLE: " + (int) angleBetweenTwoPoints(x, y, redX,
			// redY) % 360 + " FACING: "
			// + (int) red.getAngle());
			double angleToTarget = angleBetweenTwoPoints(x, y, redX, redY) % 360;

			double anglediff = (red.getAngle() - angleToTarget + 180 + 360) % 360 - 180;

			return (anglediff <= FACING_PRECISION && anglediff >= -FACING_PRECISION);
		}

	}

	public static double angleBetweenTwoPoints(double x1, double y1, double x2, double y2) {
		double angle = Math.toDegrees(Math.atan2(x2 - x1, y2 - y1));
		// Keep angle between 0 and 360
		angle = angle + Math.ceil(-angle / 360) * 360;

		return angle + 90;
	}

	public void doRobotMovement(boolean team) {

		// switcher for which team
		if (team == BLUE) {

			// if colliding with ball
			if (distance(blueX, blueY, ball.getX(), ball.getY()) < ballImage.getWidth() / 2 * BALL_SCALE_FACTOR
					+ blueImage.getWidth() / 2 * ROBOT_SCALE_FACTOR) {

				double angleToBall = angleBetweenTwoPoints(ball.getX(), ball.getY(), blueX, blueY) % 360;

				// BALL IS NOW IN POSSESSION

				// the || true disables inelastic collisions
				if (isFacing(BLUE, ball.getX(), ball.getY(), FACING_PRECISION)) {
					// System.out.println("Angle Between Vectors: " +
					// (angleBetweenTwoPoints(getBallVelX(), getBallVelY(), getRobotVelX(BLUE),
					// getRobotVelY(BLUE)) % 360));
					// blue.pointTowards(ball.getX(), ball.getY());
					double distXToBall = Math.cos(Math.toRadians(blue.getAngle()))
							* (ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0);
					double distYToBall = Math.sin(Math.toRadians(blue.getAngle()))
							* (ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0);

					ball.setX(blueX + distXToBall);
					ball.setY(blueY - distYToBall);

					ball.setXVel(0);
					ball.setYVel(0);
					/*
					 * double distXToBall = -1 * (ball.getXVel() - blue.getXVel()); double
					 * distYToBall = -1 * (ball.getYVel() - blue.getYVel()); double netDistToBall =
					 * Math.sqrt(distYToBall * distYToBall + distXToBall * distXToBall); double
					 * distanceRatio = (ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0) / netDistToBall;
					 * distXToBall *= distanceRatio; distYToBall *= distanceRatio; ball.setX(blueX +
					 * distXToBall); ball.setY(blueY + distYToBall); ball.setXVel(0);
					 * ball.setYVel(0);
					 */

					// ball.setXVel(blue.getXVel());
					// ball.setYVel(blue.getYVel());

				} else {// did not hit the front plate, perform 2d inelastic collision
					// System.out.println("Missed The Front");
					//
					// System.out.println("Blue Angle: " + blue.getAngle() + " Angle to ball: " +
					// angleToBall);

					double netBallVelocity = Math
							.sqrt((ball.getXVel() - blue.getXVel()) * (ball.getXVel() - blue.getXVel())
									+ (ball.getYVel() - blue.getYVel()) * (ball.getYVel() - blue.getYVel()));
					double angleOfBallTravel = angleBetweenTwoPoints(0, 0, ball.getXVel(), ball.getYVel());
					double resultingBallAngle = 2 * angleToBall - angleOfBallTravel;

					double resultingYComponent = Math.sin(Math.toRadians(resultingBallAngle));
					double resultingXComponent = Math.cos(Math.toRadians(resultingBallAngle));

					double netEndBallVelocity = Math.sqrt(
							resultingXComponent * resultingXComponent + resultingYComponent * resultingYComponent);
					resultingYComponent *= netBallVelocity / netEndBallVelocity;
					resultingXComponent *= netBallVelocity / netEndBallVelocity;
					// resultingXComponent += blue.getXVel();
					// resultingYComponent += blue.getYVel();
					ball.setXVel(resultingXComponent);
					ball.setYVel(-1 * resultingYComponent);

					double distXToBall = ball.getX() - blueX;
					double distYToBall = ball.getY() - blueY;
					double absDistToBall = Math.sqrt(distXToBall * distXToBall + distYToBall * distYToBall);
					double absDistFactor = (ROBOT_DIAMETER / 2 + BALL_DIAMETER / 2 + 5) / absDistToBall;
					distXToBall *= absDistFactor;
					distYToBall *= absDistFactor;
					ball.setX(blue.getX() + distXToBall);
					ball.setY(blue.getY() + distYToBall);
					// System.out.println("X Vel: " + resultingXComponent + "Y Vel: " +
					// resultingYComponent);
				}

			}
		} else {
			if (distance(redX, redY, ball.getX(), ball.getY()) < ballImage.getWidth() / 2 * BALL_SCALE_FACTOR
					+ redImage.getWidth() / 2 * ROBOT_SCALE_FACTOR) {

				double angleToBall = angleBetweenTwoPoints(ball.getX(), ball.getY(), redX, redY) % 360;

				// BALL IS NOW IN POSSESSION

				// the || true disables inelastic collisions
				if (isFacing(RED, ball.getX(), ball.getY(), FACING_PRECISION)) {
					// System.out.println("RED FACING BALL");
					// System.out.println("Angle Between Vectors: " +
					// (angleBetweenTwoPoints(getBallVelX(), getBallVelY(), getRobotVelX(RED),
					// getRobotVelY(RED)) % 360));
					// red.pointTowards(ball.getX(), ball.getY());
					double distXToBall = Math.cos(Math.toRadians(red.getAngle()))
							* (ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0);
					double distYToBall = Math.sin(Math.toRadians(red.getAngle()))
							* (ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0);

					ball.setX(redX + distXToBall);
					ball.setY(redY - distYToBall);

					ball.setXVel(0);
					ball.setYVel(0);
					/*
					 * double distXToBall = -1 * (ball.getXVel() - red.getXVel()); double
					 * distYToBall = -1 * (ball.getYVel() - red.getYVel()); double netDistToBall =
					 * Math.sqrt(distYToBall * distYToBall + distXToBall * distXToBall); double
					 * distanceRatio = (ROBOT_DIAMETER / 2.0 + BALL_DIAMETER / 2.0) / netDistToBall;
					 * distXToBall *= distanceRatio; distYToBall *= distanceRatio; ball.setX(redX +
					 * distXToBall); ball.setY(redY + distYToBall); ball.setXVel(0);
					 * ball.setYVel(0);
					 */

					// ball.setXVel(red.getXVel());
					// ball.setYVel(red.getYVel());

				} else {// did not hit the front plate, perform 2d inelastic collision
					// System.out.println("Missed The Front");
					//
					// System.out.println("Blue Angle: " + red.getAngle() + " Angle to ball: " +
					// angleToBall);

					double netBallVelocity = Math
							.sqrt((ball.getXVel() - red.getXVel()) * (ball.getXVel() - red.getXVel())
									+ (ball.getYVel() - red.getYVel()) * (ball.getYVel() - red.getYVel()));
					double angleOfBallTravel = angleBetweenTwoPoints(0, 0, ball.getXVel(), ball.getYVel());
					double resultingBallAngle = 2 * angleToBall - angleOfBallTravel;

					double resultingYComponent = Math.sin(Math.toRadians(resultingBallAngle));
					double resultingXComponent = Math.cos(Math.toRadians(resultingBallAngle));

					double netEndBallVelocity = Math.sqrt(
							resultingXComponent * resultingXComponent + resultingYComponent * resultingYComponent);
					resultingYComponent *= netBallVelocity / netEndBallVelocity;
					resultingXComponent *= netBallVelocity / netEndBallVelocity;
					// resultingXComponent += red.getXVel();
					// resultingYComponent += red.getYVel();
					ball.setXVel(resultingXComponent);
					ball.setYVel(-1 * resultingYComponent);

					double distXToBall = ball.getX() - redX;
					double distYToBall = ball.getY() - redY;
					double absDistToBall = Math.sqrt(distXToBall * distXToBall + distYToBall * distYToBall);
					double absDistFactor = (ROBOT_DIAMETER / 2 + BALL_DIAMETER / 2 + 5) / absDistToBall;
					distXToBall *= absDistFactor;
					distYToBall *= absDistFactor;
					ball.setX(red.getX() + distXToBall);
					ball.setY(red.getY() + distYToBall);
					// System.out.println("X Vel: " + resultingXComponent + "Y Vel: " +
					// resultingYComponent);
				}
			}
		}

	}

	public static double distance(double x1, double y1, double x2, double y2) {
		return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	}

	public boolean inPossession(boolean team) {
		return blue.inPossession(team);
	}

	public boolean isRunning() {
		return running;
	}

	public void rotateWithBall(Robot r, double angle) {
		r.setXVel(0);
		r.setYVel(0);
		// System.out.println("ROTATING WITH BALL");
		double distToBall;
		double oldBallX = ball.getX();
		double oldBallY = ball.getY();
		if (r == blue) {
			if (inPossession(BLUE)) {
				distToBall = distance(blueX, blueY, ball.getX(), ball.getY());
				blue.setAngle(angle);
				ball.setX(blue.getX() + Math.cos(Math.toRadians(blue.getAngle())) * distToBall);
				ball.setY(blue.getY() - Math.sin(Math.toRadians(blue.getAngle())) * distToBall);
			}
		} else if (r == red) {
			if (inPossession(RED)) {
				distToBall = distance(redX, redY, ball.getX(), ball.getY());
				red.setAngle(angle);
				ball.setX(red.getX() + (Math.cos(Math.toRadians(red.getAngle())) * distToBall));
				ball.setY(red.getY() - Math.sin(Math.toRadians(red.getAngle())) * distToBall);
			}
		}
		boolean ballOnWall = false;
		// check to make sure the ball can actually go in the new spot
		if (ball.getX() - BALL_DIAMETER / 2 <= 0) {
			ballOnWall = true;
			if (ball.getY() <= UPPER_GOAL_Y && ball.getY() >= LOWER_GOAL_Y) {
				System.out.println("Score Blue!");
				scoreBlue();
				return;
			} else {
				ball.setXVel(-1 * ball.getXVel());
				// System.out.println("Hit Left Wall");
			}

		}
		// right wall
		if (ball.getX() + BALL_DIAMETER / 2 >= WIDTH) {
			ballOnWall = true;
			if (ball.getY() <= UPPER_GOAL_Y && ball.getY() >= LOWER_GOAL_Y) {
				System.out.println("Score Red!");
				scoreRed();
				return;
			} else {
				ball.setXVel(-1 * Math.abs(ball.getXVel()));
				// System.out.println("Hit Right Wall");
			}

		}

		// top wall
		if (ball.getY() - BALL_DIAMETER / 2 <= 0) {
			ballOnWall = true;
			// System.out.println("Hit top wall");

		}

		// bottom wall
		if (ball.getY() + BALL_DIAMETER / 2 >= HEIGHT) {
			ballOnWall = true;
			// System.out.println("Hit bottom wall");

		}
		if (ballOnWall) {
			System.out.println("TRIED TO ROTATE WITH BALL, HIT A WALL");
			ball.setX(oldBallX);
			ball.setY(oldBallY);
		} else {
			ball.setYVel(0);
			ball.setXVel(0);
		}
	}

	public void kick(Robot robot, double velocity) {
		// System.out.println("KICK");
		if (robot == blue) {
			if (inPossession(BLUE)) {
				ball.setXVel(velocity * Math.cos(Math.toRadians(blue.getAngle())));
				ball.setYVel(-1 * velocity * Math.sin(Math.toRadians(blue.getAngle())));
				double minDist = BALL_DIAMETER / 2.0 + ROBOT_DIAMETER / 2.0 + BALL_KICK_DISTANCE;
				ball.setX(blue.getX() + minDist * Math.cos(Math.toRadians(blue.getAngle())));
				ball.setY(blue.getY() - minDist * Math.sin(Math.toRadians(blue.getAngle())));
			}
		} else if (robot == red) {
			if (inPossession(RED)) {
				ball.setXVel(velocity * Math.cos(Math.toRadians(red.getAngle())));
				ball.setYVel(-1 * velocity * Math.sin(Math.toRadians(red.getAngle())));
				double minDist = BALL_DIAMETER / 2.0 + ROBOT_DIAMETER / 2.0 + BALL_KICK_DISTANCE;
				ball.setX(red.getX() + minDist * Math.cos(Math.toRadians(red.getAngle())));
				ball.setY(red.getY() - minDist * Math.sin(Math.toRadians(red.getAngle())));
			}
		}

	}

}

//copied from internet lol
class Keyboard {

	private static final Map<Integer, Boolean> pressedKeys = new HashMap<>();

	static {
		KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(event -> {
			synchronized (Keyboard.class) {
				if (event.getID() == KeyEvent.KEY_PRESSED)
					pressedKeys.put(event.getKeyCode(), true);
				else if (event.getID() == KeyEvent.KEY_RELEASED)
					pressedKeys.put(event.getKeyCode(), false);
				return false;
			}
		});
	}

	public static boolean isKeyPressed(int keyCode) { // Any key code from the KeyEvent class
		return pressedKeys.getOrDefault(keyCode, false);
	}
}
