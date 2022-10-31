
public class Ball {
	//SWITCH TO Game.BALL_START_X and Game.BALL_START_Y
	private double x = 0;
	private double y = 0;
	private double xVel = 0;
	private double yVel = 0;
	
	
	public Ball() {
		//x = Game.WIDTH / 2.0;
		//y = Game.HEIGHT / 2.0;
	}
	
	public synchronized double getX() {
		return x;
	}
	
	public synchronized double getY() {
		return y;
	}
	
	public synchronized double getXVel() {
		double theta = Game.angleBetweenTwoPoints(xVel, yVel, 0, 0);
		double absoluteVelocity = Math.max(0, Math.min(Game.MAX_BALL_VELOCITY,
				Math.sqrt(xVel * xVel + yVel * yVel)));

		xVel = (Math.cos(Math.toRadians(theta)) * absoluteVelocity);
		yVel = (-1 * Math.sin(Math.toRadians(theta)) * absoluteVelocity);
		return (Math.cos(Math.toRadians(theta)) * absoluteVelocity);
	}
	
	public synchronized double getYVel() {
		double theta = Game.angleBetweenTwoPoints(xVel, yVel, 0, 0);
		double absoluteVelocity = Math.max(0, Math.min(Game.MAX_BALL_VELOCITY,
				Math.sqrt(xVel * xVel + yVel * yVel)));

		xVel = (Math.cos(Math.toRadians(theta)) * absoluteVelocity);
		yVel = (-1 * Math.sin(Math.toRadians(theta)) * absoluteVelocity);
		return (-1 * Math.sin(Math.toRadians(theta)) * absoluteVelocity);
	}
	
	public synchronized void setX(double x) {
		this.x = x;
	}
	
	public synchronized void setY(double y) {
		this.y = y;
	}
	
	public synchronized void setXVel(double xVel) {
		this.xVel = xVel;
	}
	
	public synchronized void setYVel(double yVel) {
		this.yVel = yVel;
	}
}
