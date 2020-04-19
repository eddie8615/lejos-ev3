package assignment2;

import java.util.LinkedList;
import java.util.List;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MappingRobot {
	static EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(
			MotorPort.C);
	static EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(
			MotorPort.B);
	static EV3MediumRegulatedMotor servo = new EV3MediumRegulatedMotor(MotorPort.D);
	static EV3ColorSensor color = new EV3ColorSensor(SensorPort.S1);
	static EV3UltrasonicSensor Ultra = new EV3UltrasonicSensor(SensorPort.S2);
	static EV3IRSensor IR = new EV3IRSensor(SensorPort.S3);
	static final int defaultSpeed = 180;
	static final int maxAngle = 90;
	static final int minAngle = -90;
	static final int initAngle = 0;
	SampleProvider fSampleProvider = Ultra.getDistanceMode();
	SampleProvider rSampleProvider = IR.getDistanceMode();
	SampleProvider lSampleProvider = IR.getDistanceMode();
	
	float[] fsample = new float[fSampleProvider.sampleSize()];
	float[] rsample = new float[rSampleProvider.sampleSize()];
	float[] lsample = new float[lSampleProvider.sampleSize()];
	String heading = "north";
	static final int width = 19;
	static final int height = 13;
	static int gWidth = 9;
	static int gHeight = 112;
	boolean isEnd = false;
	static List<int[]> route = new LinkedList<int[]>();
	private static Brick brick = BrickFinder.getLocal();
    private static GraphicsLCD 	g = brick.getGraphicsLCD();
	String[][] map = {{"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
					  {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0"},
						};
	
	
	public MappingRobot() {
		LEFT_MOTOR.setSpeed(defaultSpeed);
		RIGHT_MOTOR.setSpeed(defaultSpeed);
		servo.setSpeed(180);
		LEFT_MOTOR.synchronizeWith(new RegulatedMotor[] {RIGHT_MOTOR});
	}
	
	public static void main(String[] args) throws InterruptedException {
		MappingRobot mazeRobot = new MappingRobot();
		LCD.drawString("Press any buttons", 0, 0);
		Button.waitForAnyPress();
		LCD.clear();
		mazeRobot.traveling();
		mazeRobot.backTracing(route);
	}
	
	public void traveling() {
		int x = 1;
		int y = 1;
		getDis();
		fSampleProvider.fetchSample(fsample, 0);
		fsample[0] *= 100;
		//mapping process is to map to the variable "map" of each of its position
		mapping(fsample[0], lsample[0], rsample[0], x, y);
		
		//displaying is to show the maze map which the robot has explored
		//There are two cases for displaying by receiving boolean value which is "isGreen" trying to map the green tile
		displaying(false);
		Delay.msDelay(300);
		
		while(Button.ESCAPE.isUp()) {
			boolean isGreen = false;
			forward();
			Delay.msDelay(4250);
			stop();
			
			Delay.msDelay(500);
			getDis();
			fSampleProvider.fetchSample(fsample, 0);
			fsample[0] *= 100;//reason for multiply 100 is to convert the unit from meter to centimeter.
			Delay.msDelay(300);
			
			int tileColor = getColor();
			Delay.msDelay(200);
			if(tileColor == Color.GREEN) {
				isGreen = true;
				//Literally it is going to previous tile when the robot is on a green tile
				goToPrevious();
				displaying(isGreen);
				Delay.msDelay(500);
				//Fetch distance samples of sides of the robot
				getDis();
				if(lsample[0] > 30 || rsample[0] > 30) {
					if(lsample[0] > 30 && rsample[0] > 30) {
						rotateRight();
						heading = setupHeading(heading, "right");
					}
					else if(lsample[0] < 30) {
						rotateRight();
						heading = setupHeading(heading, "right");
					}
					else {
						rotateLeft();
						heading = setupHeading(heading, "left");
					}
				}
				else {
					rotateBack();
					heading = setupHeading(heading, "back");
				}
				continue;
			}
			else if(tileColor == Color.RED) {
				//LCD.drawString("Finished mapping", 0, 0);
				
				if(heading == "north" || heading == "south") {
					x = setCoordinate(x);
				}
				else {
					y = setCoordinate(y);
				}
				map[x][y] = "end";
				
				Delay.msDelay(2000);
				//This process is to set the robot's heading to north which is easy to backtracking
				switch(heading) {
				case "north":
					break;
				case "south":
					rotateBack();
					heading = setupHeading(heading, "back");
					break;
				case "east":
					rotateLeft();
					heading = setupHeading(heading, "left");
					break;
				case "west":
					rotateRight();
					heading = setupHeading(heading, "right");
					break;
				}
				Delay.msDelay(3000);
				break;
			}
			else {
				//x is row
				//y is column
				if(heading == "north" || heading == "south") {
					x = setCoordinate(x);
				}
				else {
					y = setCoordinate(y);
				}
				mapping(fsample[0],lsample[0], rsample[0], x, y);
				displaying(isGreen);
				Delay.msDelay(500);
				
				if(lsample[0] > 30) {
					rotateLeft();
					heading = setupHeading(heading, "left");
				}
				else {
					if(fsample[0] < 30 && rsample[0] < 30) {
						rotateBack();
						heading = setupHeading(heading, "back");
					}
					else if(fsample[0] < 30) {
						rotateRight();
						heading = setupHeading(heading, "right");
					}
					else if(rsample[0] < 30) {
						
					}
				}
				Delay.msDelay(500);
			}
		}
	}
	
	public void backTracing(List<int[]> route) {
		traverse(1, 1);
		int[] current = route.remove(route.size() - 1);
		int[] next = new int[] {0, 0}; //initialise
		int[] end = route.get(0);
		while(!current.equals(end)) {
			next = route.remove(route.size() - 1);
			if(next[0] == current[0]) {
				if(next[1] > current[1]) {
					switch(heading) {
					case "north":
						heading = setupHeading(heading, "right");
						rotateRight();
						break;
					case "south":
						heading = setupHeading(heading, "left");
						rotateLeft();
						break;
					case "east":
						
						break;
					case "west":
						heading = setupHeading(heading, "back");
						rotateBack();
						break;
					}
				}
				else if(next[1] < current[1]) {
					switch(heading) {
					case "north":
						heading = setupHeading(heading, "left");
						rotateLeft();
						break;
					case "south":
						heading = setupHeading(heading, "right");
						rotateRight();
						break;
					case "east":
						heading = setupHeading(heading, "back");
						rotateBack();
						break;
					case "west":
						break;
					}
				}
			}
			else {
				if(next[0] > current[0]) {
					switch(heading) {
					case "north":
						break;
					case "south":
						heading = setupHeading(heading, "back");
						rotateBack();
						break;
					case "east":
						heading = setupHeading(heading, "left");
						rotateLeft();
						break;
					case "west":
						heading = setupHeading(heading, "right");
						rotateRight();
						break;
					}
				}
				else if(next[0] < current[0]) {
					switch(heading) {
					case "north":
						heading = setupHeading(heading, "back");
						rotateBack();
						break;
					case "south":
						break;
					case "east":
						heading = setupHeading(heading, "right");
						rotateRight();
						break;
					case "west":
						heading = setupHeading(heading, "left");
						rotateLeft();
						break;
					}
				}
			}
			
			Delay.msDelay(500);
			forward();
			Delay.msDelay(2000);
			stop();
			current = new int[] {next[0], next[1]};
			LCD.drawInt(end[0], 0, 0);
			LCD.drawInt(end[1], 0, 1);
			LCD.drawInt(current[0], 0, 2);
			LCD.drawInt(current[1], 0, 3);
			Delay.msDelay(2500);
			LCD.clear();
		}
		LCD.drawString("The end", 0, 0);
		
	}
	
	public void traverse(int row, int column) {
		int[] node;
		if(isEnd) {
			return ; 
		}
		else {
			if(map[row][column] == "end") {
				node = new int[] {row, column};
				route.add(node);
				isEnd = true;
				return;
			}
			else if(map[row][column] != "0" && map[row][column] != "v" && map[row][column] != "2") {
				map[row][column] = "v";//means visited
				node = new int[] {row, column};
				route.add(node);
				if(column < width - 1 && !isEnd) {
					traverse(row, column + 1);
				}
				if(row < height - 1 && !isEnd) {
					traverse(row + 1, column);
				}
				if(column > 0 && !isEnd) {
					traverse(row, column - 1);
				}
				if(row > 0 && !isEnd) {
					traverse(row - 1, column);
				}
			}
		}
	}
	public int setCoordinate(int coo) {
		switch(heading) {
		case "north":
			coo += 2;
			gHeight -= 16;
			break;
		case "south":
			coo -= 2;
			gHeight += 16;
			break;
		case "east":
			coo += 2;
			gWidth += 16;
			break;
		case "west":
			coo -= 2;
			gWidth -= 16;
			break;
		}
		return coo;
	}
	
	public void mapping(float fsample, float lsample, float rsample, int x, int y) {
		map[x][y] = "1";
		switch(heading) {
		case "north":
			if(fsample < 30) {
				map[x + 1][y - 1] = "2";
				map[x + 1][y] = "2";
				map[x + 1][y + 1] = "2";
				map[x][y - 1] = "1";
				map[x][y + 1] = "1";
			}
			if(lsample < 30) {	
				map[x - 1][y - 1] = "2";
				map[x][y - 1] = "2";
				map[x + 1][y - 1] = "2";
				map[x - 1][y] = "1";
				map[x][y + 1] = "1";
			}
			if(rsample < 30) {
				map[x - 1][y + 1] = "2";
				map[x][y + 1] = "2";
				map[x + 1][y + 1] = "2";
				map[x][y - 1] = "1";
				map[x - 1][y] = "1";
			}
			break;
		case "south":
			if(fsample < 30) {
				map[x - 1][y - 1] = "2";
				map[x - 1][y] = "2";
				map[x - 1][y + 1] = "2";
				map[x][y + 1] = "1";
				map[x][y - 1] = "1";
			}
			if(lsample < 30) {
				map[x - 1][y + 1] = "2";
				map[x][y + 1] = "2";
				map[x + 1][y + 1] = "2";
				map[x][y - 1] = "1";
				map[x - 1][y] = "1";
			}
			if(rsample < 30) {
				map[x - 1][y - 1] = "2";
				map[x][y - 1] = "2";
				map[x + 1][y - 1] = "2";
				map[x - 1][y] = "1";
				map[x][y + 1] = "1";
			}
			break;
		case "east":
			if(fsample < 30) {
				map[x - 1][y + 1] = "2";
				map[x][y + 1] = "2";
				map[x + 1][y + 1] = "2";
				map[x][y - 1] = "1";
				map[x - 1][y] = "1";
			}
			if(lsample < 30) {
				map[x + 1][y - 1] = "2";
				map[x + 1][y] = "2";
				map[x + 1][y + 1] = "2";
				map[x][y - 1] = "1";
				map[x][y + 1] = "1";
			}
			if(rsample < 30) {
				map[x - 1][y - 1] = "2";
				map[x - 1][y] = "2";
				map[x - 1][y + 1] = "2";
				map[x][y + 1] = "1";
				map[x][y - 1] = "1";
			}
		case "west":
			if(fsample < 30) {
				map[x - 1][y - 1] = "2";
				map[x][y - 1] = "2";
				map[x + 1][y - 1] = "2";
				map[x - 1][y] = "1";
				map[x][y + 1] = "1";
			}
			if(lsample < 30) {
				map[x - 1][y - 1] = "2";
				map[x - 1][y] = "2";
				map[x - 1][y + 1] = "2";
				map[x][y + 1] = "1";
				map[x][y - 1] = "1";
			}
			if(rsample < 30) {
				map[x + 1][y - 1] = "2";
				map[x + 1][y] = "2";
				map[x + 1][y + 1] = "2";
				map[x][y - 1] = "1";
				map[x][y + 1] = "1";
			}
			break;
		}
	}
	
	public String setupHeading(String heading, String dir) {
		if(dir == "right") {
			switch(heading) {
			case "north":
				heading = "east";
				break;
			case "south":
				heading = "west";
				break;
			case "east":
				heading = "south";
				break;
			case "west":
				heading = "north";
				break;
			}
		}
		else if(dir == "left"){
			switch(heading) {
			case "north":
				heading = "west";
				break;
			case "south":
				heading = "east";
				break;
			case "east":
				heading = "north";
				break;
			case "west":
				heading = "south";
				break;
			}
		}
		else {
			switch(heading) {
			case "north":
				heading = "south";
				break;
			case "east":
				heading = "west";
				break;
			case "south":
				heading = "north";
				break;
			case "west":
				heading = "east";
				break;
			}
		}
		return heading;
	}
	
	public void getDis() {
		servo.rotateTo(maxAngle);
		Delay.msDelay(500);
		rSampleProvider.fetchSample(rsample, 0);
		
		servo.rotateTo(minAngle);
		Delay.msDelay(500);
		lSampleProvider.fetchSample(lsample, 0);
		
		servo.rotateTo(initAngle);
		Delay.msDelay(500);
		
	}
	
	public int getColor() {
		int colorID = color.getColorID();
		if(colorID == Color.GREEN)
			return Color.GREEN;
		else if(colorID == Color.RED)
			return Color.RED;
		else
			return Color.WHITE;
	}
	
	public void forward() {
		LEFT_MOTOR.startSynchronization();
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		LEFT_MOTOR.endSynchronization();
	}
	
	public void stop() {
		LEFT_MOTOR.startSynchronization();
		LEFT_MOTOR.stop();
		RIGHT_MOTOR.stop();
		Delay.msDelay(300);
		LEFT_MOTOR.endSynchronization();
	}
	
	public void backward() {
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();
		Delay.msDelay(50);//should be modified
		stop();
	}
	
	public void goToPrevious() {
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();
		Delay.msDelay(4200);
		stop();
	}
	
	public void rotateRight() {
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.backward();
		Delay.msDelay(700);
		stop();
		
	}
	
	public void rotateLeft() {
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.forward();
		Delay.msDelay(700);
		stop();
	}
	
	public void rotateBack() {
		backward();
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.backward();
		Delay.msDelay(1600);
		stop();
	}
	
	public void displaying(boolean isGreen) {
		if(isGreen) {
			switch(heading) {
			case "north":
				g.setColor(0, 0, 0);
				g.fillRect(gWidth, gHeight-8, 8, 8);
				break;
			case "south":
				g.setColor(0, 0, 0);
				g.fillRect(gWidth, gHeight+8, 8, 8);
				break;
			case "east":
				g.setColor(0, 0, 0);
				g.fillRect(gWidth+8, gHeight, 8, 8);
				break;
			case "west":
				g.setColor(0, 0, 0);
				g.fillRect(gWidth-8, gHeight, 8, 8);
				break;
			}
		}
		else {
			switch(heading) {
			case "north":
				if(fsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth, gHeight - 8, 8, 8);
					g.fillRect(gWidth - 8, gHeight - 8, 8, 8);
					g.fillRect(gWidth + 8, gHeight - 8, 8, 8);
				}
				if(lsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth - 8, gHeight, 8, 8);
					g.fillRect(gWidth - 8, gHeight - 8, 8, 8);
				}
				if(rsample[0] < 30) {
					g.setColor(0,0,0);
					g.fillRect(gWidth + 8, gHeight, 8, 8);
					g.fillRect(gWidth + 8, gHeight - 8, 8, 8);
				}
				break;
			case "south":
				if(fsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth, gHeight + 8, 8, 8);
					g.fillRect(gWidth - 8, gHeight + 8, 8, 8);
					g.fillRect(gWidth + 8, gHeight + 8, 8, 8);
				}
				if(lsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth + 8, gHeight, 8, 8);
					g.fillRect(gWidth + 8, gHeight + 8, 8, 8);
				}
				if(rsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth - 8, gHeight, 8, 8);
					g.fillRect(gWidth - 8, gHeight + 8, 8, 8);
				}
				break;
			case "east":
				if(fsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth + 8, gHeight, 8, 8);
					g.fillRect(gWidth + 8, gHeight - 8, 8, 8);
					g.fillRect(gWidth + 8, gHeight + 8, 8, 8);
				}
				if(lsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth, gHeight - 8, 8, 8);
					g.fillRect(gWidth + 8, gHeight - 8, 8, 8);
				}
				if(rsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth, gHeight + 8, 8, 8);
					g.fillRect(gWidth + 8, gHeight + 8, 8, 8);
				}
				break;
			case "west":
				if(fsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth - 8, gHeight, 8, 8);
					g.fillRect(gWidth - 8, gHeight + 8, 8, 8);
					g.fillRect(gWidth - 8, gHeight - 8, 8, 8);
				}
				if(lsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth, gHeight + 8, 8, 8);
					g.fillRect(gWidth - 8, gHeight + 8, 8, 8);
				}
				if(rsample[0] < 30) {
					g.setColor(0, 0, 0);
					g.fillRect(gWidth, gHeight - 8, 8, 8);
					g.fillRect(gWidth - 8, gHeight - 8, 8, 8);
				}
				break;
			}
		}
	}
}
