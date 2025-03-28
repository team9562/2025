/*package frc.robot.utils;

import java.util.Scanner;

import frc.robot.Main;
import frc.robot.PVector;
import frc.robot.Robot;
import frc.robot.commands.followGuzPath;

public class guzPath extends Thread {

    String name = "test";
    int startX = (int) Robot.currentPosX; // Tile index values NOT units actual
    int startY = (int) Robot.currentPosY;
    int destX = 0, destY = 0;

    public guzPath() {
    }

    public void run() {
        System.out.println("Listen Thread Engages");
        String userInput = "not N a";
        String lastUserInput = "";
        Scanner scan = new Scanner(System.in);

        while (!userInput.equals("A")) {
            System.out.println(">>" + userInput + "<<");
            userInput = scan.nextLine();
            /*
             * try {
             * sleep(1000);
             * } catch (InterruptedException e) {
             * e.printStackTrace();
             * }
             *
            if (!userInput.equals(lastUserInput) && userInput.length() > 0) {
                lastUserInput = userInput;

                if (userInput.toUpperCase().equals("T")) { // Run a test movement
                    // Pretend path of a square for TESTING only
                    Main.botPath.add(new PVector(1, 0, 0));
                    Main.botPath.add(new PVector(1, 1, 0));
                    Main.botPath.add(new PVector(0, 1, 0));
                    Main.botPath.add(new PVector(0, 0, 0));
                } else {
                    destX = -1;
                    // Other if statement for macros, id number whatever
                    if (isNumeric(userInput)) {

                        if (Integer.parseInt(userInput) > 0
                                && Integer.parseInt(userInput) <= followGuzPath.poi.length) {
                            destX = (int) Math.round(followGuzPath.poi[Integer.parseInt(userInput) - 1].x);
                            destY = (int) Math.round(followGuzPath.poi[Integer.parseInt(userInput) - 1].y);
                        }
                    }

                    if (Main.botPath.size() == 0) {
                        // startX = currentPosX / # to get tile coordinate
                        startX = 0; // (int) currentX
                        startY = 0;

                    } else {// get to end of path and append
                        startX = (int) Main.botPath.get(Main.botPath.size() - 1).x;
                        startY = (int) Main.botPath.get(Main.botPath.size() - 1).y;
                    }
                    if (destX != -1) {
                        PVector[] newMap = followGuzPath.findPath(startX, startY, destX, destY);

                        for (int i = 0; i < newMap.length; i++) {
                            // Main.botPath.add(newMap[i]); //This appends new map path to the global
                            // UNIVERSAL bot path that it will follow
                        }
                    }
                }
            } // End of user input business.
        }

        System.out.println("Listen Thread Concludes.");
    }

    private boolean isNumeric(String str) {
        return str != null && str.matches("{-=}?\\d*\\.?\\d+");
    }
}*/
