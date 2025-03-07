package frc.robot.utils;
import java.util.Scanner;

import frc.robot.Main;
import frc.robot.PVector;
import frc.robot.RobotContainer;
import frc.robot.commands.followGuzPath;
public class guzPath extends Thread{

    String name = "test";
    int startX = 0; //Tile index values NOT units actual
    int startY = 0;
    public guzPath(){}

    public void run(){
        System.out.println("Listen Thread Engages");
        String userInput = "not N a";
        String lastUserInput ="";
        Scanner scan = new Scanner(System.in);

        while(!userInput.equals("A")){
            System.out.println(">>"+userInput+"<<");
            userInput = scan.nextLine();
            /*try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }*/
            if(!userInput.equals(lastUserInput) && userInput.length() > 0){
                lastUserInput = userInput;



                
            if(userInput.toUpperCase().equals("T")){ //Run a test movement
                //Pretend path of a square for TESTING only
               Main.botPath.add(new PVector(1,0,0));
               Main.botPath.add(new PVector(1,1,0));
               Main.botPath.add(new PVector(0,1,0));
               Main.botPath.add(new PVector(0,0,0));
            }else{

                //Other if statement for macros, id number whatever

                if(Main.botPath.size() == 0){
                    //startX = currentPosX / # to get tile coordinate
                }else {//get to end of path and append
                    startX = (int)Main.botPath.get(Main.botPath.size() -1).x;
                    startY = (int)Main.botPath.get(Main.botPath.size() -1).y;
                }

                PVector[] newMap = followGuzPath.findPath(startX, startY, 15, 4);

                for(int i=0; i < newMap.length; i++){
                    //Main.botPath.add(newMap[i]); //This appends new map path to the global UNIVERSAL bot path that it will follow
                }
            }
        }//End of user input business.
        }

        System.out.println("Listen Thread Concludes.");
    }
}
