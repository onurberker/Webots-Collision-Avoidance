///Libararies for webots modules
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
///////////////////////////////////////////////////////////

import java.lang.Math;
import java.util.*;


public class MyController {
  // creating global variables for sensors and motors
  static Motor left_wheel;
  static Motor right_wheel;
  static Compass compass;
  static GPS  gps;
  static DistanceSensor distance_sensors[] = new DistanceSensor[4];//For the 4 distance sensors in front of the robot
  static DistanceSensor side_distance_sensors[] = new DistanceSensor[2];//For the 2 distancesensor on the side of the robot
  static double target[] = new double[2];// Array that contains the target
  static String state = "turning";
  ////////////////////////////////////////////////////////////////////////
  
  public static void main(String[] args) {

    
    Robot robot = new Robot();
    String robot_name = robot.getName();
    ////////////////initializing and enabling sensors
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
     left_wheel = robot.getMotor("left wheel motor");
     right_wheel = robot.getMotor("right wheel motor");
     left_wheel.setVelocity(0);
     left_wheel.setPosition(Double.POSITIVE_INFINITY);
     right_wheel.setVelocity(0);
     right_wheel.setPosition(Double.POSITIVE_INFINITY);
     
     
     String[] distance_sensor_names = new String[]{"ps1","ps0","ps7","ps6"};
     String[] side_distance_sensor_names = new String[]{"ps2","ps5"};
     for(int i=0; i<4;i++)
     {
       distance_sensors[i] = robot.getDistanceSensor(distance_sensor_names[i]);
       distance_sensors[i].enable(timeStep);
     }
     for(int i=0; i<2;i++)
     {
       side_distance_sensors[i] = robot.getDistanceSensor(side_distance_sensor_names[i]);
       side_distance_sensors[i].enable(timeStep);
     }
     gps = robot.getGPS("gps");
     gps.enable(timeStep);
     
     compass = robot.getCompass("compass");
     compass.enable(timeStep);
    ////////////////////////////////////////////////////////////
     if (robot_name.equals("puckone"))//setting target for e puck 1
     {
       target[0] = 1.3;
       target[1] = 1.3;
       
     }

     robot.step(1000);// delay of 1 second before the robot starts
   
    while (robot.step(timeStep) != -1) {
      
      ///The finite state machine of the robot
      switch(state) 
      {
        case "forward":///If the state of the robot is to go forward
          if (at_target())//checking if robot is  at the target
          {
            move_robot(0,0);//If so stop and change state to stop
            state = "stop";
          }
          else
          {
            if(obstacle_found())//checking if there is an obstacle infront
            {
              state = "avoiding"; //if soo switching state to avoiding
            }
            else
            {
              //following code checks if the robot is on its proper path 
              double robot_angle = robot_direction() ;
              double x = gps.getValues()[0];
              double y = gps.getValues()[1];
              double angle = (Math.atan2((target[1]-y),(target[0]-x))/Math.PI)*180;
              if(((target[1]-y)>0)&&((target[0]-x)>0))
            {
              angle =  angle;
            }
            else if(((target[1]-y)<0)&&((target[0]-x)>0))
            {
              angle = 360+ angle;
            }
            else if (((target[1]-y)>0)&&((target[0]-x)<0))
            {
              angle =  angle;
            }
            else
            {
              angle =  180 + angle;
            }
              if(Math.abs(robot_angle - angle)>5)//If the robots orientation and the orientation the robot should travel is greater than 5 degrees then a corrent of path is needed
              {
                move_robot(0,0);
                state = "turning";//switching state to turning
              }
              else
              {
                move_robot(5,5);//moving the robot straight
              }
             }
           }
          break;
        case "turning"://if the  state is turning
          double robot_angle = robot_direction() ;//getting robots orientation
          
          double x = gps.getValues()[0];//getting robots x coord
          double y = gps.getValues()[1];//getting robots y coord
          if (at_target() == false)//if at target
          {
            ///the below code gets the angle between the robots centroid and the target location which is the direction the robot should travel
            double angle = (Math.atan2((target[1]-y),(target[0]-x))/Math.PI)*180;
              ///following conditions are taken by testing done
            if(((target[1]-y)>0)&&((target[0]-x)>0))
            {
              angle =  angle;
            }
            else if(((target[1]-y)<0)&&((target[0]-x)>0))
            {
              angle = 360+ angle;
            }
            else if (((target[1]-y)>0)&&((target[0]-x)<0))
            {
              angle =  angle;
            }
            else
            {
              angle =  180 + angle;
            }
            /////////////////////////////////////////////////////////////
            while(Math.abs(robot_angle - angle)>1)///if the direction and the robot orientation is greater than 1 degree it needs a correction
            {
              if (robot_angle >angle)// If the robots angle is greater than angle 
              {
                move_robot(1,-1);//robot turns clockwise till the target angle is reached
              }
              else if(robot_angle <angle)
              {
                move_robot(-1,1);//anticlockwise  
              }
              robot_angle = robot_direction() ;
              robot.step(1);
              
            }
            state = "forward";//Since the robot is looking at the correct direction it switches to going forward state
          
          }
          else
          {
            state = "stop";
            move_robot(0,0);
          }
          break;
        case "avoiding":///Avoiding state
        if(obstacle_found()) //if there is a obstacle infront of the robot
          {
            avoid_obs();//turns to avoid it
            robot.step(100);
            move_robot(5,5);//slightly goes forward
            robot.step(100);
          }
          else if (side_obstacle_found())//a objetc is detected on the side
          {
             move_robot(5,5);//go forward becaude the robot wont be hitting it but has to pass it
             robot.step(1500);
          }
          else
          {
            
            move_robot(5,5);//going forward a bit 
            robot.step(100);
            state = "turning";//Switching state ot turning
          }
          
          break;
      }
     };
  }
  static void move_robot(double ls, double rs)//Function to move robot
    {
      //setting speeds to wheels
      left_wheel.setVelocity(ls);
      right_wheel.setVelocity(rs);
    }
    
   static boolean obstacle_found()//checking if a obstacle is found on the front of the robot
    {
      for(int i=0; i<4;i++)
       {
         if (distance_sensors[i].getValue() <1000)///if distance sensor value is less than 1000 there is an object
         {
           return true;
         };
         
       }
       return false;
    }
    static boolean side_obstacle_found()//checking if a obstacle is found on the side of the robot
    {
      for(int i=0; i<2;i++)
       {
         if (side_distance_sensors[i].getValue() <1000)
         {
           return true;
         };
         
       }
       return false;
    }
    static void avoid_obs()//function to turn robot when there is an object infront
    {
      double base = 3;//robots speed
      double total = 0;//variable to save the total value of weights
      double count = 0;// number of distance sensors that detected something
      double ls = 0;//left speed
      double rs = 0;//right speed
      for(int i=0; i<4;i++)
       {
         if (distance_sensors[i].getValue() <1000)
         {
           total += i;//adding distance sensor weight
           count ++;
         };
         
       }
       if (count>0)//if any distnce sensors caught a object
       {
         //algorithm to decied how much to turn 
         ls = base + (total/count - 2)*3;
         rs = base - (total/count - 2)*3;
         move_robot(ls,rs);
       }
       else
       {
         move_robot(base,base);
       }
    }
    
    static double robot_direction() //getting robots orientation
    {
      double[] values = compass.getValues();//getting robots compass reading
      
      double rad = Math.atan(values[0]/  values[1]);//getting robots angle
      //converting the received angle to the range of 0-360
      double bearing =0 ;
      if(values[0]>0 && values[1]>0)
      {
        bearing = (rad/ Math.PI) * 180.0;
      }
      else if((values[0]>0 && values[1]<0) || (values[0]<0 && values[1]<0))
      {
        bearing = ((rad + Math.PI)/ Math.PI ) * 180.0;
      }
      else
      {
        bearing = ((rad + Math.PI*2)/ Math.PI ) * 180.0;
      }
     
      bearing =  Math.round(bearing);
      if (bearing == 360)
      {
        bearing  = 0;
      }
      return bearing;
   }
   
   static boolean at_target()//checking if at target
   {
     double x = gps.getValues()[0];
     double y = gps.getValues()[1];
     if(Math.pow((Math.pow((target[0]-x),2)+Math.pow((target[1]-y),2)),0.5)<0.05)//Using pythagoras and getting distance between robot and target and if its less than 5cm telling the robot to stop
     {
       return true;
     }
     return false;
   }
}
