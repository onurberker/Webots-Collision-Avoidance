///Libararies for webots modules
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;


import java.lang.Math;
import java.util.*;


public class MyController3 {
  static Motor left_wheel;
  static Motor right_wheel;
  static Compass compass;
  static GPS  gps;
  static Node puckone;
  static Node pucktwo;
  static DistanceSensor distance_sensors[] = new DistanceSensor[4];
  static DistanceSensor side_distance_sensors[] = new DistanceSensor[2];
  static double target[] = new double[2];
  static String state = "turning";
  
  
  public static void main(String[] args) {

    Supervisor robot = new Supervisor();
    String robot_name = robot.getName();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
     left_wheel = robot.getMotor("left wheel motor");
     right_wheel = robot.getMotor("right wheel motor");
     left_wheel.setVelocity(0);
     left_wheel.setPosition(Double.POSITIVE_INFINITY);
     right_wheel.setVelocity(0);
     right_wheel.setPosition(Double.POSITIVE_INFINITY);
     puckone = robot.getFromDef("puckone");//getting node of puckone 
     pucktwo = robot.getFromDef("pucktwo");//getting node of pucktwo
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
     if (robot_name.equals("puckthree"))
     {
       target[0] = 0;
       target[1] = 1.3;
     }
     gps = robot.getGPS("gps");
     gps.enable(timeStep);
     
     compass = robot.getCompass("compass");
     compass.enable(timeStep);
     
     
     
     robot.step(1000);
   
    while (robot.step(timeStep) != -1) {
      // 
      if(check_puckone_robot() || check_pucktwo_robot())
      {
        move_robot(0,0);
      }
      else
      {
        switch(state) {
          case "forward":
            if (at_target())
            {
              move_robot(0,0);
              state = "stop";
            }
            else
            {
              if(obstacle_found())
              {
                state = "avoiding";
              }
              else{
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
                if(Math.abs(robot_angle - angle)>5)
                {
                  move_robot(0,0);
                  state = "turning";
                }
                else
                {
                  move_robot(5,5);
                  }
                }
             }
            break;
          case "turning":
            
            
            double robot_angle = robot_direction() ;
            double x = gps.getValues()[0];
            double y = gps.getValues()[1];
            if (at_target() == false)
            {
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
              
              
              
              while(Math.abs(robot_angle - angle)>1)
              {
                
                if (robot_angle >angle)
                {
                  move_robot(1,-1);
                }
                else if(robot_angle <angle)
                {
                  move_robot(-1,1);
                }
                robot_angle = robot_direction() ;
                robot.step(1);
                
              }
              state = "forward";
            
            }
            else
            {
              state = "stop";
              move_robot(0,0);
            }
            break;
          case "avoiding":
          if(obstacle_found())
            {
              avoid_obs();
              robot.step(100);
              move_robot(5,5);
              robot.step(100);
            }
            else if (side_obstacle_found())
            {
               move_robot(5,5);
               robot.step(1500);
            }
            else
            {
              move_robot(5,5);
              robot.step(100);
              state = "turning";
            }
            
            break;
        }
      }
     };
  }
  static void move_robot(double ls, double rs)
    {
      left_wheel.setVelocity(ls);
      right_wheel.setVelocity(rs);
    }
    
   static boolean obstacle_found()
    {
      for(int i=0; i<4;i++)
       {
         if (distance_sensors[i].getValue() <1000)
         {
           return true;
         };
         
       }
       return false;
    }
    static boolean side_obstacle_found()
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
    static void avoid_obs()
    {
      double base = 3;
      double total = 0;
      double count = 0;
      double ls = 0;
      double rs = 0;
      for(int i=0; i<4;i++)
       {
         if (distance_sensors[i].getValue() <1000)
         {
           total += i;
           count ++;
         };
         
       }
       if (count>0)
       {
         ls = base + (total/count - 2)*3;
         rs = base - (total/count - 2)*3;
         move_robot(ls,rs);
       }
       else
       {
         move_robot(base,base);
       }
    }
    
    static double robot_direction() 
    {
    double[] values = compass.getValues();
    
    double rad = Math.atan(values[0]/  values[1]);
    
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
   
   static boolean at_target()
   {
     double x = gps.getValues()[0];
     double y = gps.getValues()[1];
     if(Math.pow((Math.pow((target[0]-x),2)+Math.pow((target[1]-y),2)),0.5)<0.05)
     {
       return true;
     }
     return false;
   }
   
   static boolean check_puckone_robot()//avoiding puckone
   {
     double[] values = puckone.getPosition();
     double x = gps.getValues()[0];
     double y = gps.getValues()[1];
     if (values[0]>0 && values[1]>0)
     {
       return false;
     }
     else
     {
       if(Math.pow((Math.pow((values[0]-x),2)+Math.pow((values[1]-y),2)),0.5)<0.5)
       {
         if(y>values[1])
         {
           return false;
         }
         else
         {
           return true;
           
         }
       }
       return false;
     }
   }
   
   static boolean check_pucktwo_robot()//avoiding pucktwo
   {
     double[] values = pucktwo.getPosition();
     double x = gps.getValues()[0];
     double y = gps.getValues()[1];
     if (values[0]>0 && values[1]>0)
     {
       return false;
     }
     else
     {
       if(Math.pow((Math.pow((values[0]-x),2)+Math.pow((values[1]-y),2)),0.5)<0.5)
       {
         if(y>values[1])
         {
           return false;
         }
         else
         {
           return true;
           
         }
       }
       return false;
     }
   }
}
