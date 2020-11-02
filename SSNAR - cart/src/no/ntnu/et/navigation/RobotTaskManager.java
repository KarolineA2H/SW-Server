/*
 * This code is written as a part of a Master Thesis
 * the spring of 2016.
 *
 * Eirik Thon(Master 2016 @ NTNU)
 */
package no.ntnu.et.navigation;

import java.util.concurrent.ConcurrentLinkedQueue;
import no.ntnu.tem.application.Application; //tror ikke denne har noe for seg 
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;
import no.ntnu.et.general.Position;
import no.ntnu.et.map.GridMap;
import no.ntnu.et.map.MapLocation;
import no.ntnu.et.mapping.MappingController;
import no.ntnu.et.mapping.MeasurementHandler;
import no.ntnu.tem.robot.Robot;
import no.ntnu.et.navigation.NavigationRobot;
import no.ntnu.tem.communication.UpdateMessage;
import no.ntnu.tem.communication.InboxReader;
import no.ntnu.tem.robot.Measurement;
import no.ntnu.tem.application.RobotController; 
import no.ntnu.et.navigation.NavigationController; 
import no.ntnu.tem.communication.Communication;



//import no.ntnu.et.general.Navigation;

/**
 * This class is used to manage target selection for all robots. It keeps track
 * of the target points of all the robots, and the threads that are working on
 * finding new targets
 *
 * @author Eirik Thon edited by Lars Marius Strande 2016
 */
public class RobotTaskManager {

    private GridMap map;

    private NavigationRobot navRobots;

    private ConcurrentHashMap<String, MapLocation> temporaryTargets;

    private ConcurrentHashMap<String, MapLocation> currentTargets;

    private HashMap<String, RobotTaskWorker> tasksInProgress;

    private int currentNumberOfWorkers;

    private boolean paused = true;

    final private boolean debug = true;

    final private int targetSpacing = 5; //[cm]
    
    // Memory usage test variables
    private int occupiedCount;
    private int frontierCount;
    private int locationCount;
    private int memoryUsage;
    private int maxUsage;

    public RobotTaskManager(GridMap map) {
        this.map = map;
        temporaryTargets = new ConcurrentHashMap<String, MapLocation>();
        currentTargets = new ConcurrentHashMap<String, MapLocation>();
        tasksInProgress = new HashMap<String, RobotTaskWorker>();
        currentNumberOfWorkers = 0;
        
        //SLAM-test
        occupiedCount = 0;
        frontierCount = 0;
        locationCount = occupiedCount + frontierCount;
        memoryUsage = 0;
        maxUsage = memoryUsage;
    }

    public void updateCurrentTarget(String robotName, MapLocation currentLocation) {
        currentTargets.put(robotName, currentLocation);
    }

    public void createNewTask(Robot robot, NavigationRobot navRobot, String name) {
        RobotTaskWorker worker = new RobotTaskWorker(robot, navRobot, name);
        currentNumberOfWorkers++;
        tasksInProgress.put(name, worker);
        Thread t = new Thread(worker);
        t.start();
        t.setName("Task-finder " + currentNumberOfWorkers);
    }

    boolean isWorkingOnTask(String name) {
        return tasksInProgress.containsKey(name);
    }

    private class RobotTaskWorker implements Runnable {

        // These were private, may be bad practice to use protected
        protected NavigationRobot navRobot;
        protected Robot robot;
        protected String name;
        protected boolean done;

        public RobotTaskWorker(Robot robot, NavigationRobot navRobot, String name) {
            this.robot = robot;
            this.navRobot = navRobot;
            this.name = name;
            done = false;
            Position robotPosition = new Position(robot.getPosition());
            MapLocation robotLocation = map.findLocationInMap(robotPosition);
            currentTargets.put(name, robotLocation);
        }

        @Override
        public void run() {
            if (debug) {
                occupiedCount = map.getOccupiedCount();
                frontierCount = map.getFrontierCount();
                locationCount = occupiedCount + frontierCount;
                memoryUsage = locationCount*4; // 4 bytes for coordinates for each location
                //System.out.println("Memory usage: " + memoryUsage + "KB");
                if (memoryUsage > maxUsage) {
                    maxUsage = memoryUsage;
                }
                System.out.println("Max memory usage: " + maxUsage + "KB");
            }
            ArrayList<MapLocation> frontierLocations = map.getFrontierLocations();
            ArrayList<MapLocation> possibleTargets = selectSpreadLocations(frontierLocations);
            int currentOrientation = robot.getRobotOrientation();
            boolean assigned = false;
            // Assign the robot to move to one of the frontier locations if it has nothing else to do.
            while (!assigned && !robot.isGoingHome()) {
                Position robotPosition = new Position(robot.getPosition());
                MapLocation robotLocation = map.findLocationInMap(robotPosition);
                System.out.println("World frame robot position");
                robotPosition.print();
                System.out.println("MAP frame robot position");
                robotLocation.print();
                
                Measurement currentMeasurement = robot.getMeasurement();
                if (currentMeasurement != null){
                    int[] irData = currentMeasurement.getIRdata();
                    System.out.println("ROBOT IR sensor " + Arrays.toString(irData));
                    System.out.println("ROBOT IR back " + irData[2]);
                    System.out.println("ROBOT IR forward: " + irData[0]);
                    System.out.println("ROBOT IR right: " + irData[1]);
                    System.out.println("ROBOT IR left: " + irData[3]);
                }
                int[] IRMeasurement = robot.getIRMeasurement(); 
                System.out.println("From inbox: IR measurement: " + Arrays.toString(IRMeasurement));
                
                //Updatemessage update = new UpdateMessage(message.getData())
                // Find the most optimal targetpoint in the map given the robots current location and the target points of the other robots
                MapLocation bestTarget = findBestTarget(currentOrientation, robotLocation, possibleTargets, name);
                
               // Done mapping
                if (bestTarget == null) {
                    robot.setGoingHome(true);
                    robot.setAtBase(false);
                    robot.setDock(true);
                    System.out.println("Goin home set true, at base: false, setDock = true");
                    if (debug) {
                        // Sends the robot home if it is done mapping..
                        System.out.println(name + ": No targetpoint found");

                    }
                    break;
                }

                robotPosition = new Position(robot.getPosition());
                robotLocation = map.findLocationInMap(robotPosition);
                if (map.findCell(robotLocation).isRestricted()) {
                    if (debug) {
                        System.out.println(name + ": Robot in restricted position");
                    }
                    break;
                }
                // Search for a path between the robot and the best target point
                ArrayList<MapLocation> path = PathPlanningFunctions.findPath2PRINT(map, bestTarget, robotLocation);
                // If no path to bestTarget is found remove bestTarget from possibleTargets
                               
                if (path == null) {
                    for (int i = 0; i < possibleTargets.size(); i++) {
                        if (MapLocation.equals(bestTarget, possibleTargets.get(i))) {
                            possibleTargets.remove(i);
                            break;
                        }
                    }
                    // If there are no targets left in possibleTargets give up and try agin next iteration
                    if (possibleTargets.size() == 0) {
                        if (debug) {
                            System.out.println(name + ": No targetpoints reachable. Waiting");
                            robot.isGoingHome(); 
                            System.out.println("Force Robot to go home");
                        }
                        break;
                    }
                    if (debug) {
                        System.out.println(name + ": Target unreachable. Finding new target");
                    }
                } // If a path is found find waypoints along the way and set destination for the robot
                else {
                    //computeUtility(bestTarget, robotLocation, currentOrientation, name, true);
                    if (debug) {
                        System.out.println(name + ": Path found");
                    }
                    ArrayList<Position> newWaypoints = PathPlanningFunctions.generateWaypoints(map, path);
                    if (newWaypoints.isEmpty()) {
                        if (debug) {
                            System.out.println(name + ": Unable to generate waypoints");
                        }
                        temporaryTargets.remove(name);
                        break;
                    }
                    currentTargets.put(name, bestTarget);
                    temporaryTargets.remove(name);
                    navRobot.addWaypoints(newWaypoints);
                    Position destinationPos = newWaypoints.get(newWaypoints.size() - 1);
                    int[] destination = {(int) Math.round(destinationPos.getXValue()), (int) Math.round(destinationPos.getYValue())};
                    robot.setDestination(destination);
                    assigned = true;
                }
                /*  Added to remove the need of goHome btn in RobotIdGUI  */
                //robot.checkBattery(); kommentert ut 2020
                
            }
            
            // Runs the A* algorthim for return.....
            ////// Var opprinnelig kommentert ut 2020
            ///*
            while ((robot.isGoingHome() && !assigned && !robot.isAtBase() )|| robot.isGoingHome() ) {
                System.out.println(" I want to go home ");
                Position robotPosition = new Position(robot.getPosition());
                double RobX = robotPosition.getXValue();
                double RobY = robotPosition.getYValue(); 
                System.out.println("I am here X: " + RobX + " Y: "+ RobY);
                MapLocation robotLocation = map.findLocationInMap(robotPosition);

                // Find home location in the map.
                MapLocation bestTarget = findBestTarget(currentOrientation, robotLocation, possibleTargets, name);
                Position homePosition = new Position(robot.getBasePosition());
                MapLocation homeLocation = map.findLocationInMap(homePosition);
                System.out.println("Base position: " );
                homePosition.print();
                System.out.println("Base position: MAP localisation: ");
                homeLocation.print();
                robotPosition = new Position(robot.getPosition());
                robotLocation = map.findLocationInMap(robotPosition);
                double newRowPos = robotPosition.getXValue();
                double newColPos = robotPosition.getYValue();
                int newRowLoc = robotLocation.getRow(); 
                int newColLoc = robotLocation.getColumn();
                System.out.println("New robot rosition  row: " + newRowPos + " col: " + newColPos);
                System.out.println("New robot location  row: " + newRowLoc + " col: " + newColLoc);
                
                //Finding path home
                ArrayList<MapLocation> path = PathPlanningFunctions.findPath2PRINT(map, homeLocation, robotLocation);
                System.out.println("I have a new path home"); 
                if (path == null) {
                    System.out.println("BUT the PATH is NULL");
                    for (int i = 0; i < possibleTargets.size(); i++) {
                        if (MapLocation.equals(bestTarget, possibleTargets.get(i))) {
                            System.out.println("Posible target is removed");
                            possibleTargets.remove(i);
                            break;
                        }
                    }
                    // If there are no targets left in possibleTargets give up and try agin next iteration
                    if (possibleTargets.size() == 0) {
                        if (debug) {
                            System.out.println(name + ": No targetpoints reachable. Waiting");
                        }
                        break;
                    }
                    if (debug) {
                        System.out.println(name + ": Target unreachable. Finding new target");
                    }
                } // If a path is found find waypoints along the way and set destination for the robot
                else {
                    //computeUtility(bestTarget, robotLocation, currentOrientation, name, true);
                    if (debug) {
                        System.out.println(name + ": Path found");
                    }
                    ArrayList<Position> newWaypoints = PathPlanningFunctions.generateWaypoints(map, path);
                    
                    if (newWaypoints.isEmpty() || Position.distanceBetween(robotPosition, newWaypoints.get(0)) < 1) {
                    System.out.println("Robot: " + name + " is back at base.");
                    System.out.println("Robots position:");
                    robotPosition.print();

                    robot.setAtBase(true);
                    temporaryTargets.remove(name);
                    //assigned = false;
                    assigned = true;
                    break;
                    }
                    if (newWaypoints.isEmpty()) {
                        if (debug) {
                            System.out.println(name + ": Unable to generate waypoints");
                        }
                        temporaryTargets.remove(name);
                        break;
                    }
                    
                    currentTargets.put(name, homeLocation);
                temporaryTargets.remove(name);
                navRobot.addWaypoints(newWaypoints);
                Position destinationPos = newWaypoints.get(newWaypoints.size() - 1);
                int[] destination = {(int) Math.round(destinationPos.getXValue()), (int) Math.round(destinationPos.getYValue())};
                robot.setDestination(destination);
                assigned = true;
                    
                }
                    /*
                System.out.println("Str: " + path.size()); 
                for (int i = 0; i <path.size(); i ++)
                {
                    System.out.println("Path Size : " + path.get(i));
                }
                for (int i = 0; i <50 ; i ++)
                {
                    //wait
                }
                //Find waypoints along the path home
                ArrayList<Position> newWaypoints = PathPlanningFunctions.generateWaypoints(map, path);
                */
                //Print ut the path list
                
                System.out.println("I have wave point home");

                
                
                
                    if (robot.isStuck()) {
                        robot.setStuck(false);
                        System.out.println("New attempt to return");
                        break;
                    }
                }
            //*/
            ////////////////
            
            /// To park in the garasj // 
            
            while(robot.isAtBase()){
                String name = robot.getName(); 
                System.out.println("Robot NAME: "+ name);
                
                int heading = robot.getRobotOrientation(); 
                System.out.println("The Robot is heading" + heading);
                
                // Want to turn the robot 180 deg 
                //robot.setRobotOrientation(180);
                Position robotPosition = new Position(robot.getPosition());
                //MapLocation homeLocation = map.findLocationInMap(homePosition);
                MapLocation robotLocation = map.findLocationInMap(robotPosition);
                double newRowPos = robotPosition.getXValue();
                double newColPos = robotPosition.getYValue();
                int newRowLoc = robotLocation.getRow(); 
                int newColLoc = robotLocation.getColumn();
                System.out.println("New robot rosition  row: " + newRowPos + " col: " + newColPos);
                System.out.println("New robot location  row: " + newRowLoc + " col: " + newColLoc);
                
                
                
                ////////////////
                /////// ROBOTEN MÅ FINNE EN MPTE OG RYGGE PÅ SELV 
                /////// Og sende tilbake til serveren når den er hjemme! 
                ////////////////
                //Finding path home
                //Appllicaton.com = new Application.Communication(this, robot);
                //Application.com.sendOrderToRobotWithRot(robot.getAddress(), (int)robotPosition.getXValue(), (int)robotPosition.getYValue(),180);
                //Application.sendOrderToRobotWithRot(robot.getAddress(), (int)robotPosition.getXValue(), (int)robotPosition.getYValue(),180);
                //Application.com.writeCommandToRobotWithRot(robot.getId(),robot.getName() , (int)robotPosition.getXValue(), (int)robotPosition.getYValue(),180);
                
                //Application.sendOrderCOM(robot.getAddress(),name, (int)robotPosition.getXValue(), (int)robotPosition.getYValue(),180); 
                
                //sendOrderToRobotWithRot(robot.getAddress(),robotPosition.getXValue(),robotPosition.getYValue(), 180);
                
                //robot.getName().CollisionManager.setPriorityCommand(robotPosition); 
                
                //Robot applicationRobot = robotController.getRobot(name); 
                
                // Get the IR measurement 
                /*int[] IRMeasurement = robot.getIRMeasurement(); 
                System.out.println("From inbox: IR measurement: " + Arrays.toString(IRMeasurement));
                
                //int uptemumX = (int)robotPosition.getXValue() -(IRMeasurement[2] + 15); 
                //Position target = new Position(uptemumX, (int)robotPosition.getYValue()); 
                //MapLocation mapTarget = map.findLocationInMap(target);
                //ArrayList<MapLocation> path = PathPlanningFunctions.findPath2PRINT(map, mapTarget, robotLocation);
                //Drive back ass long av irData[2] > 15 cm 
                while(IRMeasurement[2]>15){
                
                
                //rygge 
                Position robotPositionTemp = new Position(robot.getPosition());
                double x = robotPositionTemp.getXValue();
                double y = robotPositionTemp.getYValue();
                double newX = x -1; 
                
                int[] destination = {(int)newX, (int)y};  
                robot.setDestination(destination);    
                IRMeasurement = robot.getIRMeasurement();
                
                }*/
                //Set the robot in manual mode 
                robot.setManualMode(true);
                System.out.println("Roboten er i manuelt modus");
              
            
            }
            currentNumberOfWorkers--;
            // Remove this thread from the current working threads
            tasksInProgress.remove(name);
        }
    }
        
    
    private class SlamRobotTaskWorker extends RobotTaskWorker {
        
        public SlamRobotTaskWorker(Robot robot, NavigationRobot navRobot, String name) {
            super(robot, navRobot, name);
        }
        
        @Override
        public void run() {
            ArrayList<MapLocation> frontierLocations = map.getFrontierLocations();
            ArrayList<MapLocation> possibleTargets = selectSpreadLocations(frontierLocations);
            int currentOrientation = robot.getRobotOrientation();
            boolean assigned = false;
            // Assign the robot to move to one of the frontier locations if it has nothing else to do.
            while (!assigned && !robot.isGoingHome()) {
                Position robotPosition = new Position(robot.getPosition());
                MapLocation robotLocation = map.findLocationInMap(robotPosition);

                // Find the most optimal targetpoint in the map given the robots current location and the target points of the other robots
                MapLocation bestTarget = simpleFindBestTarget(currentOrientation, robotLocation, possibleTargets, name);
                
                // Done mapping
                if (bestTarget == null) {
                    robot.setGoingHome(true);
                    robot.setAtBase(false);
                    robot.setDock(true);
                    if (debug) {
                        // Sends the robot home if it is done mapping..
                        System.out.println(name + ": No targetpoint found");

                    }
                    break;
                }
                
                robotPosition = new Position(robot.getPosition());
                robotLocation = map.findLocationInMap(robotPosition);
                if (map.findCell(robotLocation).isRestricted()) {
                    if (debug) {
                        System.out.println(name + ": Robot in restricted position");
                    }
                    break;
                }
                // Search for a path between the robot and the best target point
                ArrayList<MapLocation> path = PathPlanningFunctions.findPath(map, bestTarget, robotLocation);
                // If no path to bestTarget is found remove bestTarget from possibleTargets
                if (path == null) {
                    for (int i = 0; i < possibleTargets.size(); i++) {
                        if (MapLocation.equals(bestTarget, possibleTargets.get(i))) {
                            possibleTargets.remove(i);
                            break;
                        }
                    }
                    // If there are no targets left in possibleTargets give up and try agin next iteration
                    if (possibleTargets.size() == 0) {
                        if (debug) {
                            System.out.println(name + ": No targetpoints reachable. Waiting");
                        }
                        break;
                    }
                    if (debug) {
                        System.out.println(name + ": Target unreachable. Finding new target");
                    }
                } // If a path is found find waypoints along the way and set destination for the robot
                else {
                    //computeUtility(bestTarget, robotLocation, currentOrientation, name, true);
                    if (debug) {
                        System.out.println(name + ": Path found");
                    }
                    ArrayList<Position> newWaypoints = PathPlanningFunctions.generateWaypoints(map, path);
                    if (newWaypoints.isEmpty()) {
                        if (debug) {
                            System.out.println(name + ": Unable to generate waypoints");
                        }
                        temporaryTargets.remove(name);
                        break;
                    }
                    currentTargets.put(name, bestTarget);
                    temporaryTargets.remove(name);
                    navRobot.addWaypoints(newWaypoints);
                    Position destinationPos = newWaypoints.get(newWaypoints.size() - 1);
                    int[] destination = {(int) Math.round(destinationPos.getXValue()), (int) Math.round(destinationPos.getYValue())};
                    robot.setDestination(destination);
                    assigned = true;
                }
                /*  Added to remove the need of goHome btn in RobotIdGUI  */
                robot.checkBattery();
            }
        }
    }

    ArrayList<MapLocation> selectSpreadLocations(ArrayList<MapLocation> frontierLocations) {
        ArrayList<MapLocation> spreadLocations = new ArrayList<MapLocation>();
        for (MapLocation target : frontierLocations) {
            boolean done = false;
            for (MapLocation existingTarget : spreadLocations) {
                double distance = MapLocation.distance(target, existingTarget) * map.getCellSize();
                if (distance < targetSpacing) {
                    done = true;
                    break;
                }
            }
            if (done) {
                continue;
            } else {
                spreadLocations.add(target);
            }
        }
        return spreadLocations;
    }

    MapLocation findBestTarget(int currentOrientation, MapLocation currentLocation, ArrayList<MapLocation> possibleTargetLocations, String robotName) {
        MapLocation bestTargetPoint = null;
        double bestUtility = Double.NEGATIVE_INFINITY;
        for (MapLocation targetPoint : possibleTargetLocations) {
            // map.findCell(targetPoint).setTarget();
            double utility = computeUtility(targetPoint, currentLocation, currentOrientation, robotName, false);
            if (utility > bestUtility) {
                bestUtility = utility;
                bestTargetPoint = MapLocation.copy(targetPoint);
                temporaryTargets.put(robotName, bestTargetPoint);
            }
        }
        return bestTargetPoint;
    }
    
    /**
     * Simple version of findBestTarget(). Used for SLAMrobot.
     * 
     * @param currentOrientation
     * @param currentLocation
     * @param possibleTargetLocations
     * @param robotName
     * @return 
     */
    MapLocation simpleFindBestTarget(int currentOrientation, MapLocation currentLocation, ArrayList<MapLocation> possibleTargetLocations, String robotName) {
        MapLocation bestTargetPoint = null;
        double bestUtility = Double.NEGATIVE_INFINITY;
        for (MapLocation targetPoint : possibleTargetLocations) {
            // map.findCell(targetPoint).setTarget();
            double utility = computeSimpleUtility(targetPoint, currentLocation, currentOrientation, robotName, false);
            if (utility > bestUtility) {
                bestUtility = utility;
                bestTargetPoint = MapLocation.copy(targetPoint);
                temporaryTargets.put(robotName, bestTargetPoint);
            }
        }
        return bestTargetPoint;
    }
    
    private double computeUtility(MapLocation target, MapLocation currentLocation, int currentOrientation, String robotName, boolean print) {
        int mapCellSize = map.getCellSize();

        double exploration = map.countUnknownCellsAroundLocation(target, 30) * mapCellSize * mapCellSize;

        double turnDistance = MapLocation.angleBetween(currentLocation, target) - currentOrientation;
        turnDistance = Math.abs((turnDistance + 180) % 360 - 180);

        double distance = MapLocation.distance(target, currentLocation) * map.getCellSize();

        double tooNear = 0;
        if (distance < 5) {
            tooNear = Double.POSITIVE_INFINITY;
        }

        double distribution = 0;
        for (ConcurrentHashMap.Entry<String, MapLocation> entry : temporaryTargets.entrySet()) {
            if (entry.getKey() != robotName) {
                if (MapLocation.distance(target, entry.getValue()) * mapCellSize <= 50) {
                    distribution = Double.POSITIVE_INFINITY;
                } else {
                    distribution += 1 / MapLocation.distance(target, entry.getValue()) * mapCellSize;
                }
            }
        }
        for (ConcurrentHashMap.Entry<String, MapLocation> entry : currentTargets.entrySet()) {
            if (entry.getKey() != robotName) {
                if (MapLocation.distance(target, entry.getValue()) * mapCellSize <= 50) {
                    distribution = Double.POSITIVE_INFINITY;
                } else {
                    distribution += 1 / MapLocation.distance(target, entry.getValue()) * mapCellSize;
                }
            }
        }
        distribution = distribution / (currentTargets.size() + temporaryTargets.size() + 1);

        double lineOfSight = 1;
        ArrayList<MapLocation> shortestPath = MappingController.getLineBetweenPoints(currentLocation, target);
        for (MapLocation location : shortestPath) {
            if (!map.findCell(location).isWeaklyTargetable()) {
                lineOfSight = 0;
                break;
            }
        }

        double closeToWall = 0;
        if (map.findCell(target).isWeaklyRestricted()) {
            closeToWall = 1;
        }

        double weight1 = 0.2;
        double weight2 = 3;
        double weight3 = 40000;
        double weight4 = 300;
        double weight5 = 2000;
        double weight6 = 1;
        //return exploration*0.5-distance*5*+distribution*1+lineOfSight*1;
        if (print) {
            /*
            System.out.println("Base UTILITY:");
            System.out.println(exploration);
            System.out.println(distance);
            System.out.println(distribution);
            System.out.println(lineOfSight);
            System.out.println(closeToWall);
             */
            System.out.println("Calculated UTILITY:");
            System.out.println(weight1 * exploration);
            System.out.println(weight2 * distance);
            System.out.println(weight3 * distribution);
            System.out.println(weight4 * lineOfSight);
            System.out.println(weight5 * closeToWall);
            System.out.println(weight6 * turnDistance);
            System.out.println("Finished printing UTILITY");
        }
        return weight1 * exploration - weight2 * distance - weight3 * distribution + weight4 * lineOfSight - weight5 * closeToWall - tooNear - weight6 * turnDistance;
    }
    
    /**
     * Simplified version of the computeUtility()-method. Some variables are
     * not considered. This method does not take into account the targets
     * of other robots in the system.
     * 
     * @param target
     * @param currentLocation
     * @param currentOrientation
     * @param robotName
     * @param print
     * @return 
     */
    private double computeSimpleUtility(MapLocation target, MapLocation currentLocation, int currentOrientation, String robotName, boolean print) {
        int mapCellSize = map.getCellSize();

        // This should be simplified
        double exploration = map.countUnknownCellsAroundLocation(target, 30) * mapCellSize * mapCellSize;

        double turnDistance = MapLocation.angleBetween(currentLocation, target) - currentOrientation;
        turnDistance = Math.abs((turnDistance + 180) % 360 - 180);

        double distance = MapLocation.distance(target, currentLocation) * map.getCellSize();

        double tooNear = 0;
        if (distance < 5) {
            tooNear = Double.POSITIVE_INFINITY;
        }
        
        double distribution = 0;
        /* We do not account for distribution
        for (ConcurrentHashMap.Entry<String, MapLocation> entry : temporaryTargets.entrySet()) {
            if (entry.getKey() != robotName) {
                if (MapLocation.distance(target, entry.getValue()) * mapCellSize <= 50) {
                    distribution = Double.POSITIVE_INFINITY;
                } else {
                    distribution += 1 / MapLocation.distance(target, entry.getValue()) * mapCellSize;
                }
            }
        }
        
        for (ConcurrentHashMap.Entry<String, MapLocation> entry : currentTargets.entrySet()) {
            if (entry.getKey() != robotName) {
                if (MapLocation.distance(target, entry.getValue()) * mapCellSize <= 50) {
                    distribution = Double.POSITIVE_INFINITY;
                } else {
                    distribution += 1 / MapLocation.distance(target, entry.getValue()) * mapCellSize;
                }
            }
        }
        distribution = distribution / (currentTargets.size() + temporaryTargets.size() + 1);
        */

        double lineOfSight = 1;
        ArrayList<MapLocation> shortestPath = MappingController.getLineBetweenPoints(currentLocation, target);
        for (MapLocation location : shortestPath) {
            // Checks if there are locations marked as obstructed in the line
            if (map.getObstructed().contains(location)) {
                lineOfSight = 0;
                break;
            }
            // Checks if any of the locations in the line are frontiers. This
            // means that the target is on the other side of unexplored territory.
            // But it could also just be two frontiers next to eachother, check
            // the behaviour of this.
            if (map.getFrontiers().contains(location)) {
                lineOfSight = 0;
                break;
            }
        }

        double closeToWall = 0;
        if (map.isRestricted(target)) {
            closeToWall = 1;
        }

        double weight1 = 0.2;
        double weight2 = 3;
        double weight3 = 40000;
        double weight4 = 300;
        double weight5 = 2000;
        double weight6 = 1;
        //return exploration*0.5-distance*5*+distribution*1+lineOfSight*1;
        if (print) {
            /*
            System.out.println("Base UTILITY:");
            System.out.println(exploration);
            System.out.println(distance);
            System.out.println(distribution);
            System.out.println(lineOfSight);
            System.out.println(closeToWall);
             */
            System.out.println("Calculated UTILITY:");
            System.out.println(weight1 * exploration);
            System.out.println(weight2 * distance);
            System.out.println(weight3 * distribution);
            System.out.println(weight4 * lineOfSight);
            System.out.println(weight5 * closeToWall);
            System.out.println(weight6 * turnDistance);
        }
        return weight1 * exploration - weight2 * distance - weight3 * distribution + weight4 * lineOfSight - weight5 * closeToWall - tooNear - weight6 * turnDistance;
    }
}
