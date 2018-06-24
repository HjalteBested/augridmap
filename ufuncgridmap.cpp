/***************************************************************************
 *   Copyright (C) 2018 by Hjalte Bested Møller and DTU                    *
 *   hjalte.moller@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "ufuncgridmap.h"

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncNear' with your classname */
  return new UFuncGridMap();
}
#endif


bool UFuncGridMap::setResource(UResBase * resource, bool remove) { // load resource as provided by the server (or other plugins)
    bool result = true;

    if (resource->isA(UResPoseHist::getOdoPoseID())) { // pointer to server the resource that this plugin can provide too
        // but as there might be more plugins that can provide the same resource
        // use the provided
        if (remove)
            // the resource is unloaded, so reference must be removed
            poseHist = NULL;
        else if (poseHist != (UResPoseHist *) resource)
            // resource is new or is moved, save the new reference
            poseHist = (UResPoseHist *) resource;
        else
            // reference is not used
            result = false;
    }

    // other resource types may be needed by base function.
    result = UFunctionBase::setResource(resource, remove);
    return result;
}

UFuncGridMap::UFuncGridMap(){ // initialization of variables in class - as needed
    setCommand("resetmap updatemap showmap", "gridmapif", "gridmap (Compiled " __DATE__ " " __TIME__ " by Hjalte Bested Møller)");
    createBaseVar();
    gridMap.resize(10,10,0.06);
    gridMap.makeStrel(0.3-0.06); // Default width of SMR - cellSize
    gridMap.fillMode = -1;
    gridMap.clearMode = 0;
    gridMap.astar.unknownAsObstacle = true;
    gridMap.astar.krep = 10.0;
    gridMap.astar.obstThresh = 10;

    // namedWindow( "Path", WINDOW_AUTOSIZE ); // This should be removed or moved to a user callable function!
    createBaseVar();
    poseIndex = 0;
    lastScanTime.setTime(-1);
    dataID=0;

    maxAstarIter = 1e4;
    firstRun = true;
    dist_driven_since_update = 0;
    angle_turned_since_update = 0;
    targetDistance = 1e6;

    drv   = 0.1;
    ang   = 0.2;
    dist  = 1;
    maxls = 3.8;
    maxwp = 2;
}

UFuncGridMap::~UFuncGridMap() { // possibly remove allocated variables here - if needed

}

const int MVL = 200;
char value[MVL];

bool UFuncGridMap::handleCommand(UServerInMsg * msg, void * extra) { // message is unhandled
    //const int MVL = 200;
    //char value[MVL];
    bool result = false;

    bool gotamitgain = msg->tag.getAttValue("amitgain", value, MVL);
    if(gotamitgain) gridMap.astar.amitgain = strtod(value, NULL);

    bool gotallowdiagonal = msg->tag.getAttValue("allowdiag", value, MVL);
    if(gotallowdiagonal) gridMap.astar.allowDiagonal = strtod(value, NULL);

    bool gotmaxiter = msg->tag.getAttValue("maxiter", value, MVL);
    if(gotmaxiter) maxAstarIter = long(strtod(value, NULL));

    bool gotobstrep = msg->tag.getAttValue("obstrep", value, MVL);
    if(gotobstrep) gridMap.astar.krep = int(strtod(value, NULL));

    bool obstslope = msg->tag.getAttValue("obstslope", value, MVL);
    if(obstslope) gridMap.astar.p = int(strtod(value, NULL));

    bool gotobstthresh = msg->tag.getAttValue("obstthresh", value, MVL);
    if(gotobstthresh) gridMap.astar.obstThresh = int(strtod(value, NULL));

    // Test for the handled commands, and call a function to do the job
    // the tagname is not case sensitive - see the library documentation for
    // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
    if (msg->tag.isTagA("resetmap"))
        result = handleReset(msg, extra);
    else if (msg->tag.isTagA("updatemap"))
        result = handleUpdateMap(msg, extra);
    else if (msg->tag.isTagA("showmap"))
        result = handleShowMap(msg, extra);
    // else
        // sendDebug(msg, "Command not handled (by me)");
    return result;
}


///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
bool UFuncGridMap::handleReset(UServerInMsg * msg, void * extra){
    const int MVL = 200;
    char value[MVL];

    // Default Values
    int mapwidth = 10;
    int mapheight = 10;
    float cellsize = 0.05;
    double robotwidth = 0.3; // Width of SMR


    bool gotmapwidth = msg->tag.getAttValue("width", value, MVL);
    if(gotmapwidth){
      mapwidth=strtod(value, NULL);
    }
    bool gotmapheight = msg->tag.getAttValue("height", value, MVL);
    if(gotmapheight){
      mapheight=strtod(value, NULL);
    }
    bool gotcellsize = msg->tag.getAttValue("cellsize", value, MVL);
    if(gotcellsize){
      cellsize=strtod(value, NULL);
    }
    bool gotwrapmap = msg->tag.getAttValue("wrapmap", value, MVL);
    if(gotwrapmap){
      gridMap.setWrapMap(strtod(value, NULL) > 0);
    }

    // Resize the memory containers of the map and initialize all variables.
    gridMap.resize(mapwidth,mapheight,cellsize);
    cout << "GridMap:  H:" << gridMap.heightInMeters() << "m W:" << gridMap.widthInMeters() << "m with cellSize:" << gridMap.cellSize << "m^2 ---> H:" << gridMap.height << " W:" << gridMap.width << " Size:" << gridMap.size  << endl;

    // Set offset the zero position in meters
    bool gotxoffset = msg->tag.getAttValue("xoffset", value, MVL);
    if(gotxoffset) gridMap.xoffset = strtod(value, NULL);
    bool gotyoffset = msg->tag.getAttValue("yoffset", value, MVL);
    if(gotyoffset) gridMap.yoffset = strtod(value, NULL);

    // Make structuring element for Map Dialation:
    bool gotrobotwidth = msg->tag.getAttValue("robotwidth", value, MVL);
    if(gotrobotwidth) robotwidth=strtod(value, NULL);

    gridMap.makeStrel(robotwidth);
    cout << "Dialation Stucturing Element Created with size: " << robotwidth << "m" << endl;

    // Reset Bookkeeping variables
    firstRun = true;
    dist_driven_since_update = 0;
    angle_turned_since_update = 0;
    dataID=0;

    return true;
}

bool UFuncGridMap::writeMapImage(int i, vector<MapNode *> newpath){
    const int MVL = 100;
    char filename[MVL];
    snprintf(filename, MVL, "GridMap%i.png",i);

    mapToDraw = (gridMap.mapData+1);
    mapToDraw.convertTo(mapToDraw,CV_8UC3);
    mapToDraw *= 127;
    mapToDraw += gridMap.dialatedMap * 0.15;
    cvtColor(mapToDraw, mapToDraw, COLOR_GRAY2BGR);
    gridMap.astar.drawPath(mapToDraw, true);
    gridMap.astar.drawNodes(mapToDraw, newpath);

    resize(mapToDraw, mapToWrite, Size(gridMap.height*2, gridMap.width*2), 0, 0, INTER_NEAREST);
    // Create OpenCV Windows
    imwrite(filename, mapToWrite);

}

bool UFuncGridMap::handleUpdateMap(UServerInMsg * msg, void * extra){
    // handle a plugin command
    const int numLaserPar=30;
    const int MRL = 1500;
    char reply[MRL];
    const int MVL = 100;
    char value[MVL];
    int nWaypoints=0;
    ULaserData * data;
    UPose robotPose;
    UPose newRobotPose;
    int nScanLines = 0;
    double phi,r;
    int i;
    double wayp[numLaserPar];
    bool gotprint = msg->tag.getAttValue("print", NULL, 0);
    bool gotwriteimage = msg->tag.getAttValue("writeimage", NULL, 0);
    bool newPathNeeded=false;

    bool gotx = msg->tag.getAttValue("x", value, MVL);
    double x_target = strtod(value, NULL);
    bool goty = msg->tag.getAttValue("y", value, MVL);
    double y_target = strtod(value, NULL);
    bool gotxy = gotx && goty;

    // check for parameters - one parameter is tested for - 'help'
    bool ask4help = msg->tag.getAttValue("help", value, MVL);
    if (ask4help){ // create the reply in XML-like (html - like) format
        sendHelpStart(msg, "waypobst");
        sendText("--- available waypobst options\n");
        sendText("help            This message\n");
        sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
        sendText("device=N        Laser device to use (see: SCANGET help)\n");
        sendText("see also: SCANGET and SCANSET\n");
        sendHelpDone();
    }
  else
  { // do some action and send a reply
    data = getScan(msg, (ULaserData*)extra);
    if(data->isValid()){ // make analysis for closest measurement
        ULaserDevice * device = getDevice(msg, data);
        scanPoseR = device->getDevicePose();

        // Get scanner data
        nScanLines = data->getRangeCnt();
        laserScanner.resize(nScanLines);
        for(int i=0; i<nScanLines; i++){
            r = data->getRangeMeter(i);
            phi = data->getAngleRad(i);
            if(r < 0.025) r = 4;
            laserScanner.setScanPoint(i,phi,r);
        }
        // Get Robot Pose in Odo
        newRobotPose = poseHist->getPoseAtTime(data->getScanTime());

        // Compute difference from last position
        double dx = newRobotPose.x-robotPose.x;
        double dy = newRobotPose.y-robotPose.y;
        double dth = newRobotPose.h-robotPose.h;
        dist_driven_since_update  += sqrt(dx*dx+dy*dy);
        angle_turned_since_update += dth;

        // Set new position
        robotPose = newRobotPose;
        double& x = robotPose.x;
        double& y = robotPose.y;
        double& th = robotPose.h;
        double cs = cos(th);
        double sn = sin(th);

        // Set Laserpose in world coordinates
        laserScanner.setPose(
            x + cs*scanPoseR.x - sn*scanPoseR.y,
            y + cs*scanPoseR.y + sn*scanPoseR.x,
            th);

        // Find the robot position on GridMap coordinates
        Point robotCell = gridMap.worldToCell(robotPose.x,robotPose.y);
        Point scanCell  = gridMap.worldToCell(laserScanner.pose(0),laserScanner.pose(1));

        // Print Global Pose of robot, scanner and the robot cell
        if(gotprint){
            cout << "robotPose = (" << x << "," << y << "," << th << ")" << ", ";
            cout << "scanPose  = (" << laserScanner.pose(0) << "," << laserScanner.pose(1) << "," << laserScanner.pose(2) << ")" << ", ";
            cout << "robotCell: " << robotCell << endl;
        }

        // Updata Map - This must happen frequently (> 5Hz) !
        gridMap.updateMap(&laserScanner, 3.8);
        gridMap.setLine(robotCell.x, robotCell.y,scanCell.x,scanCell.y,0,2);
        // Compute the dialated map and the distance transform
        gridMap.transform();


        if(waypoints.size() > 1){
            dx = waypoints.back().x - robotPose.x;
            dy = waypoints.back().y - robotPose.y;
            targetDistance = sqrt(dx*dx+dy*dy);
        } else {
            targetDistance = 1e6;
        }

        // Do the calculations needed for route planning, conditional and with low frequency.
        if(firstRun || dist_driven_since_update > drv || abs(angle_turned_since_update) > ang || waypoints.size() < 2 || targetDistance < dist || true){
            cout << "dist_driven_since_update = " << dist_driven_since_update << ",angle_turned_since_update=" << angle_turned_since_update << ", targetdist=" << targetDistance << endl;
            cout << "drv = " << drv << ",ang=" << ang << ", dist=" << dist << endl;

            // Determine Waypoint
            if(gotxy){
                wayPointCell = gridMap.worldToCell(x_target,y_target);
                gridMap.astar.unknownAsObstacle = false;
                if(targetDistance < 0.2) return true;
            }
            else if(firstRun || dist_driven_since_update > drv || abs(angle_turned_since_update) > ang || waypoints.size() < 2 || targetDistance < dist || true){
                gridMap.astar.unknownAsObstacle = true;
                wayPointCell = gridMap.determineNextWaypointCellB(&laserScanner,2.5);
            }

            dist_driven_since_update  = 0;
            angle_turned_since_update = 0;
            firstRun = false;

            // Compute the best path using astar
            vector<MapNode *> path = gridMap.findpath(robotCell.x, robotCell.y, wayPointCell.x, wayPointCell.y, maxAstarIter);
            if(gridMap.astar.reachedTarget){
                newpath = gridMap.simplifyPath(path);
                waypoints = gridMap.pathToWorld(newpath);
                Point2f dxi = waypoints[1] - waypoints[0];
                double thdiff = th-atan2(dxi.y,dxi.x);
                if(abs(thdiff) <= 0.8*M_PI){
                    dataID++;
                    if(gotwriteimage) writeMapImage(dataID,newpath);
                }
            }
        }

        // if(gotwriteimage) writeMapImage(dataID,newpath);

        nWaypoints = waypoints.size()-1;
        if(gotprint){
            cout << "N=" << nWaypoints << " WayPoints(" << dataID << "):";
            for(int i=0; i<waypoints.size(); i++){
                cout << "->(" << waypoints[i].x << "," << waypoints[i].y << ")";
            }
            cout << endl;
        }

        for(int i=0; i<numLaserPar; i++){
            wayp[i]=0;
        }

        if(nWaypoints > 14) nWaypoints = 14;
        for(int i=0; i<nWaypoints; i++){
            wayp[2*i]   = waypoints[i+1].x;
            wayp[2*i+1] = waypoints[i+1].y;
        }


      /**
      "Normal" XML reply format */
      /*      snprintf(reply, MRL, "<%s range=\"%g\" azimuth=\"%g\" x=\"%g\" y=\"%g\" today=\"true\"/>\n",
               msg->tag.getTagName(), minRange, minAngle,
               cos(minAngle * M_PI / 180.0) * minRange,
               sin(minAngle * M_PI / 180.0) * minRange);*/
      /**
      SMRDEMO reply format */
      /*
      snprintf(reply, MRL, "<laser  l0=\"%g\"    l1=\"%g\"  l2=\"%g\"  l3=\"%g\"  l4=\"%g\"  l5=\"%g\" l6=\"%g\"  l7=\"%g\" l8=\"%g\" />\n",
                        wayp[0],wayp[1],wayp[2],wayp[3],wayp[4],wayp[5],wayp[6],wayp[7],wayp[8]);
                        */
      snprintf(reply, MRL, "<laser  l0=\"%i\" l1=\"%i\" l2=\"%g\" l3=\"%g\" l4=\"%g\" l5=\"%g\" l6=\"%g\" l7=\"%g\" l8=\"%g\" l9=\"%g\" l10=\"%g\" l11=\"%g\" l12=\"%g\" l13=\"%g\" l14=\"%g\" l15=\"%g\" l16=\"%g\" l17=\"%g\" l18=\"%g\" l19=\"%g\" l20=\"%g\" l21=\"%g\" l22=\"%g\" l23=\"%g\" l24=\"%g\" l25=\"%g\" l26=\"%g\" l27=\"%g\" l28=\"%g\" l29=\"%g\"  />\n",
                          dataID,nWaypoints,wayp[0],wayp[1],wayp[2],wayp[3],wayp[4],wayp[5],wayp[6],wayp[7],wayp[8],wayp[9],wayp[10],wayp[11],wayp[12],wayp[13],wayp[14],wayp[15],wayp[16],wayp[17],wayp[18],wayp[19],wayp[20],wayp[21],wayp[22],wayp[23],wayp[24],wayp[25],wayp[26],wayp[27]);
      // send this string as the reply to the client
      sendMsg(msg, reply);
      for(i = 0; i < 9; i++)
        var_wayp->setValued(wayp[i], i);
    }
    else
      sendWarning(msg, "No scandata available");
  }
  // return true if the function is handled with a positive result
  return true;
}


bool UFuncGridMap::handleShowMap(UServerInMsg * msg, void * extra){
    const int MVL = 100;
    char value[MVL];
    int time=0;

    bool gottime = msg->tag.getAttValue("time", value, MVL);
    if(gottime){
      time=strtod(value, NULL);
    }

    mapToDraw = (gridMap.mapData+1);
    mapToDraw.convertTo(mapToDraw,CV_8UC3);
    mapToDraw *= 127;
    mapToDraw += gridMap.dialatedMap * 0.15;
    cvtColor(mapToDraw, mapToDraw, COLOR_GRAY2BGR);
    gridMap.astar.drawPath(mapToDraw, true);

    // resize(mapToDraw, mapToDraw, Size(gridMap.height*2, gridMap.width*2), 0, 0, INTER_NEAREST);
    // Create OpenCV Windows
    imwrite("GridMap.png", mapToDraw);

    return true;
}

void UFuncGridMap::createBaseVar()
{
  var_wayp = addVarA("wayp", "0 0 0 0 0 0 0 0 0", "d", "Value of each laser wayp. Updated by waypobst.");
}
