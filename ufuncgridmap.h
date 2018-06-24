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
#ifndef UFUNC_GRIDMAP_H
#define UFUNC_GRIDMAP_H

#include <cstdlib>

#include <ulms4/ufunclaserbase.h>
#include <urob4/uresposehist.h>

#include "./GridMap/src/GridMap.hpp"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Laserscanner plugin for mapping and planning and solving one-way labyrinths
 * @author Hjalte Bested Møller
*/
class UFuncGridMap : public UFuncLaserBase
{
public:
  /**
  Constructor */
  UFuncGridMap();
  virtual ~UFuncGridMap();

  GridMap gridMap;
  LaserScanner laserScanner;
  // Mat mapToPlot;
  Mat mapToDraw;
  Mat mapToWrite;

  float drv;
  float ang;
  float dist;
  float maxls;
  float maxwp;

  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);



  /** Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  virtual bool setResource(UResBase * resource, bool remove);

  protected:
    void createBaseVar();
    UVariable *var_wayp;

    UResPoseHist * poseHist;
    int poseIndex;
    UTime lastScanTime;
    UPosRot scanPoseR;

    // Command Handlers
    bool handleReset(UServerInMsg * msg, void * extra);
    bool handleUpdateMap(UServerInMsg * msg, void * extra);
    bool handleShowMap(UServerInMsg * msg, void * extra);
    bool writeMapImage(int i, vector<MapNode *> newpath);

    vector<Point2f> waypoints;
    int dataID;
    bool firstRun;
    float dist_driven_since_update;
    float angle_turned_since_update;
    unsigned int maxAstarIter;
    float targetDistance;
    Point wayPointCell;
    vector<MapNode *> newpath;
};



#endif

