/***************************************************************************
*               Generated by StarUML(tm) C++ Add-In                        *
***************************************************************************/
/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
/***************************************************************************
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
 
 
 

#include "obstacle.h"
//#include "robot.h"

namespace libProblem {
  Obstacle::Obstacle(string modFile, KthReal pos[3], KthReal ori[4], KthReal scale, LIBUSED lib, bool flagCol){
	  enablecollisions = flagCol;
    libs = lib;
    switch(libs){
      case IVPQP:
      case IVSOLID:
        element = new IVPQPElement(modFile,scale);
    }
		element->setPosition(pos);
		element->setOrientation(ori);
		for(int i=0; i<3; i++)
			linVel[i] = angVel[i] = (KthReal) 0.0;
	}
	
	void Obstacle::setLinVelocity(KthReal vel[3]){
    for(int i=0; i<3; i++)
			linVel[i] = vel[i];
	}

	void Obstacle::setAngVelocity(KthReal vel[3]){
		for(int i=0; i<3; i++)
			angVel[i] = vel[i];
	}

  void* Obstacle::getModel(bool tran){
    switch(libs){
      case IVPQP:
      case IVSOLID:
        return (void*)((IVElement*)element)->ivModel(tran);
        break;
      default:
        return NULL;
    }
  }

  void* Obstacle::getModelFromColl(bool tran){
    switch(libs){
      case IVPQP:
        return (void*)((IVPQPElement*)element)->getIvFromPQPModel(tran);
        break;
      case IVSOLID:
        break;
      default:
        return NULL;
    }
    return NULL;
  }
	
}


