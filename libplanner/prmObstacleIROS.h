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
 
 

#if !defined(_PRMOBSTACLEIROS_H)
#define _PRMOBSTACLEIROS_H

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include "localplanner.h"
#include "planner.h"
#include "prmplanner.h"

using namespace std;
using namespace libSampling;

namespace libPlanner {
  namespace PRM{
   namespace prmObstaclesIROS{
    class prmObstaclesPlanner:public PRMPlanner {
	    public:
    	prmObstaclesPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler,
          WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize);
        ~prmObstaclesPlanner();

		bool trySolve();

		protected:
			vector<prmEdge*> edges;
			//!edge weights
			vector<cost> weights;
			//!bool to determine if the graph has been loaded
			bool _isGraphSet;
			KthReal _neighThress;
			int     _kNeighs;
			std::map<int, SampleSet*> _ccMap;
			int _labelCC;
			int _drawnLink; //!>flag to show which link path is to be drawn
			KthReal _probabilityConnectionIniGoal; //probability to connect last samp`le to init and goal samp`les

		private:
			prmObstaclesPlanner();
			//!boost graph
			prmGraph *g;
			//!solution to query
			list<prmVertex> shortest_path;
			//!pointer to the samples of cspace to be used by the distance_heuristic function used in A*
			vector<location> locations;
	  };
   }
  }
}

#endif  //_MYPRMPLANNER_H

