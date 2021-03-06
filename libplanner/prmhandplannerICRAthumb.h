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
 
#include "prmhandplanner.h"

#if !defined(_PRMHANDPLANNERICRATHUMB_H)
#define _PRMHANDPLANNERICRATHUMB_H
 namespace libPlanner {
  namespace PRM{
      typedef std::pair<KthReal, KthReal> thumbLimits;
	  

	class PRMHandPlannerICRAthumb:public PRMHandPlanner{
		public:
			PRMHandPlannerICRAthumb(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
				WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize, int cloundSize, KthReal cloudRad, int numHC);
	  
			~PRMHandPlannerICRAthumb();
      
			bool  setParameters();
			bool  trySolve();
      bool getHandConfig(vector<KthReal>& coord, thumbLimits *thlimits, bool randhand, int numPMDs);
			void pruneNeighs(Sample* smp); 
			bool getSampleInGoalRegion(); //reimplemented
			void moveAlongPath(unsigned int step); //reimplemented
			void saveData();//reimplemented
//      void setIniGoal();//reimplemented
      void writeFiles(FILE *fpr, FILE *fph, RobConf* joints);
			void computeMaxSteps(KthReal radius, int *bits, int *steps);

	 
		private:
			vector<thumbLimits*> _vectthl;
			int	_numberHandConf;
			int _incrementalPMDs;

	};	
  }
}
 
#endif  //_PRMHANDPLANNERICRATHUMB_H

