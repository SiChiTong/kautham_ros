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
 
#include "inversekinematic.h" 

namespace libProblem {
  InverseKinematic::InverseKinematic(Robot* const rob){
    _robot = rob;
	}

  /*KthReal InverseKinematic::getParameter(string key){
    HASH_S_K::iterator it = _parameters.find(key);
    if(it != _parameters.end() )
      return it->second;
    else
      throw -1;
  }

  bool InverseKinematic::setParameter(string key, KthReal value){
    HASH_S_K::iterator it = _parameters.find(key);
    if(it != _parameters.end() ){
      it->second = value;
      return true;
    }
    return false;
  }

  string InverseKinematic::getParametersAsString(){
    stringstream par;
    string p;
    par.precision(10);
	  HASH_S_K::iterator it;
	  for(it = _parameters.begin(); it != _parameters.end(); ++it) { 
		  par << it->first << "|";
		  par << it->second << "|"; 
	  }
    p = par.str();
	  return p.substr(0,p.length()-1);
  }

  bool InverseKinematic::setParametersFromString(string par){
    string prop="", sval="";
    KthReal val = 0.0;
    HASH_S_K::iterator it;
    size_t from=0, to=0;
    while(from != string::npos){
      to = par.find('|', from + 1);
      prop = par.substr(from, to);
      from = to + 1;
      to = par.find('|', from );
      if( to != string::npos ) 
        sval = par.substr(from, to);
      else
        sval = par.substr(from);
      val = (KthReal)atof(sval.c_str());
	    it = _parameters.find(prop);
	    if(it != _parameters.end()) 
		    it->second = val;
      else 
        throw -1;
      from = to ;
    }
    return setParameters();
  }*/

  //! This method receives the target as a parameter and the most used way is
  //! configured the target as a se3.coordinates vector that includes the 
  //! position and the orientation as a quaternion.
  void InverseKinematic::setTarget(vector<KthReal> &target, const string& param){
    if(param != "" ) setParametersFromString( param );
    _target.clear();
    for(size_t i =0; i< target.size(); i++)
      _target.push_back(target.at(i));
  }

  
  void InverseKinematic::setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist){
	   //loads the target: the tcp transform
    _target.clear();
    for(size_t i =0; i< target.size(); i++)
      _target.push_back(target.at(i));  
	  
	  //masterconf not used -  maintainSameWrist not used
	  //the derived classes may use it to complete the target with configuration parameters
  }

  RobLayout& InverseKinematic::getRobLayout(vector<KthReal> &target){
    return _robLay;
  }

}

