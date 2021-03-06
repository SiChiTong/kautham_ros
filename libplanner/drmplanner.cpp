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
 
 


#include <libproblem/ivworkspace.h>
#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/algorithm/string.hpp>
#include "localplanner.h"
#include "drmplanner.h"
#include <stdio.h>
#include <libutil/pugixml/pugixml.hpp>

#include <libsampling/lcprng.h>


#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>

using namespace libSampling;
using namespace SDK;
using namespace pugi;

namespace libPlanner {
  namespace DRM{
    DRMPlanner::DRMPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize):
              Planner(stype, init, goal, samples, sampler, ws, lcPlan, ssize){

      _guiName = _idName = "DRM";
      _neighThress = 1.5;//0.5//50000.0;
      _kNeighs = 10;
      _isGraphSet = false;
	    _maxNumSamples = 300;//1000;
	    _speedFactor = 1;
	    _solved = false;
	    setStepSize(ssize);//also changes stpssize of localplanner
      _drawnLink = -1; //the path of last link is defaulted
	  _probabilityConnectionIniGoal = 0.1;
	  
      _samplerHalton = new HaltonSampler(_wkSpace->getDimension());
	  _levelSDK = 5;
      _samplerSDK = new SDKSampler(_wkSpace->getDimension(), _levelSDK);
      //_samplerGaussian = new GaussianSampler(_wkSpace->getDimension());
	  KthReal sigma = 0.1;
      _samplerGaussian = new GaussianSampler(_wkSpace->getDimension(),sigma,_wkSpace);
      _samplerGaussianLike = new GaussianLikeSampler(_wkSpace->getDimension(), _levelSDK,_wkSpace);
  	  
	  _samplertype = 1;
	  setsampler(_samplertype);
      addParameter("Step Size", ssize);
      addParameter("Sampler 1(sdk),2(h),3(g),4(gl),5(r)", _samplertype);
      addParameter("Neigh Thresshold", _neighThress);
      addParameter("Max. Neighs", _kNeighs);
      addParameter("Max. Samples", _maxNumSamples);
      addParameter("Speed Factor", _speedFactor);
      addParameter("Drawn Path Link",_drawnLink);
	  addParameter("P(connect to Ini-Goal)",_probabilityConnectionIniGoal);

	    _labelCC=0;

	    _samples->setTypeSearch(ANNMETHOD);//(BRUTEFORCE);//
	    _samples->setWorkspacePtr(_wkSpace);
	    _samples->setANNdatastructures(_kNeighs, _maxNumSamples);

      for(int i=0; i<_wkSpace->robotsCount();i++)
        _wkSpace->getRobot(i)->setLinkPathDrawn(_drawnLink);
    }


	//!Void destructor
	DRMPlanner::~DRMPlanner(){
			
	}

	//!Function to set the parameters of the DRM planner 
    bool DRMPlanner::setParameters(){
      try{
        HASH_S_K::iterator it = _parameters.find("Step Size");
        if(it != _parameters.end())
			    setStepSize(it->second);//also changes stpssize of localplanner
        else
          return false;

		it = _parameters.find("P(connect to Ini-Goal)");
        if(it != _parameters.end())
          _probabilityConnectionIniGoal = it->second;
        else
          return false;

		it = _parameters.find("Sampler 1(sdk),2(h),3(g),4(gl),5(r)");
		if(it != _parameters.end()){
          if(it->second != _samplertype) setsampler(it->second);
		}
        else
          return false;

        it = _parameters.find("Speed Factor");
        if(it != _parameters.end())
          _speedFactor = it->second;
        else
          return false;

        it = _parameters.find("Drawn Path Link");
		if(it != _parameters.end()){
          _drawnLink = it->second;
          for(int i=0; i<_wkSpace->robotsCount();i++)
            _wkSpace->getRobot(i)->setLinkPathDrawn(_drawnLink);
		}else
          return false;

        it = _parameters.find("Max. Samples");
        if(it != _parameters.end()){
          _maxNumSamples = it->second;
		      _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
			 _samples->loadAnnData();
		    }else
          return false;

        it = _parameters.find("Neigh Thresshold");
        if(it != _parameters.end())
          _neighThress = it->second;
        else
          return false;

        it = _parameters.find("Max. Neighs");
        if(it != _parameters.end()){
          _kNeighs = (int)it->second;
		      _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
			 _samples->loadAnnData();
		    }else
          return false;
      }catch(...){
        return false;
      }
      return true;
    }

	SoSeparator *DRMPlanner::getIvCspaceScene()
	{
		if(_wkSpace->getDimension()==2)
		{
			//_sceneCspace = ((IVWorkSpace*)_wkSpace)->getIvScene();
			_sceneCspace = new SoSeparator();
			_sceneCspace->ref();
		}
		else _sceneCspace=NULL;
		return Planner::getIvCspaceScene();
		
	}

	void DRMPlanner::drawCspace()
	{
		if(_wkSpace->getDimension()==2)
		{			
			//first delete whatever is already drawn
			while (_sceneCspace->getNumChildren() > 0)
			{
				_sceneCspace->removeChild(0);
			}

			//draw points
			SoSeparator *psep = new SoSeparator();
			SoCoordinate3 *points  = new SoCoordinate3();
			SoPointSet *pset  = new SoPointSet();
			vector<Sample*>::iterator itera = _samples->getBeginIterator();
			int i=0;
			KthReal xmin=100000000.0;
			KthReal xmax=-100000000.0;
			KthReal ymin=100000000.0;
			KthReal ymax=-100000000.0;
			while((itera != _samples->getEndIterator()))
			{
				KthReal x=(*itera)->getCoords()[0];
				KthReal y=(*itera)->getCoords()[1];

				points->point.set1Value(i,x,y,0);

				if(x<xmin) xmin=x;
				if(x>xmax) xmax=x;
				if(y<ymin) ymin=y;
				if(y>ymax) ymax=y;

				itera++;
				i++;
			}
			SoDrawStyle *pstyle = new SoDrawStyle;
			pstyle->pointSize = 2;
			SoMaterial *color = new SoMaterial;
			color->diffuseColor.setValue(0.2,0.8,0.2);

			//draw samples
			psep->addChild(color);
			psep->addChild(points);
			psep->addChild(pstyle);
			psep->addChild(pset);

			_sceneCspace->addChild(psep);
			

			//draw edges: 
			SoSeparator *lsep = new SoSeparator();
			vector<prmEdge*>::iterator itC;
			for(itC = edges.begin(); itC != edges.end(); ++itC)
			{	
				SoCoordinate3 *edgepoints  = new SoCoordinate3();
				edgepoints->point.set1Value(0,points->point[(*itC)->first]);
				edgepoints->point.set1Value(1,points->point[(*itC)->second]);
				lsep->addChild(edgepoints);

				SoLineSet *ls = new SoLineSet;
				ls->numVertices.set1Value(0,2);//two values
				//cout<<"EDGE "<<(*itC)->first<<" "<<(*itC)->second<<endl;
				lsep->addChild(ls);
			}
			_sceneCspace->addChild(lsep);

			//draw path: 
			if(_solved)
			{
				SoSeparator *pathsep = new SoSeparator();
				list<prmVertex>::iterator spi = shortest_path.begin();
				list<prmVertex>::iterator spi_init = shortest_path.begin();

				for(++spi; spi != shortest_path.end(); ++spi)
				{	
					SoCoordinate3 *edgepoints  = new SoCoordinate3();
					edgepoints->point.set1Value(0,points->point[*spi_init]);
					edgepoints->point.set1Value(1,points->point[*spi]);
					pathsep->addChild(edgepoints);

					SoLineSet *ls = new SoLineSet;
					ls->numVertices.set1Value(0,2);//two values
					SoDrawStyle *lstyle = new SoDrawStyle;
					lstyle->lineWidth=2;
					SoMaterial *path_color = new SoMaterial;
					path_color->diffuseColor.setValue(0.8,0.2,0.2);
					pathsep->addChild(path_color);
					pathsep->addChild(lstyle);
					pathsep->addChild(ls);
					spi_init = spi;
				}
				_sceneCspace->addChild(pathsep);
			}


			//draw floor
			SoSeparator *floorsep = new SoSeparator();
			SoCube *cs = new SoCube();
			cs->width = xmax-xmin;
			cs->depth = (xmax-xmin)/50.0;
			cs->height = ymax-ymin;
			
			SoTransform *cub_transf = new SoTransform;
			SbVec3f centre;
			centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,-cs->depth.getValue());
			cub_transf->translation.setValue(centre);
			cub_transf->recenter(centre);	
			
			SoMaterial *cub_color = new SoMaterial;
			cub_color->diffuseColor.setValue(0.2,0.2,0.2);

			floorsep->addChild(cub_color);
			floorsep->addChild(cub_transf);
			floorsep->addChild(cs);
			_sceneCspace->addChild(floorsep);
		}
	}

	void DRMPlanner::setsampler(int i)
	{
		clearGraph();
		_samplertype = i;
		if(i==1) _samplerUsed = _samplerSDK;
		else if(i==2) _samplerUsed = _samplerHalton;
		else if(i==3) _samplerUsed = _samplerGaussian;
		else if(i==4) _samplerUsed = _samplerGaussianLike;
		else _samplerUsed = _sampler;
	}
    bool DRMPlanner::saveData(string path){
      //  First the Planner method is invoked to store the planner's 
      //  parameters and the samples in sample set. 
      if(Planner::saveData(path)){
      
        //  Then, the saved file is loaded again in order to appending 
        //  the planner own information
        xml_document doc;
        xml_parse_result result = doc.load_file(path.c_str());

        if(result){
          xml_node conNode = doc.child("Planner").append_child();
          conNode.set_name("Conectivity");
          vector<prmEdge*>::iterator itC;
          vector<cost>::iterator itW = weights.begin();
          stringstream ss;
          for(itC = edges.begin(); itC != edges.end(); ++itC){
            ss.str("");
            ss << (*itC)->first << " " << (*itC)->second;
            xml_node pairNode = conNode.append_child();
            pairNode.set_name("Pair");
            pairNode.append_child(node_pcdata).set_value(ss.str().c_str());
            ss.str("");
            ss << *itW++;
            pairNode.append_attribute("Weight") = ss.str().c_str();
          }

          //Adding the connected components
          xml_node compNode = doc.child("Planner").append_child();
          compNode.set_name("ConnComponents");
          compNode.append_attribute("size") = (int) _ccMap.size();
          map<int,SampleSet*>::iterator it;
          for ( it=_ccMap.begin(); it != _ccMap.end(); it++ ){
            xml_node ccNode = compNode.append_child();
            ccNode.set_name("Component");
            ccNode.append_attribute("name") = (*it).first;
            ccNode.append_attribute("size") = (*it).second->getSize();
            ss.str("");
            vector<Sample*>::iterator itera = (*it).second->getBeginIterator();
            for(itera = (*it).second->getBeginIterator(); itera != (*it).second->getEndIterator();
                ++itera)
              ss <<_samples->indexOf( (*itera) ) << " ";

            ccNode.append_child(node_pcdata).set_value(ss.str().c_str());
          }

          return doc.save_file(path.c_str());
        }
      }
      return false;
    }

    bool DRMPlanner::loadData(string path){
      // First, the current graph is reset.
      clearGraph();
      if(Planner::loadData(path)){
        //  If it is correct, the planner has loaded the 
        //  SampleSet with the that samples from the file.
        xml_document doc;
        xml_parse_result result = doc.load_file(path.c_str());
        if(result){
          xml_node tempNode = doc.child("Planner").child("Conectivity");
          prmEdge *e;
          for (pugi::xml_node_iterator it = tempNode.begin(); it != tempNode.end(); ++it){
            string sentence = it->child_value();
            vector<string> tokens;
            boost::split(tokens, sentence, boost::is_any_of("| "));
            if(tokens.size() != 2){
              std::cout << "Connectivity information is wrong. Graph is not set."
                  << std::endl;
              return false;
            }
            e = new prmEdge(atoi(tokens[0].c_str()), atoi(tokens[1].c_str()));
            edges.push_back(e);
			  	  weights.push_back((KthReal)(it->attribute("Weight").as_double()));
          } 
          loadGraph(); // Calling this method, the graph is regenerated.
          std::cout << "The PRM connectivity has been loaded correctly from the file." << std::endl;
          return true;
        }
      }
      return false;
    }

	  void DRMPlanner::saveData()
	  {  // Put here the code you want in order to save your data in an informal way.

	  }

	//!function that constructs the PRM and finds a solution path
    bool DRMPlanner::trySolve(){
      _solved = false;
      int count = 0;
      if( _isGraphSet ){  //If graph already available
        //If new configurations have been sampled the graph is rebuild
        if( _samples->changed())
        {
          clearGraph();
          _samples->findNeighs(_neighThress, _kNeighs);
          connectSamples();
          loadGraph();
        }
      }
      //No grah is already avaliable, then build it
      else
      {
        _samples->findNeighs(_neighThress, _kNeighs);
        connectSamples();
        loadGraph();
      }
      //try to find a path with the samples already available in the sampleSet
      //If found, smooth it.
      if( findPath() )
      {
        printConnectedComponents();
        smoothPath();
		drawCspace();
        _solved = true;
        count = _samples->getSize();
      }
      //if not found, keep trying until a max number of samples
      //iteratively growing the PRM
      else
      {

        LCPRNG* rgen = new LCPRNG(15485341);//15485341 is a big prime number
        count = _samples->getSize();
        Sample* smp = NULL;
        count = _samples->getSize();
        while( count < _maxNumSamples)
        {
          smp = NULL;
          do{
            if( smp != NULL ) delete smp;
            smp = _samplerUsed->nextSample();
            count++;
          }while(_wkSpace->collisionCheck(smp) == true);
          _samples->add(smp);
          double r=rgen->d_rand();
          if(r < _probabilityConnectionIniGoal) connectLastSample(_init);
          else if(r < 2*_probabilityConnectionIniGoal) connectLastSample(_goal);
          else connectLastSample();
          if( findPath() )
          {
            printConnectedComponents();
            smoothPath();
            cout << "Calls to collision-check = " << count <<endl;
            _solved = true;
			
			drawCspace();
            break;
          }
        }
      }

      printConnectedComponents();
      _triedSamples = count;
	  
	  drawCspace();
      return _solved;
    }



    //!Finds a solution path in the graph using A*
    bool DRMPlanner::findPath()
	{
		_solved = false;

		if(_init->getConnectedComponent() != _goal->getConnectedComponent()) return false;


		clearSimulationPath();
	    shortest_path.clear(); //path as a vector of prmvertex
	    _path.clear();//path as a vector of samples

	    prmVertex start = _samples->indexOf(_init);
	    prmVertex  goal = _samples->indexOf(_goal);

		//vector to store the parent information of each vertex
		vector<prmGraph::vertex_descriptor> p(num_vertices(*g));
		//vector with cost of reaching each vertex
		vector<cost> d(num_vertices(*g));

		try {
			// call astar named parameter interface
			astar_search(*g, start, 
                      distance_heuristic<prmGraph, cost, vector<location> >(locations, goal, _locPlanner),
                      predecessor_map(&p[0]).distance_map(&d[0]).
                      visitor(astar_goal_visitor<prmVertex>(goal)));

		}catch(DRM::found_goal fg){ // found a path to the goal
			_solved = true;
			//Load the vector shortest_path that represents the solution as a sequence of prmvertex
			for( prmVertex v = goal; ; v = p[v] )
			{
				shortest_path.push_front(v);
				if( p[v] == v ) break;
			}
			//Print solution path
			cout << "Shortest path from " << start << " to " << goal << ": ";
			cout << start;
			list<prmVertex>::iterator spi = shortest_path.begin();
			for(++spi; spi != shortest_path.end(); ++spi)
				cout << " -> " << *spi;
			cout << endl;


			//Load the vector _path that represents the solution as a sequence of samples
			list<prmVertex>::iterator spi2 = shortest_path.begin();
			_path.push_back(_samples->getSampleAt(start));  
			for(++spi2; spi2 != shortest_path.end(); ++spi2)
			{
				_path.push_back(_samples->getSampleAt(*spi2));
			}
			return true;
    }
      
		cout << "Didn't find a path from " << start << " to "
			 << goal << "!" << endl;
		return false;
    }




    //!Load boost graph data
    void DRMPlanner::loadGraph()
	{
	    int maxNodes = this->_samples->getSize();
		unsigned int num_edges = edges.size(); 
      
		// create graph
		g = new prmGraph(maxNodes);
		WeightMap weightmap = get(edge_weight, *g);

		for(std::size_t j = 0; j < num_edges; ++j) 
		{
			edge_descriptor e; 
			bool inserted;//when the efge already exisits or is a self-loop
						  //then this flag is set to false and the edge is not inserted 
			tie(e, inserted) = add_edge(edges[j]->first,edges[j]->second, *g);
			if(inserted) weightmap[e] = weights[j];
		}

		//locations are the pointer to the samples of cspace that 
		//are at each node of the graph and is used to compute the
		//distance when using the heuristic (function distance_heuristic)
		for(unsigned int i=0;i<num_vertices(*g); i++)
		    locations.push_back( _samples->getSampleAt(i) );

		_isGraphSet = true;
	}


   //!Update boost graph data
    void DRMPlanner::updateGraph()
	{
        unsigned int newVertices = _samples->getSize();
	    unsigned int oldVertices = num_vertices(*g);
        unsigned int newEdges = edges.size();
	    unsigned int oldEdges = num_edges(*g);

	    // add new vertices
		for(std::size_t i=oldVertices; i<newVertices;i++)
		{	
			prmVertex vd=add_vertex(*g);
		}
      
		WeightMap weightmap = get(edge_weight, *g);

		//add new edges
		for(std::size_t j = oldEdges; j < newEdges; j++) 
		{
			edge_descriptor e; 
			bool inserted;//when the efge already exisits or is a self-loop
						  //then this flag is set to false and the edge is not inserted 
			tie(e, inserted) = add_edge(edges[j]->first,
                                    edges[j]->second, *g);
			if(inserted) weightmap[e] = weights[j];
		}

		//locations are the pointer to the samples of cspace that 
		//are at each node of the graph and is used to compute the
		//distance when using the heuristic (function distance_heuristic)
		for(unsigned int i=oldVertices; i<newVertices; i++)
		    locations.push_back( _samples->getSampleAt(i) );
    }


	//! Computes neighbors for the last sample, tries to connect to them, and updates graph 
	//! If parameter connectToSmp is not NULL then the sample pointed is set as a neighbor
	//! of the sample to be connected.
	//! Finally adds the sample to the graph
    void DRMPlanner::connectLastSample(Sample* connectToSmp)
	{
	    int n;
	    Sample *smpFrom;
	    Sample *smpTo;
	    prmEdge *e; //prmEdge is a type defined in PRM.h as std::pair<int, int>
      
 		typedef std::pair<int, SampleSet*> ccPair;
	    
		int ccFrom; //label co connected component of intial sample
		int ccTo; //label co connected component of goal sample

		
		int indexFrom = _samples->getSize() - 1;
	    smpFrom = _samples->getSampleAt(indexFrom);
		
	    //set initial sample of local planner
		_locPlanner->setInitSamp(smpFrom);

		//if already labeled, return
		if(smpFrom->getConnectedComponent() != -1) return;

		smpFrom->setConnectedComponent(_labelCC);	//label sample with connected component
		SampleSet *tmpSS = new SampleSet();			//create new sample set
		tmpSS->add( smpFrom );						//add sample to sample set	
		_ccMap.insert(ccPair(_labelCC,tmpSS));		//create connected component as a labeled sample set
		_labelCC++;

		//compute neighs
		_samples->findNeighs(smpFrom, _neighThress, _kNeighs);
		if(connectToSmp != NULL)
		{
			//smpFrom->addNeigh( _samples->indexOf( connectToSmp ) );
			//add sample connectToSmp as the first neighbor of sample smpFrom (set distance = 0) 
			//smpFrom->addNeighOrdered(_samples->indexOf( connectToSmp ), 0.0, _kNeighs);
		
			// Modified to avoid the false order.
			KthReal dis = smpFrom->getDistance(connectToSmp, Kautham::CONFIGSPACE );
			smpFrom->addNeighOrdered(_samples->indexOf( connectToSmp ), dis, _kNeighs);
		}

		//srtart connecting with neighs
	    for(unsigned int j=0; j<smpFrom->getNeighs()->size(); j++)
		{
			n = smpFrom->getNeighs()->at(j);
			smpTo = _samples->getSampleAt(n);

			//if same connected component do no try to connect them
			if(smpFrom->getConnectedComponent() == smpTo->getConnectedComponent()) continue;

		    //set goal sample of local planner
			_locPlanner->setGoalSamp(smpTo);

		    //local planner collision checks the edge (if required)
			int canconnect=0;
			//if(smpFrom==goalSamp() || smpTo==goalSamp()) canconnect=1; //do not check
			//else canconnect = _locPlanner->canConect();
			canconnect = _locPlanner->canConect();
		    if( canconnect )
			{
				e = new prmEdge(indexFrom,n);
			  	edges.push_back(e);
			  	weights.push_back(_locPlanner->distance(smpFrom,smpTo));

				//set neigh sample with same label as current sample
				ccTo = smpTo->getConnectedComponent();
				ccFrom = smpFrom->getConnectedComponent();
				if(ccTo == -1)
				{
					smpTo->setConnectedComponent( ccFrom );
					_ccMap[ccFrom]->add( smpTo );
				}
				//set all samples of the connected component of the neigh sample to 
				//the same label as the current sample
				else
				{
					//unify lists under the lowest label
					if( ccFrom < ccTo)
					{
						vector<Sample*>::iterator itera = _ccMap[ccTo]->getBeginIterator();
						while((itera != _ccMap[ccTo]->getEndIterator()))
						{
							(*itera)->setConnectedComponent( ccFrom );
							_ccMap[ccFrom]->add( (*itera) );
							itera++;
						}
            _ccMap[ccTo]->clean();
						_ccMap.erase(ccTo);
					}
					else
					{
						vector<Sample*>::iterator itera = _ccMap[ccFrom]->getBeginIterator();
						while((itera != _ccMap[ccFrom]->getEndIterator()))
						{
							(*itera)->setConnectedComponent( ccTo );
							_ccMap[ccTo]->add( (*itera) );
							itera++;
						}
            _ccMap[ccFrom]->clean();
						_ccMap.erase(ccFrom);
					}
				}
			}
			else
			{
			  //cout<<": FAILED"<<endl;
			  //cout << "edge from " << i << " to " << n << " is NOT free" << endl;
			}
	    }

		//Adds the sample as node of the graph
		updateGraph();
    }


	//!connect samples - put weights 
    bool DRMPlanner::connectSamples(bool assumeAllwaysFree)
	{
	    int n;
	    Sample *smpFrom;
	    Sample *smpTo;
	    prmEdge *e; //prmEdge is a type defined in PRM.h as std::pair<int, int>
      
      cout << "CONNECTING  " << _samples->getSize() << " FREE SAMPLES" << endl;

      typedef std::pair<int, SampleSet*> ccPair;

      int ccFrom; //label co connected component of intial sample
      int ccTo;   //label co connected component of goal sample

	  	unsigned int max;
      if(_samples->getSize() < _maxNumSamples)
          max = _samples->getSize();
      else {
        max = _maxNumSamples;
        cout<<"connectSamples::Using a maximum of "<<max<<" samples"<<endl;
      }

	    for(unsigned int i=0; i<max; i++){
		    //cout<<"Connect sample "<<i<<" with:"<<endl;
        smpFrom = _samples->getSampleAt(i);

        //if not yet labeled, labelwith a new connected component
        if(smpFrom->getConnectedComponent() == -1){
          smpFrom->setConnectedComponent(_labelCC);	//label sample with connected component
          SampleSet *tmpSS = new SampleSet();       //create new sample set
          tmpSS->add( smpFrom );                    //add sample to sample set
          _ccMap.insert(ccPair(_labelCC,tmpSS));		//create connected component as a labeled sample set
          _labelCC++;
        }else
          continue;//already in a connected component

        //set initial sample of local planner
        _locPlanner->setInitSamp(smpFrom);

        //srtart connecting with neighs
		    for(unsigned int j=0; j<smpFrom->getNeighs()->size(); j++){
          n = smpFrom->getNeighs()->at(j);
          smpTo = _samples->getSampleAt(n);

          //if same connected component do no try to connect them
          if(smpFrom->getConnectedComponent() == smpTo->getConnectedComponent()) continue;

			    //set goal sample of local planner
          _locPlanner->setGoalSamp(smpTo);

			    //local planner collision checks the edge (if required)
			    if(assumeAllwaysFree || _locPlanner->canConect())
          {
			  		e = new prmEdge(i,n);
			  		edges.push_back(e);
			  		weights.push_back(_locPlanner->distance(smpFrom,smpTo));

            //set neigh sample with same label as current sample
            ccTo = smpTo->getConnectedComponent();
            ccFrom = smpFrom->getConnectedComponent();
            if(ccTo == -1)
            {
              smpTo->setConnectedComponent( ccFrom );
              _ccMap[ccFrom]->add( smpTo );
            }
            else                        //set all samples of the connected component of the neigh sample to
            {                           //the same label as the current sample
              if( ccFrom < ccTo)        //unify lists under the lowest label
              {
                vector<Sample*>::iterator itera = _ccMap[ccTo]->getBeginIterator();
                while((itera != _ccMap[ccTo]->getEndIterator()))
                {
                  (*itera)->setConnectedComponent( ccFrom );
                  _ccMap[ccFrom]->add( (*itera) );
                  //itera = NULL;
                  itera++;
                }
                 _ccMap[ccTo]->clean();
                 _ccMap.erase(ccTo);
              }
              else
              {
                vector<Sample*>::iterator itera = _ccMap[ccFrom]->getBeginIterator();
                while((itera != _ccMap[ccFrom]->getEndIterator()))
                {
                  (*itera)->setConnectedComponent( ccTo );
                  _ccMap[ccTo]->add( (*itera) );
                  //itera = NULL;
                  itera++;
                }
                _ccMap[ccFrom]->clean();
                _ccMap.erase(ccFrom);
              }
            }
          }
          else
          {
              //cout<<": FAILED"<<endl;
              //cout << "edge from " << i << " to " << n << " is NOT free" << endl;
          }
        }
	    }
	    cout << "END CONNECTING  " << max << " FREE SAMPLES" << endl;

	    return true;
    }


    //!Delete the graph g
    void DRMPlanner::clearGraph(){
  	  weights.clear();
	    edges.clear();
      _samples->clearNeighs();
	    if(_isGraphSet){
		    locations.clear();
		    delete g;
	    }
	    _isGraphSet = false;
      _solved = false;
      _labelCC = 0;
      _ccMap.clear();
      for(int i=0;i<_samples->getSize();i++)
        _samples->getSampleAt(i)->setConnectedComponent(-1);
    }



	//!Print connected components 
    void DRMPlanner::printConnectedComponents()
	{
		std::map<int, SampleSet*>::iterator it;
		cout<<"NUM CONNECTED COMPONENTS = "<<_ccMap.size()<<endl;
		for ( it=_ccMap.begin(); it != _ccMap.end(); it++ )
		{
			cout << "CC " << (*it).first << " => " << (*it).second->getSize()<<" samples: " ;
			vector<Sample*>::iterator itera = (*it).second->getBeginIterator();
			while((itera != (*it).second->getEndIterator()))
			{
				cout <<_samples->indexOf( (*itera) )<<", ";
				itera++;
			}
			cout << endl;
		}	
		cout<<"TOTAL NUMBER OF NODES = "<< _samples->getSize() <<endl;
		//cout << "Weights.size = "<< weights.size() <<" Edges size = " << edges.size()<<endl;
		

	}



	//!Smooths the path. 
	//!If maintainfirst is set then the first edge is maintained in the smoothed path 
	//!If maintainlast is set then the last edge is maintained in the smoothed path 
	//! They are both initialized to false
	void DRMPlanner::smoothPath(bool maintainfirst, bool maintainlast)
	{
		if(!_solved){
			cout<<"Cannot smooth path - path is not yet solved"<<endl;
			return;
		}
		//START CREATING AN AUXILIAR GRAPH
		//Create a graph with all the samples but with edges connecting those samples
		//of the original path (if collision-free)  
	    int maxNodesPath = _path.size();

		if(maxNodesPath==2)
		{
			cout<<"smoothPath not performed: Path has only two nodes!!"<<endl;
			return;
		}


        vector<prmEdge*> edgesPath;
		vector<cost> weightsPath;

		int last;
		if(maintainlast==true) last=maxNodesPath-1;//do not try to connect other nodes to the final
		else last=maxNodesPath; 

		for(int i=0;i<maxNodesPath-1;i++)
		{
			//connect node i with the next sample (i+1) in path (known to be connectable)
			prmEdge *e = new prmEdge(_samples->indexOf(_path[i]),_samples->indexOf(_path[i+1]));
			edgesPath.push_back(e);
			weightsPath.push_back(_locPlanner->distance(_path[i],_path[i+1]));
			//try to connect with the other samples in the path
			if(i==0 && maintainfirst==true) continue; //do not try to connect initial node with others
			for(int n=i+2;n<last;n++)
			{
				_locPlanner->setInitSamp(_path[i]);
				_locPlanner->setGoalSamp(_path[n]);
				if(_locPlanner->canConect())
				{
					prmEdge *e = new prmEdge(_samples->indexOf(_path[i]),_samples->indexOf(_path[n]));
					edgesPath.push_back(e);
					weightsPath.push_back(_locPlanner->distance(_path[i],_path[n]));
				}
			}
		}
            
		// create graph
		prmGraph *gPath = new prmGraph(_samples->getSize());
		WeightMap weightmapPath = get(edge_weight, *gPath);

		for(std::size_t j = 0; j < edgesPath.size(); ++j) {
			edge_descriptor e; 
			bool inserted;
			tie(e, inserted) = add_edge(edgesPath[j]->first, edgesPath[j]->second, *gPath);
			weightmapPath[e] = weightsPath[j];
		}
	  //END CREATING AN AUXILIAR GRAPH


	  //START FINDING A SMOOTHER PATH
	  //Find path from cini to cgoal along the nodes of gPath
	    prmVertex start = shortest_path.front();
	    prmVertex goal = shortest_path.back();

		//Now clear the unsmoothed solution
        shortest_path.clear(); //path as a vector of prmvertex
	    _path.clear();//path as a vector of samples

		//vector to store the parent information of each vertex
		vector<prmGraph::vertex_descriptor> p(num_vertices(*gPath));
		//vector with cost of reaching each vertex
		vector<cost> d(num_vertices(*gPath));

		try {
        // call astar named parameter interface
        astar_search(*gPath, start, 
                      distance_heuristic<prmGraph, cost, vector<location> >(locations, goal, _locPlanner),
                      predecessor_map(&p[0]).distance_map(&d[0]).
                      visitor(astar_goal_visitor<prmVertex>(goal)));

    }catch(DRM::found_goal fg) { // found a path to the goal
			//list<prmVertex> shortest_path; now is a class parameter
			for( prmVertex v = goal; ; v = p[v] ){
				shortest_path.push_front(v);
				if( p[v] == v ) break;
			}
			cout << "Smoothed path from " << start << " to " << goal << ": ";
			cout << start;
			list<prmVertex>::iterator spi = shortest_path.begin();
			for(++spi; spi != shortest_path.end(); ++spi)
				cout << " -> " << *spi;
			cout << endl;

			//set solution vector
			list<prmVertex>::iterator spi2 = shortest_path.begin();
			_path.push_back(_samples->getSampleAt(start));
	      
			for(++spi2; spi2 != shortest_path.end(); ++spi2){
		      _path.push_back(_samples->getSampleAt(*spi2));
			}
    }
  }

  } //namespace DRMPlanner
} //namespace libPlanner


