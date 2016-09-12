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
 
 
 

#include "ivpqpelement.h"

	
namespace libProblem {

  IVPQPElement::IVPQPElement(string visFile, KthReal sc)
    :IVElement(visFile,sc){
      pqpmodel = NULL;
      makePQPModel();
  }

// get triangles from inventor models
	void IVPQPElement::triang_CB(void *data, SoCallbackAction *action,
		      const SoPrimitiveVertex *vertex1,
		      const SoPrimitiveVertex *vertex2,
		      const SoPrimitiveVertex *vertex3)
	{
		tri_info* info = (tri_info*)data;
		if ( !info->pqp_model ) return;

		SbVec3f points[] = { vertex1->getPoint(), vertex2->getPoint(), vertex3->getPoint() };
		double verts[3][3];

		const SbMatrix  mm = action->getModelMatrix();
		for (int i = 0; i < 3; i++) {
			mm.multVecMatrix(points[i], points[i]);
			for (int j = 0; j < 3; j++) {
				verts[i][j] = points[i][j] ;
			}
		}
		info->pqp_model->AddTri(verts[0], verts[1], verts[2], info->tri_cnt++);
	}

  SoSeparator* IVPQPElement::getIvFromPQPModel(bool tran) {
	  LCPRNG* gen1 = new LCPRNG(15485341);//15485341 is a big prime number
	  SoSeparator **MyRobot;
	  SoVertexProperty **myRobotVertexProperty;
	  SoFaceSet **myRobotFaceSet;
	  SoSeparator *myRootRobot;
	  SbVec3f *vertices;
	  int nTriRobot;
	  float r,g;
	  float x,y,z;
	  int npunts = 3;

	  vertices = new SbVec3f[3];
	  myRootRobot = new SoSeparator;

	  nTriRobot = pqpmodel->num_tris;
	  MyRobot = new SoSeparator*[nTriRobot];
	  myRobotVertexProperty = new SoVertexProperty*[nTriRobot];
	  myRobotFaceSet = new SoFaceSet*[nTriRobot];

	  for(int j=0; j< nTriRobot; j++){
		  x = (float)pqpmodel->tris[j].p1[0];
		  y = (float)pqpmodel->tris[j].p1[1];
		  z = (float)pqpmodel->tris[j].p1[2];
		  vertices[0].setValue(x,y,z);

		  x = (float)pqpmodel->tris[j].p2[0];
		  y = (float)pqpmodel->tris[j].p2[1];
		  z = (float)pqpmodel->tris[j].p2[2];
		  vertices[1].setValue(x,y,z);

		  x = (float)pqpmodel->tris[j].p3[0];
		  y = (float)pqpmodel->tris[j].p3[1];
		  z = (float)pqpmodel->tris[j].p3[2];
		  vertices[2].setValue(x,y,z);

		  MyRobot[j] = new SoSeparator();
		  MyRobot[j]->ref();

		  myRobotVertexProperty[j] = new SoVertexProperty;
		  myRobotVertexProperty[j]->normalBinding = SoNormalBinding::PER_FACE;
		  myRobotVertexProperty[j]->vertex.setValues(0, 3, vertices);
		  r = 0.5f +((float)gen1->d_rand()/2.0f);
		  g = 0.5f +((float)gen1->d_rand()/2.0f);
		  myRobotVertexProperty[j]->orderedRGBA.setValue(SbColor(r,g,(float)0.0).getPackedValue());

		  myRobotFaceSet[j] = new SoFaceSet;
		  myRobotFaceSet[j]->numVertices.setValues(0, 1, &npunts);
		  myRobotFaceSet[j]->vertexProperty.setValue(myRobotVertexProperty[j]);
		  MyRobot[j]->addChild(myRobotFaceSet[j]);

		  MyRobot[j]->unrefNoDelete();

		  myRootRobot->addChild(MyRobot[j]);
	  }
    if(tran){
		  SoSeparator* temp = new SoSeparator;
      temp->addChild(getTrans());
		  temp->addChild(getRot());
		  temp->addChild(myRootRobot);
		  return temp;
	  }else
      return myRootRobot;
  }


  bool IVPQPElement::collideTo(Element* other) {
    Element::increaseCollCheckCounter();
	  try{
		  PQP_CollideResult pqp_coll_result;
		  KthReal *pR,*pO;
		  double posR[3];
		  double oriR[3][3];
		  double posO[3];
		  double oriO[3][3];
		  SbMatrix matO, matR; 
		  SbRotation vcR;
  		
		  pR = getPosition();
		  pO = other->getPosition();
		  for(int i=0;i<3;i++){
			  posR[i]= (double)pR[i];
			  posO[i]= (double)pO[i];
		  }
  		
		  matR = orientationMatrix();				// To Get rotation of Robot at matriz form
		  matO = ((IVPQPElement*)other)->orientationMatrix();
		  for (int i = 0; i < 3; i++){
			  for (int j = 0; j < 3; j++){
				  oriR[i][j] = matR[i][j];
				  oriO[i][j] = matO[i][j];
			  }
		  }

		  PQP_Collide(&pqp_coll_result,
	        oriR, posR, pqpModel(),
	        oriO, posO, ((IVPQPElement*)other)->pqpModel(),
	        PQP_FIRST_CONTACT);
		  return (pqp_coll_result.Colliding()!= 0);
	  }catch(...){
		  return true;
	  }
  }

  KthReal IVPQPElement::getDistanceTo(Element* other) {
    Element::increaseCollCheckCounter();
	  try{
		  PQP_DistanceResult pqp_dist_result;
		  KthReal *pR,*pO;
		  double posR[3];
		  double oriR[3][3]; 
		  double posO[3];
		  double oriO[3][3];
		  SbMatrix matO, matR; 
  		
		  pR = getPosition();
		  pO = other->getPosition();
		  for(int i=0;i<3;i++){
			  posR[i]= (double)pR[i];
			  posO[i]= (double)pO[i];
		  }

		  matR = orientationMatrix();
		  matO = ((IVPQPElement*)other)->orientationMatrix();
		  for (int i = 0; i < 3; i++){
			  for (int j = 0; j < 3; j++){
				  oriR[i][j] = matR[i][j];
				  oriO[i][j] = matO[i][j];
			  }
		  }

		  PQP_REAL rel_err = 0.1; //?
		  PQP_REAL abs_err = 1000;//make big - rel_error interests us (?)
		  int qsize = 20;
		  PQP_Distance(&pqp_dist_result,
			  oriR, posR, pqpModel(),
	      oriO, posO, ((IVPQPElement*)other)->pqpModel(),
			  rel_err,abs_err,qsize);

		  return (KthReal)pqp_dist_result.Distance();
	  }catch(...){
		  return (KthReal)0.0;
	  }
  }

  bool IVPQPElement::makePQPModel() {
    if(ivModel() != NULL){
		  if(this->pqpmodel == NULL){
			  this->pqpmodel = new PQP_Model;
			  SoCallbackAction triAction;
			  tri_info info(pqpmodel,scale);
			  pqpmodel->BeginModel();
			  triAction.addTriangleCallback(SoShape::getClassTypeId(),
							  triang_CB, (void*)&info);
			  triAction.apply(ivModel());
			  pqpmodel->EndModel();
			  return true;
		  }
	  }
	  return false;
  }
}

