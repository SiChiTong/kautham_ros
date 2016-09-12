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
 
 

#include "tmat.h"
#include <string>
#include <sstream>
#include <math.h>

using namespace std;

namespace SDK {
	TMat::TMat(int d) {

		this->d = d;
    _tMat = new char*[d]; //The matrix Td
    for(int i=0; i<d; i++) {
        _tMat[i] = new char[d];
    }
	  
    createTd();
  
    matRes = new char*[d];
    for(char i=0; i<d; i++)
      matRes[i] = new char[d];
		for(char i=0;i<d;i++)
			for(char j=0; j<d; j++)
				matRes[i][j] = 0;

	}

	TMat::~TMat() {
	 /********************
     * Heap cleanup
     ********************/
    for(int i=0 ; i < d ; i++ ) {
			//delete[] matRes[i];
			delete[] _tMat[i];
    }
    delete[] _tMat;
		delete[] matRes;
	}

  /************************************************************
 * Function: prime_factorization
 ************************************************************/
/*! Does the prime factorization of number 'x'. Puts the result in
*   parameter 'fact' and the number of prime factors in parameter 'numfactors'.
*   This function is addapted from Steven S. Skiena (www.programming-challenges.com).
 */
  void TMat::prime_factorization(long int x, int *fact, int *numfactors){
    //Addapted from:
    /*
    Copyright 2003 by Steven S. Skiena; all rights reserved. 

    Permission is granted for use in non-commerical applications
    provided this copyright notice remains intact and unchanged.

    This program appears in my book:

    "Programming Challenges: The Programming Contest Training Manual"
    by Steven Skiena and Miguel Revilla, Springer-Verlag, New York 2003.

    See our website www.programming-challenges.com for additional information.

    This book can be ordered from Amazon.com at

    http://www.amazon.com/exec/obidos/ASIN/0387001638/thealgorithmrepo/

    */
	  long i;			/* counter */
	  long c;			/* remaining product to factor */
	  *numfactors=0;

	  c = x;

	  while ((c % 2) == 0) {
		  fact[(*numfactors)++] = 2;
		  c = c / 2;
	  }

	  i = 3;

	  while (i <= (sqrt((long double)c)+1)) {
		  if ((c % i) == 0) {
			  fact[(*numfactors)++] = i;
			  c = c / i;
		  }
		  else
			  i = i + 2;
	  }

	  if (c > 1) 
	  {
		  fact[(*numfactors)++] = c;
	  }
  }

  void TMat::insert(char **vA, char **vB, int dimA, int dimB){
	  int dimC = dimA*dimB;

	  //copy of vA
	  char **tempvA;
	  tempvA = new char*[dimA];
	  for(int i=0; i<dimA; i++)
	  {
		  tempvA[i] = new char[dimA];
		  for(int j=0; j<dimA; j++)
		  {
			  tempvA[i][j] = vA[i][j];
		  }
	  }

	  for(int i=0; i<dimA; i++)
	  {
		  for(int j=0;j<dimA;j++)
		  {
			  if(tempvA[i][j] != 0)
			  {
        			  for(int k=0; k<dimB; k++) {
            				  for(int l=0; l<dimB; l++) {
                				  vA[i*dimB + k][j*dimB + l]  = vB[k][l]; 
            				  }
				  }
			  }
			  else
			  {
				  for(int k=0; k<dimB; k++) {
            				  for(int l=0; l<dimB; l++) {
                				  vA[i*dimB + k][j*dimB + l] = 0; 
            				  }
				  }
  				
			  }	
		  }
	  }
  }

  //! This method creates the base matrices of the prime numbers involved.
  void TMat::compose(int *primefactors, int numfactors, char **vC, int dimC, int trunc){	
	  //Create the base matrices of the prime numbers involved
	  char ***vv;
	  vv = new char**[numfactors];

	  //vectors to do the prime decomposition if necessary
	  int *nf;
	  nf = new int[numfactors];
	  int **pf;
	  pf = new int*[numfactors];
	  for(int i=0;i<numfactors;i++) pf[i] = new int[100];

	  //start creating matrices for all the factors
	  //for all primes different from 2 and 3 it is done by a 
	  //recursive call to 'compose' with trunc set to 1.
	  for(int i=0; i<numfactors; i++)
	  {
		  switch(primefactors[i])
		  { 
			  case 2:
				  //printf("factor 2\n");
				  //base matrix T2
				  vv[i] = new char*[2]; 
				  for(int j=0;j<2;j++) vv[i][j] = new char[2];
				  vv[i][0][0] = 1; vv[i][0][1] = 0; 
				  vv[i][1][0] = 1; vv[i][1][1] = 1;
				  break;

			  case 3:
				  //printf("factor 3\n");
    			  //base matrix T3
				  vv[i] = new char*[3]; 
				  for(int j=0;j<3;j++) vv[i][j] = new char[3];
    			  vv[i][0][0] = 1; vv[i][0][1] = 0; vv[i][0][2] = 0;
    			  vv[i][1][0] = 0; vv[i][1][1] = 1; vv[i][1][2] = 0;
    			  vv[i][2][0] = 1; vv[i][2][1] = 1; vv[i][2][2] = 1;
				  break;
			  default:
				  //printf("factor %d\n",primefactors[i]);
				  prime_factorization(primefactors[i]+1, pf[i], &nf[i]);
				  vv[i] = new char*[primefactors[i]];
				  for(int j=0;j< (primefactors[i]);j++) vv[i][j] = new char[primefactors[i]];
				  compose(pf[i], nf[i], vv[i], primefactors[i]+1, 1); //compose with trunc option
		  }
	  }

	  //recursively insert one inside the other
	  char **tempvC;
	  tempvC = new char*[dimC];
	  for(int i=0;i<dimC;i++) 
	  {
		  tempvC[i] = new char[dimC+trunc];
		  for(int j=0;j<dimC;j++) 
		  {
			  tempvC[i][j] = 0;
		  }
	  }

	  //load vC with the vv matrix of the first prime factor
	  for(int i=0;i<primefactors[0];i++) 
	  {
		  for(int j=0;j<primefactors[0];j++) 
		  {
			  tempvC[i][j] = vv[0][i][j];
		  }
	  }


	  //loop for all factors
	  int prodprimes = primefactors[0];
	  for(int i=1;i<numfactors;i++)
	  {
		  insert(tempvC,vv[i],prodprimes,primefactors[i]);	
		  prodprimes *= primefactors[i];
	  }

	  //trunc if required
	  int dvC;
	  if(trunc) dvC = dimC - 1;
	  else dvC = dimC;
  	
	  for(int i=0; i<dvC; i++)
	  {
		  for(int j=0;j<dvC;j++)
		  {
			  vC[i][j] = tempvC[i][j];
		  }
	  }
  	
  	
	  /********************
       * Heap cleanup
       ********************/
      for(int i=0 ; i < numfactors ; i++ ) {
		  switch(primefactors[i])
		  { 
			  case 2:	for(int j=0 ; j < 2 ; j++ ) delete[] vv[i][j]; break;
			  case 3:	for(int j=0 ; j < 3 ; j++ ) delete[] vv[i][j]; break;
			  default: for(int j=0 ; j < primefactors[i] ; j++ ) delete[] vv[i][j]; break;
		  }
		  delete[] vv[i];
	  }
      delete[] vv;

	  for(int i=0 ; i < dimC ; i++ ) delete[] tempvC[i]; 
	  delete[] tempvC;
  }


  void TMat::createTd(){	

	  int numfactors;
	  int primefactors[100];
  	
	  prime_factorization(d, primefactors, &numfactors);

	  //if d is prime then factorize (d+1)
	  if(numfactors==1 && d!=2 && d!=3) 
	  {
		  prime_factorization(d+1, primefactors, &numfactors);
		  compose(primefactors, numfactors, _tMat, d+1, 1);
	  }
	  else{
		  //compose(primefactors, numfactors, v, d);
		  compose(primefactors, numfactors, _tMat, d);
	  }
  }

	int TMat::multiply(const int k) {
		int resk;
    int i;

    char *const w = new char[d];
    char *const res = new char[d];

    //Convert k to a binary vector w
    for( i=0; i<d; i++){
        w[i] = (k>>i) & 0x01;
    }

    //multiply T*w
    multiply(w,res);

    //Convert the resulting binary vector res to an integer
    resk=0;
    for( i=0; i<d; i++)
    {
        resk += (res[i]<<i);
    }

    /********************
     * Heap cleanup
     ********************/
    delete[] w;
    delete[] res;

    return resk;
	}

	void TMat::multiply(const char* const k, char* const l) {
		// Falta por implementar. Verificar posibilidad de  borrado.
	}

	void TMat::multiply(const char* const k, char* const l, const int m) {
		// Falta por implementar. Verificar posibilidad de  borrado.
	}
/************************************************************
 *                             multiply
 ************************************************************
 /*! This function performs the multipication of T (d x d) by
 *  a binary matrix v (d x m) representing the coordinates of
 *  a sample.
 * The result is also a binary matrix that is set in matRes parameter.
 * The partion level m is passed as a parameter.
 ************************************************************/
	char** TMat::multiply( const char *const *const v, const int m) const {
		for(int k=0;k<m;k++) {
			for(int i=0;i<d;i++) {
				matRes[i][k] = 0;
				for(int j=0;j<d;j++)
					matRes[i][k] += (_tMat[i][j])*(v[j][k]);
				matRes[i][k] = matRes[i][k] % 2;
			}
		}
		return matRes;
	}

/***************************************************************************/
/*                             multiply                                    */
/***************************************************************************/
/*! Multiplies the matrix by a binary vector "w". Returns the
* result at the parameter "res"
* A standard matrix-vector operation is performed and then
* a mod2 operation is done. Therefore the resulting vector is
* a binary vector.
*/
	void TMat::multiply( const char* const w, char* const res ) const {
		for(int i=0; i<d; i++) {
			res[i]=0;
			for( int j=0; j<d; j++) {
				if(w[j]!=-1)
					res[i]+=_tMat[i][j]*w[j];
			}
			res[i] = res[i]%2;
		}
	}

	string TMat::printMatrix( void ) {
		stringstream out;
		out << "matrix T:\n";
		for( int i=0; i<d; i++) {
			out << "[";
			for( int j=0; j<d; j++) {
				out << "\t";
				out << (int)_tMat[i][j];
			}
			out << "]\n";
		}
		return out.str();
	}
}
