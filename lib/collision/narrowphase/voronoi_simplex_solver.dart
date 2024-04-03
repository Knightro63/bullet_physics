/*
 * Dart port of Bullet (c) 2024 @Knightro
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import 'package:bullet_physics/collision/narrowphase/simplex_solver_interface.dart';
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class VoronoiSimplexSolver extends SimplexSolverInterface {
	static const int voronoiSimplexMaxVerts = 5;
	static const int _vertA = 0;
	static const int _vertB = 1;
	static const int _vertC = 2;
	//static const int _vertD = 3;

	int numVerts = 0;

	final List<Vector3> simplexVectorW = [Vector3.zero(),Vector3.zero(),Vector3.zero(),Vector3.zero(),Vector3.zero()];
	final List<Vector3> simplexPointsP = [Vector3.zero(),Vector3.zero(),Vector3.zero(),Vector3.zero(),Vector3.zero()];
	final List<Vector3> simplexPointsQ = [Vector3.zero(),Vector3.zero(),Vector3.zero(),Vector3.zero(),Vector3.zero()];

	final Vector3 cachedP1 = Vector3.zero();
	final Vector3 cachedP2 = Vector3.zero();
	final Vector3 cachedV = Vector3.zero();
	final Vector3 lastW = Vector3.zero();
	bool cachedValidClosest = false;

	final SubSimplexClosestResult cachedBC = SubSimplexClosestResult();

	bool needsUpdate = false;

	void removeVertex(int index) {
		assert(numVerts>0);
		numVerts--;
		simplexVectorW[index].setFrom(simplexVectorW[numVerts]);
		simplexPointsP[index].setFrom(simplexPointsP[numVerts]);
		simplexPointsQ[index].setFrom(simplexPointsQ[numVerts]);
	}
	
	void	reduceVertices(UsageBitfield usedVerts) {
		if ((numVertices() >= 4) && (!usedVerts.usedVertexD)){
			removeVertex(3);
    }

		if ((numVertices() >= 3) && (!usedVerts.usedVertexC)){
			removeVertex(2);
    }

		if ((numVertices() >= 2) && (!usedVerts.usedVertexB)){
			removeVertex(1);
    }

		if ((numVertices() >= 1) && (!usedVerts.usedVertexA)){
			removeVertex(0);
    }
	}
	

	bool updateClosestVectorAndPoints() {
		if (needsUpdate)
		{
			cachedBC.reset();

			needsUpdate = false;

			switch (numVertices())
			{
			case 0:
					cachedValidClosest = false;
					break;
			case 1:
				{
					cachedP1.setFrom(simplexPointsP[0]);
					cachedP2.setFrom(simplexPointsQ[0]);
					cachedV.sub2(cachedP1, cachedP2); //== m_simplexVectorW[0]
					cachedBC.reset();
					cachedBC.setBarycentricCoordinates(1, 0, 0, 0);
					cachedValidClosest = cachedBC.isValid();
					break;
				}
			case 2:
				{
					Vector3 tmp = Vector3.zero();
					
					//closest point origin from line segment
					Vector3 from = simplexVectorW[0];
					Vector3 to = simplexVectorW[1];
					Vector3 nearest = Vector3.zero();

					Vector3 p = Vector3.zero();
					p.setValues(0, 0, 0);
					Vector3 diff = Vector3.zero();
					diff.sub2(p, from);

					Vector3 v = Vector3.zero();
					v.sub2(to, from);

					double t = v.dot(diff);

					if (t > 0) {
						double dotVV = v.dot(v);
						if (t < dotVV) {
							t /= dotVV;
							tmp.scaleFrom(t, v);
							diff.sub(tmp);
							cachedBC.usedVertices.usedVertexA = true;
							cachedBC.usedVertices.usedVertexB = true;
						} else {
							t = 1;
							diff.sub(v);
							// reduce to 1 point
							cachedBC.usedVertices.usedVertexB = true;
						}
					} else
					{
						t = 0;
						//reduce to 1 point
						cachedBC.usedVertices.usedVertexA = true;
					}
					cachedBC.setBarycentricCoordinates(1-t, t, 0, 0);
					
					tmp.scaleFrom(t, v);
					nearest.add2(from, tmp);

					tmp.sub2(simplexPointsP[1], simplexPointsP[0]);
					tmp.scale(t);
					cachedP1.add2(simplexPointsP[0], tmp);

					tmp.sub2(simplexPointsQ[1], simplexPointsQ[0]);
					tmp.scale(t);
					cachedP2.add2(simplexPointsQ[0], tmp);

					cachedV.sub2(cachedP1, cachedP2);

					reduceVertices(cachedBC.usedVertices);

					cachedValidClosest = cachedBC.isValid();
					break;
				}
			case 3: 
				{ 
					Vector3 tmp1 = Vector3.zero();
					Vector3 tmp2 = Vector3.zero();
					Vector3 tmp3 = Vector3.zero();
					
					// closest point origin from triangle 
					Vector3 p = Vector3.zero();
					p.setValues(0, 0, 0);

					Vector3 a = simplexVectorW[0]; 
					Vector3 b = simplexVectorW[1]; 
					Vector3 c = simplexVectorW[2]; 

					closestPtPointTriangle(p,a,b,c,cachedBC);

					tmp1.scaleFrom(cachedBC.barycentricCoords[0], simplexPointsP[0]);
					tmp2.scaleFrom(cachedBC.barycentricCoords[1], simplexPointsP[1]);
					tmp3.scaleFrom(cachedBC.barycentricCoords[2], simplexPointsP[2]);
					VectorUtil.add(cachedP1, tmp1, tmp2, tmp3);

					tmp1.scaleFrom(cachedBC.barycentricCoords[0], simplexPointsQ[0]);
					tmp2.scaleFrom(cachedBC.barycentricCoords[1], simplexPointsQ[1]);
					tmp3.scaleFrom(cachedBC.barycentricCoords[2], simplexPointsQ[2]);
					VectorUtil.add(cachedP2, tmp1, tmp2, tmp3);

					cachedV.sub2(cachedP1, cachedP2);

					reduceVertices(cachedBC.usedVertices);
					cachedValidClosest = cachedBC.isValid(); 

					break; 
				}
			case 4:
				{
					Vector3 tmp1 = Vector3.zero();
					Vector3 tmp2 = Vector3.zero();
					Vector3 tmp3 = Vector3.zero();
					Vector3 tmp4 = Vector3.zero();
					
					Vector3 p = Vector3.zero();

					Vector3 a = simplexVectorW[0];
					Vector3 b = simplexVectorW[1];
					Vector3 c = simplexVectorW[2];
					Vector3 d = simplexVectorW[3];

					bool hasSeperation = closestPtPointTetrahedron(p,a,b,c,d,cachedBC);

					if (hasSeperation)
					{
						tmp1.scaleFrom(cachedBC.barycentricCoords[0], simplexPointsP[0]);
						tmp2.scaleFrom(cachedBC.barycentricCoords[1], simplexPointsP[1]);
						tmp3.scaleFrom(cachedBC.barycentricCoords[2], simplexPointsP[2]);
						tmp4.scaleFrom(cachedBC.barycentricCoords[3], simplexPointsP[3]);
						VectorUtil.add(cachedP1, tmp1, tmp2, tmp3, tmp4);

						tmp1.scaleFrom(cachedBC.barycentricCoords[0], simplexPointsQ[0]);
						tmp2.scaleFrom(cachedBC.barycentricCoords[1], simplexPointsQ[1]);
						tmp3.scaleFrom(cachedBC.barycentricCoords[2], simplexPointsQ[2]);
						tmp4.scaleFrom(cachedBC.barycentricCoords[3], simplexPointsQ[3]);
						VectorUtil.add(cachedP2, tmp1, tmp2, tmp3, tmp4);

						cachedV.sub2(cachedP1, cachedP2);
						reduceVertices (cachedBC.usedVertices);
					} else
					{
	//					printf("sub distance got penetration\n");

						if (cachedBC.degenerate)
						{
							cachedValidClosest = false;
						} else
						{
							cachedValidClosest = true;
							//degenerate case == false, penetration = true + zero
							cachedV.setValues(0, 0, 0);
						}
						break;
					}

					cachedValidClosest = cachedBC.isValid();

					//closest point origin from tetrahedron
					break;
				}
			default:
				{
					cachedValidClosest = false;
				}
			}
		}

		return cachedValidClosest;
	}


	bool closestPtPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c, SubSimplexClosestResult result) {
		result.usedVertices.reset();

		// Check if P in vertex region outside A
		Vector3 ab = Vector3.zero();
		ab.sub2(b, a);

		Vector3 ac = Vector3.zero();
		ac.sub2(c, a);

		Vector3 ap = Vector3.zero();
		ap.sub2(p, a);

		double d1 = ab.dot(ap);
		double d2 = ac.dot(ap);

		if (d1 <= 0 && d2 <= 0) 
		{
			result.closestPointOnSimplex.setFrom(a);
			result.usedVertices.usedVertexA = true;
			result.setBarycentricCoordinates(1, 0, 0, 0);
			return true; // a; // barycentric coordinates (1,0,0)
		}

		// Check if P in vertex region outside B
		Vector3 bp = Vector3.zero();
		bp.sub2(p, b);

		double d3 = ab.dot(bp);
		double d4 = ac.dot(bp);

		if (d3 >= 0 && d4 <= d3) 
		{
			result.closestPointOnSimplex.setFrom(b);
			result.usedVertices.usedVertexB = true;
			result.setBarycentricCoordinates(0, 1, 0, 0);

			return true; // b; // barycentric coordinates (0,1,0)
		}

		// Check if P in edge region of AB, if so return projection of P onto AB
		double vc = d1*d4 - d3*d2;
		if (vc <= 0 && d1 >= 0 && d3 <= 0) {
			double v = d1 / (d1 - d3);
			result.closestPointOnSimplex.scaleAdd(v, ab, a);
			result.usedVertices.usedVertexA = true;
			result.usedVertices.usedVertexB = true;
			result.setBarycentricCoordinates(1-v, v, 0, 0);
			return true;
			//return a + v * ab; // barycentric coordinates (1-v,v,0)
		}

		// Check if P in vertex region outside C
		Vector3 cp = Vector3.zero();
		cp.sub2(p, c);

		double d5 = ab.dot(cp);
		double d6 = ac.dot(cp);

		if (d6 >= 0 && d5 <= d6) 
		{
			result.closestPointOnSimplex.setFrom(c);
			result.usedVertices.usedVertexC = true;
			result.setBarycentricCoordinates(0, 0, 1, 0);
			return true;//c; // barycentric coordinates (0,0,1)
		}

		// Check if P in edge region of AC, if so return projection of P onto AC
		double vb = d5*d2 - d1*d6;
		if (vb <= 0 && d2 >= 0 && d6 <= 0) {
			double w = d2 / (d2 - d6);
			result.closestPointOnSimplex.scaleAdd(w, ac, a);
			result.usedVertices.usedVertexA = true;
			result.usedVertices.usedVertexC = true;
			result.setBarycentricCoordinates(1-w, 0, w, 0);
			return true;
			//return a + w * ac; // barycentric coordinates (1-w,0,w)
		}

		// Check if P in edge region of BC, if so return projection of P onto BC
		double va = d3*d6 - d5*d4;
		if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
			double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

			Vector3 tmp = Vector3.zero();
			tmp.sub2(c, b);
			result.closestPointOnSimplex.scaleAdd(w, tmp, b);

			result.usedVertices.usedVertexB = true;
			result.usedVertices.usedVertexC = true;
			result.setBarycentricCoordinates(0, 1-w, w, 0);
			return true;		
		   // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
		}

		// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
		double denom = 1 / (va + vb + vc);
		double v = vb * denom;
		double w = vc * denom;

		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		tmp1.scaleFrom(v, ab);
		tmp2.scaleFrom(w, ac);
		VectorUtil.add(result.closestPointOnSimplex, a, tmp1, tmp2);
		result.usedVertices.usedVertexA = true;
		result.usedVertices.usedVertexB = true;
		result.usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(1-v-w, v, w, 0);

		return true;
		//	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = btScalar(1.0) - v - w
	}
	
	/// Test if point p and d lie on opposite sides of plane through abc

	static int pointOutsideOfPlane(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
	{
		Vector3 tmp = Vector3.zero();

		Vector3 normal = Vector3.zero();
		normal.sub2(b, a);
		tmp.sub2(c, a);
		normal.cross2(normal, tmp);

		tmp.sub2(p, a);
		double signp = tmp.dot(normal); // [AP AB AC]

		tmp.sub2(d, a);
		double signd = tmp.dot(normal); // [AD AB AC]

	//#ifdef CATCH_DEGENERATE_TETRAHEDRON
//	#ifdef BT_USE_DOUBLE_PRECISION
//	if (signd * signd < (btScalar(1e-8) * btScalar(1e-8)))
//		{
//			return -1;
//		}
//	#else
		if (signd * signd < ((1e-4) * (1e-4)))
		{
	//		printf("affine dependent/degenerate\n");//
			return -1;
		}
	//#endif

	//#endif
		// Points on opposite sides if expression signs are opposite
		return (signp * signd < 0)? 1 : 0;
	}
	

	bool closestPtPointTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d, SubSimplexClosestResult finalResult) {
		SubSimplexClosestResult tempResult = SubSimplexClosestResult();
		tempResult.reset();
		try {
			Vector3 tmp = Vector3.zero();
			Vector3 q = Vector3.zero();

			// Start out assuming point inside all halfspaces, so closest to itself
			finalResult.closestPointOnSimplex.setFrom(p);
			finalResult.usedVertices.reset();
			finalResult.usedVertices.usedVertexA = true;
			finalResult.usedVertices.usedVertexB = true;
			finalResult.usedVertices.usedVertexC = true;
			finalResult.usedVertices.usedVertexD = true;

			int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
			int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
			int	pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
			int	pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);

		   if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
		   {
			   finalResult.degenerate = true;
			   return false;
		   }

		   if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
			 {
				 return false;
			 }


			double bestSqDist = double.infinity;
			// If point outside face abc then compute closest point on abc
			if (pointOutsideABC != 0) 
			{
				closestPtPointTriangle(p, a, b, c,tempResult);
				q.setFrom(tempResult.closestPointOnSimplex);

				tmp.sub2(q, p);
				double sqDist = tmp.dot(tmp);
				// Update best closest point if (squared) distance is less than current best
				if (sqDist < bestSqDist) {
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.setFrom(q);
					//convert result bitmask!
					finalResult.usedVertices.reset();
					finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
					finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexB;
					finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
					finalResult.setBarycentricCoordinates(
							tempResult.barycentricCoords[_vertA],
							tempResult.barycentricCoords[_vertB],
							tempResult.barycentricCoords[_vertC],
							0
					);

				}
			}


			// Repeat test for face acd
			if (pointOutsideACD != 0) 
			{
				closestPtPointTriangle(p, a, c, d,tempResult);
				q.setFrom(tempResult.closestPointOnSimplex);
				//convert result bitmask!

				tmp.sub2(q, p);
				double sqDist = tmp.dot(tmp);
				if (sqDist < bestSqDist) 
				{
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.setFrom(q);
					finalResult.usedVertices.reset();
					finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;

					finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexB;
					finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexC;
					finalResult.setBarycentricCoordinates(
							tempResult.barycentricCoords[_vertA],
							0,
							tempResult.barycentricCoords[_vertB],
							tempResult.barycentricCoords[_vertC]
					);

				}
			}
			// Repeat test for face adb


			if (pointOutsideADB != 0)
			{
				closestPtPointTriangle(p, a, d, b,tempResult);
				q.setFrom(tempResult.closestPointOnSimplex);
				//convert result bitmask!

				tmp.sub2(q, p);
				double sqDist = tmp.dot(tmp);
				if (sqDist < bestSqDist) 
				{
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.setFrom(q);
					finalResult.usedVertices.reset();
					finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
					finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexC;

					finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;
					finalResult.setBarycentricCoordinates(
							tempResult.barycentricCoords[_vertA],
							tempResult.barycentricCoords[_vertC],
							0,
							tempResult.barycentricCoords[_vertB]
					);

				}
			}
			// Repeat test for face bdc


			if (pointOutsideBDC != 0)
			{
				closestPtPointTriangle(p, b, d, c,tempResult);
				q.setFrom(tempResult.closestPointOnSimplex);
				//convert result bitmask!
				tmp.sub2(q, p);
				double sqDist = tmp.dot(tmp);
				if (sqDist < bestSqDist) 
				{
					bestSqDist = sqDist;
					finalResult.closestPointOnSimplex.setFrom(q);
					finalResult.usedVertices.reset();
					//
					finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexA;
					finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
					finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;

					finalResult.setBarycentricCoordinates(
							0,
							tempResult.barycentricCoords[_vertA],
							tempResult.barycentricCoords[_vertC],
							tempResult.barycentricCoords[_vertB]
					);

				}
			}

			//help! we ended up full !

			if (finalResult.usedVertices.usedVertexA &&
				finalResult.usedVertices.usedVertexB &&
				finalResult.usedVertices.usedVertexC &&
				finalResult.usedVertices.usedVertexD) 
			{
				return true;
			}

			return true;
		}
		finally {
		}
	}
	
	/**
	 * Clear the simplex, remove all the vertices.
	 */
  @override
	void reset() {
		cachedValidClosest = false;
		numVerts = 0;
		needsUpdate = true;
		lastW.setValues(1e30, 1e30, 1e30);
		cachedBC.reset();
	}
  @override
	void addVertex(Vector3 w, Vector3 p, Vector3 q) {
		lastW.setFrom(w);
		needsUpdate = true;

		simplexVectorW[numVerts].setFrom(w);
		simplexPointsP[numVerts].setFrom(p);
		simplexPointsQ[numVerts].setFrom(q);

		numVerts++;
	}

	/**
	 * Return/calculate the closest vertex.
	 */
  @override
	bool closest(Vector3 v) {
		bool succes = updateClosestVectorAndPoints();
		v.setFrom(cachedV);
		return succes;
	}
  @override
	double maxVertex() {
		int i, numverts = numVertices();
		double maxV = 0;
		for (i = 0; i < numverts; i++) {
			double curLen2 = simplexVectorW[i].length2;
			if (maxV < curLen2) {
				maxV = curLen2;
			}
		}
		return maxV;
	}
  @override
	bool fullSimplex() {
		return (numVerts == 4);
	}
  @override
	int getSimplex(List<Vector3> pBuf, List<Vector3> qBuf, List<Vector3> yBuf) {
		for (int i = 0; i < numVertices(); i++) {
			yBuf[i].setFrom(simplexVectorW[i]);
			pBuf[i].setFrom(simplexPointsP[i]);
			qBuf[i].setFrom(simplexPointsQ[i]);
		}
		return numVertices();
	}
  @override
	bool inSimplex(Vector3 w) {
		bool found = false;
		int i, numverts = numVertices();
		//btScalar maxV = btScalar(0.);

		//w is in the current (reduced) simplex
		for (i = 0; i < numverts; i++) {
			if (simplexVectorW[i].equals(w)) {
				found = true;
			}
		}

		//check in case lastW is already removed
		if (w.equals(lastW)) {
			return true;
		}

		return found;
	}
  @override
	void backupClosest(Vector3 v) {
		v.setFrom(cachedV);
	}
  @override
	bool emptySimplex() {
		return (numVertices() == 0);
	}
  @override
	void computePoints(Vector3 p1, Vector3 p2) {
		updateClosestVectorAndPoints();
		p1.setFrom(cachedP1);
		p2.setFrom(cachedP2);
	}
  @override
	int numVertices() {
		return numVerts;
	}
}


////////////////////////////////////////////////////////////////////////////
class UsageBitfield {
  bool usedVertexA = false;
  bool usedVertexB = false;
  bool usedVertexC = false;
  bool usedVertexD = false;
  
  void reset() {
    usedVertexA = false;
    usedVertexB = false;
    usedVertexC = false;
    usedVertexD = false;
  }
}
	
class SubSimplexClosestResult {
  final Vector3 closestPointOnSimplex = Vector3.zero();
  //MASK for m_usedVertices
  //stores the simplex vertex-usage, using the MASK, 
  // if m_usedVertices & MASK then the related vertex is used
  final UsageBitfield usedVertices = UsageBitfield();
  final List<double> barycentricCoords = [0,0,0,0];
  bool degenerate = false;
  
  void reset() {
    degenerate = false;
    setBarycentricCoordinates(0, 0, 0, 0);
    usedVertices.reset();
  }

  bool isValid() {
    bool valid = (barycentricCoords[0] >= 0) &&
        (barycentricCoords[1] >= 0) &&
        (barycentricCoords[2] >= 0) &&
        (barycentricCoords[3] >= 0);
    return valid;
  }

  void setBarycentricCoordinates(double a, double b, double c, double d) {
    barycentricCoords[0] = a;
    barycentricCoords[1] = b;
    barycentricCoords[2] = c;
    barycentricCoords[3] = d;
  }
}