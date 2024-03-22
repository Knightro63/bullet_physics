/*
 * Dart port of Bullet (c) 2024 @Knightro63
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

import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

//notes:
// Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
// which makes the btJacobianEntry memory layout 16 bytes
// if you only are interested in angular part, just feed massInvA and massInvB zero

/**
 * Jacobian entry is an abstraction that allows to describe constraints.
 * It can be used in combination with a constraint solver.
 * Can be used to relate the effect of an impulse to the constraint error.
 * 
 * @author jezek2
 */
class JacobianEntry {
	//final BulletStack stack = BulletStack.get();
	final Vector3 linearJointAxis = Vector3.zero();
	final Vector3 aJ = Vector3.zero();
	final Vector3 bJ = Vector3.zero();
	final Vector3 m_0MinvJt = Vector3.zero();
	final Vector3 m_1MinvJt = Vector3.zero();
	// Optimization: can be stored in the w/last component of one of the vectors
	double Adiag = 0;

	JacobianEntry();

	/**
	 * Constraint between two different rigidbodies.
	 */
	void initAll(
    Matrix3 world2A,
    Matrix3 world2B,
    Vector3 relPos1, 
    Vector3 relPos2,
    Vector3 jointAxis,
    Vector3 inertiaInvA,
    double massInvA,
    Vector3 inertiaInvB,
    double massInvB
  ){
		linearJointAxis.setFrom(jointAxis);

		aJ.cross2(relPos1, linearJointAxis);
		world2A.transform(aJ);

		bJ.setFrom(linearJointAxis);
		bJ.negate();
		bJ.cross2(relPos2, bJ);
		world2B.transform(bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = massInvA + m_0MinvJt.dot(aJ) + massInvB + m_1MinvJt.dot(bJ);

		assert (Adiag > 0);
	}

	/**
	 * Angular constraint between two different rigidbodies.
	 */
	void initMultiWorld(
    Vector3 jointAxis,
		Matrix3 world2A,
		Matrix3 world2B,
		Vector3 inertiaInvA,
		Vector3 inertiaInvB
  ){
		linearJointAxis.setValues(0, 0, 0);

		aJ.setFrom(jointAxis);
		world2A.transform(aJ);

		bJ.setFrom(jointAxis);
		bJ.negate();
		world2B.transform(bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0);
	}

	/**
	 * Angular constraint between two different rigidbodies.
	 */
	void initMinimal(
    Vector3 axisInA,
		Vector3 axisInB,
		Vector3 inertiaInvA,
		Vector3 inertiaInvB
  ){
		linearJointAxis.setValues(0, 0, 0);
		aJ.setFrom(axisInA);

		bJ.setFrom(axisInB);
		bJ.negate();

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0);
	}

	/**
	 * Constraint on one rigidbody.
	 */
	void initSingleWorld(
		Matrix3 world2A,
		Vector3 relPos1, 
    Vector3 relPos2,
		Vector3 jointAxis,
		Vector3 inertiaInvA, 
		double massInvA
  ){
		linearJointAxis.setFrom(jointAxis);

		aJ.cross2(relPos1, jointAxis);
		world2A.transform(aJ);

		bJ.setFrom(jointAxis);
		bJ.negate();
		bJ.cross2(relPos2, bJ);
		world2A.transform(bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		m_1MinvJt.setValues(0, 0, 0);
		Adiag = massInvA + m_0MinvJt.dot(aJ);

		assert (Adiag > 0);
	}

	double getDiagonal() { return Adiag; }

	/**
	 * For two constraints on the same rigidbody (for example vehicle friction).
	 */
	double getNonDiagonalFromSingleMass(JacobianEntry jacB, double massInvA) {
		JacobianEntry jacA = this;
		double lin = massInvA * jacA.linearJointAxis.dot(jacB.linearJointAxis);
		double ang = jacA.m_0MinvJt.dot(jacB.aJ);
		return lin + ang;
	}

	/**
	 * For two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies).
	 */
	double getNonDiagonalFromDoubleMass(JacobianEntry jacB, double massInvA, double massInvB) {
		JacobianEntry jacA = this;

		Vector3 lin = Vector3.zero();
		VectorUtil.mul(lin, jacA.linearJointAxis, jacB.linearJointAxis);

		Vector3 ang0 = Vector3.zero();
		VectorUtil.mul(ang0, jacA.m_0MinvJt, jacB.aJ);

		Vector3 ang1 = Vector3.zero();
		VectorUtil.mul(ang1, jacA.m_1MinvJt, jacB.bJ);

		Vector3 lin0 = Vector3.zero();
		lin0.scaleFrom(massInvA, lin);

		Vector3 lin1 = Vector3.zero();
		lin1.scaleFrom(massInvB, lin);

		Vector3 sum = Vector3.zero();
		VectorUtil.add(sum, ang0, ang1, lin0, lin1);

		return sum.x + sum.y + sum.z;
	}

	double getRelativeVelocity(Vector3 linvelA, Vector3 angvelA, Vector3 linvelB, Vector3 angvelB) {
		Vector3 linrel = Vector3.zero();
		linrel.sub2(linvelA, linvelB);

		Vector3 angvela = Vector3.zero();
		VectorUtil.mul(angvela, angvelA, aJ);

		Vector3 angvelb = Vector3.zero();
		VectorUtil.mul(angvelb, angvelB, bJ);

		VectorUtil.mul(linrel, linrel, linearJointAxis);

		angvela.add(angvelb);
		angvela.add(linrel);

		double rel_vel2 = angvela.x + angvela.y + angvela.z;
		return rel_vel2 + BulletGlobals.fltEpsilon;
	}
	
}
