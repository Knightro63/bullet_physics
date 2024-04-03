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

import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/core/bullet_stats.dart";
import "package:bullet_physics/core/contact_destroyed_callback.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/narrowphase/manifold_point.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/dynamics/constraintsolver/constraint_persistent_data.dart";
import "package:bullet_physics/dynamics/constraintsolver/constraint_solver.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_constraint_enum.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_solver_func.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart";
import "package:bullet_physics/dynamics/constraintsolver/jacobian_entry.dart";
import "package:bullet_physics/dynamics/constraintsolver/solver_body.dart";
import "package:bullet_physics/dynamics/constraintsolver/solver_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/solver_constraint_type.dart";
import "package:bullet_physics/dynamics/constraintsolver/solver_mode.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/int_array_list.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * SequentialImpulseConstraintSolver uses a Propagation Method and Sequentially applies impulses.
 * The approach is the 3D version of Erin Catto's GDC 2006 tutorial. See http://www.gphysics.com<p>
 * 
 * Although Sequential Impulse is more intuitive, it is mathematically equivalent to Projected
 * Successive Overrelaxation (iterative LCP).<p>
 * 
 * Applies impulses for combined restitution and penetration recovery and to simulate friction.
 * 
 * @author jezek2
 */

class _CDC extends ContactDestroyedCallback {
  @override
  bool contactDestroyed(Object? userPersistentData) {
    assert (userPersistentData != null);
    //ConstraintPersistentData? cpd = userPersistentData as ConstraintPersistentData?;
    //btAlignedFree(cpd);
   SequentialImpulseConstraintSolver._totalCpd--;
    //printf("totalCpd = %i. DELETED Ptr %x\n",totalCpd,userPersistentData);
    return true;
  }
}
class SequentialImpulseConstraintSolver extends ConstraintSolver {
	static final int _maxContactSolverTypes = ContactSolverType.maxCST.index;

	static const int _sequentialImpulseMaxSolverPoints = 16384;
	final List<_OrderIndex?> _gOrder = List.filled(_sequentialImpulseMaxSolverPoints, null);//_OrderIndex[_sequentialImpulseMaxSolverPoints];
	static int _totalCpd = 0;
	
	////////////////////////////////////////////////////////////////////////////
	final ObjectArrayList<SolverBody> _tmpSolverBodyPool = ObjectArrayList();//List<SolverBody>();
	final ObjectArrayList<SolverConstraint> _tmpSolverConstraintPool = ObjectArrayList();//List<SolverConstraint>();
	final ObjectArrayList<SolverConstraint> _tmpSolverFrictionConstraintPool = ObjectArrayList();//List<SolverConstraint>();
	final IntArrayList _orderTmpConstraintPool = IntArrayList();
	final IntArrayList _orderFrictionConstraintPool = IntArrayList();

	final List<List<ContactSolverFunc?>> contactDispatch = List.filled(_maxContactSolverTypes, new List.filled(_maxContactSolverTypes, null));//ContactSolverFunc[_maxContactSolverTypes][_maxContactSolverTypes];
	final List<List<ContactSolverFunc?>> frictionDispatch = List.filled(_maxContactSolverTypes, new List.filled(_maxContactSolverTypes, null));//ContactSolverFunc[_maxContactSolverTypes][_maxContactSolverTypes];
	
	// btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
	int btSeed2 = 0;

	SequentialImpulseConstraintSolver() {
    for (int i=0; i<_gOrder.length; i++) {
      _gOrder[i] = _OrderIndex();
    }
  
		BulletGlobals.setContactDestroyedCallback(_CDC());

		// initialize default friction/contact funcs
		for (int i = 0; i < _maxContactSolverTypes; i++) {
			for (int j = 0; j < _maxContactSolverTypes; j++) {
				contactDispatch[i][j] = ContactConstraint.resolveSingleCollision;
				frictionDispatch[i][j] = ContactConstraint.resolveSingleFriction;
			}
		}
	}
	
	int rand2() {
		btSeed2 = (1664525 * btSeed2 + 1013904223) & 0xffffffff;
		return btSeed2;
	}
	
	// See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
	int randInt2(int n) {
		// seems good; xor-fold and modulus
		int un = n;
		int r = rand2();

		// note: probably more aggressive than it needs to be -- might be
		//       able to get away without one or two of the innermost branches.
		if (un <= 0x00010000) {
			r ^= (r >>> 16);
			if (un <= 0x00000100) {
				r ^= (r >>> 8);
				if (un <= 0x00000010) {
					r ^= (r >>> 4);
					if (un <= 0x00000004) {
						r ^= (r >>> 2);
						if (un <= 0x00000002) {
							r ^= (r >>> 1);
						}
					}
				}
			}
		}

		// TODO: check modulo C vs Java mismatch
		return (r % un).abs();
	}
	
	void _initSolverBody(SolverBody solverBody, CollisionObject? collisionObject) {
		RigidBody? rb = RigidBody.upcast(collisionObject);
		if (rb != null) {
			rb.getAngularVelocity(solverBody.angularVelocity);
			solverBody.centerOfMassPosition.setFrom(collisionObject?.getWorldTransform(Transform()).origin ?? Vector3.zero());
			solverBody.friction = collisionObject?.getFriction() ?? 0;
			solverBody.invMass = rb.getInvMass();
			rb.getLinearVelocity(solverBody.linearVelocity);
			solverBody.originalBody = rb;
			solverBody.angularFactor = rb.getAngularFactor();
		}
		else {
			solverBody.angularVelocity.setValues(0, 0, 0);
			solverBody.centerOfMassPosition.setFrom(collisionObject?.getWorldTransform(Transform()).origin ?? Vector3.zero());
			solverBody.friction = collisionObject?.getFriction() ?? 0;
			solverBody.invMass = 0;
			solverBody.linearVelocity.setValues(0, 0, 0);
			solverBody.originalBody = null;
			solverBody.angularFactor = 1;
		}

		solverBody.pushVelocity.setValues(0, 0, 0);
		solverBody.turnVelocity.setValues(0, 0, 0);
	}
	
	double _restitutionCurve(double relVel, double restitution) {
		double rest = restitution * -relVel;
		return rest;
	}
	
	void _resolveSplitPenetrationImpulseCacheFriendly(
    SolverBody? body1,
    SolverBody? body2,
    SolverConstraint? contactConstraint,
    ContactSolverInfo? solverInfo
  ) {
		
		if ((contactConstraint?.penetration ?? 0) < (solverInfo?.splitImpulsePenetrationThreshold ?? 0)) {
			BulletStats.gNumSplitImpulseRecoveries++;
			double normalImpulse;

			//  Optimized version of projected relative velocity, use precomputed cross products with normal
			//      body1.getVelocityInLocalPoint(contactConstraint.m_rel_posA,vel1);
			//      body2.getVelocityInLocalPoint(contactConstraint.m_rel_posB,vel2);
			//      btVector3 vel = vel1 - vel2;
			//      btScalar  relVel = contactConstraint.m_contactNormal.dot(vel);

			double relVel;
			double vel1Dotn = (contactConstraint?.contactNormal.dot(body1?.pushVelocity ?? Vector3.zero()) ?? 0) + (contactConstraint?.relpos1CrossNormal.dot(body1?.turnVelocity ?? Vector3.zero()) ?? 0);
			double vel2Dotn = (contactConstraint?.contactNormal.dot(body2?.pushVelocity ?? Vector3.zero()) ?? 0) + (contactConstraint?.relpos2CrossNormal.dot(body2?.turnVelocity ?? Vector3.zero()) ?? 0);

			relVel = vel1Dotn - vel2Dotn;

			double positionalError = solverInfo != null?-(contactConstraint?.penetration ?? 0) * solverInfo.erp2 / solverInfo.timeStep:0;
			//      btScalar positionalError = contactConstraint.m_penetration;

			double velocityError = (contactConstraint?.restitution ?? 0) - relVel;// * damping;

			double penetrationImpulse = positionalError * (contactConstraint?.jacDiagABInv ?? 0);
			double velocityImpulse = velocityError * (contactConstraint?.jacDiagABInv ?? 0);
			normalImpulse = penetrationImpulse + velocityImpulse;

			// See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
			double oldNormalImpulse = contactConstraint?.appliedPushImpulse ?? 0;
			double sum = oldNormalImpulse + normalImpulse;
			contactConstraint?.appliedPushImpulse = 0 > sum ? 0 : sum;

			normalImpulse = (contactConstraint?.appliedPushImpulse ?? 0) - oldNormalImpulse;

			Vector3 tmp = Vector3.zero();

			tmp.scaleFrom((body1?.invMass ?? 0), contactConstraint?.contactNormal);
			body1?.internalApplyPushImpulse(tmp, contactConstraint?.angularComponentA ?? Vector3.zero(), normalImpulse);

			tmp.scaleFrom((body2?.invMass ?? 0), contactConstraint?.contactNormal);
			body2?.internalApplyPushImpulse(tmp, contactConstraint?.angularComponentB ?? Vector3.zero(), -normalImpulse);
		}
	}

	/**
	 * velocity + friction
	 * response  between two dynamic objects with friction
	 */
	double _resolveSingleCollisionCombinedCacheFriendly(
    SolverBody? body1,
    SolverBody? body2,
    SolverConstraint? contactConstraint,
    ContactSolverInfo? solverInfo
  ) {

		double normalImpulse;

		{
			//  Optimized version of projected relative velocity, use precomputed cross products with normal
			//	body1.getVelocityInLocalPoint(contactConstraint.m_rel_posA,vel1);
			//	body2.getVelocityInLocalPoint(contactConstraint.m_rel_posB,vel2);
			//	btVector3 vel = vel1 - vel2;
			//	btScalar  relVel = contactConstraint.m_contactNormal.dot(vel);

			double relVel;
			double vel1Dotn = (contactConstraint?.contactNormal.dot(body1?.linearVelocity ?? Vector3.zero()) ?? 0) + (contactConstraint?.relpos1CrossNormal.dot(body1?.angularVelocity ?? Vector3.zero()) ?? 0);
			double vel2Dotn = (contactConstraint?.contactNormal.dot(body2?.linearVelocity ?? Vector3.zero()) ?? 0) + (contactConstraint?.relpos2CrossNormal.dot(body2?.angularVelocity ?? Vector3.zero()) ?? 0);

			relVel = vel1Dotn - vel2Dotn;

			double positionalError = 0;
			if (!(solverInfo?.splitImpulse ?? false) || ((contactConstraint?.penetration ?? 0) > (solverInfo?.splitImpulsePenetrationThreshold ?? 0))) {
				positionalError = solverInfo != null?-(contactConstraint?.penetration ?? 0) * solverInfo.erp / solverInfo.timeStep:0;
			}

			double velocityError = (contactConstraint?.restitution ?? 0) - relVel;// * damping;

			double penetrationImpulse = positionalError * (contactConstraint?.jacDiagABInv ?? 0);
			double velocityImpulse = velocityError * (contactConstraint?.jacDiagABInv ?? 0);
			normalImpulse = penetrationImpulse + velocityImpulse;


			// See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
			double oldNormalImpulse = contactConstraint?.appliedImpulse ?? 0;
			double sum = oldNormalImpulse + normalImpulse;
			contactConstraint?.appliedImpulse = 0 > sum ? 0 : sum;

			normalImpulse = (contactConstraint?.appliedImpulse ?? 0) - oldNormalImpulse;

			Vector3 tmp = Vector3.zero();

			tmp.scaleFrom((body1?.invMass ?? 0), contactConstraint?.contactNormal);
			body1?.internalApplyImpulse(tmp, contactConstraint?.angularComponentA ?? Vector3.zero(), normalImpulse);

			tmp.scaleFrom((body2?.invMass ?? 0), contactConstraint?.contactNormal);
			body2?.internalApplyImpulse(tmp, contactConstraint?.angularComponentB ?? Vector3.zero(), -normalImpulse);
		}

		return normalImpulse;
	}
	
	double _resolveSingleFrictionCacheFriendly(
    SolverBody? body1,
    SolverBody? body2,
    SolverConstraint? contactConstraint,
    ContactSolverInfo? solverInfo,
    double appliedNormalImpulse
  ) {
		double combinedFriction = contactConstraint?.friction ?? 0;

		double limit = appliedNormalImpulse * combinedFriction;

		if (appliedNormalImpulse > 0) //friction
		{

			double j1;
			{

				double relVel;
				double vel1Dotn = (contactConstraint?.contactNormal.dot(body1?.linearVelocity ?? Vector3.zero()) ?? 0) + (contactConstraint?.relpos1CrossNormal.dot(body1?.angularVelocity ?? Vector3.zero()) ?? 0);
				double vel2Dotn = (contactConstraint?.contactNormal.dot(body2?.linearVelocity ?? Vector3.zero()) ?? 0) + (contactConstraint?.relpos2CrossNormal.dot(body2?.angularVelocity ?? Vector3.zero()) ?? 0);
				relVel = vel1Dotn - vel2Dotn;

				// calculate j that moves us to zero relative velocity
				j1 = -relVel * (contactConstraint?.jacDiagABInv ?? 0);
				//#define CLAMP_ACCUMULATED_FRICTION_IMPULSE 1
				//#ifdef CLAMP_ACCUMULATED_FRICTION_IMPULSE
				double oldTangentImpulse = (contactConstraint?.appliedImpulse ?? 0);
				contactConstraint?.appliedImpulse = oldTangentImpulse + j1;

				if (limit < (contactConstraint?.appliedImpulse ?? 0)) {
					contactConstraint?.appliedImpulse = limit;
				}
				else {
					if ((contactConstraint?.appliedImpulse ?? 0) < -limit) {
						contactConstraint?.appliedImpulse = -limit;
					}
				}
				j1 = (contactConstraint?.appliedImpulse ?? 0) - oldTangentImpulse;
				//	#else
				//	if (limit < j1)
				//	{
				//		j1 = limit;
				//	} else
				//	{
				//		if (j1 < -limit)
				//			j1 = -limit;
				//	}
				//	#endif

				//GEN_set_min(contactConstraint.m_appliedImpulse, limit);
				//GEN_set_max(contactConstraint.m_appliedImpulse, -limit);
			}

			Vector3 tmp = Vector3.zero();
			
			tmp.scaleFrom((body1?.invMass ?? 0), contactConstraint?.contactNormal);
			body1?.internalApplyImpulse(tmp, contactConstraint?.angularComponentA ?? Vector3.zero(), j1);
			
			tmp.scaleFrom((body2?.invMass ?? 0), contactConstraint?.contactNormal);
			body2?.internalApplyImpulse(tmp, contactConstraint?.angularComponentB ?? Vector3.zero(), -j1);
		}
		return 0;
	}
	

	void addFrictionConstraint(Vector3 normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, ManifoldPoint cp, Vector3 relPos1, Vector3 relPos2, CollisionObject? colObj0, CollisionObject? colObj1, double relaxation) {
		RigidBody? body0 = RigidBody.upcast(colObj0);
		RigidBody? body1 = RigidBody.upcast(colObj1);

		SolverConstraint solverConstraint = SolverConstraint();
		_tmpSolverFrictionConstraintPool.add(solverConstraint);

		solverConstraint.contactNormal.setFrom(normalAxis);

		solverConstraint.solverBodyIdA = solverBodyIdA;
		solverConstraint.solverBodyIdB = solverBodyIdB;
		solverConstraint.constraintType = SolverConstraintType.friction1d;
		solverConstraint.frictionIndex = frictionIndex;

		solverConstraint.friction = cp.combinedFriction;
		solverConstraint.originalContactPoint = null;

		solverConstraint.appliedImpulse = 0;
		solverConstraint.appliedPushImpulse = 0;
		solverConstraint.penetration = 0;
		
		Vector3 ftorqueAxis1 = Vector3.zero();
		Matrix3 tmpMat = Matrix3.zero();
		
		{
			ftorqueAxis1.cross2(relPos1, solverConstraint.contactNormal);
			solverConstraint.relpos1CrossNormal.setFrom(ftorqueAxis1);
			if (body0 != null) {
				solverConstraint.angularComponentA.setFrom(ftorqueAxis1);
				body0.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentA);
			}
			else {
				solverConstraint.angularComponentA.setValues(0, 0, 0);
			}
		}
		{
			ftorqueAxis1.cross2(relPos2, solverConstraint.contactNormal);
			solverConstraint.relpos2CrossNormal.setFrom(ftorqueAxis1);
			if (body1 != null) {
				solverConstraint.angularComponentB.setFrom(ftorqueAxis1);
				body1.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentB);
			}
			else {
				solverConstraint.angularComponentB.setValues(0, 0, 0);
			}
		}

		//#ifdef COMPUTE_IMPULSE_DENOM
		//	btScalar denom0 = rb0->computeImpulseDenominator(pos1,solverConstraint.m_contactNormal);
		//	btScalar denom1 = rb1->computeImpulseDenominator(pos2,solverConstraint.m_contactNormal);
		//#else
		Vector3 vec = Vector3.zero();
		double denom0 = 0;
		double denom1 = 0;
		if (body0 != null) {
			vec.cross2(solverConstraint.angularComponentA, relPos1);
			denom0 = body0.getInvMass() + normalAxis.dot(vec);
		}
		if (body1 != null) {
			vec.cross2(solverConstraint.angularComponentB, relPos2);
			denom1 = body1.getInvMass() + normalAxis.dot(vec);
		}
		//#endif //COMPUTE_IMPULSE_DENOM

		double denom = relaxation / (denom0 + denom1);
		solverConstraint.jacDiagABInv = denom;
	}
	
	double solveGroupCacheFriendlySetup(
    ObjectArrayList<CollisionObject>? bodies, 
    int numBodies, 
    ObjectArrayList<PersistentManifold>? manifoldPtr, 
    int manifoldOffset, 
    int numManifolds, 
    ObjectArrayList<TypedConstraint>? constraints, 
    int constraintsOffset, 
    int numConstraints, 
    ContactSolverInfo? infoGlobal, 
    IDebugDraw? debugDrawer/*,btStackAlloc* stackAlloc*/
  ) {
		BulletStats.pushProfile("solveGroupCacheFriendlySetup");
		try {

			if ((numConstraints + numManifolds) == 0) {
				// printf("empty\n");
				return 0;
			}
			PersistentManifold? manifold;
			CollisionObject? colObj0, colObj1;
			Transform tmpTrans = Transform();

			//if (1)
			{
				{
					Vector3 relPos1 = Vector3.zero();
					Vector3 relPos2 = Vector3.zero();

					Vector3 pos1 = Vector3.zero();
					Vector3 pos2 = Vector3.zero();
					Vector3 vel = Vector3.zero();
					Vector3 torqueAxis0 = Vector3.zero();
					Vector3 torqueAxis1 = Vector3.zero();
					Vector3 vel1 = Vector3.zero();
					Vector3 vel2 = Vector3.zero();
					//Vector3 frictionDir1 = Vector3.zero();
					//Vector3 frictionDir2 = Vector3.zero();
					Vector3 vec = Vector3.zero();

					Matrix3 tmpMat = Matrix3.zero();
					
					for (int i = 0; i < numManifolds; i++) {
						manifold = manifoldPtr?[manifoldOffset+i];
						colObj0 = manifold?.getBody0() as CollisionObject?;
						colObj1 = manifold?.getBody1() as CollisionObject?;

						int solverBodyIdA = -1;
						int solverBodyIdB = -1;

						if (manifold?.getNumContacts() != 0) {
							if ((colObj0?.getIslandTag() ?? 0) >= 0) {
								if ((colObj0?.getCompanionId() ?? 0) >= 0) {
									// body has already been converted
									solverBodyIdA = colObj0?.getCompanionId()?? 0;
								}
								else {
									solverBodyIdA = _tmpSolverBodyPool.size;
									SolverBody solverBody = SolverBody();
									_tmpSolverBodyPool.add(solverBody);
									_initSolverBody(solverBody, colObj0);
									colObj0?.setCompanionId(solverBodyIdA);
								}
							}
							else {
								// create a static body
								solverBodyIdA = _tmpSolverBodyPool.size;
								SolverBody solverBody = SolverBody();
								_tmpSolverBodyPool.add(solverBody);
								_initSolverBody(solverBody, colObj0);
							}

							if ((colObj1?.getIslandTag() ?? 0) >= 0) {
								if ((colObj1?.getCompanionId() ?? 0) >= 0) {
									solverBodyIdB = colObj1?.getCompanionId() ?? 0;
								}
								else {
									solverBodyIdB = _tmpSolverBodyPool.size;
									SolverBody solverBody = SolverBody();
									_tmpSolverBodyPool.add(solverBody);
									_initSolverBody(solverBody, colObj1);
									colObj1?.setCompanionId(solverBodyIdB);
								}
							}
							else {
								// create a static body
								solverBodyIdB = _tmpSolverBodyPool.size;
								SolverBody solverBody = SolverBody();
								_tmpSolverBodyPool.add(solverBody);
								_initSolverBody(solverBody, colObj1);
							}
						}

						double relaxation;

						for (int j = 0; j < (manifold?.getNumContacts() ?? 0); j++) {

							ManifoldPoint? cp = manifold?.getContactPoint(j);

							if ((cp?.getDistance() ?? 0) <= 0) {
								cp?.getPositionWorldOnA(pos1);
								cp?.getPositionWorldOnB(pos2);

								relPos1.sub2(pos1, colObj0?.getWorldTransform(tmpTrans).origin);
								relPos2.sub2(pos2, colObj1?.getWorldTransform(tmpTrans).origin);

								relaxation = 1;
								double relVel;

								int frictionIndex = _tmpSolverConstraintPool.size;

								{
									SolverConstraint solverConstraint = SolverConstraint();
									_tmpSolverConstraintPool.add(solverConstraint);
									RigidBody? rb0 = RigidBody.upcast(colObj0);
									RigidBody? rb1 = RigidBody.upcast(colObj1);

									solverConstraint.solverBodyIdA = solverBodyIdA;
									solverConstraint.solverBodyIdB = solverBodyIdB;
									solverConstraint.constraintType = SolverConstraintType.contact1d;
									
									solverConstraint.originalContactPoint = cp;

									torqueAxis0.cross2(relPos1, cp?.normalWorldOnB);

									if (rb0 != null) {
										solverConstraint.angularComponentA.setFrom(torqueAxis0);
										rb0.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentA);
									}
									else {
										solverConstraint.angularComponentA.setValues(0, 0, 0);
									}

									torqueAxis1.cross2(relPos2, cp?.normalWorldOnB);

									if (rb1 != null) {
										solverConstraint.angularComponentB.setFrom(torqueAxis1);
										rb1.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentB);
									}
									else {
										solverConstraint.angularComponentB.setValues(0, 0, 0);
									}

									{					
										double denom0 = 0;
										double denom1 = 0;
										if (rb0 != null) {
											vec.cross2(solverConstraint.angularComponentA, relPos1);
											denom0 = rb0.getInvMass() + (cp?.normalWorldOnB.dot(vec) ?? 0);
										}
										if (rb1 != null) {
											vec.cross2(solverConstraint.angularComponentB, relPos2);
											denom1 = rb1.getInvMass() + (cp?.normalWorldOnB.dot(vec) ?? 0);
										}

										double denom = relaxation / (denom0 + denom1);
										solverConstraint.jacDiagABInv = denom;
									}
                  if(cp != null){
									  solverConstraint.contactNormal.setFrom(cp.normalWorldOnB);
                  }
									solverConstraint.relpos1CrossNormal.cross2(relPos1, cp?.normalWorldOnB);
									solverConstraint.relpos2CrossNormal.cross2(relPos2, cp?.normalWorldOnB);

									if (rb0 != null) {
										rb0.getVelocityInLocalPoint(relPos1, vel1);
									}
									else {
										vel1.setValues(0, 0, 0);
									}
									
									if (rb1 != null) {
										rb1.getVelocityInLocalPoint(relPos2, vel2);
									}
									else {
										vel2.setValues(0, 0, 0);
									}

									vel.sub2(vel1, vel2);

									relVel = (cp?.normalWorldOnB.dot(vel) ?? 0);

									solverConstraint.penetration = min((cp?.getDistance() ?? 0) + (infoGlobal?.linearSlop ?? 0), 0);
									//solverConstraint.m_penetration = cp.getDistance();
									
									solverConstraint.friction = (cp?.combinedFriction ?? 0);
									solverConstraint.restitution = _restitutionCurve(relVel, (cp?.combinedRestitution ?? 0));
									if (solverConstraint.restitution <= 0) {
										solverConstraint.restitution = 0;
									}

									double penVel = infoGlobal?.timeStep == null?0:-solverConstraint.penetration / infoGlobal!.timeStep;

									if (solverConstraint.restitution > penVel) {
										solverConstraint.penetration = 0;
									}
									
									Vector3 tmp = Vector3.zero();

									// warm starting (or zero if disabled)
									if (((infoGlobal?.solverMode ?? 0) & SolverMode.useWarmStarting) != 0) {
										solverConstraint.appliedImpulse = (cp?.appliedImpulse ?? 0) * (infoGlobal?.warmstartingFactor ?? 0);
										if (rb0 != null) {
											tmp.scaleFrom(rb0.getInvMass(), solverConstraint.contactNormal);
											_tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdA)?.internalApplyImpulse(tmp, solverConstraint.angularComponentA, solverConstraint.appliedImpulse);
										}
										if (rb1 != null) {
											tmp.scaleFrom(rb1.getInvMass(), solverConstraint.contactNormal);
											_tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdB)?.internalApplyImpulse(tmp, solverConstraint.angularComponentB, -solverConstraint.appliedImpulse);
										}
									}
									else {
										solverConstraint.appliedImpulse = 0;
									}

									solverConstraint.appliedPushImpulse = 0;

									solverConstraint.frictionIndex = _tmpSolverFrictionConstraintPool.size;
									if (!(cp?.lateralFrictionInitialized ?? false)) {
										cp?.lateralFrictionDir1.scaleFrom(relVel, cp.normalWorldOnB);
										cp?.lateralFrictionDir1.sub2(vel, cp.lateralFrictionDir1);

										double latRelVel = (cp?.lateralFrictionDir1.length2 ?? 0);
										if (latRelVel > BulletGlobals.fltEpsilon){//0.0)
											cp?.lateralFrictionDir1.scale(1 / sqrt(latRelVel));
											addFrictionConstraint(cp!.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
											cp.lateralFrictionDir2.cross2(cp.lateralFrictionDir1, cp.normalWorldOnB);
											cp.lateralFrictionDir2.normalize(); //??
											addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
										}
										else {
											// re-calculate friction direction every frame, todo: check if this is really needed

											TransformUtil.planeSpace1(cp!.normalWorldOnB, cp.lateralFrictionDir1, cp.lateralFrictionDir2);
											addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
											addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
										}
										cp.lateralFrictionInitialized = true;

									}
									else {
										addFrictionConstraint(cp!.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
										addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
									}

									{
										SolverConstraint? frictionConstraint1 = _tmpSolverFrictionConstraintPool.getQuick(solverConstraint.frictionIndex);
										if (((infoGlobal?.solverMode ?? 0) & SolverMode.useWarmStarting) != 0) {
											frictionConstraint1?.appliedImpulse = cp.appliedImpulseLateral1 * (infoGlobal?.warmstartingFactor ?? 0);
											if (rb0 != null) {
												tmp.scaleFrom(rb0.getInvMass(), frictionConstraint1!.contactNormal);
												_tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdA)?.internalApplyImpulse(tmp, frictionConstraint1.angularComponentA, frictionConstraint1.appliedImpulse);
											}
											if (rb1 != null) {
												tmp.scaleFrom(rb1.getInvMass(), frictionConstraint1!.contactNormal);
												_tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdB)?.internalApplyImpulse(tmp, frictionConstraint1.angularComponentB, -frictionConstraint1.appliedImpulse);
											}
										}
										else {
											frictionConstraint1?.appliedImpulse = 0;
										}
									}
									{
										SolverConstraint? frictionConstraint2 = _tmpSolverFrictionConstraintPool.getQuick(solverConstraint.frictionIndex + 1);
										if (((infoGlobal?.solverMode ?? 0) & SolverMode.useWarmStarting) != 0) {
											frictionConstraint2?.appliedImpulse = cp.appliedImpulseLateral2 * (infoGlobal?.warmstartingFactor ?? 0);
											if (rb0 != null) {
												tmp.scaleFrom(rb0.getInvMass(), frictionConstraint2!.contactNormal);
												_tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdA)?.internalApplyImpulse(tmp, frictionConstraint2.angularComponentA, frictionConstraint2.appliedImpulse);
											}
											if (rb1 != null) {
												tmp.scaleFrom(rb1.getInvMass(), frictionConstraint2!.contactNormal);
												_tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdB)?.internalApplyImpulse(tmp, frictionConstraint2.angularComponentB, -frictionConstraint2.appliedImpulse);
											}
										}
										else {
											frictionConstraint2?.appliedImpulse = 0;
										}
									}
								}
							}
						}
					}
				}
			}

			for (int j = 0; j < numConstraints; j++) {
        TypedConstraint? constraint = constraints![constraintsOffset+j];
        constraint?.buildJacobian();
      }

      for (int j = 0; j < numConstraints; j++) {
        TypedConstraint? constraint = constraints![constraintsOffset+j];
        constraint?.getInfo2(infoGlobal as ContactSolverInfo);
      }

			int numConstraintPool = _tmpSolverConstraintPool.size;
			int numFrictionPool = _tmpSolverFrictionConstraintPool.size;

			// todo: use stack allocator for such temporarily memory, same for solver bodies/constraints
			MiscUtil.resizeArray(_orderTmpConstraintPool, numConstraintPool, 0);
			MiscUtil.resizeArray(_orderFrictionConstraintPool, numFrictionPool, 0);

      for (int i = 0; i < numConstraintPool; i++) {
        _orderTmpConstraintPool.set(i, i);
      }
      for (int i = 0; i < numFrictionPool; i++) {
        _orderFrictionConstraintPool.set(i, i);
      }
			
			return 0;
		}
		finally {
			BulletStats.popProfile();
		}
	}
	
	double solveGroupCacheFriendlyIterations(
    ObjectArrayList<CollisionObject>? bodies, 
    int numBodies, 
    ObjectArrayList<PersistentManifold>? manifoldPtr, 
    int manifoldOffset, 
    int numManifolds, 
    ObjectArrayList<TypedConstraint>? constraints, 
    int constraintsOffset, 
    int numConstraints, 
    ContactSolverInfo? infoGlobal, 
    IDebugDraw? debugDrawer
  ) {
		BulletStats.pushProfile("solveGroupCacheFriendlyIterations");
		try {
			int numConstraintPool = _tmpSolverConstraintPool.size;
			int numFrictionPool = _tmpSolverFrictionConstraintPool.size;

			// should traverse the contacts random order...
			int iteration;
			{
				for (iteration = 0; iteration < (infoGlobal?.numIterations ?? 0); iteration++) {

					int j;
					if (((infoGlobal?.solverMode ?? 0) & SolverMode.randomOrder) != 0) {
						if ((iteration & 7) == 0) {
							for (j = 0; j < numConstraintPool; ++j) {
								int tmp = _orderTmpConstraintPool.get(j);
								int swapi = randInt2(j + 1);
								_orderTmpConstraintPool.set(j, _orderTmpConstraintPool.get(swapi));
								_orderTmpConstraintPool.set(swapi, tmp);
							}

							for (j = 0; j < numFrictionPool; ++j) {
								int tmp = _orderFrictionConstraintPool.get(j);
								int swapi = randInt2(j + 1);
								_orderFrictionConstraintPool.set(j, _orderFrictionConstraintPool.get(swapi));
								_orderFrictionConstraintPool.set(swapi, tmp);
							}
						}
					}

					for (j = 0; j < numConstraints; j++) {
						TypedConstraint constraint = constraints![constraintsOffset+j]!;
						// todo: use solver bodies, so we don't need to copy from/to btRigidBody

						if ((constraint.getRigidBodyA().getIslandTag() >= 0) && (constraint.getRigidBodyA().getCompanionId() >= 0)) {
							_tmpSolverBodyPool.getQuick(constraint.getRigidBodyA().getCompanionId())?.writebackVelocity();
						}
						if ((constraint.getRigidBodyB().getIslandTag() >= 0) && (constraint.getRigidBodyB().getCompanionId() >= 0)) {
							_tmpSolverBodyPool.getQuick(constraint.getRigidBodyB().getCompanionId())?.writebackVelocity();
						}

						constraint.solveConstraint(infoGlobal?.timeStep ?? 0);

						if ((constraint.getRigidBodyA().getIslandTag() >= 0) && (constraint.getRigidBodyA().getCompanionId() >= 0)) {
							_tmpSolverBodyPool.getQuick(constraint.getRigidBodyA().getCompanionId())?.readVelocity();
						}
						if ((constraint.getRigidBodyB().getIslandTag() >= 0) && (constraint.getRigidBodyB().getCompanionId() >= 0)) {
							_tmpSolverBodyPool.getQuick(constraint.getRigidBodyB().getCompanionId())?.readVelocity();
						}
					}

					{
						int numPoolConstraints = _tmpSolverConstraintPool.size;
						for (j = 0; j < numPoolConstraints; j++) {
							SolverConstraint? solveManifold = _tmpSolverConstraintPool.getQuick(_orderTmpConstraintPool.get(j));
							_resolveSingleCollisionCombinedCacheFriendly(
                _tmpSolverBodyPool.getQuick(solveManifold?.solverBodyIdA ?? 0),
                _tmpSolverBodyPool.getQuick(solveManifold?.solverBodyIdB ?? 0), 
                solveManifold, 
                infoGlobal
              );
						}
					}

					{
						int numFrictionPoolConstraints = _tmpSolverFrictionConstraintPool.size;

						for (j = 0; j < numFrictionPoolConstraints; j++) {
							SolverConstraint? solveManifold = _tmpSolverFrictionConstraintPool.getQuick(_orderFrictionConstraintPool.get(j));
							
							double totalImpulse = (_tmpSolverConstraintPool.getQuick(solveManifold?.frictionIndex ?? 0)?.appliedImpulse ?? 0) +
							  (_tmpSolverConstraintPool.getQuick(solveManifold?.frictionIndex ?? 0)?.appliedPushImpulse ?? 0);
							
							_resolveSingleFrictionCacheFriendly(
                _tmpSolverBodyPool.getQuick(solveManifold?.solverBodyIdA ?? 0),
							  _tmpSolverBodyPool.getQuick(solveManifold?.solverBodyIdB ?? 0), 
                solveManifold, 
                infoGlobal,
							  totalImpulse
              );
						}
					}
				}
				
				if (infoGlobal?.splitImpulse ?? false) {
					for (iteration = 0; iteration < (infoGlobal?.numIterations ?? 0); iteration++) {
						{
							int numPoolConstraints = _tmpSolverConstraintPool.size;
							int j;
							for (j = 0; j < numPoolConstraints; j++) {
								SolverConstraint? solveManifold = _tmpSolverConstraintPool.getQuick(_orderTmpConstraintPool.get(j));

								_resolveSplitPenetrationImpulseCacheFriendly(
                  _tmpSolverBodyPool.getQuick(solveManifold?.solverBodyIdA ?? 0),
									_tmpSolverBodyPool.getQuick(solveManifold?.solverBodyIdB ?? 0), 
                  solveManifold, 
                  infoGlobal
                );
							}
						}
					}
				}
			}

			return 0;
		}
		finally {
			BulletStats.popProfile();		
		}
	}

	double solveGroupCacheFriendly(
    ObjectArrayList<CollisionObject>? bodies, 
    int numBodies, 
    ObjectArrayList<PersistentManifold>? manifoldPtr, 
    int manifoldOffset, 
    int numManifolds, 
    ObjectArrayList<TypedConstraint>? constraints, 
    int constraintsOffset, 
    int numConstraints, 
    ContactSolverInfo? infoGlobal, 
    IDebugDraw? debugDrawer/*,btStackAlloc* stackAlloc*/
  ) {
		solveGroupCacheFriendlySetup(bodies, numBodies, manifoldPtr, manifoldOffset, numManifolds, constraints, constraintsOffset, numConstraints, infoGlobal, debugDrawer/*, stackAlloc*/);
		solveGroupCacheFriendlyIterations(bodies, numBodies, manifoldPtr, manifoldOffset, numManifolds, constraints, constraintsOffset, numConstraints, infoGlobal, debugDrawer/*, stackAlloc*/);
		
		int numPoolConstraints = _tmpSolverConstraintPool.size;
		for (int j=0; j<numPoolConstraints; j++) {

			SolverConstraint? solveManifold = _tmpSolverConstraintPool.getQuick(j);
			ManifoldPoint? pt = solveManifold?.originalContactPoint as ManifoldPoint?;
			assert (pt != null);
			pt!.appliedImpulse = solveManifold!.appliedImpulse;
			pt.appliedImpulseLateral1 = _tmpSolverFrictionConstraintPool.getQuick(solveManifold.frictionIndex)!.appliedImpulse;
			pt.appliedImpulseLateral1 = _tmpSolverFrictionConstraintPool.getQuick(solveManifold.frictionIndex + 1)!.appliedImpulse;

			// do a callback here?
		}

		if (infoGlobal?.splitImpulse ?? false) {
			for (int i=0; i<_tmpSolverBodyPool.size; i++) {
				_tmpSolverBodyPool.getQuick(i)?.writebackVelocity(infoGlobal?.timeStep ?? 0);
			}
		}
		else {
			for (int i=0; i<_tmpSolverBodyPool.size; i++) {
				_tmpSolverBodyPool.getQuick(i)?.writebackVelocity();
			}
		}

		_tmpSolverBodyPool.clear();
		_tmpSolverConstraintPool.clear();
		_tmpSolverFrictionConstraintPool.clear();

		return 0;
	}
	
	/**
	 * Sequentially applies impulses.
	 */
	@override
	double solveGroup(
    ObjectArrayList<CollisionObject>? bodies, 
    int numBodies, 
    ObjectArrayList<PersistentManifold>? manifoldPtr, 
    int manifoldOffset, 
    int numManifolds,
    ObjectArrayList<TypedConstraint>? constraints, 
    int constraintsOffset, 
    int numConstraints, 
    ContactSolverInfo? infoGlobal, 
    IDebugDraw? debugDrawer, 
    Dispatcher? dispatcher
  ) {
		BulletStats.pushProfile("solveGroup");
		try {
			// TODO: solver cache friendly
			// if ((infoGlobal!.solverMode & SolverMode.cacheFriendly) != 0) {
			// 	// you need to provide at least some bodies
			// 	// SimpleDynamicsWorld needs to switch off SOLVER_CACHE_FRIENDLY
			// 	assert (bodies != null);
			// 	assert (numBodies != 0);
			// 	double value = solveGroupCacheFriendly(bodies, numBodies, manifoldPtr, manifoldOffset, numManifolds, constraints, constraintsOffset, numConstraints, infoGlobal, debugDrawer);
      //   return value;
			// }

			ContactSolverInfo info = ContactSolverInfo(infoGlobal);
			int numiter = infoGlobal!.numIterations;
			int totalPoints = 0;
      for (int j = 0; j < numManifolds; j++) {
        PersistentManifold? manifold = manifoldPtr?[manifoldOffset+j];
        prepareConstraints(manifold, info, debugDrawer);

        for (int p = 0; p < (manifoldPtr?[manifoldOffset+j]?.getNumContacts() ?? 0); p++) {
          _gOrder[totalPoints]?.manifoldIndex = j;
          _gOrder[totalPoints]?.pointIndex = p;
          totalPoints++;
        }
      }

      for (int j = 0; j < numConstraints; j++) {
        TypedConstraint? constraint = constraints?[constraintsOffset+j];
        constraint?.buildJacobian();
      }
			
      // should traverse the contacts random order...
      for (int iteration = 0; iteration < numiter; iteration++) {
        if ((infoGlobal.solverMode & SolverMode.randomOrder) != 0) {
          if ((iteration & 7) == 0) {
            for (int j = 0; j < totalPoints; ++j) {
              // JAVA NOTE: swaps references instead of copying values (but that's fine in this context)
              _OrderIndex tmp = _gOrder[j]!;
              int swapi = randInt2(j + 1);
              _gOrder[j] = _gOrder[swapi];
              _gOrder[swapi] = tmp;
            }
          }
        }

        for (int j = 0; j < numConstraints; j++) {
          TypedConstraint? constraint = constraints![constraintsOffset+j];
          constraint?.solveConstraint(info.timeStep);
        }

        for (int j = 0; j < totalPoints; j++) {
          PersistentManifold? manifold = manifoldPtr![manifoldOffset+_gOrder[j]!.manifoldIndex];
          solve(
            manifold?.getBody0() as RigidBody?,
            manifold?.getBody1() as RigidBody?, 
            manifold?.getContactPoint(_gOrder[j]!.pointIndex), 
            info, 
            iteration, 
            debugDrawer
          );
        }

        for (int j = 0; j < totalPoints; j++) {
          PersistentManifold? manifold = manifoldPtr![manifoldOffset+_gOrder[j]!.manifoldIndex];
          solveFriction(
            manifold?.getBody0() as RigidBody?,
            manifold?.getBody1() as RigidBody?, 
            manifold?.getContactPoint(_gOrder[j]!.pointIndex), 
            info, 
            iteration, 
            debugDrawer
          );
        }
      }
			

			return 0;
		}
		finally {
			BulletStats.popProfile();
		}
	}
	
	void prepareConstraints(PersistentManifold? manifoldPtr, ContactSolverInfo info, IDebugDraw? debugDrawer) {
		RigidBody? body0 = manifoldPtr?.getBody0() as RigidBody;
		RigidBody? body1 = manifoldPtr?.getBody1() as RigidBody;

		// only necessary to refresh the manifold once (first iteration). The integration is done outside the loop
		{
			int numpoints = manifoldPtr?.getNumContacts() ?? 0;

			BulletStats.gTotalContactPoints += numpoints;
			
			Vector3 tmpVec = Vector3.zero();
			Matrix3 tmpMat3 = Matrix3.zero();

			Vector3 pos1 = Vector3.zero();
			Vector3 pos2 = Vector3.zero();
			Vector3 relPos1 = Vector3.zero();
			Vector3 relPos2 = Vector3.zero();
			Vector3 vel1 = Vector3.zero();
			Vector3 vel2 = Vector3.zero();
			Vector3 vel = Vector3.zero();
			Vector3 totalImpulse = Vector3.zero();
			Vector3 torqueAxis0 = Vector3.zero();
			Vector3 torqueAxis1 = Vector3.zero();
			Vector3 ftorqueAxis0 = Vector3.zero();
			Vector3 ftorqueAxis1 = Vector3.zero();
			
			for (int i = 0; i < numpoints; i++) {
				ManifoldPoint cp = manifoldPtr!.getContactPoint(i);
				if (cp.getDistance() <= 0) {
					cp.getPositionWorldOnA(pos1);
					cp.getPositionWorldOnB(pos2);

					relPos1.sub2(pos1, body0.getCenterOfMassPosition(tmpVec));
					relPos2.sub2(pos2, body1.getCenterOfMassPosition(tmpVec));

					// this jacobian entry is re-used for all iterations
					Matrix3 mat1 = body0.getCenterOfMassTransform(Transform()).basis;
					mat1.transpose();

					Matrix3 mat2 = body1.getCenterOfMassTransform(Transform()).basis;
					mat2.transpose();

					JacobianEntry jac = JacobianEntry();
					jac.initAll(mat1, mat2,
							relPos1, relPos2, cp.normalWorldOnB,
							body0.getInvInertiaDiagLocal(Vector3.zero()), body0.getInvMass(),
							body1.getInvInertiaDiagLocal(Vector3.zero()), body1.getInvMass());

					double jacDiagAB = jac.getDiagonal();

					ConstraintPersistentData? cpd = cp.userPersistentData as ConstraintPersistentData?;
					if (cpd != null) {
						// might be invalid
						cpd.persistentLifeTime++;
						if (cpd.persistentLifeTime != cp.getLifeTime()) {
							//printf("Invalid: cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd->m_persistentLifeTime,cp.getLifeTime());
							//(cpd) btConstraintPersistentData;
							cpd.reset();
							cpd.persistentLifeTime = cp.getLifeTime();

						}
						else {
						//printf("Persistent: cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd->m_persistentLifeTime,cp.getLifeTime());
						}
					}
					else {
						// todo: should this be in a pool?
						//void* mem = btAlignedAlloc(sizeof(btConstraintPersistentData),16);
						//cpd = (mem)btConstraintPersistentData;
						cpd = ConstraintPersistentData();
						//assert(cpd != null);

						_totalCpd++;
						//printf("totalCpd = %i Created Ptr %x\n",totalCpd,cpd);
						cp.userPersistentData = cpd;
						cpd.persistentLifeTime = cp.getLifeTime();
					//printf("CREATED: %x . cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd,cpd->m_persistentLifeTime,cp.getLifeTime());
					}
					//assert (cpd != null);

					cpd.jacDiagABInv = 1 / jacDiagAB;

					// Dependent on Rigidbody A and B types, fetch the contact/friction response func
					// perhaps do a similar thing for friction/restutution combiner funcs...

					cpd.frictionSolverFunc = frictionDispatch[body0.frictionSolverType][body1.frictionSolverType];
					cpd.contactSolverFunc = contactDispatch[body0.contactSolverType][body1.contactSolverType];

					body0.getVelocityInLocalPoint(relPos1, vel1);
					body1.getVelocityInLocalPoint(relPos2, vel2);
					vel.sub2(vel1, vel2);

					double relVel;
					relVel = cp.normalWorldOnB.dot(vel);

					double combinedRestitution = cp.combinedRestitution;

					cpd.penetration = cp.getDistance(); ///btScalar(info.m_numIterations);
					cpd.friction = cp.combinedFriction;
					cpd.restitution = _restitutionCurve(relVel, combinedRestitution);
					if (cpd.restitution <= 0) {
						cpd.restitution = 0;
					}

					// restitution and penetration work in same direction so
					// relVel 

					double penVel = -cpd.penetration / info.timeStep;

					if (cpd.restitution > penVel) {
						cpd.penetration = 0;
					}

					double relaxation = info.damping;
					if ((info.solverMode & SolverMode.useWarmStarting) != 0) {
						cpd.appliedImpulse *= relaxation;
					}
					else {
						cpd.appliedImpulse = 0;
					}

					// for friction
					cpd.prevAppliedImpulse = cpd.appliedImpulse;

					// re-calculate friction direction every frame, todo: check if this is really needed
					TransformUtil.planeSpace1(cp.normalWorldOnB, cpd.frictionWorldTangential0, cpd.frictionWorldTangential1);

					//#define NO_FRICTION_WARMSTART 1
					//#ifdef NO_FRICTION_WARMSTART
					cpd.accumulatedTangentImpulse0 = 0;
					cpd.accumulatedTangentImpulse1 = 0;
					//#endif //NO_FRICTION_WARMSTART
					double denom0 = body0.computeImpulseDenominator(pos1, cpd.frictionWorldTangential0);
					double denom1 = body1.computeImpulseDenominator(pos2, cpd.frictionWorldTangential0);
					double denom = relaxation / (denom0 + denom1);
					cpd.jacDiagABInvTangent0 = denom;

					denom0 = body0.computeImpulseDenominator(pos1, cpd.frictionWorldTangential1);
					denom1 = body1.computeImpulseDenominator(pos2, cpd.frictionWorldTangential1);
					denom = relaxation / (denom0 + denom1);
					cpd.jacDiagABInvTangent1 = denom;

					totalImpulse.scaleFrom(cpd.appliedImpulse, cp.normalWorldOnB);

					{
						torqueAxis0.cross2(relPos1, cp.normalWorldOnB);

						cpd.angularComponentA.setFrom(torqueAxis0);
						body0.getInvInertiaTensorWorld(tmpMat3).transform(cpd.angularComponentA);

						torqueAxis1.cross2(relPos2, cp.normalWorldOnB);

						cpd.angularComponentB.setFrom(torqueAxis1);
						body1.getInvInertiaTensorWorld(tmpMat3).transform(cpd.angularComponentB);
					}
					{
						ftorqueAxis0.cross2(relPos1, cpd.frictionWorldTangential0);

						cpd.frictionAngularComponent0A.setFrom(ftorqueAxis0);
						body0.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent0A);
					}
					{
						ftorqueAxis1.cross2(relPos1, cpd.frictionWorldTangential1);

						cpd.frictionAngularComponent1A.setFrom(ftorqueAxis1);
						body0.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent1A);
					}
					{
						ftorqueAxis0.cross2(relPos2, cpd.frictionWorldTangential0);

						cpd.frictionAngularComponent0B.setFrom(ftorqueAxis0);
						body1.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent0B);
					}
					{
						ftorqueAxis1.cross2(relPos2, cpd.frictionWorldTangential1);

						cpd.frictionAngularComponent1B.setFrom(ftorqueAxis1);
						body1.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent1B);
					}

					///

					// apply previous frames impulse on both bodies
					body0.applyImpulse(totalImpulse, relPos1);

					tmpVec.negateFrom(totalImpulse);
					body1.applyImpulse(tmpVec, relPos2);
				}

			}
		}
	}

	double solveCombinedContactFriction(RigidBody body0, RigidBody body1, ManifoldPoint cp, ContactSolverInfo info, int iter, IDebugDraw debugDrawer) {
		double maxImpulse = 0;
    if (cp.getDistance() <= 0) {
      //btConstraintPersistentData* cpd = (btConstraintPersistentData*) cp.m_userPersistentData;
      double impulse = ContactConstraint.resolveSingleCollisionCombined(body0, body1, cp, info);
      if (maxImpulse < impulse) {
        maxImpulse = impulse;
      }
    }
		return maxImpulse;
	}
	
	double solve(RigidBody? body0, RigidBody? body1, ManifoldPoint? cp, ContactSolverInfo info, int iter, IDebugDraw? debugDrawer) {
		double maxImpulse = 0;
  
    if ((cp?.getDistance() ?? 0) <= 0) {{
        ConstraintPersistentData? cpd = cp?.userPersistentData as ConstraintPersistentData?;
        if(cpd != null){
          double impulse = cpd.contactSolverFunc!(body0, body1, cp, info);
          if (maxImpulse < impulse) {
            maxImpulse = impulse;
          }
        }
      }
    }
		
		return maxImpulse;
	}

	double solveFriction(RigidBody? body0, RigidBody? body1, ManifoldPoint? cp, ContactSolverInfo? info, int iter, IDebugDraw? debugDrawer) {
    if ((cp?.getDistance() ?? 0) <= 0) {
      ConstraintPersistentData? cpd = cp?.userPersistentData as ConstraintPersistentData?;
      if(cpd != null){
        cpd.frictionSolverFunc!(body0, body1, cp, info);
      }
    }
		return 0;
	}
	
	@override
	void reset() {
		btSeed2 = 0;
	}
	
	/**
	 * Advanced: Override the default contact solving function for contacts, for certain types of rigidbody<br>
	 * See RigidBody.contactSolverType and RigidBody.frictionSolverType
	 */
	void setContactSolverFunc(ContactSolverFunc func, int type0, int type1) {
		contactDispatch[type0][type1] = func;
	}
	
	/**
	 * Advanced: Override the default friction solving function for contacts, for certain types of rigidbody<br>
	 * See RigidBody.contactSolverType and RigidBody.frictionSolverType
	 */
	void setFrictionSolverFunc(ContactSolverFunc func, int type0, int type1) {
		frictionDispatch[type0][type1] = func;
	}

	void setRandSeed(int seed) {
		btSeed2 = seed;
	}

	int getRandSeed() {
		return btSeed2;
	}
}


////////////////////////////////////////////////////////////////////////////
class _OrderIndex {
  int manifoldIndex = 0;
  int pointIndex = 0;
}