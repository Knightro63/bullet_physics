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
import "package:bullet_physics/collision/narrowphase/manifold_point.dart";
import "package:bullet_physics/dynamics/constraintsolver/constraint_persistent_data.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart";
import "package:bullet_physics/dynamics/constraintsolver/jacobian_entry.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * Functions for resolving contacts.
 * 
 * @author jezek2
 */
class ContactConstraint {
	/**
	 * Bilateral constraint between two dynamic objects.
	 */
	static void resolveSingleBilateral(RigidBody body1, Vector3 pos1,
			RigidBody body2, Vector3 pos2,
			double distance, Vector3 normal, double impulse, double timeStep) {
		double normalLenSqr = normal.length2;
		assert (normalLenSqr.abs() < 1.1);
		if (normalLenSqr > 1.1) {
			impulse = 0;
			return;
		}

		Vector3 tmp = Vector3.zero();
		
		Vector3 relPos1 = Vector3.zero();
		relPos1.sub2(pos1, body1.getCenterOfMassPosition(tmp));

		Vector3 relPos2 = Vector3.zero();
		relPos2.sub2(pos2, body2.getCenterOfMassPosition(tmp));

		//this jacobian entry could be re-used for all iterations

		Vector3 vel1 = Vector3.zero();
		body1.getVelocityInLocalPoint(relPos1, vel1);

		Vector3 vel2 = Vector3.zero();
		body2.getVelocityInLocalPoint(relPos2, vel2);

		Vector3 vel = Vector3.zero();
		vel.sub2(vel1, vel2);

		Matrix3 mat1 = body1.getCenterOfMassTransform(Transform()).basis;
		mat1.transpose();

		Matrix3 mat2 = body2.getCenterOfMassTransform(Transform()).basis;
		mat2.transpose();

		JacobianEntry jac = JacobianEntry();
		jac.initAll(
      mat1, 
      mat2,
			relPos1, 
      relPos2, 
      normal,
			body1.getInvInertiaDiagLocal(Vector3.zero()), 
      body1.getInvMass(),
			body2.getInvInertiaDiagLocal(Vector3.zero()), 
      body2.getInvMass()
    );

		double jacDiagAB = jac.getDiagonal();
		double jacDiagABInv = 1 / jacDiagAB;

		Vector3 tmp1 = body1.getAngularVelocity(Vector3.zero());
		mat1.transform(tmp1);

		Vector3 tmp2 = body2.getAngularVelocity(Vector3.zero());
		mat2.transform(tmp2);

		double relVel = jac.getRelativeVelocity(
				body1.getLinearVelocity(Vector3.zero()),
				tmp1,
				body2.getLinearVelocity(Vector3.zero()),
				tmp2);

		//double a = jacDiagABInv;
		relVel = normal.dot(vel);

		// todo: move this into proper structure
		double contactDamping = 0.2;
		double velocityImpulse = -contactDamping * relVel * jacDiagABInv;
		impulse = velocityImpulse;
	}

	/**
	 * Response between two dynamic objects with friction.
	 */
	static double resolveSingleCollision(
    RigidBody? body1,
    RigidBody? body2,
    ManifoldPoint? contactPoint,
    ContactSolverInfo? solverInfo,
  ){
		Vector3 tmpVec = Vector3.zero();

		Vector3 pos1_ = contactPoint?.getPositionWorldOnA(Vector3.zero()) ?? Vector3.zero();
		Vector3 pos2_ = contactPoint?.getPositionWorldOnB(Vector3.zero()) ?? Vector3.zero();
		Vector3 normal = contactPoint?.normalWorldOnB ?? Vector3.zero();

		// constant over all iterations
		Vector3 relPos1 = Vector3.zero();
		relPos1.sub2(pos1_, body1?.getCenterOfMassPosition(tmpVec));

		Vector3 relPos2 = Vector3.zero();
		relPos2.sub2(pos2_, body2?.getCenterOfMassPosition(tmpVec));

		Vector3 vel1 = body1?.getVelocityInLocalPoint(relPos1, Vector3.zero()) ?? Vector3.zero();
		Vector3 vel2 = body2?.getVelocityInLocalPoint(relPos2, Vector3.zero()) ?? Vector3.zero();
		Vector3 vel = Vector3.zero();
		vel.sub2(vel1, vel2);

		double relVel;
		relVel = normal.dot(vel);

		double kFps = solverInfo == null?0:1 / solverInfo.timeStep;

		// btScalar damping = solverInfo.m_damping ;
		double kErp = solverInfo?.erp ?? 0;
		double kCor = kErp * kFps;

		ConstraintPersistentData? cpd = contactPoint?.userPersistentData as ConstraintPersistentData?;
		assert (cpd != null);
		double distance = cpd!.penetration;
		double positionalError = kCor * -distance;
		double velocityError = cpd.restitution - relVel; // * damping;

		double penetrationImpulse = positionalError * cpd.jacDiagABInv;

		double velocityImpulse = velocityError * cpd.jacDiagABInv;

		double normalImpulse = penetrationImpulse + velocityImpulse;

		// See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
		double oldNormalImpulse = cpd.appliedImpulse;
		double sum = oldNormalImpulse + normalImpulse;
		cpd.appliedImpulse = 0 > sum ? 0 : sum;

		normalImpulse = cpd.appliedImpulse - oldNormalImpulse;

		Vector3 tmp = Vector3.zero();
		if ((body1?.getInvMass() ?? 0) != 0) {
			tmp.scaleFrom(body1!.getInvMass(), contactPoint!.normalWorldOnB);
			body1.internalApplyImpulse(tmp, cpd.angularComponentA, normalImpulse);
		}
		if ((body2?.getInvMass() ?? 0) != 0) {
			tmp.scaleFrom(body2!.getInvMass(), contactPoint!.normalWorldOnB);
			body2.internalApplyImpulse(tmp, cpd.angularComponentB, -normalImpulse);
		}

		return normalImpulse;
	}
	
	static double resolveSingleFriction(
    RigidBody? body1,
    RigidBody? body2,
    ManifoldPoint? contactPoint,
    ContactSolverInfo? solverInfo
  ) {
		
		Vector3 tmpVec = Vector3.zero();
		
		Vector3 pos1 = contactPoint?.getPositionWorldOnA(Vector3.zero()) ?? Vector3.zero();
		Vector3 pos2 = contactPoint?.getPositionWorldOnB(Vector3.zero()) ?? Vector3.zero();

		Vector3 relPos1 = Vector3.zero();
		relPos1.sub2(pos1, body1?.getCenterOfMassPosition(tmpVec));

		Vector3 relPos2 = Vector3.zero();
		relPos2.sub2(pos2, body2?.getCenterOfMassPosition(tmpVec));

		ConstraintPersistentData? cpd = contactPoint?.userPersistentData as ConstraintPersistentData?;
		assert (cpd != null);

		double combinedFriction = cpd!.friction;

		double limit = cpd.appliedImpulse * combinedFriction;

		if (cpd.appliedImpulse > 0){
			//apply friction in the 2 tangential directions

			// 1st tangent
			Vector3 vel1 = Vector3.zero();
			body1?.getVelocityInLocalPoint(relPos1, vel1);

			Vector3 vel2 = Vector3.zero();
			body2?.getVelocityInLocalPoint(relPos2, vel2);

			Vector3 vel = Vector3.zero();
			vel.sub2(vel1, vel2);

			double j1, j2;

			{
				double vrel = cpd.frictionWorldTangential0.dot(vel);

				// calculate j that moves us to zero relative velocity
				j1 = -vrel * cpd.jacDiagABInvTangent0;
				double oldTangentImpulse = cpd.accumulatedTangentImpulse0;
				cpd.accumulatedTangentImpulse0 = oldTangentImpulse + j1;

				cpd.accumulatedTangentImpulse0 = min(cpd.accumulatedTangentImpulse0, limit);
				cpd.accumulatedTangentImpulse0 = max(cpd.accumulatedTangentImpulse0, -limit);
				j1 = cpd.accumulatedTangentImpulse0 - oldTangentImpulse;
			}
			{
				// 2nd tangent

				double vrel = cpd.frictionWorldTangential1.dot(vel);

				// calculate j that moves us to zero relative velocity
				j2 = -vrel * cpd.jacDiagABInvTangent1;
				double oldTangentImpulse = cpd.accumulatedTangentImpulse1;
				cpd.accumulatedTangentImpulse1 = oldTangentImpulse + j2;

				cpd.accumulatedTangentImpulse1 = min(cpd.accumulatedTangentImpulse1, limit);
				cpd.accumulatedTangentImpulse1 = max(cpd.accumulatedTangentImpulse1, -limit);
				j2 = cpd.accumulatedTangentImpulse1 - oldTangentImpulse;
			}

			//#ifdef USE_INTERNAL_APPLY_IMPULSE
			Vector3 tmp = Vector3.zero();

			if ((body1?.getInvMass() ?? 0) != 0) {
				tmp.scaleFrom(body1!.getInvMass(), cpd.frictionWorldTangential0);
				body1.internalApplyImpulse(tmp, cpd.frictionAngularComponent0A, j1);

				tmp.scaleFrom(body1.getInvMass(), cpd.frictionWorldTangential1);
				body1.internalApplyImpulse(tmp, cpd.frictionAngularComponent1A, j2);
			}
			if ((body2?.getInvMass() ?? 0) != 0) {
				tmp.scaleFrom(body2!.getInvMass(), cpd.frictionWorldTangential0);
				body2.internalApplyImpulse(tmp, cpd.frictionAngularComponent0B, -j1);

				tmp.scaleFrom(body2.getInvMass(), cpd.frictionWorldTangential1);
				body2.internalApplyImpulse(tmp, cpd.frictionAngularComponent1B, -j2);
			}
			//#else //USE_INTERNAL_APPLY_IMPULSE
			//	body1.applyImpulse((j1 * cpd->m_frictionWorldTangential0)+(j2 * cpd->m_frictionWorldTangential1), relPos1);
			//	body2.applyImpulse((j1 * -cpd->m_frictionWorldTangential0)+(j2 * -cpd->m_frictionWorldTangential1), relPos2);
			//#endif //USE_INTERNAL_APPLY_IMPULSE
		}
		return cpd.appliedImpulse;
	}
	
	/**
	 * velocity + friction<br>
	 * response between two dynamic objects with friction
	 */
	static double resolveSingleCollisionCombined(
    RigidBody? body1,
    RigidBody? body2,
    ManifoldPoint? contactPoint,
    ContactSolverInfo? solverInfo
  ) {
		
		Vector3 tmpVec = Vector3.zero();
		
		Vector3 pos1 = contactPoint?.getPositionWorldOnA(Vector3.zero()) ?? Vector3.zero();
		Vector3 pos2 = contactPoint?.getPositionWorldOnB(Vector3.zero()) ?? Vector3.zero();
		Vector3 normal = contactPoint?.normalWorldOnB ?? Vector3.zero();

		Vector3 relPos1 = Vector3.zero();
		relPos1.sub2(pos1, body1?.getCenterOfMassPosition(tmpVec));

		Vector3 relPos2 = Vector3.zero();
		relPos2.sub2(pos2, body2?.getCenterOfMassPosition(tmpVec));

		Vector3 vel1 = body1?.getVelocityInLocalPoint(relPos1, Vector3.zero()) ?? Vector3.zero();
		Vector3 vel2 = body2?.getVelocityInLocalPoint(relPos2, Vector3.zero()) ?? Vector3.zero();
		Vector3 vel = Vector3.zero();
		vel.sub2(vel1, vel2);

		double relVel;
		relVel = normal.dot(vel);

		double kFps = solverInfo == null?0:(1 / solverInfo.timeStep);

		//btScalar damping = solverInfo.m_damping ;
		double kErp = solverInfo?.erp ?? 0;
		double kCor = kErp * kFps;

		ConstraintPersistentData? cpd = contactPoint?.userPersistentData as ConstraintPersistentData?;
		assert (cpd != null);
		double distance = cpd!.penetration;
		double positionalError = kCor * -distance;
		double velocityError = cpd.restitution - relVel;// * damping;

		double penetrationImpulse = positionalError * cpd.jacDiagABInv;

		double velocityImpulse = velocityError * cpd.jacDiagABInv;

		double normalImpulse = penetrationImpulse + velocityImpulse;

		// See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
		double oldNormalImpulse = cpd.appliedImpulse;
		double sum = oldNormalImpulse + normalImpulse;
		cpd.appliedImpulse = 0 > sum ? 0 : sum;

		normalImpulse = cpd.appliedImpulse - oldNormalImpulse;


		//#ifdef USE_INTERNAL_APPLY_IMPULSE
		Vector3 tmp = Vector3.zero();
		if ((body1?.getInvMass() ?? 0) != 0) {
			tmp.scaleFrom(body1!.getInvMass(), contactPoint?.normalWorldOnB);
			body1.internalApplyImpulse(tmp, cpd.angularComponentA, normalImpulse);
		}
		if ((body2?.getInvMass() ?? 0) != 0) {
			tmp.scaleFrom(body2!.getInvMass(), contactPoint?.normalWorldOnB);
			body2.internalApplyImpulse(tmp, cpd.angularComponentB, -normalImpulse);
		}
		//#else //USE_INTERNAL_APPLY_IMPULSE
		//	body1.applyImpulse(normal*(normalImpulse), relPos1);
		//	body2.applyImpulse(-normal*(normalImpulse), relPos2);
		//#endif //USE_INTERNAL_APPLY_IMPULSE

		{
			//friction
			body1?.getVelocityInLocalPoint(relPos1, vel1);
			body2?.getVelocityInLocalPoint(relPos2, vel2);
			vel.sub2(vel1, vel2);

			relVel = normal.dot(vel);

			tmp.scaleFrom(relVel, normal);
			Vector3 latVel = Vector3.zero();
			latVel.sub2(vel, tmp);
			double latRelVel = latVel.length;

			double combinedFriction = cpd.friction;

			if (cpd.appliedImpulse > 0) {
				if (latRelVel > BulletGlobals.fltEpsilon) {
					latVel.scale(1 / latRelVel);

					Vector3 temp1 = Vector3.zero();
					temp1.cross2(relPos1, latVel);
					body1?.getInvInertiaTensorWorld(Matrix3.zero()).transform(temp1);

					Vector3 temp2 = Vector3.zero();
					temp2.cross2(relPos2, latVel);
					body2?.getInvInertiaTensorWorld(Matrix3.zero()).transform(temp2);

					Vector3 javaTmp1 = Vector3.zero();
					javaTmp1.cross2(temp1, relPos1);

					Vector3 javaTmp2 = Vector3.zero();
					javaTmp2.cross2(temp2, relPos2);

					tmp.add2(javaTmp1, javaTmp2);

					double frictionImpulse = latRelVel / ((body1?.getInvMass() ?? 0) + (body2?.getInvMass() ?? 0) + latVel.dot(tmp));
					double normalImpulse = cpd.appliedImpulse * combinedFriction;

					frictionImpulse = min(frictionImpulse, normalImpulse);
					frictionImpulse = max(frictionImpulse, -normalImpulse);

					tmp.scaleFrom(-frictionImpulse, latVel);
					body1?.applyImpulse(tmp, relPos1);

					tmp.scaleFrom(frictionImpulse, latVel);
					body2?.applyImpulse(tmp, relPos2);
				}
			}
		}

		return normalImpulse;
	}

	static double resolveSingleFrictionEmpty(
			RigidBody body1,
			RigidBody body2,
			ManifoldPoint contactPoint,
			ContactSolverInfo solverInfo) {
		return 0;
	}
	
}
