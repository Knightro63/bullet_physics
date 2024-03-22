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

import 'package:bullet_physics/collision/broadphase/broadphase_native_type.dart';
import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:bullet_physics/linearmath/transform.dart';
import 'package:vector_math/vector_math.dart';


/**
 * CollisionShape class provides an interface for collision shapes that can be
 * shared among {@link CollisionObject}s.
 * 
 * @author jezek2
 */
abstract class CollisionShape {

	//final BulletStack stack = BulletStack.get();

	Object? userPointer;
	
	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax);

	void getBoundingSphere(Vector3 center, List<double> radius) {
		Vector3 tmp = Vector3.zero();

		Transform tr = Transform();
		tr.setIdentity();
		Vector3 aabbMin = Vector3.zero(), aabbMax = Vector3.zero();

		getAabb(tr, aabbMin, aabbMax);

		tmp.sub2(aabbMax,aabbMin);
		radius[0] = tmp.length * 0.5;

		tmp.add2(aabbMin,aabbMax);
		center.scaleFrom(0.5, tmp);
	}

	///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
	double getAngularMotionDisc() {
		Vector3 center = Vector3.zero();
		List<double> disc = []; // TODO: stack
		getBoundingSphere(center, disc);
		disc[0] += center.length;
		return disc[0];
	}

	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	void calculateTemporalAabb(Transform curTrans, Vector3 linvel, Vector3 angvel, double timeStep, Vector3 temporalAabbMin, Vector3 temporalAabbMax) {
		//start with static aabb
		getAabb(curTrans, temporalAabbMin, temporalAabbMax);

		double temporalAabbMaxx = temporalAabbMax.x;
		double temporalAabbMaxy = temporalAabbMax.y;
		double temporalAabbMaxz = temporalAabbMax.z;
		double temporalAabbMinx = temporalAabbMin.x;
		double temporalAabbMiny = temporalAabbMin.y;
		double temporalAabbMinz = temporalAabbMin.z;

		// add linear motion
		Vector3 linMotion = Vector3.copy(linvel);
		linMotion.scale(timeStep);

		//todo: simd would have a vector max/min operation, instead of per-element access
		if (linMotion.x > 0) {
			temporalAabbMaxx += linMotion.x;
		}
		else {
			temporalAabbMinx += linMotion.x;
		}
		if (linMotion.y > 0) {
			temporalAabbMaxy += linMotion.y;
		}
		else {
			temporalAabbMiny += linMotion.y;
		}
		if (linMotion.z > 0) {
			temporalAabbMaxz += linMotion.z;
		}
		else {
			temporalAabbMinz += linMotion.z;
		}

		//add conservative angular motion
		double angularMotion = angvel.length * getAngularMotionDisc() * timeStep;
		Vector3 angularMotion3d = Vector3.zero();
		angularMotion3d.setValues(angularMotion, angularMotion, angularMotion);
		temporalAabbMin.setValues(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
		temporalAabbMax.setValues(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);

		temporalAabbMin.sub(angularMotion3d);
		temporalAabbMax.add(angularMotion3d);
	}

//#ifndef __SPU__
	bool isPolyhedral() {
		return getShapeType().isPolyhedral();
	}

	bool isConvex() {
		return getShapeType().isConvex();
	}

	bool isConcave() {
		return getShapeType().isConcave();
	}

	bool isCompound() {
		return getShapeType().isCompound();
	}

	///isInfinite is used to catch simulation error (aabb check)
	bool isInfinite() {
		return getShapeType().isInfinite();
	}

	BroadphaseNativeType getShapeType();

	void setLocalScaling(Vector3 scaling);
	
	// TODO: returns const
	Vector3 getLocalScaling(Vector3 out);

	void calculateLocalInertia(double mass, Vector3 inertia);


  //debugging support
	String getName();
  //#endif //__SPU__
	void setMargin(double margin);

	double getMargin();
	
	// optional user data pointer
	void setUserPointer(Object userPtr) {
		userPointer = userPtr;
	}

	Object? getUserPointer() {
		return userPointer;
	}
	
}
