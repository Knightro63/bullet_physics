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

import 'package:vector_math/vector_math.dart';

class ManifoldPoint {
	final Vector3 localPointA = Vector3.zero();
	final Vector3 localPointB = Vector3.zero();
	final Vector3 positionWorldOnB = Vector3.zero();
	///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
	final Vector3 positionWorldOnA = Vector3.zero();
	final Vector3 normalWorldOnB = Vector3.zero();
	
	late double distance1;
	double combinedFriction = 0;
	double combinedRestitution = 0;
	
	// BP mod, store contact triangles.
	int? partId0;
	int? partId1;
	int? index0;
	int? index1;
	
	Object? userPersistentData;
	double appliedImpulse = 0;
	
	bool lateralFrictionInitialized = false;
	double appliedImpulseLateral1 = 0;
	double appliedImpulseLateral2 = 0;
	int lifeTime = 0; //lifetime of the contactpoint in frames

	final Vector3 lateralFrictionDir1 = Vector3.zero();
	final Vector3 lateralFrictionDir2 = Vector3.zero();
	
	ManifoldPoint([Vector3? pointA, Vector3? pointB, Vector3? normal, double? distance]) {
		init(pointA, pointB, normal, distance);
	}

	void init([Vector3? pointA, Vector3? pointB, Vector3? normal, double? distance]) {
        
    if(pointA != null){
		  localPointA.setFrom(pointA);
    }
    if(pointB != null){
		  localPointB.setFrom(pointB);
    }
    if(normal != null){
		  normalWorldOnB.setFrom(normal);
    }
		distance1 = distance ?? 0;
	}

	double getDistance() {
		return distance1;
	}

	int getLifeTime() {
		return lifeTime;
	}
	
	void set(ManifoldPoint p) {
		localPointA.setFrom(p.localPointA);
		localPointB.setFrom(p.localPointB);
		positionWorldOnA.setFrom(p.positionWorldOnA);
		positionWorldOnB.setFrom(p.positionWorldOnB);
		normalWorldOnB.setFrom(p.normalWorldOnB);
		distance1 = p.distance1;
		combinedFriction = p.combinedFriction;
		combinedRestitution = p.combinedRestitution;
		partId0 = p.partId0;
		partId1 = p.partId1;
		index0 = p.index0;
		index1 = p.index1;
		userPersistentData = p.userPersistentData;
		appliedImpulse = p.appliedImpulse;
		lateralFrictionInitialized = p.lateralFrictionInitialized;
		appliedImpulseLateral1 = p.appliedImpulseLateral1;
		appliedImpulseLateral2 = p.appliedImpulseLateral2;
		lifeTime = p.lifeTime;
		lateralFrictionDir1.setFrom(p.lateralFrictionDir1);
		lateralFrictionDir2.setFrom(p.lateralFrictionDir2);
	}
	
	Vector3 getPositionWorldOnA(Vector3 out) {
		out.setFrom(positionWorldOnA);
		return out;
		//return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
	}

	Vector3 getPositionWorldOnB(Vector3 out) {
		out.setFrom(positionWorldOnB);
		return out;
	}

	void setDistance(double dist) {
		distance1 = dist;
	}
}
