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

import "package:bullet_physics/collision/narrowphase/manifold_point.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * PersistentManifold is a contact point cache, it stays persistent as int as objects
 * are overlapping in the broadphase. Those contact points are created by the collision
 * narrow phase.<p>
 * 
 * The cache can be empty, or hold 1, 2, 3 or 4 points. Some collision algorithms (GJK)
 * might only add one point at a time, updates/refreshes old contact points, and throw
 * them away if necessary (distance becomes too large).<p>
 * 
 * Reduces the cache to 4 points, when more then 4 points are added, using following rules:
 * the contact point with deepest penetration is always kept, and it tries to maximize the
 * area covered by the points.<p>
 * 
 * Note that some pairs of objects might have more then one contact manifold.
 * 
 * @author jezek2
 */
class PersistentManifold {

	//final BulletStack stack = BulletStack.get();
	
	static const int manifoleCacheSize = 4;
	
	final List<ManifoldPoint> _pointCache = [];//ManifoldPoint[MANIFOLD_CACHE_SIZE];
	/// this two body pointers can point to the physics rigidbody class.
	/// void* will allow any rigidbody class
	Object? _body0;
	Object? _body1;
	int _cachedPoints = 0;
	
	int index1a = 0;

	PersistentManifold([Object? body0, Object? body1, int bla = 0]) {
    for (int i=0; i<_pointCache.length; i++){ 
      _pointCache.add(ManifoldPoint());
    }
		init(body0, body1, bla);
	}

	void init(Object? body0, Object? body1, int bla) {
		_body0 = body0;
		_body1 = body1;
		_cachedPoints = 0;
		index1a = 0;
	}

	/// sort cached points so most isolated points come first
	int _sortCachedPoints(ManifoldPoint pt) {
		//calculate 4 possible cases areas, and take biggest area
		//also need to keep 'deepest'

		int maxPenetrationIndex = -1;
//#define KEEP_DEEPEST_POINT 1
//#ifdef KEEP_DEEPEST_POINT
		double maxPenetration = pt.getDistance();
		for (int i = 0; i < 4; i++) {
			if (_pointCache[i].getDistance() < maxPenetration) {
				maxPenetrationIndex = i;
				maxPenetration = _pointCache[i].getDistance();
			}
		}
//#endif //KEEP_DEEPEST_POINT

		double res0 = 0, res1 = 0, res2 = 0, res3 = 0;
		if (maxPenetrationIndex != 0) {
			Vector3 a0 = Vector3.copy(pt.localPointA);
			a0.sub(_pointCache[1].localPointA);

			Vector3 b0 = Vector3.copy(_pointCache[3].localPointA);
			b0.sub(_pointCache[2].localPointA);

			Vector3 cross = Vector3.zero();
			cross.cross2(a0, b0);

			res0 = cross.length2;
		}

		if (maxPenetrationIndex != 1) {
			Vector3 a1 = Vector3.copy(pt.localPointA);
			a1.sub(_pointCache[0].localPointA);

			Vector3 b1 = Vector3.copy(_pointCache[3].localPointA);
			b1.sub(_pointCache[2].localPointA);

			Vector3 cross = Vector3.zero();
			cross.cross2(a1, b1);
			res1 = cross.length2;
		}

		if (maxPenetrationIndex != 2) {
			Vector3 a2 = Vector3.copy(pt.localPointA);
			a2.sub(_pointCache[0].localPointA);

			Vector3 b2 = Vector3.copy(_pointCache[3].localPointA);
			b2.sub(_pointCache[1].localPointA);

			Vector3 cross = Vector3.zero();
			cross.cross2(a2, b2);

			res2 = cross.length2;
		}

		if (maxPenetrationIndex != 3) {
			Vector3 a3 = Vector3.copy(pt.localPointA);
			a3.sub(_pointCache[0].localPointA);

			Vector3 b3 = Vector3.copy(_pointCache[2].localPointA);
			b3.sub(_pointCache[1].localPointA);

			Vector3 cross = Vector3.zero();
			cross.cross2(a3, b3);
			res3 = cross.length2;
		}

		Vector4 maxvec = Vector4.zero();
		maxvec.setValues(res0, res1, res2, res3);
		int biggestarea = VectorUtil.closestAxis4(maxvec);
		return biggestarea;
	}

	//private int findContactPoint(ManifoldPoint unUsed, int numUnused, ManifoldPoint pt);

	Object? getBody0() {
		return _body0;
	}

	Object? getBody1() {
		return _body1;
	}

	void setBodies(Object? body0, Object? body1) {
		_body0 = body0;
		_body1 = body1;
	}
	
	void clearUserCache(ManifoldPoint pt) {
		Object? oldPtr = pt.userPersistentData;
		if (oldPtr != null) {
    //#ifdef DEBUG_PERSISTENCY
    //			int i;
    //			int occurance = 0;
    //			for (i = 0; i < cachedPoints; i++) {
    //				if (pointCache[i].userPersistentData == oldPtr) {
    //					occurance++;
    //					if (occurance > 1) {
    //						throw InternalError();
    //					}
    //				}
    //			}
    //			assert (occurance <= 0);
    //#endif //DEBUG_PERSISTENCY

    if (pt.userPersistentData != null && BulletGlobals.getContactDestroyedCallback() != null) {
      BulletGlobals.getContactDestroyedCallback()?.contactDestroyed(pt.userPersistentData!);
      pt.userPersistentData = null;
    }

    //#ifdef DEBUG_PERSISTENCY
    //			DebugPersistency();
    //#endif
		}
	}

	int getNumContacts() {
		return _cachedPoints;
	}

	ManifoldPoint getContactPoint(int index) {
		return _pointCache[index];
	}

	// todo: get this margin from the current physics / collision environment
	double getContactBreakingThreshold() {
		return BulletGlobals.getContactBreakingThreshold();
	}

	int getCacheEntry(ManifoldPoint newPoint) {
		double intestDist = getContactBreakingThreshold() * getContactBreakingThreshold();
		int size = getNumContacts();
		int nearestPoint = -1;
		Vector3 diffA = Vector3.zero();
		for (int i = 0; i < size; i++) {
			ManifoldPoint mp = _pointCache[i];

			diffA.sub2(mp.localPointA,newPoint.localPointA);

			double distToManiPoint = diffA.dot(diffA);
			if (distToManiPoint < intestDist) {
				intestDist = distToManiPoint;
				nearestPoint = i;
			}
		}
		return nearestPoint;
	}

	int addManifoldPoint(ManifoldPoint newPoint) {
		assert (_validContactDistance(newPoint));

		int insertIndex = getNumContacts();
		if (insertIndex == manifoleCacheSize) {
			//#if MANIFOLD_CACHE_SIZE >= 4
			if (manifoleCacheSize >= 4) {
				//sort cache so best points come first, based on area
				insertIndex = _sortCachedPoints(newPoint);
			}
			else {
				//#else
				insertIndex = 0;
			}
			//#endif
			
			clearUserCache(_pointCache[insertIndex]);
		}
		else {
			_cachedPoints++;
		}
		assert (_pointCache[insertIndex].userPersistentData == null);
		_pointCache[insertIndex].set(newPoint);
		return insertIndex;
	}

	void removeContactPoint(int index) {
		clearUserCache(_pointCache[index]);

		int lastUsedIndex = getNumContacts() - 1;
//		m_pointCache[index] = m_pointCache[lastUsedIndex];
		if (index != lastUsedIndex) {
			// TODO: possible bug
			_pointCache[index].set(_pointCache[lastUsedIndex]);
			//get rid of duplicated userPersistentData pointer
			_pointCache[lastUsedIndex].userPersistentData = null;
			_pointCache[lastUsedIndex].appliedImpulse = 0;
			_pointCache[lastUsedIndex].lateralFrictionInitialized = false;
			_pointCache[lastUsedIndex].appliedImpulseLateral1 = 0;
			_pointCache[lastUsedIndex].appliedImpulseLateral2 = 0;
			_pointCache[lastUsedIndex].lifeTime = 0;
		}

		assert (_pointCache[lastUsedIndex].userPersistentData == null);
		_cachedPoints--;
	}

	void replaceContactPoint(ManifoldPoint newPoint, int insertIndex) {
		assert (_validContactDistance(newPoint));

//#define MAINTAIN_PERSISTENCY 1
//#ifdef MAINTAIN_PERSISTENCY
		int lifeTime = _pointCache[insertIndex].getLifeTime();
		double appliedImpulse = _pointCache[insertIndex].appliedImpulse;
		double appliedLateralImpulse1 = _pointCache[insertIndex].appliedImpulseLateral1;
		double appliedLateralImpulse2 = _pointCache[insertIndex].appliedImpulseLateral2;

		assert (lifeTime >= 0);
		Object? cache = _pointCache[insertIndex].userPersistentData;

		_pointCache[insertIndex].set(newPoint);

		_pointCache[insertIndex].userPersistentData = cache;
		_pointCache[insertIndex].appliedImpulse = appliedImpulse;
		_pointCache[insertIndex].appliedImpulseLateral1 = appliedLateralImpulse1;
		_pointCache[insertIndex].appliedImpulseLateral2 = appliedLateralImpulse2;

		_pointCache[insertIndex].lifeTime = lifeTime;
//#else
//		clearUserCache(m_pointCache[insertIndex]);
//		m_pointCache[insertIndex] = newPoint;
//#endif
	}

	bool _validContactDistance(ManifoldPoint pt) {
		return pt.distance1 <= getContactBreakingThreshold();
	}

	/// calculated worldspace coordinates and depth, and reject points that exceed the collision margin
	void refreshContactPoints(Transform trA, Transform trB) {
		Vector3 tmp = Vector3.zero();
		int i;
//#ifdef DEBUG_PERSISTENCY
//	printf("refreshContactPoints posA = (%f,%f,%f) posB = (%f,%f,%f)\n",
//		trA.getOrigin().getX(),
//		trA.getOrigin().getY(),
//		trA.getOrigin().getZ(),
//		trB.getOrigin().getX(),
//		trB.getOrigin().getY(),
//		trB.getOrigin().getZ());
//#endif //DEBUG_PERSISTENCY
		// first refresh worldspace positions and distance
		for (i = getNumContacts() - 1; i >= 0; i--) {
			ManifoldPoint manifoldPoint = _pointCache[i];

			manifoldPoint.positionWorldOnA.setFrom(manifoldPoint.localPointA);
			trA.transform(manifoldPoint.positionWorldOnA);

			manifoldPoint.positionWorldOnB.setFrom(manifoldPoint.localPointB);
			trB.transform(manifoldPoint.positionWorldOnB);

			tmp.setFrom(manifoldPoint.positionWorldOnA);
			tmp.sub(manifoldPoint.positionWorldOnB);
			manifoldPoint.distance1 = tmp.dot(manifoldPoint.normalWorldOnB);

			manifoldPoint.lifeTime++;
		}

		// then 
		double distance2d;
		Vector3 projectedDifference = Vector3.zero(), projectedPoint = Vector3.zero();

		for (i = getNumContacts() - 1; i >= 0; i--) {

			ManifoldPoint manifoldPoint = _pointCache[i];
			// contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
			if (!_validContactDistance(manifoldPoint)) {
				removeContactPoint(i);
			}
			else {
				// contact also becomes invalid when relative movement orthogonal to normal exceeds margin
				tmp.scaleFrom(manifoldPoint.distance1, manifoldPoint.normalWorldOnB);
				projectedPoint.sub2(manifoldPoint.positionWorldOnA,tmp);
				projectedDifference.sub2(manifoldPoint.positionWorldOnB,projectedPoint);
				distance2d = projectedDifference.dot(projectedDifference);
				if (distance2d > getContactBreakingThreshold() * getContactBreakingThreshold()) {
					removeContactPoint(i);
				}
				else {
					// contact point processed callback
					if (BulletGlobals.getContactProcessedCallback() != null) {
						BulletGlobals.getContactProcessedCallback()?.contactProcessed(manifoldPoint, _body0, _body1);
					}
				}
			}
		}
//#ifdef DEBUG_PERSISTENCY
//	DebugPersistency();
//#endif //
	}

	void clearManifold() {
		int i;
		for (i = 0; i < _cachedPoints; i++) {
			clearUserCache(_pointCache[i]);
		}
		_cachedPoints = 0;
	}
}
