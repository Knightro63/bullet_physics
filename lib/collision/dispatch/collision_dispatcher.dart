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



import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/overlap_callback.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart";
import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_configuration.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/default_near_callback.dart";
import "package:bullet_physics/collision/dispatch/near_callback.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/utils/collections.dart";
import "package:bullet_physics/utils/object_array_list.dart";

/**
 * CollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
 * Time of Impact, Closest Points and Penetration Depth.
 * 
 * @author jezek2
 */
class CollisionDispatcher extends Dispatcher {

	static final int _maxBroadphaseCollisionTypes = BroadphaseNativeType.maxBroadphaseCollisionTypes.index;
	//int _count = 0;
	final ObjectArrayList<PersistentManifold> _manifoldsPtr = ObjectArrayList();
	//bool _useIslands = true;
	bool _staticWarningReported = false;
	//ManifoldResult? _defaultManifoldResult;
	NearCallback? _nearCallback;
	//private PoolAllocator*	m_collisionAlgorithmPoolAllocator;
	//private PoolAllocator*	m_persistentManifoldPoolAllocator;
	final List<List<CollisionAlgorithmCreateFunc?>> _doubleDispatch = List.filled(_maxBroadphaseCollisionTypes, List.filled(_maxBroadphaseCollisionTypes, null));//CollisionAlgorithmCreateFunc[_maxBroadphaseCollisionTypes][_maxBroadphaseCollisionTypes];
	late CollisionConfiguration _collisionConfiguration;
	//private static int gNumManifold = 0;
	
	CollisionAlgorithmConstructionInfo _tmpCI = CollisionAlgorithmConstructionInfo();

	CollisionDispatcher(CollisionConfiguration collisionConfiguration) {
		_collisionConfiguration = collisionConfiguration;

		setNearCallback(DefaultNearCallback());

		//m_collisionAlgorithmPoolAllocator = collisionConfiguration->getCollisionAlgorithmPool();
		//m_persistentManifoldPoolAllocator = collisionConfiguration->getPersistentManifoldPool();

		for (int i = 0; i < _maxBroadphaseCollisionTypes; i++) {
			for (int j = 0; j < _maxBroadphaseCollisionTypes; j++) {
				_doubleDispatch[i][j] = collisionConfiguration.getCollisionAlgorithmCreateFunc(
					BroadphaseNativeType.forValue(i),
					BroadphaseNativeType.forValue(j)
				);
				assert(_doubleDispatch[i][j] != null);
			}
		}
	}

	void registerCollisionCreateFunc(int proxyType0, int proxyType1, CollisionAlgorithmCreateFunc createFunc) {
		_doubleDispatch[proxyType0][proxyType1] = createFunc;
	}

	NearCallback? getNearCallback() {
		return _nearCallback;
	}

	void setNearCallback(NearCallback nearCallback) {
		_nearCallback = nearCallback;
	}

	CollisionConfiguration getCollisionConfiguration() {
		return _collisionConfiguration;
	}

	void setCollisionConfiguration(CollisionConfiguration collisionConfiguration) {
		_collisionConfiguration = collisionConfiguration;
	}

	@override
	CollisionAlgorithm? findAlgorithm([CollisionObject? body0, CollisionObject? body1, PersistentManifold? sharedManifold]) {
		CollisionAlgorithmConstructionInfo ci = _tmpCI;
		ci.dispatcher1 = this;
		ci.manifold = sharedManifold;
		CollisionAlgorithmCreateFunc? createFunc = body0 == null || body1 == null?null:_doubleDispatch[body0.getCollisionShape()?.getShapeType().index ?? 0][body1.getCollisionShape()?.getShapeType().index ?? 0];
		CollisionAlgorithm? algo = createFunc?.createCollisionAlgorithm(ci, body0, body1);
		algo?.internalSetCreateFunc(createFunc);

		return algo;
	}

	@override
	void freeCollisionAlgorithm(CollisionAlgorithm? algo) {
    assert(algo != null);
		CollisionAlgorithmCreateFunc? createFunc = algo!.internalGetCreateFunc();
		algo.internalSetCreateFunc(null);
		createFunc?.releaseCollisionAlgorithm(algo);
		algo.destroy();
	}

	@override
	PersistentManifold getNewManifold(Object? b0, Object? b1) {
		//gNumManifold++;

		//btAssert(gNumManifold < 65535);

		CollisionObject body0 = b0 as CollisionObject;
		CollisionObject body1 = b1 as CollisionObject;

		/*
		void* mem = 0;

		if (m_persistentManifoldPoolAllocator->getFreeCount())
		{
			mem = m_persistentManifoldPoolAllocator->allocate(sizeof(btPersistentManifold));
		} else
		{
			mem = btAlignedAlloc(sizeof(btPersistentManifold),16);

		}
		btPersistentManifold* manifold = new(mem) btPersistentManifold (body0,body1,0);
		manifold->m_index1a = m_manifoldsPtr.length;
		m_manifoldsPtr.push_back(manifold);
		*/
		
		PersistentManifold manifold = PersistentManifold();
		manifold.init(body0,body1,0);
		
		manifold.index1a = _manifoldsPtr.size;
		_manifoldsPtr.add(manifold);

		return manifold;
	}

	@override
	void releaseManifold(PersistentManifold? manifold) {
		//gNumManifold--;

		//printf("releaseManifold: gNumManifold %d\n",gNumManifold);
		clearManifold(manifold);

		// TODO: optimize
		int findIndex = manifold?.index1a ?? 0;
		assert (findIndex < _manifoldsPtr.size);
		Collections.swapObjectArray(_manifoldsPtr, findIndex, _manifoldsPtr.size-1);
		_manifoldsPtr.getQuick(findIndex)?.index1a = findIndex;
		_manifoldsPtr.removeAt(_manifoldsPtr.size-1);

	}

	@override
	void clearManifold(PersistentManifold? manifold) {
		manifold?.clearManifold();
	}

	@override
	bool needsCollision(CollisionObject? body0, CollisionObject? body1) {
		assert (body0 != null);
		assert (body1 != null);

		bool needsCollision = true;

		//#ifdef BT_DEBUG
		if (!_staticWarningReported) {
			// broadphase filtering already deals with this
			if ((body0!.isStaticObject() || body0.isKinematicObject()) &&
					(body1!.isStaticObject() || body1.isKinematicObject())) {
				_staticWarningReported = true;
				print("warning CollisionDispatcher.needsCollision: static-static collision!");
			}
		}
		//#endif //BT_DEBUG

		if ((!body0!.isActive()) && (!body1!.isActive())) {
			needsCollision = false;
		}
		else if (!body0.checkCollideWith(body1!)) {
			needsCollision = false;
		}

		return needsCollision;
	}

	@override
	bool needsResponse(CollisionObject body0, CollisionObject body1) {
		//here you can do filtering
		bool hasResponse = (body0.hasContactResponse() && body1.hasContactResponse());
		//no response between two static/kinematic bodies:
		hasResponse = hasResponse && ((!body0.isStaticOrKinematicObject()) || (!body1.isStaticOrKinematicObject()));
		return hasResponse;
	}
	
	_CollisionPairCallback _collisionPairCallback = _CollisionPairCallback();
	
	@override
	void dispatchAllCollisionPairs(OverlappingPairCache? pairCache, DispatcherInfo dispatchInfo, Dispatcher? dispatcher) {
		//m_blockedForChanges = true;
		_collisionPairCallback.init(dispatchInfo, this);
		pairCache?.processAllOverlappingPairs(_collisionPairCallback, dispatcher);
		//m_blockedForChanges = false;
	}

	@override
	int getNumManifolds() {
		return _manifoldsPtr.size;
	}

	@override
	PersistentManifold? getManifoldByIndexInternal(int index) {
		return _manifoldsPtr.getQuick(index);
	}

	@override
	ObjectArrayList<PersistentManifold> getInternalManifoldPointer() {
		return _manifoldsPtr;
	}
	
}

class _CollisionPairCallback extends OverlapCallback {
  late DispatcherInfo _dispatchInfo;
  late CollisionDispatcher _dispatcher;

  void init(DispatcherInfo dispatchInfo, CollisionDispatcher dispatcher) {
    _dispatchInfo = dispatchInfo;
    _dispatcher = dispatcher;
  }
  @override
  bool processOverlap(BroadphasePair pair) {
    _dispatcher.getNearCallback()?.handleCollision(pair, _dispatcher, _dispatchInfo);
    return false;
  }
}
