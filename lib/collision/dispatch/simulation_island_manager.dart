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

import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/collision/dispatch/union_find.dart";
import "package:bullet_physics/core/bullet_stats.dart";
import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";

/**
 * SimulationIslandManager creates and handles simulation islands, using {@link UnionFind}.
 * 
 * @author jezek2
 */
class SimulationIslandManager {

	final UnionFind _unionFind = UnionFind();

	final ObjectArrayList<PersistentManifold> _islandmanifold = ObjectArrayList();
	final ObjectArrayList<CollisionObject> _islandBodies = ObjectArrayList();
	
	void initUnionFind(int n) {
		_unionFind.reset(n);
	}
	
	UnionFind getUnionFind() {
		return _unionFind;
	}
	
	void findUnions(Dispatcher? dispatcher, CollisionWorld colWorld) {
		ObjectArrayList<BroadphasePair?>? pairPtr = colWorld.getPairCache()?.getOverlappingPairArray();
		for (int i=0; i<(pairPtr?.size ?? 0); i++) {
			BroadphasePair? collisionPair = pairPtr?.getQuick(i);
			
			CollisionObject? colObj0 = collisionPair?.pProxy0?.clientObject as CollisionObject?;
			CollisionObject? colObj1 = collisionPair?.pProxy1?.clientObject  as CollisionObject?;

			if (((colObj0 != null) && ((colObj0).mergesSimulationIslands())) &&
					((colObj1 != null) && ((colObj1).mergesSimulationIslands()))) {
				_unionFind.unite((colObj0).getIslandTag(), (colObj1).getIslandTag());
			}
		}
	}

	void updateActivationState(CollisionWorld colWorld, Dispatcher? dispatcher) {
		initUnionFind(colWorld.getCollisionObjectArray().size);
    for (int i = 0; i < colWorld.getCollisionObjectArray().size; i++) {
      CollisionObject? collisionObject = colWorld.getCollisionObjectArray().getQuick(i);
      collisionObject?.setIslandTag(i);
      collisionObject?.setCompanionId(-1);
      collisionObject?.setHitFraction(1);
    }
		findUnions(dispatcher, colWorld);
	}
	
	void storeIslandActivationState(CollisionWorld colWorld) {
    for (int i = 0; i < colWorld.getCollisionObjectArray().size; i++) {
      CollisionObject? collisionObject = colWorld.getCollisionObjectArray().getQuick(i);
      if (!(collisionObject!.isStaticOrKinematicObject())) {
        collisionObject?.setIslandTag(_unionFind.find(i));
        collisionObject?.setCompanionId(-1);
      }
      else {
        collisionObject?.setIslandTag(-1);
        collisionObject?.setCompanionId(-2);
      }
		}
	}
	
	static int _getIslandId(PersistentManifold? lhs) {
		int islandId;
		CollisionObject? rcolObj0 = lhs?.getBody0() as CollisionObject?;
		CollisionObject? rcolObj1 = lhs?.getBody1() as CollisionObject?;
		islandId = (rcolObj0?.getIslandTag() ?? 0) >= 0? (rcolObj0?.getIslandTag() ?? 0) : (rcolObj1?.getIslandTag() ?? 0);
		return islandId;
	}

	void buildIslands(Dispatcher? dispatcher, ObjectArrayList<CollisionObject> collisionObjects) {
		BulletStats.pushProfile("islandUnionFindAndQuickSort");
		try {
			_islandmanifold.clear();

			// we are going to sort the unionfind array, and store the element id in the size
			// afterwards, we clean unionfind, to make sure no-one uses it anymore

			getUnionFind().sortIslands();
			int numElem = getUnionFind().getNumElements();

			int endIslandIndex = 1;
			int startIslandIndex;

			// update the sleeping state for bodies, if all are sleeping
			for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
				int islandId = getUnionFind().getElement(startIslandIndex)?.id ?? 0;
				//for (endIslandIndex = startIslandIndex + 1; (endIslandIndex < numElem) && (getUnionFind().getElement(endIslandIndex)?.id == islandId); endIslandIndex++) {}

				bool allSleeping = true;
				for (int idx = startIslandIndex; idx < endIslandIndex; idx++) {
					int i = getUnionFind().getElement(idx)?.sz ?? 0;

					CollisionObject? colObj0 = collisionObjects.getQuick(i);
					// if ((colObj0?.getIslandTag() != islandId) && (colObj0?.getIslandTag() != -1)) {
					// 	//System.err.println("error in island management\n");
					// }

					assert ((colObj0?.getIslandTag() == islandId) || (colObj0?.getIslandTag() == -1));
					if (colObj0?.getIslandTag() == islandId) {
						if (colObj0?.getActivationState() == CollisionObject.activeTag) {
							allSleeping = false;
						}
						if (colObj0?.getActivationState() == CollisionObject.disableDeactivation) {
							allSleeping = false;
						}
					}
				}


				if (allSleeping) {
					for (int idx = startIslandIndex; idx < endIslandIndex; idx++) {
						int i = getUnionFind().getElement(idx)?.sz ?? 0;
						CollisionObject? colObj0 = collisionObjects.getQuick(i);
						// if ((colObj0?.getIslandTag() != islandId) && (colObj0?.getIslandTag() != -1)) {
						// 	//System.err.println("error in island management\n");
						// }

						assert ((colObj0?.getIslandTag() == islandId) || (colObj0?.getIslandTag() == -1));

						if (colObj0?.getIslandTag() == islandId) {
							colObj0?.setActivationState(CollisionObject.islandSleeping);
						}
					}
				}
				else {
					for (int idx = startIslandIndex; idx < endIslandIndex; idx++) {
						int i = getUnionFind().getElement(idx)?.sz ?? 0;

						CollisionObject? colObj0 = collisionObjects.getQuick(i);
						// if ((colObj0?.getIslandTag() != islandId) && (colObj0?.getIslandTag() != -1)) {
						// 	//System.err.println("error in island management\n");
						// }

						assert ((colObj0?.getIslandTag() == islandId) || (colObj0?.getIslandTag() == -1));

						if (colObj0?.getIslandTag() == islandId) {
							if (colObj0?.getActivationState() == CollisionObject.islandSleeping) {
								colObj0?.setActivationState(CollisionObject.wantsDeactivation);
							}
						}
					}
				}
			}

			int maxNumManifolds = dispatcher?.getNumManifolds() ?? 0;
			for (int i = 0; i < maxNumManifolds; i++) {
				PersistentManifold? manifold = dispatcher?.getManifoldByIndexInternal(i);

				CollisionObject? colObj0 = manifold?.getBody0() as CollisionObject?;
				CollisionObject? colObj1 = manifold?.getBody1() as CollisionObject?;

				// todo: check sleeping conditions!
				if (((colObj0 != null) && colObj0.getActivationState() != CollisionObject.islandSleeping) ||
						((colObj1 != null) && colObj1.getActivationState() != CollisionObject.islandSleeping)) {

					// kinematic objects don't merge islands, but wake up all connected objects
					if (colObj0!.isKinematicObject() && colObj0.getActivationState() != CollisionObject.islandSleeping) {
						colObj1!.activate();
					}
					if (colObj1!.isKinematicObject() && colObj1.getActivationState() != CollisionObject.islandSleeping) {
						colObj0.activate();
					}
					if (dispatcher?.needsResponse(colObj0, colObj1) ?? false) {
						_islandmanifold.add(manifold!);
					}
				}
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}

	void buildAndProcessIslands(Dispatcher? dispatcher, ObjectArrayList<CollisionObject> collisionObjects, IslandCallback callback) {
		buildIslands(dispatcher, collisionObjects);

		int endIslandIndex = 1;
		int startIslandIndex;
		int numElem = getUnionFind().getNumElements();

		BulletStats.pushProfile("processIslands");
		try {
			int numManifolds = _islandmanifold.size;
			MiscUtil.quickSortObjectArray(_islandmanifold, _persistentManifoldComparator);

			int startManifoldIndex = 0;
			int endManifoldIndex = 1;

			for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
				int islandId = getUnionFind().getElement(startIslandIndex)?.id ?? 0;
				bool islandSleeping = false;

				for (endIslandIndex = startIslandIndex; (endIslandIndex < numElem) && (getUnionFind().getElement(endIslandIndex)?.id == islandId); endIslandIndex++) {
					int i = getUnionFind().getElement(endIslandIndex)?.sz ?? 0;
					CollisionObject? colObj0 = collisionObjects.getQuick(i);
					_islandBodies.add(colObj0);
					if (!colObj0!.isActive()) {
						islandSleeping = true;
					}
				}


				// find the accompanying contact manifold for this islandId
				int numIslandManifolds = 0;
				//List<PersistentManifold> startManifold = null;
				int startManifoldIdx = -1;

				if (startManifoldIndex < numManifolds) {
					int curIslandId = _getIslandId(_islandmanifold.getQuick(startManifoldIndex));
					if (curIslandId == islandId) {
						startManifoldIdx = startManifoldIndex;

						//for (endManifoldIndex = startManifoldIndex + 1; (endManifoldIndex < numManifolds) && (islandId == _getIslandId(_islandmanifold.getQuick(endManifoldIndex))); endManifoldIndex++) {}
						numIslandManifolds = endManifoldIndex - startManifoldIndex;
					}

				}

				if (!islandSleeping) {
					callback.processIsland(_islandBodies, _islandBodies.size, _islandmanifold, startManifoldIdx, numIslandManifolds, islandId);
				}

				if (numIslandManifolds != 0) {
					startManifoldIndex = endManifoldIndex;
				}

				_islandBodies.clear();
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}

  static Comparator<PersistentManifold?> _persistentManifoldComparator = (PersistentManifold? lhs, PersistentManifold? rhs){
    if(lhs == null || rhs == null) return -1;
    return _getIslandId(lhs) < _getIslandId(rhs)? -1 : 1;
  };
}

////////////////////////////////////////////////////////////////////////////
abstract class IslandCallback {
  void processIsland(ObjectArrayList<CollisionObject> bodies, int numBodies, ObjectArrayList<PersistentManifold> manifolds, int manifoldsOffset, int numManifolds, int islandId);
}
	
