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

// Dbvt implementation by Nathanael Presson
import 'package:bullet_physics/collision/broadphase/broadphase_interface.dart';
import 'package:bullet_physics/collision/broadphase/broadphase_native_type.dart';
import 'package:bullet_physics/collision/broadphase/broadphase_pair.dart';
import 'package:bullet_physics/collision/broadphase/broadphase_proxy.dart';
import 'package:bullet_physics/collision/broadphase/dbvt.dart';
import 'package:bullet_physics/collision/broadphase/dbvt_aabb_mm.dart';
import 'package:bullet_physics/collision/broadphase/dbvt_proxy.dart';
import 'package:bullet_physics/collision/broadphase/dbvt_tree_collider.dart';
import 'package:bullet_physics/collision/broadphase/dispatcher.dart';
import 'package:bullet_physics/collision/broadphase/hashed_overlapping_pair_cache.dart';
import 'package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart';
import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';


class DbvtBroadphase extends BroadphaseInterface {
	static const double dbvtBPmargin = 0.05;

	static const int dynamicSet = 0; // Dynamic set index
	static const int fixedSet   = 1; // Fixed set index
	static const int stageCount  = 2; // Number of stages

	final List<Dbvt> sets = [Dbvt(),Dbvt()];                        // Dbvt sets
	List<DbvtProxy?> stageRoots = List.filled(stageCount+1, null);
	late OverlappingPairCache paircache;                         // Pair cache
	double predictedframes = 2;                                  // Frames predicted
	int stageCurrent = 0;                                       // Current stage
	int fupdates = 1;                                           // % of fixed updates per frame
	int dupdates = 1;                                           // % of dynamic updates per frame
	int pid = 0;                                                // Parse id
	int gid = 0;                                                // Gen id
	bool releasepaircache = false;                               // Release pair cache on delete

	DbvtBroadphase([OverlappingPairCache? paircache]) {
		releasepaircache = (paircache != null? false : true);
		this.paircache = paircache ?? HashedOverlappingPairCache();
		for (int i=0; i<=stageCount; i++) {
			stageRoots[i] = null;
    }
	}

	void collide(Dispatcher? dispatcher) {
		// optimize:
		sets[0].optimizeIncremental(1 + (sets[0].leaves * dupdates) ~/ 100);
		sets[1].optimizeIncremental(1 + (sets[1].leaves * fupdates) ~/ 100);
		stageCurrent = (stageCurrent + 1) % stageCount;

		DbvtProxy? current = stageRoots[stageCurrent];
		if (current != null) {
			DbvtTreeCollider collider = DbvtTreeCollider(this);
			do {
				DbvtProxy? next = current!.links[1];
				stageRoots[current.stage] = _listremove(current, stageRoots[current.stage]);
				stageRoots[stageCount] = _listappend(current, stageRoots[stageCount]);
				Dbvt.collideTT(sets[1].root, current.leaf, collider);
				sets[0].remove(current.leaf);
				current.leaf = sets[1].insert(current.aabb, current);
				current.stage = stageCount;
				current = next;
			} while (current != null);
		}

		// collide dynamics:
    DbvtTreeCollider collider = DbvtTreeCollider(this);
    Dbvt.collideTT(sets[0].root, sets[1].root, collider);
    Dbvt.collideTT(sets[0].root, sets[0].root, collider);

		// clean up:
    ObjectArrayList<BroadphasePair?> pairs = paircache.getOverlappingPairArray();
    if (pairs.isNotEmpty) {//pairs != null && 
      for (int i=0, ni=pairs.size; i<ni; i++) {
        BroadphasePair? p = pairs.getQuick(i);
        DbvtProxy? pa = p?.pProxy0 as DbvtProxy?;
        DbvtProxy? pb = p?.pProxy1 as DbvtProxy?;
        if (!DbvtAabbMm.intersect(pa!.aabb, pb!.aabb)) {
          
          if (pa.hashCode > pb.hashCode) {
            DbvtProxy? tmp = pa;
            pa = pb;
            pb = tmp;
          }
          paircache.removeOverlappingPair(pa, pb, dispatcher);
          ni--;
          i--;
        }
      }
    }
		pid++;
	}

	static DbvtProxy? _listappend(DbvtProxy? item, DbvtProxy? list) {
		item?.links[0] = null;
		item?.links[1] = list;
		if (list != null) list.links[0] = item;
		list = item;
		return list;
	}

	static DbvtProxy? _listremove(DbvtProxy? item, DbvtProxy? list) {
		if (item?.links[0] != null) {
			item?.links[0]!.links[1] = item.links[1];
		}
		else {
			list = item?.links[1];
		}
		
		if (item?.links[1] != null) {
			item?.links[1]!.links[0] = item.links[0];
		}
		return list;
	}
  @override
	BroadphaseProxy createProxy(
    Vector3 aabbMin, 
    Vector3 aabbMax, 
    BroadphaseNativeType shapeType, 
    Object? userPtr, 
    int collisionFilterGroup, 
    int collisionFilterMask, 
    Dispatcher? dispatcher, 
    Object? multiSapProxy
  ) {
		DbvtProxy proxy = DbvtProxy(userPtr, collisionFilterGroup, collisionFilterMask);
		DbvtAabbMm.fromMM(aabbMin, aabbMax, proxy.aabb);
		proxy.leaf = sets[0].insert(proxy.aabb, proxy);
		proxy.stage = stageCurrent;
		proxy.uniqueId = ++gid;
		stageRoots[stageCurrent] = _listappend(proxy, stageRoots[stageCurrent]);
		return proxy;
	}
  @override
	void destroyProxy(BroadphaseProxy absproxy, Dispatcher? dispatcher) {
		DbvtProxy proxy = absproxy as DbvtProxy;
		if (proxy.stage == stageCount) {
			sets[1].remove(proxy.leaf!);
		}
		else {
			sets[0].remove(proxy.leaf!);
		}
		stageRoots[proxy.stage] = _listremove(proxy, stageRoots[proxy.stage]);
		paircache.removeOverlappingPairsContainingProxy(proxy, dispatcher);
	}
  @override
	void setAabb(BroadphaseProxy? absproxy, Vector3 aabbMin, Vector3 aabbMax, Dispatcher? dispatcher) {
		DbvtProxy? proxy = absproxy as DbvtProxy?;
		DbvtAabbMm aabb = DbvtAabbMm.fromMM(aabbMin, aabbMax, DbvtAabbMm());
		if (proxy?.stage == stageCount) {
			sets[1].remove(proxy!.leaf!);
			proxy.leaf = sets[0].insert(aabb, proxy);
		}
		else {
			if (DbvtAabbMm.intersect(proxy?.leaf?.volume, aabb)) {
				Vector3 delta = Vector3.zero();
				delta.add2(aabbMin, aabbMax);
				delta.scale(0.5);
				delta.sub(proxy!.aabb.center(Vector3.zero()));
				delta.scale(predictedframes);
				sets[0].updateWithVelocityAndMargin(proxy.leaf!, aabb, delta, DbvtBroadphase.dbvtBPmargin);
			}
			else {
				sets[0].updateWithVolume(proxy?.leaf, aabb);
			}
		}
		
		stageRoots[proxy?.stage ?? 0] = _listremove(proxy, stageRoots[proxy?.stage ?? 0]);
		proxy?.aabb.set(aabb);
		proxy?.stage = stageCurrent;
		stageRoots[stageCurrent] = _listappend(proxy, stageRoots[stageCurrent]);
	}

  @override
	void calculateOverlappingPairs(Dispatcher? dispatcher) {
		collide(dispatcher);
	}

  @override
	OverlappingPairCache? getOverlappingPairCache() {
		return paircache;
	}
  @override
	void getBroadphaseAabb(Vector3 aabbMin, Vector3 aabbMax) {
		DbvtAabbMm bounds = DbvtAabbMm();
		if (!sets[0].empty()) {
			if (!sets[1].empty()) {
				DbvtAabbMm.merge(sets[0].root!.volume, sets[1].root!.volume, bounds);
			}
			else {
				bounds.set(sets[0].root!.volume);
			}
		}
		else if (!sets[1].empty()) {
			bounds.set(sets[1].root!.volume);
		}
		else {
			DbvtAabbMm.fromCR(Vector3(0, 0, 0), 0, bounds);
		}
		aabbMin.setFrom(bounds.mins());
		aabbMax.setFrom(bounds.maxs());
	}
  @override
	void printStats() {}
}
