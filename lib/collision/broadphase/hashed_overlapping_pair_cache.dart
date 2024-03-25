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

import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/overlap_callback.dart";
import "package:bullet_physics/collision/broadphase/overlap_filter_callback.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_callback.dart";
import "package:bullet_physics/core/bullet_stats.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/utils/int_array_list.dart";
import "package:bullet_physics/utils/object_array_list.dart";

class HashedOverlappingPairCache extends OverlappingPairCache {
	static const int _nullPair = 0xffffffff;
	
  final ObjectArrayList<BroadphasePair?> _overlappingPairArray = ObjectArrayList(2);
	OverlapFilterCallback? _overlapFilterCallback;
	final IntArrayList _hashTable = IntArrayList();
	final IntArrayList _next = IntArrayList();
	OverlappingPairCallback? _ghostPairCallback;

	HashedOverlappingPairCache() {
		_growTables();
	}

	/**
	 * Add a pair and return the pair. If the pair already exists,
	 * no pair is created and the old one is returned.
	 */
  @override
	BroadphasePair? addOverlappingPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1) {
		BulletStats.gAddedPairs++;
		if (!needsBroadphaseCollision(proxy0, proxy1)) {
			return null;
		}
		return _internalAddPair(proxy0,proxy1);
	}
  @override
	Object? removeOverlappingPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1, Dispatcher? dispatcher) {
		BulletStats.gRemovePairs++;
		if (proxy0!.getUid() > proxy1!.getUid()) {
			BroadphaseProxy? tmp = proxy0;
			proxy0 = proxy1;
			proxy1 = tmp;
		}
		int proxyId1 = proxy0.getUid();
		int proxyId2 = proxy1.getUid();

		int hash = _getHash(proxyId1, proxyId2) & (_overlappingPairArray.capacity - 1);

		BroadphasePair? pair = _internalFindPair(proxy0, proxy1, hash);
		if (pair == null) {
			return null;
		}

		cleanOverlappingPair(pair, dispatcher);

		Object? userData = pair.userInfo;

		assert (pair.pProxy0?.getUid() == proxyId1);
		assert (pair.pProxy1?.getUid() == proxyId2);

		int pairIndex = _overlappingPairArray.indexOf(pair);
		assert (pairIndex != -1);
		assert (pairIndex < _overlappingPairArray.size);

		// Remove the pair from the hash table.
		int index = _hashTable[hash];
		assert (index != _nullPair);

		int previous = _nullPair;
		while (index != pairIndex) {
			previous = index;
			index = _next[index];
		}

		if (previous != _nullPair) {
			assert (_next[previous] == pairIndex);
			_next.set(previous, _next.get(pairIndex));
		}
		else {
			_hashTable.set(hash, _next.get(pairIndex));
		}

		// We now move the last pair into spot of the
		// pair being removed. We need to fix the hash
		// table indices to support the move.

		int lastPairIndex = _overlappingPairArray.size - 1;

		_ghostPairCallback?.removeOverlappingPair(proxy0, proxy1, dispatcher);

		// If the removed pair is the last pair, we are done.
		if (lastPairIndex == pairIndex) {
			_overlappingPairArray.removeAt(_overlappingPairArray.size - 1);
			return userData;
		}

		// Remove the last pair from the hash table.
		BroadphasePair? last = _overlappingPairArray.getQuick(lastPairIndex)!;
		/* missing swap here too, Nat. */
		int lastHash = _getHash(last.pProxy0!.getUid(), last.pProxy1!.getUid()) & (_overlappingPairArray.capacity - 1);

		index = _hashTable[lastHash];
		assert (index != _nullPair);

		previous = _nullPair;

		while (index != lastPairIndex) {
			previous = index;
			index = _next.get(index);
		}

		if (previous != _nullPair) {
			assert (_next.get(previous) == lastPairIndex);
			_next.set(previous, _next.get(lastPairIndex));
		}
		else {
			_hashTable.set(lastHash, _next.get(lastPairIndex));
		}

		// Copy the last pair into the remove pair's spot.
		_overlappingPairArray.getQuick(pairIndex)?.set(_overlappingPairArray.getQuick(lastPairIndex));

		// Insert the last pair into the hash table
		_next.set(pairIndex, _hashTable.get(lastHash));
		_hashTable.set(lastHash, pairIndex);

		_overlappingPairArray.removeAt(_overlappingPairArray.size - 1);

		return userData;
	}

	bool needsBroadphaseCollision(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1) {
		if (_overlapFilterCallback != null) {
			return _overlapFilterCallback!.needBroadphaseCollision(proxy0, proxy1);
		}

		bool collides = (proxy0!.collisionFilterGroup & proxy1!.collisionFilterMask) != 0;
		collides = collides && (proxy1.collisionFilterGroup & proxy0.collisionFilterMask) != 0;

		return collides;
	}
	
	@override
	void processAllOverlappingPairs(OverlapCallback callback, Dispatcher? dispatcher) {
		for (int i=0; i<_overlappingPairArray.size; ) {

			BroadphasePair? pair = _overlappingPairArray.getQuick(i);
			if(pair != null && callback.processOverlap(pair)) {
				removeOverlappingPair(pair.pProxy0!, pair.pProxy1!, dispatcher);
				BulletStats.gOverlappingPairs--;
			}
			else {
				i++;
			}
		}
	}

  @override
	void removeOverlappingPairsContainingProxy(BroadphaseProxy proxy0, Dispatcher? dispatcher) {
		processAllOverlappingPairs(_RemovePairCallback(proxy0), dispatcher);
	}

	@override
	void cleanProxyFromPairs(BroadphaseProxy proxy, Dispatcher? dispatcher) {
		processAllOverlappingPairs(_CleanPairCallback(proxy, this, dispatcher), dispatcher);
	}

	@override
	ObjectArrayList<BroadphasePair?> getOverlappingPairArray() {
		return _overlappingPairArray;
	}

	@override
	void cleanOverlappingPair(BroadphasePair? pair, Dispatcher? dispatcher) {
		if (pair?.algorithm != null) {
			dispatcher?.freeCollisionAlgorithm(pair!.algorithm);
			pair?.algorithm = null;
		}
	}

	@override
	BroadphasePair? findPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1) {
		BulletStats.gFindPairs++;
		if (proxy0!.getUid() > proxy1!.getUid()) {
			proxy0 = proxy1;
			proxy1 = proxy0;
		}

		int proxyId1 = proxy0.getUid();
		int proxyId2 = proxy1.getUid();
		int hash = _getHash(proxyId1, proxyId2) & (_overlappingPairArray.capacity-1);

		if (hash >= _hashTable.size()){
			return null;
		}

		int index = _hashTable.get(hash);
		while (index != _nullPair && !_equalsPair(_overlappingPairArray.getQuick(index)!, proxyId1, proxyId2)){
			index = _next.get(index);
		}

		if (index == _nullPair){
			return null;
		}

		assert(index < _overlappingPairArray.size);

		return _overlappingPairArray[index];
	}

	int getCount() {
		return _overlappingPairArray.size;
	}

	OverlapFilterCallback? getOverlapFilterCallback() {
		return _overlapFilterCallback;
	}

	@override
	void setOverlapFilterCallback(OverlapFilterCallback overlapFilterCallback) {
		_overlapFilterCallback = overlapFilterCallback;
	}

	@override
	int getNumOverlappingPairs() {
		return _overlappingPairArray.size;
	}

	@override
	bool hasDeferredRemoval() {
		return false;
	}
	
	BroadphasePair _internalAddPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1) {
		if (proxy0!.getUid() > proxy1!.getUid()) {
			BroadphaseProxy? tmp = proxy0;
			proxy0 = proxy1;
			proxy1 = tmp;
		}

		int proxyId1 = proxy0.getUid();
		int proxyId2 = proxy1.getUid();

		int hash = _getHash(proxyId1, proxyId2) & (_overlappingPairArray.capacity - 1); // hash value with mask

		BroadphasePair? pair = _internalFindPair(proxy0, proxy1, hash);
		if (pair != null) {
			return pair;
		}

		int count = _overlappingPairArray.size;
		int oldCapacity = _overlappingPairArray.capacity;
		_overlappingPairArray.add(null);

		// this is where we add an actual pair, so also call the 'ghost'
		_ghostPairCallback?.addOverlappingPair(proxy0, proxy1);
		
		int newCapacity = _overlappingPairArray.capacity;
		if (oldCapacity < newCapacity) {
			_growTables();
			// hash with length
			hash = _getHash(proxyId1, proxyId2) & (_overlappingPairArray.capacity - 1);
		}
    pair = BroadphasePair(proxy0, proxy1);
		pair.algorithm = null;
		pair.userInfo = null;

    _overlappingPairArray.set(_overlappingPairArray.size - 1, pair);

		_next.set(count, _hashTable.get(hash));
		_hashTable.set(hash, count);

		return pair;
	}

	void _growTables() {
		final int newCapacity = _overlappingPairArray.capacity;
		if (_hashTable.size() < newCapacity) {
			// grow hashtable and next table
			final int curHashtableSize = _hashTable.size();

			MiscUtil.resizeArray(_hashTable, newCapacity, 0);
			MiscUtil.resizeArray(_next, newCapacity, 0);

			for (int i=0; i<newCapacity; ++i) {
				_hashTable.set(i, _nullPair);
        _next.set(i, _nullPair);
			}

			for (int i=0; i<curHashtableSize; i++) {
				BroadphasePair pair = _overlappingPairArray[i]!;
				int proxyId1 = pair.pProxy0!.getUid();
				int proxyId2 = pair.pProxy1!.getUid();
				int hashValue = _getHash(proxyId1, proxyId2) & (_overlappingPairArray.capacity - 1); // hash value with mask
				_next.set(i, _hashTable[hashValue]);
				_hashTable.set(hashValue, i);
			}
		}
	}

	bool _equalsPair(BroadphasePair pair, int proxyId1, int proxyId2) {
		return pair.pProxy0!.getUid() == proxyId1 && pair.pProxy1!.getUid() == proxyId2;
	}

	int _getHash(int? proxyId1, int? proxyId2) {
		int key = (proxyId1!) | (proxyId2! << 16);
		// Thomas Wang's hash

		key += ~(key << 15);
		key ^= (key >>> 10);
		key += (key << 3);
		key ^= (key >>> 6);
		key += ~(key << 11);
		key ^= (key >>> 16);
		return key;
	}

	BroadphasePair? _internalFindPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1, int hash) {
		int proxyId1 = proxy0!.getUid();
		int proxyId2 = proxy1!.getUid();
    
		int index = _hashTable.get(hash);

		while(index != _nullPair && _equalsPair(_overlappingPairArray.getQuick(index)!, proxyId1, proxyId2) == false) {
      index = _next.get(index);
		}

		if (index == _nullPair) {
			return null;
		}
    assert(index < _overlappingPairArray.size);
		return _overlappingPairArray.getQuick(index);
	}
  @override
	void setInternalGhostPairCallback(OverlappingPairCallback ghostPairCallback) {
		_ghostPairCallback = ghostPairCallback;
	}
}

////////////////////////////////////////////////////////////////////////////

class _RemovePairCallback extends OverlapCallback {
  late BroadphaseProxy _obsoleteProxy;

  _RemovePairCallback(BroadphaseProxy obsoleteProxy) {
    _obsoleteProxy = obsoleteProxy;
  }

  @override
  bool processOverlap(BroadphasePair pair) {
    return ((pair.pProxy0 == _obsoleteProxy) ||
        (pair.pProxy1 == _obsoleteProxy));
  }
}

class _CleanPairCallback extends OverlapCallback {
  late BroadphaseProxy _cleanProxy;
  late OverlappingPairCache _pairCache;
  late Dispatcher? _dispatcher;

  _CleanPairCallback(BroadphaseProxy cleanProxy, OverlappingPairCache pairCache, Dispatcher? dispatcher) {
    _cleanProxy = cleanProxy;
    _pairCache = pairCache;
    _dispatcher = dispatcher;
  }

  @override
  bool processOverlap(BroadphasePair pair) {
    if ((pair.pProxy0 == _cleanProxy) ||
        (pair.pProxy1 == _cleanProxy)) {
      _pairCache.cleanOverlappingPair(pair, _dispatcher);
    }
    return false;
  }
}