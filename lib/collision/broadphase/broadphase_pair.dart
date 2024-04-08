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
import 'package:bullet_physics/collision/broadphase/broadphase_proxy.dart';
import 'package:bullet_physics/collision/broadphase/collision_algorithm.dart';

class BroadphasePair {
	BroadphaseProxy? pProxy0;
	BroadphaseProxy? pProxy1;
	CollisionAlgorithm? algorithm;
	Object? userInfo;

	BroadphasePair([this.pProxy0, this.pProxy1]);
	
	void set(BroadphasePair? p) {
		pProxy0 = p?.pProxy0;
		pProxy1 = p?.pProxy1;
		algorithm = p?.algorithm;
		userInfo = p?.userInfo;
	}
	
	bool equals(BroadphasePair p) {
		return pProxy0 == p.pProxy0 && pProxy1 == p.pProxy1;
	}
  @override
	int get hashCode => _getHash();
  int _getHash(){
    return 31 * (pProxy0?.hashCode ?? 0) + (pProxy1?.hashCode ?? 0);
  }

	static Comparator<BroadphasePair> broadphasePairSortPredicate = (BroadphasePair a, BroadphasePair b) {
    bool result = a.pProxy0!.getUid() > b.pProxy0!.getUid() ||
        (a.pProxy0!.getUid() == b.pProxy0!.getUid() && a.pProxy1!.getUid() > b.pProxy1!.getUid()) ||
        (a.pProxy0!.getUid() == b.pProxy0!.getUid() && a.pProxy1!.getUid() == b.pProxy1!.getUid());
    return result? -1 : 1;
  };
}
