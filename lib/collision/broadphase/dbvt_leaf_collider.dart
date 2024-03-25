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

import 'package:bullet_physics/collision/broadphase/dbvt_aabb_mm.dart';
import 'package:bullet_physics/collision/broadphase/dbvt_broadphase.dart';
import 'package:bullet_physics/collision/broadphase/dbvt_proxy.dart';
import 'package:bullet_physics/collision/broadphase/node.dart';

class DbvtLeafCollider extends ICollide {
	DbvtBroadphase pbp;
	DbvtProxy ppx;
	DbvtLeafCollider(this.pbp, this.ppx);

	@override
	void process(Node na,[Node? nb, double f = 0]) {
		nb = ppx.leaf;
		if (nb != na) {
			DbvtProxy pa = na.data as DbvtProxy;
			DbvtProxy pb = nb!.data as DbvtProxy;

			if (DbvtAabbMm.intersect(pa.aabb, pb.aabb)){
				if (pa.hashCode > pb.hashCode) {
					DbvtProxy tmp = pa;
					pa = pb;
					pb = tmp;
				}
				pbp.paircache.addOverlappingPair(pa, pb);
			}
		}
	}

}
