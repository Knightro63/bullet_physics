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

import "package:bullet_physics/collision/broadphase/dbvt_aabb_mm.dart";
import "package:bullet_physics/collision/broadphase/node.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/collections.dart";
import "package:bullet_physics/utils/int_array_list.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

class Dbvt {
	static const int simpleStackSize = 64;
	static const int doubleStackSize = simpleStackSize * 2;
	
	Node? root;
	Node? free;
	int lkhd = -1;
	int leaves = 0;
	int opath = 0;

	Dbvt();

	void clear() {
		if (root != null) {
			_recursedeletenode(this, root);
		}
		free = null;
    lkhd = -1;
    opath = 0;
    //stkStack.clear();
	}

	bool empty() {
		return (root == null);
	}

	void optimizeBottomUp() {
		if (root != null) {
			ObjectArrayList<Node> leaves = ObjectArrayList(this.leaves);
			_fetchleaves(this, root, leaves);
			_bottomup(this, leaves);
			root = leaves.getQuick(0);
		}
	}

	void optimizeTopDown([int buTreshold = 128]) {
		if (root != null) {
			ObjectArrayList<Node> leaves = ObjectArrayList(this.leaves);
			_fetchleaves(this, root, leaves);
			root = _topdown(this, leaves, buTreshold);
		}
	}

	void optimizeIncremental(int passes) {
		if (passes < 0) {
			passes = leaves;
		}
		
		if (root != null && (passes > 0)) {
			List<Node?> rootRef = [null];
			do {
				Node? node = root;
				int bit = 0;
				while (node?.isinternal() ?? false) {
					rootRef[0] = root;
					node = _sort(node, rootRef)?.childs[(opath >>> bit) & 1];
					root = rootRef[0];
					
					bit = (bit + 1) & (4 * 8 - 1);
				}
				update(node);
				++opath;
			}
			while ((--passes) != 0);
		}
	}

	Node? insert(DbvtAabbMm box, Object data) {
		Node? leaf = _createnode(this, null, box, data);
		_insertleaf(this, root, leaf);
		leaves++;
		return leaf;
	}

	void update(Node? leaf, [int lookahead = -1]) {
		Node? root = _removeleaf(this, leaf);
		if (root != null) {
			if (lookahead >= 0) {
				for (int i = 0; (i < lookahead) && root?.parent != null; i++) {
					root = root?.parent;
				}
			}
			else {
				root = this.root;
			}
		}
		_insertleaf(this, root, leaf);
	}

	void updateWithVolume(Node? leaf, DbvtAabbMm volume) {
		Node? root = _removeleaf(this, leaf);
		if (root != null) {
			if (lkhd >= 0) {
				for (int i = 0; (i < lkhd) && root?.parent != null; i++) {
					root = root?.parent;
				}
			}
			else {
				root = this.root;
			}
		}
		leaf?.volume.set(volume);
		_insertleaf(this, root, leaf);
	}

	bool updateWithVelocityAndMargin(Node leaf, DbvtAabbMm volume, Vector3 velocity, double margin) {
		if (leaf.volume.contain(volume)) {
			return false;
		}
		Vector3 tmp = Vector3.zero();
		tmp.setValues(margin, margin, margin);
		volume.expand(tmp);
		volume.signedExpand(velocity);
		updateWithVolume(leaf, volume);
		return true;
	}

	bool updateWithVelocity(Node leaf, DbvtAabbMm volume, Vector3 velocity) {
		if (leaf.volume.contain(volume)) {
			return false;
		}
		volume.signedExpand(velocity);
		updateWithVolume(leaf, volume);
		return true;
	}

	bool updateWithMargin(Node leaf, DbvtAabbMm volume, double margin) {
		if (leaf.volume.contain(volume)) {
			return false;
		}
		Vector3 tmp = Vector3.zero();
		tmp.setValues(margin, margin, margin);
		volume.expand(tmp);
		updateWithVolume(leaf, volume);
		return true;
	}

	void remove(Node? leaf) {
		_removeleaf(this, leaf);
		_deletenode(this, leaf);
		leaves--;
	}

	void write(IWriter iwriter) {
		throw'UnsupportedOperationException()';
	}

	void clone(Dbvt dest, [IClone? iclone]) {
		throw'UnsupportedOperationException()';
	}

	static int countLeaves(Node? node) {
		if (node?.isinternal() ?? false) {
			return countLeaves(node?.childs[0]) + countLeaves(node?.childs[1]);
		}
		else {
			return 1;
		}
	}

	static void extractLeaves(Node? node, ObjectArrayList<Node?> leaves) {
		if (node?.isinternal() ?? false) {
			extractLeaves(node?.childs[0], leaves);
			extractLeaves(node?.childs[1], leaves);
		}
		else {
			leaves.add(node);
		}
	}

	static void enumNodes(Node? root, ICollide? policy) {
		  policy?.process(root);
      if (root?.isinternal() ?? false) {
        enumNodes(root?.childs[0], policy);
        enumNodes(root?.childs[1], policy);
      }
	}

	static void enumLeaves(Node? root, ICollide? policy) {
		//DBVT_CHECKTYPE
		if (root?.isinternal() ?? false) {
			enumLeaves(root?.childs[0], policy);
			enumLeaves(root?.childs[1], policy);
		}
		else{
			policy?.process(root);
		}
	}

	static void collideTT(Node? root0, Node? root1, ICollide policy) {
		//DBVT_CHECKTYPE
		if (root0 != null && root1 != null) {
			ObjectArrayList<sStkNN> stack = ObjectArrayList(doubleStackSize);
			stack.add(sStkNN(root0, root1));
			do {
				sStkNN p = stack.removeAt(stack.size - 1)!;
				if (p.a == p.b) {
					if (p.a!.isinternal()) {
						stack.add(sStkNN(p.a!.childs[0], p.a!.childs[0]));
						stack.add(sStkNN(p.a!.childs[1], p.a!.childs[1]));
						stack.add(sStkNN(p.a!.childs[0], p.a!.childs[1]));
					}
				}
				else if (DbvtAabbMm.intersect(p.a!.volume, p.b!.volume)) {
					if (p.a!.isinternal()) {
						if (p.b!.isinternal()) {
							stack.add(sStkNN(p.a!.childs[0], p.b!.childs[0]));
							stack.add(sStkNN(p.a!.childs[1], p.b!.childs[0]));
							stack.add(sStkNN(p.a!.childs[0], p.b!.childs[1]));
							stack.add(sStkNN(p.a!.childs[1], p.b!.childs[1]));
						}
						else {
							stack.add(sStkNN(p.a!.childs[0], p.b));
							stack.add(sStkNN(p.a!.childs[1], p.b));
						}
					}
					else {
						if (p.b!.isinternal()) {
							stack.add(sStkNN(p.a, p.b?.childs[0]));
							stack.add(sStkNN(p.a, p.b?.childs[1]));
						}
						else {
							policy.process(p.a!, p.b);
						}
					}
				}
			}
			while (stack.size > 0);
		}
	}

	static void collideTTWithSingle(Node? root0, Node? root1, Transform xform, ICollide policy) {
		//DBVT_CHECKTYPE
		if (root0 != null && root1 != null) {
      int depth = 1;
      int treshold = doubleStackSize - 4;
			ObjectArrayList<sStkNN> stack = ObjectArrayList(doubleStackSize);
			stack[0] = sStkNN(root0, root1);
			do {
				sStkNN? p = stack[--depth];
        if (depth > treshold){
          stack.resize(stack.size * 2);
          treshold = stack.size - 4;
        }
				if (p?.a == p?.b) {
					if (p?.a?.isinternal() ?? false) {
						stack.add(sStkNN(p!.a?.childs[0], p.a?.childs[0]));
						stack.add(sStkNN(p.a?.childs[1], p.a?.childs[1]));
						stack.add(sStkNN(p.a?.childs[0], p.a?.childs[1]));
					}
				}
				else if (DbvtAabbMm.intersectWithTransform(p?.a?.volume, p?.b?.volume, xform)) {
					if (p?.a?.isinternal() ?? false) {
						if (p?.b?.isinternal() ?? false) {
							stack.add(sStkNN(p!.a?.childs[0], p.b?.childs[0]));
							stack.add(sStkNN(p.a?.childs[1], p.b?.childs[0]));
							stack.add(sStkNN(p.a?.childs[0], p.b?.childs[1]));
							stack.add(sStkNN(p.a?.childs[1], p.b?.childs[1]));
						}
						else {
							stack.add(sStkNN(p!.a?.childs[0], p.b));
							stack.add(sStkNN(p.a?.childs[1], p.b));
						}
					}
					else {
						if (p?.b?.isinternal() ?? false) {
							stack.add(sStkNN(p!.a, p.b?.childs[0]));
							stack.add(sStkNN(p.a, p.b?.childs[1]));
						}
						else{
							policy.process(p!.a!, p.b);
						}
					}
				}
			}
			while (stack.isNotEmpty);
		}
	}

	static void collideTTWithDouble(Node root0, Transform xform0, Node root1, Transform xform1, ICollide policy) {
		Transform xform = Transform();
		xform.inverse(xform0);
		xform.mul(xform1);
		collideTTWithSingle(root0, root1, xform, policy);
	}

	static void collideTV(Node? root, DbvtAabbMm volume, ICollide policy) {
		//DBVT_CHECKTYPE
		if (root != null) {
			ObjectArrayList<Node?> stack = ObjectArrayList(doubleStackSize);
			stack.add(root);
			do {
				Node? n = stack.removeAt(stack.size - 1);
				if (DbvtAabbMm.intersect(n!.volume, volume)) {
					if (n.isinternal()) {
						stack.add(n.childs[0]);
						stack.add(n.childs[1]);
					}
					else{
						policy.process(n);
					}
				}
			}
			while (stack.isNotEmpty);
		}
	}

	static void collideRAY(Node? root, Vector3 origin, Vector3 direction, ICollide policy) {
		//DBVT_CHECKTYPE
		if (root != null) {
			Vector3 normal = Vector3.zero();
			normal.normalizeFrom(direction);
			Vector3 invdir = Vector3.zero();
			invdir.setValues(1 / normal.x, 1 / normal.y, 1 / normal.z);
			List<int> signs = [ direction.x<0 ? 1:0, direction.y<0 ? 1:0, direction.z<0 ? 1:0 ];
			ObjectArrayList<Node?> stack = ObjectArrayList(simpleStackSize);//List<Node>(simpleStackSize);
      //List<Node?> stack = [];
			stack.add(root);
			do {
				Node? node = stack.removeAt(stack.size - 1);
				if (DbvtAabbMm.intersectWithSigns(node?.volume, origin, invdir, signs)) {
					if (node?.isinternal() ?? false) {
						stack.add(node?.childs[0]);
						stack.add(node?.childs[1]);
					}
					else{
						policy.process(node);
					}
				}
			}
			while (stack.isNotEmpty);
		}
	}

	static void collideKDOP(Node? root, List<Vector3> normals, List<double> offsets, int count, ICollide policy) {
		//DBVT_CHECKTYPE
		if (root != null) {
			int inside = (1 << count) - 1;
			//ObjectArrayList<sStkNP> stack = ObjectArrayList(simpleStackSize);
      List<sStkNP> stack = [];
			List<int> signs = List.filled(4 * 8, 0);//int[4 * 8];
			assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
			for (int i=0; i<count; ++i) {
				signs[i] = ((normals[i].x >= 0) ? 1 : 0) +
						((normals[i].y >= 0) ? 2 : 0) +
						((normals[i].z >= 0) ? 4 : 0);
			}
			stack.add(sStkNP(root, 0));
			do {
				sStkNP se = stack.removeAt(stack.length - 1);
				bool out = false;
				for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
					if (0 == (se.mask & j)) {
						int side = se.node?.volume.classify(normals[i], offsets[i], signs[i]) ?? 0;
						switch (side) {
							case -1:
								out = true;
								break;
							case 1:
								se.mask |= j;
								break;
						}
					}
				}
				if (!out) {
					if ((se.mask != inside) && (se.node?.isinternal() ?? false)) {
						stack.add(sStkNP(se.node?.childs[0], se.mask));
						stack.add(sStkNP(se.node?.childs[1], se.mask));
					}
					else {
						if (policy.allLeaves(se.node)) {
							enumLeaves(se.node, policy);
						}
					}
				}
			}
			while (stack.isNotEmpty);
		}
	}

	static void collideOCL(Node? root, List<Vector3> normals, List<double> offsets, Vector3 sortaxis, int count, ICollide policy, [bool fullsort = true]) {
		//DBVT_CHECKTYPE
		if (root != null) {
			int srtsgns = (sortaxis.x >= 0 ? 1 : 0) +
					(sortaxis.y >= 0 ? 2 : 0) +
					(sortaxis.z >= 0 ? 4 : 0);
			int inside = (1 << count) - 1;
			ObjectArrayList<sStkNPS> stock = ObjectArrayList();//List<sStkNPS>();
			IntArrayList ifree = IntArrayList();
			IntArrayList stack = IntArrayList();
			List<int> signs = List.filled(4*8, 0);//int[/*sizeof(unsigned)*8*/4 * 8];
			assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
			for (int i = 0; i < count; i++) {
				signs[i] = ((normals[i].x >= 0) ? 1 : 0) +
						((normals[i].y >= 0) ? 2 : 0) +
						((normals[i].z >= 0) ? 4 : 0);
			}
			//stock.reserve(simpleStackSize);
			//stack.reserve(simpleStackSize);
			//ifree.reserve(simpleStackSize);
			stack.add(allocate(ifree, stock, sStkNPS(root, 0, root.volume.projectMinimum(sortaxis, srtsgns))));
			do {
				// JAVA NOTE: check
				int id = stack.remove(stack.size() - 1)!;
				sStkNPS se = stock.getQuick(id)!;
				ifree.add(id);
				if (se.mask != inside) {
					bool out = false;
					for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
						if (0 == (se.mask & j)) {
							int side = se.node?.volume.classify(normals[i], offsets[i], signs[i]) ?? 0;
							switch (side) {
								case -1:
									out = true;
									break;
								case 1:
									se.mask |= j;
									break;
							}
						}
					}
					if (out) {
						continue;
					}
				}
				if (policy.descent(se.node)) {
					if (se.node?.isinternal() ?? false) {
						List<Node?> pns = [se.node?.childs[0], se.node?.childs[1]];
						List<sStkNPS> nes = [sStkNPS(pns[0], se.mask, pns[0]?.volume.projectMinimum(sortaxis, srtsgns) ?? 0),
							sStkNPS(pns[1], se.mask, pns[1]?.volume.projectMinimum(sortaxis, srtsgns) ?? 0)
            ];
						int q = nes[0].value < nes[1].value ? 1 : 0;
						int j = stack.size();
						if (fullsort && (j > 0)) {
							j = nearest(stack, stock, nes[q].value, 0, stack.size());
							stack.add(0);
							for (int k = stack.size() - 1; k > j; --k) {
								stack.set(k, stack.get(k - 1));
							}
							stack.set(j, allocate(ifree, stock, nes[q]));
							j = nearest(stack, stock, nes[1 - q].value, j, stack.size());
							stack.add(0);
							for (int k = stack.size() - 1; k > j; --k) {
								stack.set(k, stack.get(k - 1));
							}
							stack.set(j, allocate(ifree, stock, nes[1 - q]));
						}
						else {
							stack.add(allocate(ifree, stock, nes[q]));
							stack.add(allocate(ifree, stock, nes[1 - q]));
						}
					}
					else if(se.node != null){
						policy.process(se.node!, null ,se.value);
					}
				}
			}
			while (stack.isNotEmpty);
		}
	}

	static void collideTU(Node? root, ICollide policy) {
		//DBVT_CHECKTYPE
		if (root != null) {
			ObjectArrayList<Node?> stack = ObjectArrayList(simpleStackSize);
			stack.add(root);
			do {
				Node? n = stack.removeAt(stack.size - 1);
				if (policy.descent(n)) {
					if (n?.isinternal() ?? false) {
						stack.add(n?.childs[0]);
						stack.add(n?.childs[1]);
					}
					else if(n != null){
						policy.process(n);
					}
				}
			}
			while (stack.isNotEmpty);
		}
	}
	
	static int nearest(IntArrayList i, ObjectArrayList<sStkNPS> a, double v, int l, int h) {
		int m = 0;
		while (l < h) {
			m = (l + h) >> 1;
			if ((a.getQuick(i.get(m))?.value ?? 0) >= v) {
				l = m + 1;
			}
			else {
				h = m;
			}
		}
		return h;
	}
	
	static int allocate(IntArrayList ifree, ObjectArrayList<sStkNPS> stock, sStkNPS value) {
		int i;
		if (ifree.isNotEmpty) {
			i = ifree.get(ifree.size() - 1);
			ifree.remove(ifree.size() - 1);
			stock.getQuick(i)?.set(value);
		}
		else {
			i = stock.size;
			stock.add(value);
		}
		return (i);
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	static int _indexof(Node? node) {
		return (node?.parent?.childs[1] == node)? 1:0;
	}

	static DbvtAabbMm _merge(DbvtAabbMm? a, DbvtAabbMm? b, DbvtAabbMm out) {
    if(a == null || b == null) return out;
		DbvtAabbMm.merge(a, b, out);
		return out;
	}
	
	// volume+edge lengths
	static double _size(DbvtAabbMm a) {
		Vector3 edges = a.lengths(Vector3.zero());
		return (edges.x * edges.y * edges.z +
		        edges.x + edges.y + edges.z);
	}

	static void _deletenode(Dbvt? pdbvt, Node? node) {
		//btAlignedFree(pdbvt->m_free);
		pdbvt?.free = node;
	}

	static void _recursedeletenode(Dbvt? pdbvt, Node? node) {
		if (!(node?.isleaf() ?? true)) {
			_recursedeletenode(pdbvt, node?.childs[0]);
			_recursedeletenode(pdbvt, node?.childs[1]);
		}
		if (node == pdbvt?.root) {
			pdbvt?.root = null;
		}
		_deletenode(pdbvt, node);
	}
	
	static Node? _createnode(Dbvt pdbvt, Node? parent, DbvtAabbMm volume, Object? data) {
		Node? node;
		if (pdbvt.free != null) {
			node = pdbvt.free;
			pdbvt.free = null;
		}
		else {
			node = Node();
		}
		node?.parent = parent;
		node?.volume.set(volume);
		node?.data = data;
		node?.childs[1] = null;
		return node;
	}

	static void _insertleaf(Dbvt pdbvt, Node? root, Node? leaf) {
		if (pdbvt.root == null) {
			pdbvt.root = leaf;
			leaf?.parent = null;
		}
		else {
			if (!(root?.isleaf() ?? true)) {
				do {
					if (DbvtAabbMm.proximity(root?.childs[0]?.volume, leaf?.volume) < DbvtAabbMm.proximity(root?.childs[1]?.volume, leaf?.volume)) {
						root = root?.childs[0];
					}
					else {
						root = root?.childs[1];
					}
				}
				while (!(root?.isleaf() ?? true));
			}
			Node? prev = root?.parent;
			Node? node = _createnode(pdbvt, prev, _merge(leaf?.volume, root?.volume, DbvtAabbMm()), null);
			if (prev != null) {
				prev.childs[_indexof(root)] = node;
				node?.childs[0] = root;
				root?.parent = node;
				node?.childs[1] = leaf;
				leaf?.parent = node;
				do {
					if (!(prev?.volume.contain(node!.volume) ?? true)) {
						DbvtAabbMm.merge(prev!.childs[0]!.volume, prev.childs[1]!.volume, prev.volume);
					}
					else {
						break;
					}
					node = prev;
				}
				while (null != (prev = node.parent));
			}
			else {
				node?.childs[0] = root;
				root?.parent = node;
				node?.childs[1] = leaf;
				leaf?.parent = node;
				pdbvt.root = node;
			}
		}
	}
	
	static Node? _removeleaf(Dbvt pdbvt, Node? leaf) {
		if (leaf == pdbvt.root) {
			pdbvt.root = null;
			return null;
		}
		else {
			Node? parent = leaf?.parent;
			Node? prev = parent?.parent;
			Node? sibling = parent?.childs[1 - _indexof(leaf)];
			if (prev != null) {
				prev.childs[_indexof(parent)] = sibling;
				sibling?.parent = prev;
				_deletenode(pdbvt, parent);
				while (prev != null) {
					DbvtAabbMm pb = prev.volume;
					DbvtAabbMm.merge(prev.childs[0]!.volume, prev.childs[1]!.volume, prev.volume);
					if (DbvtAabbMm.notEqual(pb, prev.volume)) {
						prev = prev.parent;
					}
					else {
						break;
					}
				}
				return prev ?? pdbvt.root;
			}
			else {
				pdbvt.root = sibling;
				sibling?.parent = null;
				_deletenode(pdbvt, parent);
				return pdbvt.root;
			}
		}
	}
	

	static void _fetchleaves(Dbvt pdbvt, Node? root, ObjectArrayList<Node?> leaves, [int depth = -1]) {
		if ((root?.isinternal() ?? false) && depth != 0) {
			_fetchleaves(pdbvt, root?.childs[0], leaves, depth - 1);
			_fetchleaves(pdbvt, root?.childs[1], leaves, depth - 1);
			_deletenode(pdbvt, root);
		}
		else {
			leaves.add(root);
		}
	}
	
	static void _split(ObjectArrayList<Node> leaves, ObjectArrayList<Node> left, ObjectArrayList<Node> right, Vector3 org, Vector3 axis) {
		Vector3 tmp = Vector3.zero();
		MiscUtil.resizeObjectArray(left, 0, Node);
		MiscUtil.resizeObjectArray(right, 0, Node);
		for (int i=0, ni=leaves.size; i<ni; i++) {
			leaves.getQuick(i)?.volume.center(tmp);
			tmp.sub(org);
			if (axis.dot(tmp) < 0) {
				left.add(leaves.getQuick(i));
			}
			else {
				right.add(leaves.getQuick(i));
			}
		}
	}
	
	static DbvtAabbMm _bounds(ObjectArrayList<Node> leaves) {
		DbvtAabbMm volume = DbvtAabbMm(leaves.getQuick(0)?.volume);
		for (int i=1, ni=leaves.size; i<ni; i++) {
			_merge(volume, leaves.getQuick(i)?.volume, volume);
		}
		return volume;
	}
	
	static void _bottomup(Dbvt pdbvt, ObjectArrayList<Node?> leaves) {
		DbvtAabbMm tmpVolume = DbvtAabbMm();
		while (leaves.size > 1) {
			double minsize = BulletGlobals.simdInfinity;
			List<int> minidx = [-1, -1];
			for (int i=0; i<leaves.size; i++) {
				for (int j=i+1; j<leaves.size; j++) {
					double sz = _size(_merge(leaves.getQuick(i)?.volume, leaves.getQuick(j)?.volume, tmpVolume));
					if (sz < minsize) {
						minsize = sz;
						minidx[0] = i;
						minidx[1] = j;
					}
				}
			}
			List<Node?> n = [ leaves.getQuick(minidx[0]), leaves.getQuick(minidx[1]) ];
			Node? p = _createnode(pdbvt, null, _merge(n[0]?.volume, n[1]?.volume, DbvtAabbMm()), null);
			p?.childs[0] = n[0];
			p?.childs[1] = n[1];
			n[0]?.parent = p;
			n[1]?.parent = p;
      
			leaves.setQuick(minidx[0], p);
			Collections.swapObjectArray(leaves, minidx[1], leaves.size - 1);
			leaves.removeAt(leaves.size - 1);
		}
	}

	static final List<Vector3> _axis = [ Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1) ];
	
	static Node? _topdown(Dbvt pdbvt, ObjectArrayList<Node> leaves, int buTreshold) {
		if (leaves.size > 1) {
			if (leaves.size > buTreshold) {
				DbvtAabbMm vol = _bounds(leaves);
				Vector3 org = vol.center(Vector3.zero());
				List<ObjectArrayList<Node>> sets = [ObjectArrayList<Node>(),ObjectArrayList<Node>()];
				// for (int i=0; i < sets.length; i++) {
				// 	sets[i] = ObjectArrayList();//List();
				// }
				int bestaxis = -1;
				int bestmidp = leaves.size;
				List<List<int>> splitcount = [[0, 0], [0, 0], [0, 0]];

				Vector3 x = Vector3.zero();

				for (int i=0; i<leaves.size; i++) {
					leaves.getQuick(i)?.volume.center(x);
					x.sub(org);
					for (int j=0; j<3; j++) {
						splitcount[j][x.dot(_axis[j]) > 0? 1 : 0]++;
					}
				}
				for (int i=0; i<3; i++) {
					if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0)) {
						int midp = (splitcount[i][0] - splitcount[i][1]).abs();
						if (midp < bestmidp) {
							bestaxis = i;
							bestmidp = midp;
						}
					}
				}
				if (bestaxis >= 0) {
					//sets[0].reserve(splitcount[bestaxis][0]);
					//sets[1].reserve(splitcount[bestaxis][1]);
					_split(leaves, sets[0], sets[1], org, _axis[bestaxis]);
				}
				else {
					//sets[0].reserve(leaves.length/2+1);
					//sets[1].reserve(leaves.length/2);
					for (int i=0, ni=leaves.size; i<ni; i++) {
						sets[i & 1].add(leaves.getQuick(i));
					}
				}
				Node? node = _createnode(pdbvt, null, vol, null);
				node?.childs[0] = _topdown(pdbvt, sets[0], buTreshold);
				node?.childs[1] = _topdown(pdbvt, sets[1], buTreshold);
				node?.childs[0]?.parent = node;
				node?.childs[1]?.parent = node;
				return node;
			}
			else {
				_bottomup(pdbvt, leaves);
				return leaves.getQuick(0);
			}
		}
		return leaves.getQuick(0);
	}
	
	static Node? _sort(Node? n, List<Node?> r) {
		Node? p = n?.parent;
		assert (n?.isinternal() ?? false);
		if (p != null && p.hashCode > n.hashCode) {
			int i = _indexof(n);
			int j = 1 - i;
			Node? s = p.childs[j];
			Node? q = p.parent;
			assert (n == p.childs[i]);
			if (q != null) {
				q.childs[_indexof(p)] = n;
			}
			else {
				r[0] = n;
			}
			s?.parent = n;
			p.parent = n;
			n?.parent = q;
			p.childs[0] = n?.childs[0];
			p.childs[1] = n?.childs[1];
			n?.childs[0]?.parent = p;
			n?.childs[1]?.parent = p;
			n?.childs[i] = p;
			n?.childs[j] = s;
      
			DbvtAabbMm.swap(p.volume, n?.volume);
			return p;
		}
		return n;
	}
	
	static Node? _walkup(Node? n, int count) {
		while (n != null && (count--) != 0) {
			n = n.parent;
		}
		return n;
	}	
}
