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

import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
/**
 * UnionFind calculates connected subsets. Implements weighted Quick Union with
 * path compression.
 * 
 * @author jezek2
 */
class UnionFind {
	
	// Optimization: could use int ints instead of ints (halving memory, would limit the number of rigid bodies to 64k, sounds reasonable).

	final ObjectArrayList<Element> _elements = ObjectArrayList();//List<Element>();
	
	/**
	 * This is a special operation, destroying the content of UnionFind.
	 * It sorts the elements, based on island id, in order to make it easy to iterate over islands.
	 */
	void sortIslands() {
		// first store the original body index, and islandId
		int numElements = _elements.size;

		for (int i = 0; i < numElements; i++) {
			_elements.getQuick(i)?.id = find(i);
			_elements.getQuick(i)?.sz = i;
		}

		// Sort the vector using predicate and std::sort
		//std::sort(m_elements.begin(), m_elements.end(), btUnionFindElementSortPredicate);
		//perhaps use radix sort?
		//elements.heapSort(btUnionFindElementSortPredicate());
		
		//Collections.sort(elements);
		MiscUtil.quickSortObjectArray(_elements, _elementComparator);
	}

	void reset(int N) {
		allocate(N);

		for (int i = 0; i < N; i++) {
			_elements.getQuick(i)?.id = i;
			_elements.getQuick(i)?.sz = 1;
		}
	}

	int getNumElements() {
		return _elements.size;
	}

	bool isRoot(int x) {
		return (x == _elements.getQuick(x)?.id);
	}

	Element? getElement(int index) {
		return _elements.getQuick(index);
	}

	void allocate(int N) {
		MiscUtil.resizeObjectArray(_elements, N, Element());
	}

	void free() {
		_elements.clear();
	}

	void unite(int p, int q) {
		int i = find(p), j = find(q);
		if (i == j) {
			return;
		}
		_elements.getQuick(i)?.id = j;
		_elements.getQuick(j)?.sz += _elements.getQuick(i)?.sz ?? 0;
	}

	int findCompare(int p, int q) {
		return (find(p) == find(q))? 1 : 0;
	}

	int find(int x) {
		while (x != _elements.getQuick(x)?.id) {
			_elements.getQuick(x)?.id = _elements.getQuick(_elements.getQuick(x)?.id ?? 0)?.id ?? 0;
			x = _elements.getQuick(x)?.id ?? 0;
		}
		return x;
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	static final Comparator<Element?> _elementComparator = (Element? o1, Element? o2) {
    if(o1 == null || o2 == null) return -1;
		return o1.id < o2.id? -1 : 1;
	};
}

class Element {
  int id = 0;
  int sz = 0;
}