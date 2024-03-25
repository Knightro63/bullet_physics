/*
 * Dart port of Bullet (c) 2024 @Knightro
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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

import "package:bullet_physics/extras/gimpact/bvh_data_array.dart";
import "package:bullet_physics/linearmath/aabb.dart";

/**
 *
 * @author jezek2
 */
class BvhTreeNodeArray {
	int size = 0;
	List<double> _bound = [];//double[0];
	List<int> _escapeIndexOrDataIndex = [];//int[0];

	void clear() {
		size = 0;
	}

	void resize(int newSize) {
		List<double> newBound = List.filled(newSize*6, 0);//double[newSize*6];
		List<int> newEIODI = List.filled(newSize, 0);//int[newSize];
		_bound = newBound.sublist(0);
    _escapeIndexOrDataIndex = newEIODI.sublist(0);
		
		_bound = newBound;
		_escapeIndexOrDataIndex = newEIODI;
		
		size = newSize;
	}
	
	void setFromTreeNode(int destIdx, BvhTreeNodeArray array, int srcIdx) {
		int dpos = destIdx*6;
		int spos = srcIdx*6;
		
		_bound[dpos+0] = array._bound[spos+0];
		_bound[dpos+1] = array._bound[spos+1];
		_bound[dpos+2] = array._bound[spos+2];
		_bound[dpos+3] = array._bound[spos+3];
		_bound[dpos+4] = array._bound[spos+4];
		_bound[dpos+5] = array._bound[spos+5];
		_escapeIndexOrDataIndex[destIdx] = array._escapeIndexOrDataIndex[srcIdx];
	}

	void setFromData(int destIdx, BvhDataArray array, int srcIdx) {
		int dpos = destIdx*6;
		int spos = srcIdx*6;
		
		_bound[dpos+0] = array.bound[spos+0];
		_bound[dpos+1] = array.bound[spos+1];
		_bound[dpos+2] = array.bound[spos+2];
		_bound[dpos+3] = array.bound[spos+3];
		_bound[dpos+4] = array.bound[spos+4];
		_bound[dpos+5] = array.bound[spos+5];
		_escapeIndexOrDataIndex[destIdx] = array.data[srcIdx];
	}
	
	AABB getBound(int nodeIndex, AABB out) {
		int pos = nodeIndex*6;
		out.min.setValues(_bound[pos+0], _bound[pos+1], _bound[pos+2]);
		out.max.setValues(_bound[pos+3], _bound[pos+4], _bound[pos+5]);
		return out;
	}
	
	void setBound(int nodeIndex, AABB aabb) {
		int pos = nodeIndex*6;
		_bound[pos+0] = aabb.min.x;
		_bound[pos+1] = aabb.min.y;
		_bound[pos+2] = aabb.min.z;
		_bound[pos+3] = aabb.max.x;
		_bound[pos+4] = aabb.max.y;
		_bound[pos+5] = aabb.max.z;
	}
	
	bool isLeafNode(int nodeIndex) {
		// skipindex is negative (internal node), triangleindex >=0 (leafnode)
		return (_escapeIndexOrDataIndex[nodeIndex] >= 0);
	}

	int getEscapeIndex(int nodeIndex) {
		//btAssert(m_escapeIndexOrDataIndex < 0);
		return -_escapeIndexOrDataIndex[nodeIndex];
	}

	void setEscapeIndex(int nodeIndex, int index) {
		_escapeIndexOrDataIndex[nodeIndex] = -index;
	}

	int getDataIndex(int nodeIndex) {
		//btAssert(m_escapeIndexOrDataIndex >= 0);
		return _escapeIndexOrDataIndex[nodeIndex];
	}

	void setDataIndex(int nodeIndex, int index) {
		_escapeIndexOrDataIndex[nodeIndex] = index;
	}
	
}
