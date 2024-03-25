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

import 'package:bullet_physics/collision/shapes/optimized_bvh.dart';
import 'dart:math';

class QuantizedBvhNodes{ // implements Serializable 
	//static const int _serialVersionUID = 1;
	static const int _stride = 4; // 16 bytes
	
	late List<int> _buf;
	int _size = 0;

	QuantizedBvhNodes() {
		_buf = List.filled(16*_stride, 0);//resize(16);
	}
	
	int add() {
		while (_size+1 >= capacity()) {
			resize(capacity()*2);
		}
		return _size++;
	}
	
	int get length => _size;

	
	int capacity() {
		return (_buf.length) ~/ _stride;
	}
	
	void clear() {
		_size = 0;
	}
	
	void resize(int num) {
		List<int> oldBuf = _buf.sublist(0);
		_buf = List.filled(num*_stride, 0);
    _buf.length = min(oldBuf.length, _buf.length);
	}
	
	static int getNodeSize() {
		return _stride*4;
	}
	
	void set(int destId, QuantizedBvhNodes srcNodes, int srcId) {
		assert (_stride == 4);

		// save field access:
		List<int> buf = _buf;
		List<int> srcBuf = srcNodes._buf;
		
		buf[destId*_stride+0] = srcBuf[srcId*_stride+0];
		buf[destId*_stride+1] = srcBuf[srcId*_stride+1];
		buf[destId*_stride+2] = srcBuf[srcId*_stride+2];
		buf[destId*_stride+3] = srcBuf[srcId*_stride+3];
	}
	
	void swap(int id1, int id2) {
		assert (_stride == 4);
		
		// save field access:
		List<int> buf = _buf;
		
		int temp0 = buf[id1*_stride+0];
		int temp1 = buf[id1*_stride+1];
		int temp2 = buf[id1*_stride+2];
		int temp3 = buf[id1*_stride+3];
		
		buf[id1*_stride+0] = buf[id2*_stride+0];
		buf[id1*_stride+1] = buf[id2*_stride+1];
		buf[id1*_stride+2] = buf[id2*_stride+2];
		buf[id1*_stride+3] = buf[id2*_stride+3];
		
		buf[id2*_stride+0] = temp0;
		buf[id2*_stride+1] = temp1;
		buf[id2*_stride+2] = temp2;
		buf[id2*_stride+3] = temp3;
	}
	
	int getQuantizedAabbMinWithIndex(int nodeId, int index) {
		switch (index) {
			case 0: return (_buf[nodeId*_stride+0]) & 0xFFFF;
			case 1: return (_buf[nodeId*_stride+0] >>> 16) & 0xFFFF;
			case 2: return (_buf[nodeId*_stride+1]) & 0xFFFF;
      default: return 0;
		}
	}

	int getQuantizedAabbMin(int nodeId) {
		return (_buf[nodeId*_stride+0] & 0xFFFFFFFF) | ((_buf[nodeId*_stride+1] & 0xFFFF) << 32);
	}

	void setQuantizedAabbMin(int nodeId, int value) {
		_buf[nodeId*_stride+0] = value;
		setQuantizedAabbMinWithIndex(nodeId, 2, ((value & 0xFFFF00000000) >>> 32));
	}
	void setQuantizedAabbMinWithIndex(int nodeId, int index, int value) {
		switch (index) {
			case 0: _buf[nodeId*_stride+0] = (_buf[nodeId*_stride+0] & 0xFFFF0000) | (value & 0xFFFF); break;
			case 1: _buf[nodeId*_stride+0] = (_buf[nodeId*_stride+0] & 0x0000FFFF) | ((value & 0xFFFF) << 16); break;
			case 2: _buf[nodeId*_stride+1] = (_buf[nodeId*_stride+1] & 0xFFFF0000) | (value & 0xFFFF); break;
		}
	}

	int getQuantizedAabbMaxWithIndex(int nodeId, int index) {
		switch (index) {
			case 0: return (_buf[nodeId*_stride+1] >>> 16) & 0xFFFF;
			case 1: return (_buf[nodeId*_stride+2]) & 0xFFFF;
			case 2: return (_buf[nodeId*_stride+2] >>> 16) & 0xFFFF;
      default: return 0;
		}
	}

	int getQuantizedAabbMax(int nodeId) {
		return ((_buf[nodeId*_stride+1] & 0xFFFF0000) >>> 16) | ((_buf[nodeId*_stride+2] & 0xFFFFFFFF) << 16);
	}
	void setQuantizedAabbMax(int nodeId, int value) {
		setQuantizedAabbMaxWithIndex(nodeId, 0, value);
		_buf[nodeId*_stride+2] = (value >>> 16);
	}
	void setQuantizedAabbMaxWithIndex(int nodeId, int index, int value) {
		switch (index) {
			case 0: _buf[nodeId*_stride+1] = (_buf[nodeId*_stride+1] & 0x0000FFFF) | ((value & 0xFFFF) << 16); break;
			case 1: _buf[nodeId*_stride+2] = (_buf[nodeId*_stride+2] & 0xFFFF0000) | (value & 0xFFFF); break;
			case 2: _buf[nodeId*_stride+2] = (_buf[nodeId*_stride+2] & 0x0000FFFF) | ((value & 0xFFFF) << 16); break;
		}
	}
	
	int getEscapeIndexOrTriangleIndex(int nodeId) {
		return _buf[nodeId*_stride+3];
	}
	
	void setEscapeIndexOrTriangleIndex(int nodeId, int value) {
		_buf[nodeId*_stride+3] = value;
	}
	
	bool isLeafNode(int nodeId) {
		// skipindex is negative (internal node), triangleindex >=0 (leafnode)
		return (getEscapeIndexOrTriangleIndex(nodeId) >= 0);
	}

	int getEscapeIndex(int nodeId) {
		assert (!isLeafNode(nodeId));
		return -getEscapeIndexOrTriangleIndex(nodeId);
	}

	int getTriangleIndex(int nodeId) {
		assert (isLeafNode(nodeId));
		// Get only the lower bits where the triangle index is stored
		return (getEscapeIndexOrTriangleIndex(nodeId) & ~((~0) << (31 - OptimizedBvh.maxNumPartsInBits)));
	}

	int getPartId(int nodeId) {
		assert (isLeafNode(nodeId));
		// Get only the highest bits where the part index is stored
		return (getEscapeIndexOrTriangleIndex(nodeId) >>> (31 - OptimizedBvh.maxNumPartsInBits)); 
	}
	
	static int getCoord(int vec, int index) {
		switch (index) {
			case 0: return ((vec & 0x00000000FFFF)) & 0xFFFF;
			case 1: return ((vec & 0x0000FFFF0000) >>> 16) & 0xFFFF;
			case 2: return ((vec & 0xFFFF00000000) >>> 32) & 0xFFFF;
      default: return 0;
		}
	}
	
}
