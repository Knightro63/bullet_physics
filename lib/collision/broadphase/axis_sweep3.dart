/*
 * Dart port of Bullet (c) 2024 @Knightro63
 * 
 * AxisSweep3
 * Copyright (c) 2006 Simon Hobbs
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

import 'package:bullet_physics/collision/broadphase/axis_sweep3_internal.dart';
import 'package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart';
import 'package:vector_math/vector_math.dart';

/**
 * AxisSweep3 is an efficient implementation of the 3D axis sweep and prune broadphase.<p>
 * 
 * It uses arrays rather then lists for storage of the 3 axis. Also it operates using 16 bit
 * integer coordinates instead of doubles. For large worlds and many objects, use {@link AxisSweep3_32}
 * instead. AxisSweep3_32 has higher precision and allows more than 16384 objects at the cost
 * of more memory and bit of performance.
 *
 * @author jezek2
 */
class AxisSweep3 extends AxisSweep3Internal {
	
	AxisSweep3(Vector3 worldAabbMin, Vector3 worldAabbMax, [int maxHandles = 16384, OverlappingPairCache? pairCache/* = 0*/]):super(worldAabbMin, worldAabbMax, 0xfffe, 0xffff, maxHandles, pairCache){
		// 1 handle is reserved as sentinel
		assert (maxHandles > 1 && maxHandles < 32767);
	}
	
	@override
	EdgeArray createEdgeArray(int size) {
		return _EdgeArrayImpl(size);
	}

	@override
	Handle createHandle() {
		return _HandleImpl();
	}
	
	int getMask() {
		return 0xFFFF;
	}
}

class _EdgeArrayImpl extends EdgeArray {
  late List<int> _pos;
  late List<int> _handle;

  _EdgeArrayImpl(int size) {
    _pos = List.filled(size, 0);
    _handle = List.filled(size, 0);
  }
  
  @override
  void swap(int idx1, int idx2) {
    int tmpPos = _pos[idx1];
    int tmpHandle = _handle[idx1];
    
    _pos[idx1] = _pos[idx2];
    _handle[idx1] = _handle[idx2];
    
    _pos[idx2] = tmpPos;
    _handle[idx2] = tmpHandle;
  }
  
  @override
  void set(int dest, int src) {
    _pos[dest] = _pos[src];
    _handle[dest] = _handle[src];
  }
  
  @override
  int getPos(int index) {
    return _pos[index] & 0xFFFF;
  }

  @override
  void setPos(int index, int value) {
    _pos[index] = value;
  }

  @override
  int getHandle(int index) {
    return _handle[index] & 0xFFFF;
  }

  @override
  void setHandle(int index, int value) {
    _handle[index] = value;
  }
}

class _HandleImpl extends Handle {
  int _minEdges0 = 0;
  int _minEdges1 = 0;
  int _minEdges2 = 0;

  int _maxEdges0 = 0;
  int _maxEdges1 = 0;
  int _maxEdges2 = 0;
  
  @override
  int getMinEdges(int edgeIndex) {
    switch (edgeIndex) {
      case 0: return _minEdges0 & 0xFFFF;
      case 1: return _minEdges1 & 0xFFFF;
      case 2: return _minEdges2 & 0xFFFF;
      default: return 0;
    }
  }
  
  @override
  void setMinEdges(int edgeIndex, int value) {
    switch (edgeIndex) {
      case 0: _minEdges0 = value; break;
      case 1: _minEdges1 = value; break;
      case 2: _minEdges2 = value; break;
    }
  }
  
  @override
  int getMaxEdges(int edgeIndex) {
    switch (edgeIndex) {
      case 0: return _maxEdges0 & 0xFFFF;
      case 1: return _maxEdges1 & 0xFFFF;
      case 2: return _maxEdges2 & 0xFFFF;
      default: return 0;
    }
  }
  
  @override
  void setMaxEdges(int edgeIndex, int value) {
    switch (edgeIndex) {
      case 0: _maxEdges0 = value; break;
      case 1: _maxEdges1 = value; break;
      case 2: _maxEdges2 = value; break;
    }
  }
}