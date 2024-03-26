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

import 'package:bullet_physics/extras/gimpact/pair.dart';

class PairSet {
	List<Pair> _array = [];
	int _size = 0;
	
	PairSet() {
    for(int i = 0; i < 32; i++){
		  _array.add(Pair());
    }
	}
	
	void clear() {
		_size = 0;
	}
	
	int get length => _size;
	
	
	Pair get(int index) {
		if (index >= _size) throw'IndexOutOfBoundsException()';
		return _array[index];
	}
	
	void _expand() {
		List<Pair> newArray = List.filled(_array.length << 1, Pair());
    _array = newArray.sublist(0);
		// System.arraycopy(array, 0, newArray, 0, array.length);
		// _array = newArray;
	}

	void pushPair(int index1, int index2) {
		if (_size == _array.length) {
			_expand();
		}
		_array[_size].index1 = index1;
		_array[_size].index2 = index2;
		_size++;
	}

	void pushPairInv(int index1, int index2) {
		if (_size == _array.length) {
			_expand();
		}
		_array[_size].index1 = index2;
		_array[_size].index2 = index1;
		_size++;
	}
	
}
