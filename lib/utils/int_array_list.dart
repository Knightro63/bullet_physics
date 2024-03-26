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

class IntArrayList {
	List<int> _array = List.filled(16, 0,growable: true);
	int _size = 0;

  int operator [](i) => _array[i];
	
	void add(int value) {
		if (_size == _array.length) {
      _array.add(value);
		}
		
		_array[_size++] = value;
	}

	int? remove(int index) {
		if (index >= _size) throw 'new IndexOutOfBoundsException()';
		int? old = _array.removeAt(index);
		_size--;
		return old;
	}

	int get(int index) {
		if (index >= _size) throw 'new IndexOutOfBoundsException $index, $_size';
		return _array[index];
	}

	void set(int index, int value) {
		if (index >= _size) throw 'new IndexOutOfBoundsException()';
		_array[index] = value;
	}

	int size() {
		return _size;
	}
	
	void clear() {
		_size = 0;
	}

  bool get isNotEmpty => _size > 0;
  bool get isEmpty => _size == 0;

  @override
  String toString() {
    return _array.toString();
  }
}