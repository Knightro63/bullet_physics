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


final class ObjectArrayList<T>{

	late List<T?> array;
	int _size = 0;

	ObjectArrayList([int initialCapacity = 16]) {
		array = List.filled(initialCapacity, null, growable: true);
	}

  bool get isNotEmpty => _size > 0;
  bool get isEmpty => _size == 0;
  set size(int v ) => _size = v;

  T? operator [](i) => array[i];
  void operator []=(int i,T? v)=>(){
    array[i] = v;
  };
	
	bool add(T? value) {
		if (_size == array.length) {
			//_expand();
      array.add(value);
		}
    array[_size++] = value;
		return true;
	}

  bool contains(T? element){
    return array.contains(element);
  }

  void reverse(){
    array = array.reversed.toList();
  }

	void addAt(int index, T value) {
		// if (_size == array.length) {
		// 	_expand();
		// }

		int num = _size - index;
		if (num > 0) {
			//System.arraycopy(array, index, array, index+1, num);
      array = array..replaceRange(index, index+1+num, array.sublist(index+1,index+1+num));
		}

		array[index] = value;
		_size++;
	}

	T? removeAt(int index) {
		_size--;
    return array.removeAt(index);
  }
	T? remove(T? object) {
		_size--;
		return array.remove(object)?object:null;
  }
  void resize(int newSize){
    array.length = newSize;
  }
	void _expand() {
		List<T?> newArray = List.filled(array.length << 1, null, growable: true);//(T[])new Object[array.length << 1];
		//System.arraycopy(array, 0, newArray, 0, array.length);
    if(newArray.isNotEmpty){
		  array = newArray..replaceRange(0, array.length-1, array);
    }
    else{
      array = newArray;
    }
	}

	void removeQuick(int index) {
    _size--;
    array.removeAt(index);
	}

	T? get(int index) {
		if (index >= _size) throw 'new IndexOutOfBoundsException()';
		return array[index];
	}

	T? getQuick(int index) {
		return array[index];
	}

	T? set(int index, T value) {
		if (index >= _size) throw 'new IndexOutOfBoundsException()';
		T? old = array[index];
		array[index] = value;
		return old;
	}

	void setQuick(int index, T? value) {
		array[index] = value;
	}

	int get size => _size;
	int get capacity => array.length;
	
	void clear() {
		_size = 0;
	}

	int indexOf(Object? o) {
		int _size = size;
		List<T?> _array = array;
		for (int i=0; i<_size; i++) {
			if (o == null? _array[i] == null : o == _array[i]) {
				return i;
			}
		}
		return -1;
	}
	// int indexOf(T o) {
  //   return array.indexOf(o);
	// }
}