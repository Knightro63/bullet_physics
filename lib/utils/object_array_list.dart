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

	T? removeAt(int index) {
    T? temp = array.removeAt(index);
    _size--;
    return temp;
  }
  T? removeLast(){
    _size--;
    return array.removeLast();
  }
	T? remove(T? object) {
    bool temp = array.remove(object);
		_size--;
		return temp?object:null;
  }
  void resize(int newSize){
    array.length = newSize;
  }

	void removeQuick(int index) {
    T? temp = array.removeAt(index);
    if(temp != null){
      _size--;
    }
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

	int indexOf(T o) {
    return array.indexOf(o);
	}
}