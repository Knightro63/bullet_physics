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

import 'dart:typed_data';
import 'package:bullet_physics/bullet_physics.dart';
import 'package:bullet_physics/collision/broadphase/broadphase_pair.dart';
import 'package:bullet_physics/utils/int_array_list.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';

extension M on Map{
  V? get<K,V>(K i){
    return this[i];
  }
  void set<K,V>(int i, int v){
    // Insertion sort. This way the diff will have linear complexity.
    final key = i;//getKey(i, j);
    final current = this;
    int index = 0;
    while (current[index] != null && key > current[index]!) {
      index++;
    }
    if (key == current[index]) {
      return; // Pair was already added
    }
    for (int j = current.length - 1; j >= index; j--) {
      current[j + 1] = current[j]!;
    }
    current[index] = key;
  }
}

extension F64L on Float64List{
  double get(int index){
    return this[index];
  }
}

extension I64L on IntArrayList{
  int get(int index){
    return this[index];
  }
}

extension L<T> on List<T>{
  void set(int index, T value){
    this[index] = value;
  }
  void setQuick(int index, T value){
    this[index] = value;
  }
  T getQuick(int index){
    return this[index];
  }
  T get(int index){
    return this[index];
  }
  // int get capacity => length;
  // int get size => length;
	// T remove2(int index) {
	// 	if (index < 0 || index >= length) throw 'IndexOutOfBoundsException()';
	// 	T prev = this[index];
	// 	//System.arraycopy(array, index+1, array, index, size-index-1);
  //   List.copyRange(this,index,this,length-index-1);
	// 	//this[length-1] = null;
	// 	length--;
	// 	return prev;
  // }
}

class MiscUtil {
	// static int getListCapacityForHash(List list) {
	// 	return getListCapacityForHash(list.length);
	// }
	
	// static int getListCapacityForHash(int size) {
	// 	int n = 2;
	// 	while (n < size) {
	// 		n <<= 1;
	// 	}
	// 	return n;
	// }

	/**
	 * Ensures valid index in provided list by filling list with provided values
	 * until the index is valid.
	 */
	static void ensureIndex<T>(List<T> list, int index, T value) {
		while (list.length <= index) {
			list.add(value);
		}
	}
	
	/**
	 * Resizes list to exact size, filling with given value when expanding.
	 */
	static void resize<T>(List<T?> list, int size, T? value) {
   if(list.length > size || value == null){
      list.length = size;
    }
		else if(list.length < size) {
      for(int i = list.length;i < size; i++){
			  list.add(value);
      }
		}
	}
	/**
	 * Resizes list to exact size, filling with new instances of given class type
	 * when expanding.
	 */
	static void resizeObjectArray<T>(ObjectArrayList<T> list, int size, Type? valueCls) {
		try {
			while (list.size < size) {
        if(valueCls == Element){
          list.add(new Element() as T);//
        }
        else if(valueCls == Vector3){
          list.add(new Vector3.zero() as T);//
        }
        else if(valueCls == OptimizedBvhNode){
          list.add(new OptimizedBvhNode() as T);//
        }
        else if(valueCls == Node){
          list.add(new Node() as T);//
        }
        else if(valueCls == BroadphasePair){
          list.add(new BroadphasePair() as T);//
        }
        else{
          list.add(null);//
        }
				
			}
			while (list.size > size) {
				list.removeQuick(list.size - 1);
			}
		}
		catch (e) {
			throw '$e';
		}
	}
	/**
	 * Resizes list to exact size, filling with given value when expanding.
	 */
	static void resizeArray(IntArrayList list, int size, int value) {
		while (list.size() < size) {
			list.add(value);
		}
		while (list.size() > size) {
			list.remove(list.size() - 1);
		}
	}
	static void resizeMap<T>(Map map, int size, T? value) {
   if(map.length > size || value == null){
      List list = map.keys.toList();
      for(int i = list.length;i < size; i++){
			  map.remove(map[list[i]]);
      }
    }
		else if(map.length < size) {
      for(int i = map.length;i < size; i++){
			  map[i] = value;
      }
		}
	}
	/**
	 * Searches object in array.
	 * 
	 * @return first index of match, or -1 when not found
	 */
	static int indexOf<T>(List<T> array, T obj) {
		for (int i=0; i<array.length; i++) {
			if (array[i] == obj) return i;
		}
		return -1;
	}
	
	static double genClamped(double a, double lb, double ub) {
		return a < lb ? lb : (ub < a ? ub : a);
	}

	static void _downHeap<T>(List<T> pArr, int k, int n, Comparator<T> comparator) {
		/*  PRE: a[k+1..N] is a heap */
		/* POST:  a[k..N]  is a heap */

		T temp = pArr[k - 1];
		/* k has child(s) */
		while (k <= n / 2) {
			int child = 2 * k;

			if ((child < n) && comparator(pArr[child - 1], pArr[child]) < 0) {
				child++;
			}
			/* pick larger child */
			if (comparator(temp, pArr[child - 1]) < 0) {
				/* move child up */
				pArr[k - 1] = pArr[child - 1];
				k = child;
			}
			else {
				break;
			}
		}
		pArr[k - 1] = temp;
	}

	/**
	 * Sorts list using heap sort.<p>
	 * 
	 * Implementation from: http://www.csse.monash.edu.au/~lloyd/tildeAlgDS/Sort/Heap/
	 */
	static void heapSort<T>(List<T> list, Comparator<T> comparator) {
		/* sort a[0..N-1],  N.B. 0 to N-1 */
		int k;
		int n = list.length;
		for (k = n ~/ 2; k > 0; k--) {
			_downHeap(list, k, n, comparator);
		}

		/* a[1..N] is now a heap */
		while (n >= 1) {
			_swap(list, 0, n - 1); /* largest of a[0..n-1] */

			n = n - 1;
			/* restore a[1..i-1] heap */
			_downHeap(list, 1, n, comparator);
		}
	}

	static void _swap<T>(List<T> list, int index0, int index1) {
		T temp = list[index0];
		list[index0] = list[index1];
		list[index1] = temp;
	}
	
	/**
	 * Sorts list using quick sort.<p>
	 */
	static void quickSort<T>(List<T?> list, Comparator<T?> comparator) {
		// don't sort 0 or 1 elements
		if (list.length > 1) {
			_quickSortInternal(list, comparator, 0, list.length - 1);
		}
	}
	/**
	 * Sorts list using quick sort.<p>
	 */
	static void quickSortObjectArray<T>(ObjectArrayList<T?> list, Comparator<T?> comparator) {
		// don't sort 0 or 1 elements
		if (list.size > 1) {
			_quickSortInternalObjectArray(list, comparator, 0, list.size - 1);
		}
	}
	static void _quickSortInternalObjectArray<T>(ObjectArrayList<T?> list, Comparator<T?> comparator, int lo, int hi) {
		// lo is the lower index, hi is the upper index
		// of the region of array a that is to be sorted
		int i = lo, j = hi;
		T? x = list.getQuick(((lo + hi) / 2).floor());

		// partition
		do {
			while (comparator(list.getQuick(i), x) < 0) {
        i++;
      }
			while (comparator(x, list.getQuick(j)) < 0){
        j--;
      }
			
			if (i <= j) {
				_swapObjectArray(list, i, j);
				i++;
				j--;
			}
		}
		while (i <= j);

		// recursion
		if (lo < j) {
			_quickSortInternalObjectArray(list, comparator, lo, j);
		}
		if (i < hi) {
			_quickSortInternalObjectArray(list, comparator, i, hi);
		}
	}
	static  void _swapObjectArray<T>(ObjectArrayList<T> list, int index0, int index1) {
		final T? temp = list.getQuick(index0);
		list.setQuick(index0, list.getQuick(index1));
		list.setQuick(index1, temp);
	}
	static void _quickSortInternal<T>(List<T> list, Comparator<T> comparator, int lo, int hi) {
		// lo is the lower index, hi is the upper index
		// of the region of array a that is to be sorted
		int i = lo, j = hi;
		T x = list[((lo + hi) / 2).floor()];

		// partition
		do {
			while (comparator(list[i], x) < 0){ i++;}
			while (comparator(x, list[j]) < 0){ j--;}
			
			if (i <= j) {
				_swap(list, i, j);
				i++;
				j--;
			}
      
		}
		while (i <= j);

		// recursion
		if (lo < j) {
			_quickSortInternal(list, comparator, lo, j);
		}
		if (i < hi) {
			_quickSortInternal(list, comparator, i, hi);
		}
	}
}
