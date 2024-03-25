/*
 * Dart port of Bullet (c) 2024 @Knightro
 *
 * Stan Melax Convex Hull Computation
 * Copyright (c) 2024 Stan Melax http://www.melax.com/
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

import 'package:bullet_physics/linearmath/convexhull/int3.dart';

/**
 *
 * @author jezek2
 */
class Tri extends Int3{
	
	late Int3 n = Int3(-1, -1, -1);
	int id = 0;
	int vmax = -1;
	double rise = 0;

	Tri([int? a, int? b, int? c]):super(a, b, c);

	static int _er = -1;
	
  int getRef(){
    return _er;
  }
  
  void setRef(int value) {
    _er = value;
  }
  
	// static IntRef _erRef = IntRef() {
	// 	@override
	// 	int get() {
	// 		return _er;
	// 	}

  //   @override
  //   void set(int value) {
  //     _er = value;
  //   }
	// };
	
	int neib(int a, int b) {
		for (int i = 0; i < 3; i++) {
			int i1 = (i + 1) % 3;
			int i2 = (i + 2) % 3;
			
			if (getCoord(i) == a && getCoord(i1) == b) {
				return n.getRefCoord(i2);
			}
			if (getCoord(i) == b && getCoord(i1) == a) {
				return n.getRefCoord(i2);
			}
		}
		assert(false);
		return _er;
	}
}
