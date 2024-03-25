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

//import 'package:bullet_physics/linearmath/convexhull/int_ref.dart';

/**
 *
 * @author jezek2
 */
class Int3{

	late int x;
  late int y;
  late int z;

	Int3([int? x, int? y, int? z]) {
		this.x = x ?? 0;
		this.y = y ?? 0;
		this.z = z ?? 0;
	}
	
	Int3.fromInt3(Int3 i) {
		x = i.x;
		y = i.y;
		z = i.z;
	}
	
	void set(int x, int y, int z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	void copy(Int3 i) {
		x = i.x;
		y = i.y;
		z = i.z;
	}
	
	int getCoord(int coord) {
		switch (coord) {
			case 0: return x;
			case 1: return y;
			default: return z;
		}
	}

	void setCoord(int coord, int value) {
		switch (coord) {
			case 0: x = value; break;
			case 1: y = value; break;
			case 2: z = value; break;
		}
	}
	
	bool equals(Int3 i) {
		return (x == i.x && y == i.y && z == i.z);
	}

  int getRefCoord(int coord) {
    return getCoord(coord);
  }
  void setRefCoord(int coord, int value) {
    setCoord(coord, value);
  }
	// IntRef getRef(final int coord) {
	// 	return IntRef() {
	// 		@override
	// 		int get() {
	// 			return getCoord(coord);
	// 		}

	// 		@override
	// 		void set(int value) {
	// 			setCoord(coord, value);
	// 		}
	// 	};
	// }
}
