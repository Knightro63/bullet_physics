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

class Int4 {

	late int x;
  late int y;
  late int z;
  late int w;

	Int4([int? x, int? y, int? z, int? w]) {
		this.x = x ?? 0;
		this.y = y ?? 0;
		this.z = z ?? 0;
		this.w = w ?? 0;
	}

	void set(int x, int y, int z, int w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}
	
	int getCoord(int coord) {
		switch (coord) {
			case 0: return x;
			case 1: return y;
			case 2: return z;
			default: return w;
		}
	}
}
