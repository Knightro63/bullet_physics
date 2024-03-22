/*
 * Dart port of Bullet (c) 2024 @Knightro63
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

import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class GImpactMassUtil {
	static Vector3 getPointInertia(Vector3 point, double mass, Vector3 out) {
		double x2 = point.x * point.x;
		double y2 = point.y * point.y;
		double z2 = point.z * point.z;
		out.setValues(mass * (y2 + z2), mass * (x2 + z2), mass * (x2 + y2));
		return out;
	}
}
