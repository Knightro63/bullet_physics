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

import "package:bullet_physics/collision/shapes/static_plane_shape.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class PlaneShape {
	static void getPlaneEquation(StaticPlaneShape shape, Vector4 equation) {
		Vector3 tmp = Vector3.zero();
		equation.setFromVector3(shape.getPlaneNormal(tmp));
		equation.w = shape.getPlaneConstant();
	}
	
	static void getPlaneEquationTransformed(StaticPlaneShape shape, Transform? trans, Vector4 equation) {
		getPlaneEquation(shape, equation);
		Vector3 tmp = Vector3.zero();
		trans?.basis.getRowWith(0, tmp);
		double x = VectorUtil.dot3(tmp, equation);
		trans?.basis.getRowWith(1, tmp);
		double y = VectorUtil.dot3(tmp, equation);
		trans?.basis.getRowWith(2, tmp);
		double z = VectorUtil.dot3(tmp, equation);
		double w = VectorUtil.dot3(trans?.origin, equation) + equation.w;
		equation.setValues(x, y, z, w);
	}
}
