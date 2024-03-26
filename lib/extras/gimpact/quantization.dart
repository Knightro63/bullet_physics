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

import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class Quantization {

	static void btCalcQuantizationParameters(Vector3 outMinBound, Vector3 outMaxBound, Vector3 bvhQuantization, Vector3 srcMinBound, Vector3 srcMaxBound, double quantizationMargin) {
		// enlarge the AABB to avoid division by zero when initializing the quantization values
		Vector3 clampValue = Vector3.zero();
		clampValue.setValues(quantizationMargin, quantizationMargin, quantizationMargin);
		outMinBound.sub2(srcMinBound, clampValue);
		outMaxBound.add2(srcMaxBound, clampValue);
		Vector3 aabbSize = Vector3.zero();
		aabbSize.sub2(outMaxBound, outMinBound);
		bvhQuantization.setValues(65535.0, 65535.0, 65535.0);
		VectorUtil.div(bvhQuantization, bvhQuantization, aabbSize);
	}

	static void btQuantizeClamp(List<int> out, Vector3 point, Vector3 minBound, Vector3 maxBound, Vector3 bvhQuantization) {
		Vector3 clampedPoint = Vector3.copy(point);
		VectorUtil.setMax(clampedPoint, minBound);
		VectorUtil.setMin(clampedPoint, maxBound);

		Vector3 v = Vector3.zero();
		v.sub2(clampedPoint, minBound);
		VectorUtil.mul(v, v, bvhQuantization);

		out[0] = (v.x + 0.5).toInt();
		out[1] = (v.y + 0.5).toInt();
		out[2] = (v.z + 0.5).toInt();
	}

	static Vector3 btUnquantize(List<int> vecIn, Vector3 offset, Vector3 bvhQuantization, Vector3 out) {
		out.setValues((vecIn[0] & 0xFFFF) / (bvhQuantization.x),
		        (vecIn[1] & 0xFFFF) / (bvhQuantization.y),
		        (vecIn[2] & 0xFFFF) / (bvhQuantization.z));
		out.add(offset);
		return out;
	}
	
}
