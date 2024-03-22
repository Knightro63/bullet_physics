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

import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * IDebugDraw interface class allows hooking up a debug renderer to visually debug
 * simulations.<p>
 * 
 * Typical use case: create a debug drawer object, and assign it to a {@link CollisionWorld}
 * or {@link DynamicsWorld} using setDebugDrawer and call debugDrawWorld.<p>
 * 
 * A class that implements the IDebugDraw interface has to implement the drawLine
 * method at a minimum.
 * 
 * @author jezek2
 */
abstract class IDebugDraw {
	
	//final BulletStack stack = BulletStack.get();

	void drawLine(Vector3 from, Vector3 to, Vector3 color);
	
	// void drawTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 n0, Vector3 n1, Vector3 n2, Vector3 color, double alpha) {
	// 	drawTriangle(v0, v1, v2, color, alpha);
	// }
	
	void drawTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 color, double alpha) {
		drawLine(v0, v1, color);
		drawLine(v1, v2, color);
		drawLine(v2, v0, color);
	}

	void drawContactPoint(Vector3 pointOnB, Vector3 normalOnB, double distance, int lifeTime, Vector3 color);

	void reportErrorWarning(String warningString);

	void draw3dText(Vector3 location, String textString);

	void setDebugMode(int debugMode);

	int getDebugMode();

	void drawAabb(Vector3 from, Vector3 to, Vector3 color) {
		Vector3 halfExtents = Vector3.copy(to);
		halfExtents.sub(from);
		halfExtents.scale(0.5);

		Vector3 center = Vector3.copy(to);
		center.add(from);
		center.scale(0.5);

		int i, j;

		Vector3 edgecoord = Vector3.zero();
		edgecoord.setValues(1, 1, 1);
		Vector3 pa = Vector3.zero(), pb = Vector3.zero();
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 3; j++) {
				pa.setValues(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y, edgecoord.z * halfExtents.z);
				pa.add(center);

				int othercoord = j % 3;

				VectorUtil.mulCoord(edgecoord, othercoord, -1);
				pb.setValues(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y, edgecoord.z * halfExtents.z);
				pb.add(center);

				drawLine(pa, pb, color);
			}
			edgecoord.setValues(-1, -1, -1);
			if (i < 3) {
				VectorUtil.mulCoord(edgecoord, i, -1);
			}
		}
	}
}
