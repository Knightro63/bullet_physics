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

enum BroadphaseNativeType {
	
	// polyhedral convex shapes:
	boxShapeProxyType,
	triangleShapeProxyType,
	tetrahedralShapeProxyType,
	convexTriangleMeshShapeProxytype,
	convexHullShapeProxytype,
	
	// implicit convex shapes:
	implicitConvexShapesStartHere,
	sphereShapeProxytype,
	multiSphereShapeProxytype,
	capsuleShapeProxytype,
	coneShapeProxytype,
	convexShapeProxytype,
	cylinderShapeProxytype,
	uniformScalingShapeProxytype,
	minkowskiSumShapeProxytype,
	minkowskiDifferenceShapeProxytype,
	
	// concave shapes:
	concaveShapesStartHere,
	
	// keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
	triangleMeshShapeProxytype,
	scaledTriangleMeshShapeProxytype,
	
	// used for demo integration FAST/Swift collision library and Bullet:
	fastConcaveMeshProxytype,
	
	// terrain:
	terrainShapeProxytype,
	
	// used for GIMPACT Trimesh integration:
	gimpactShapeProxytype,
	
	// multimaterial mesh:
	multimaterialTriangleMeshProxytoye,
	
	emptyShapeProxytype,
	staticPlaneProxytype,
	concaveShapesEndHere,
	compoundShapeProxyType,
	
	softbodyShapeProxytype,
	invalidShapeProxytype,
	maxBroadphaseCollisionTypes;
	
	//static List<BroadphaseNativeType> _values = values();
	
	static BroadphaseNativeType forValue(int value) {
		return values[value];
	}
	
	bool isPolyhedral() {
		return (index < implicitConvexShapesStartHere.index);
	}

	bool isConvex() {
		return (index < concaveShapesStartHere.index);
	}

	bool isConcave() {
		return ((index > concaveShapesStartHere.index) &&
				(index < concaveShapesEndHere.index));
	}

	bool isCompound() {
		return (index == compoundShapeProxyType.index);
	}

	bool isInfinite() {
		return (index == staticPlaneProxytype.index);
	}
	
}
