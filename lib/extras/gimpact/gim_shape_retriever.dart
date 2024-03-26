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

import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/extras/gimpact/g_impact_shape_interface.dart";
import "package:bullet_physics/extras/gimpact/tetrahedron_shape_ex.dart";
import "package:bullet_physics/extras/gimpact/triangle_shape_ex.dart";

class GIM_ShapeRetriever {
	GImpactShapeInterface? gimShape;
	TriangleShapeEx trishape = TriangleShapeEx();
	TetrahedronShapeEx tetrashape = TetrahedronShapeEx();

	ChildShapeRetriever childRetriever = ChildShapeRetriever();
	TriangleShapeRetriever triRetriever = TriangleShapeRetriever();
	TetraShapeRetriever tetraRetriever = TetraShapeRetriever();
	ChildShapeRetriever? currentRetriever;

	GIM_ShapeRetriever(this.gimShape) {
		// select retriever
		if (gimShape?.needsRetrieveTriangles() ?? false) {
			currentRetriever = triRetriever;
		}
		else if (gimShape?.needsRetrieveTetrahedrons() ?? false) {
			currentRetriever = tetraRetriever;
		}
		else {
			currentRetriever = childRetriever;
		}

		currentRetriever?.parent = this;
	}

	CollisionShape? getChildShape(int index) {
		return currentRetriever?.getChildShape(index);
	}	
}
////////////////////////////////////////////////////////////////////////////

class ChildShapeRetriever {
  GIM_ShapeRetriever? parent;

  ChildShapeRetriever([this.parent]);

  CollisionShape? getChildShape(int index) {
    return parent?.gimShape?.getChildShape(index);
  }
}

class TriangleShapeRetriever extends ChildShapeRetriever {
  @override
  CollisionShape? getChildShape(int index) {
    parent?.gimShape?.getBulletTriangle(index, parent?.trishape);
    return parent?.trishape;
  }
}

class TetraShapeRetriever extends ChildShapeRetriever {
  @override
  CollisionShape? getChildShape(int index) {
    parent?.gimShape?.getBulletTetrahedron(index, parent?.tetrashape);
    return parent?.tetrashape;
  }
}
