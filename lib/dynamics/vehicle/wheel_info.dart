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

import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/dynamics/vehicle/wheel_info_construction_info.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class WheelInfo {
	//final BulletStack stack = BulletStack.get();
	final RaycastInfo raycastInfo = RaycastInfo();

	final Transform worldTransform = Transform();
	
	final Vector3 chassisConnectionPointCS = Vector3.zero(); // const
	final Vector3 wheelDirectionCS = Vector3.zero(); // const
	final Vector3 wheelAxleCS = Vector3.zero(); // const or modified by steering
	late double suspensionRestLength1; // const
	late double maxSuspensionTravelCm;
	late double maxSuspensionForce;
	late double wheelsRadius; // const
	late double suspensionStiffness; // const
	late double wheelsDampingCompression; // const
	late double wheelsDampingRelaxation; // const
	late double frictionSlip;
	double steering = 0;
	double rotation = 0;
	double deltaRotation = 0;
	double rollInfluence = 0.1;

	double engineForce = 0;

	double brake = 0;
	
	late bool bIsFrontWheel;
	
	Object? clientInfo; // can be used to store pointer to sync transforms...

	double clippedInvContactDotSuspension = 0;
	double suspensionRelativeVelocity = 0;
	// calculated by suspension
	double wheelsSuspensionForce = 0;
	double skidInfo = 0;
	
	WheelInfo(WheelInfoConstructionInfo ci) {
		suspensionRestLength1 = ci.suspensionRestLength;
		maxSuspensionTravelCm = ci.maxSuspensionTravelCm;
    maxSuspensionForce = ci.maxSuspensionForce;

		wheelsRadius = ci.wheelRadius;
		suspensionStiffness = ci.suspensionStiffness;
		wheelsDampingCompression = ci.wheelsDampingCompression;
		wheelsDampingRelaxation = ci.wheelsDampingRelaxation;
		chassisConnectionPointCS.setFrom(ci.chassisConnectionCS);
		wheelDirectionCS.setFrom(ci.wheelDirectionCS);
		wheelAxleCS.setFrom(ci.wheelAxleCS);
		frictionSlip = ci.frictionSlip;
		bIsFrontWheel = ci.bIsFrontWheel;
	}
	
	double get getSuspensionRestlength => suspensionRestLength1;
	

	void updateWheel(RigidBody chassis, RaycastInfo raycastInfo) {
		if (raycastInfo.isInContact) {
			double project = raycastInfo.contactNormalWS.dot(raycastInfo.wheelDirectionWS);
			Vector3 chassisVelocityAtContactPoint = Vector3.zero();
			Vector3 relpos = Vector3.zero();
			relpos.sub2(raycastInfo.contactPointWS, chassis.getCenterOfMassPosition(Vector3.zero()));
			chassis.getVelocityInLocalPoint(relpos, chassisVelocityAtContactPoint);
			double projVel = raycastInfo.contactNormalWS.dot(chassisVelocityAtContactPoint);
			if (project >= -0.1) {
				suspensionRelativeVelocity = 0;
				clippedInvContactDotSuspension = 1 / 0.1;
			}
			else {
				double inv = -1 / project;
				suspensionRelativeVelocity = projVel * inv;
				clippedInvContactDotSuspension = inv;
			}
		}
		else {
			// Not in contact : position wheel in a nice (rest length) position
			raycastInfo.suspensionLength = getSuspensionRestlength;
			suspensionRelativeVelocity = 0;
			raycastInfo.contactNormalWS.negateFrom(raycastInfo.wheelDirectionWS);
			clippedInvContactDotSuspension = 1;
		}
	}
}


////////////////////////////////////////////////////////////////////////////

class RaycastInfo {
  // set by raycaster
  final Vector3 contactNormalWS = Vector3.zero(); // contactnormal
  final Vector3 contactPointWS = Vector3.zero(); // raycast hitpoint
  double suspensionLength = 0;
  final Vector3 hardPointWS = Vector3.zero(); // raycast starting point
  final Vector3 wheelDirectionWS = Vector3.zero(); // direction in worldspace
  final Vector3 wheelAxleWS = Vector3.zero(); // axle in worldspace
  bool isInContact = false;
  Object? groundObject; // could be general void* ptr
}