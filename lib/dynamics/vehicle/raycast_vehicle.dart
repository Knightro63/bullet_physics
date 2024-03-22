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

import "dart:typed_data";

import "package:bullet_physics/dynamics/constraintsolver/contact_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/dynamics/vehicle/vehicle_raycaster.dart";
import "package:bullet_physics/dynamics/vehicle/vehicle_raycaster_result.dart";
import "package:bullet_physics/dynamics/vehicle/vehicle_tuning.dart";
import "package:bullet_physics/dynamics/vehicle/wheel_info.dart";
import "package:bullet_physics/dynamics/vehicle/wheel_info_construction_info.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/quaternion_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * Raycast vehicle, very special constraint that turn a rigidbody into a vehicle.
 * 
 * @author jezek2
 */
class RaycastVehicle extends TypedConstraint {
	static RigidBody _sFixedObject = RigidBody();
	static final double _sideFrictionStiffness2 = 1.0;
	
	ObjectArrayList<Vector3> forwardWS = ObjectArrayList();//List<Vector3>();
	ObjectArrayList<Vector3> axle = ObjectArrayList();//List<Vector3>();
	Float64List forwardImpulse = Float64List(0);
	Float64List sideImpulse = Float64List(0);

	// double _tau = 0;
	// double _damping = 0;
	VehicleRaycaster? _vehicleRaycaster;
	double _pitchControl = 0;
	double _steeringValue = 0; 
	double _currentVehicleSpeedKmHour = 0;

	late RigidBody _chassisBody;

	int _indexRightAxis = 0;
	int _indexUpAxis = 2;
	int _indexForwardAxis = 1;
	
	List<WheelInfo> wheelInfo = [];//List<WheelInfo>();

	// constructor to create a car from an existing rigidbody
	RaycastVehicle(VehicleTuning tuning, RigidBody chassis, VehicleRaycaster? raycaster):super(TypedConstraintType.vehicle){
		_vehicleRaycaster = raycaster;
		_chassisBody = chassis;
		_defaultInit(tuning);
	}
	
	void _defaultInit(VehicleTuning tuning) {
		_currentVehicleSpeedKmHour = 0;
		_steeringValue = 0;
	}

	/**
	 * Basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed.
	 */
	WheelInfo addWheel(Vector3 connectionPointCS, Vector3 wheelDirectionCS0, Vector3 wheelAxleCS, double suspensionRestLength, double wheelRadius, VehicleTuning tuning, bool isFrontWheel) {
		WheelInfoConstructionInfo ci = WheelInfoConstructionInfo();

		ci.chassisConnectionCS.setFrom(connectionPointCS);
		ci.wheelDirectionCS.setFrom(wheelDirectionCS0);
		ci.wheelAxleCS.setFrom(wheelAxleCS);
		ci.suspensionRestLength = suspensionRestLength;
		ci.wheelRadius = wheelRadius;
		ci.suspensionStiffness = tuning.suspensionStiffness;
		ci.wheelsDampingCompression = tuning.suspensionCompression;
		ci.wheelsDampingRelaxation = tuning.suspensionDamping;
		ci.frictionSlip = tuning.frictionSlip;
		ci.bIsFrontWheel = isFrontWheel;
		ci.maxSuspensionTravelCm = tuning.maxSuspensionTravelCm;

		wheelInfo.add(WheelInfo(ci));

		WheelInfo wheel = wheelInfo.getQuick(getNumWheels() - 1);

		updateWheelTransformsWS(wheel, false);
		updateWheelTransform(getNumWheels() - 1, false);
		return wheel;
	}

	Transform getWheelTransformWS(int wheelIndex, Transform out) {
		assert (wheelIndex < getNumWheels());
		WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
		out.copy(wheel.worldTransform);
		return out;
	}
	
	void updateWheelTransform(int wheelIndex, [bool interpolatedTransform = true]) {
		WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
		updateWheelTransformsWS(wheel, interpolatedTransform);
		Vector3 up = Vector3.zero();
		up.negateFrom(wheel.raycastInfo.wheelDirectionWS);
		Vector3 right = wheel.raycastInfo.wheelAxleWS;
		Vector3 fwd = Vector3.zero();
		fwd.cross2(up, right);
		fwd.normalize();
		// up = right.cross(fwd);
		// up.normalize();

		// rotate around steering over de wheelAxleWS
		double steering = wheel.steering;

		Quaternion steeringOrn = Quaternion(0,0,0,0);
		QuaternionUtil.setRotation(steeringOrn, up, steering); //wheel.m_steering);
		Matrix3 steeringMat = Matrix3.zero();
		MatrixUtil.setRotation(steeringMat, steeringOrn);

		Quaternion rotatingOrn = Quaternion(0,0,0,0);
		QuaternionUtil.setRotation(rotatingOrn, right, -wheel.rotation);
		Matrix3 rotatingMat = Matrix3.zero();
		MatrixUtil.setRotation(rotatingMat, rotatingOrn);

		Matrix3 basis2 = Matrix3.zero();
		basis2.setRowByValues(0, right.x, fwd.x, up.x);
		basis2.setRowByValues(1, right.y, fwd.y, up.y);
		basis2.setRowByValues(2, right.z, fwd.z, up.z);

		Matrix3 wheelBasis = wheel.worldTransform.basis;
		wheelBasis.mul2(steeringMat, rotatingMat);
		wheelBasis.mul(basis2);

		wheel.worldTransform.origin.scaleAdd(wheel.raycastInfo.suspensionLength, wheel.raycastInfo.wheelDirectionWS, wheel.raycastInfo.hardPointWS);
	}
	
	void resetSuspension() {
		int i;
		for (i = 0; i < wheelInfo.length; i++) {
			WheelInfo wheel = wheelInfo.getQuick(i);
			wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestlength;
			wheel.suspensionRelativeVelocity = 0;

			wheel.raycastInfo.contactNormalWS.negateFrom(wheel.raycastInfo.wheelDirectionWS);
			//wheelInfo.setContactFriction(btScalar(0.0));
			wheel.clippedInvContactDotSuspension = 1;
		}
	}
	
	void updateWheelTransformsWS(WheelInfo wheel, [bool interpolatedTransform = true]) {
		wheel.raycastInfo.isInContact = false;

		Transform chassisTrans = getChassisWorldTransform(Transform());
		if (interpolatedTransform && (getRigidBody().getMotionState() != null)) {
			getRigidBody().getMotionState()?.getWorldTransform(chassisTrans);
		}

		wheel.raycastInfo.hardPointWS.setFrom(wheel.chassisConnectionPointCS);
		chassisTrans.transform(wheel.raycastInfo.hardPointWS);

		wheel.raycastInfo.wheelDirectionWS.setFrom(wheel.wheelDirectionCS);
		chassisTrans.basis.transform(wheel.raycastInfo.wheelDirectionWS);

		wheel.raycastInfo.wheelAxleWS.setFrom(wheel.wheelAxleCS);
		chassisTrans.basis.transform(wheel.raycastInfo.wheelAxleWS);
	}

	double rayCast(WheelInfo wheel) {
		updateWheelTransformsWS(wheel, false);

		double depth = -1;

		double raylen = wheel.getSuspensionRestlength + wheel.wheelsRadius;

		Vector3 rayvector = Vector3.zero();
		rayvector.scaleFrom(raylen, wheel.raycastInfo.wheelDirectionWS);
		Vector3 source = wheel.raycastInfo.hardPointWS;
		wheel.raycastInfo.contactPointWS.add2(source, rayvector);
		Vector3 target = wheel.raycastInfo.contactPointWS;

		double param = 0;

		VehicleRaycasterResult rayResults = VehicleRaycasterResult();

		assert (_vehicleRaycaster != null);

		Object? object = _vehicleRaycaster?.castRay(source, target, rayResults);

		wheel.raycastInfo.groundObject = null;

		if (object != null) {
			param = rayResults.distFraction;
			depth = raylen * rayResults.distFraction;
			wheel.raycastInfo.contactNormalWS.setFrom(rayResults.hitNormalInWorld);
			wheel.raycastInfo.isInContact = true;

			wheel.raycastInfo.groundObject = _sFixedObject; // todo for driving on dynamic/movable objects!;
			//wheel.m_raycastInfo.m_groundObject = object;

			double hitDistance = param * raylen;
			wheel.raycastInfo.suspensionLength = hitDistance - wheel.wheelsRadius;
			// clamp on max suspension travel

			double minSuspensionLength = wheel.getSuspensionRestlength - wheel.maxSuspensionTravelCm * 0.01;
			double maxSuspensionLength = wheel.getSuspensionRestlength + wheel.maxSuspensionTravelCm * 0.01;
			if (wheel.raycastInfo.suspensionLength < minSuspensionLength) {
				wheel.raycastInfo.suspensionLength = minSuspensionLength;
			}
			if (wheel.raycastInfo.suspensionLength > maxSuspensionLength) {
				wheel.raycastInfo.suspensionLength = maxSuspensionLength;
			}

			wheel.raycastInfo.contactPointWS.setFrom(rayResults.hitPointInWorld);

			double denominator = wheel.raycastInfo.contactNormalWS.dot(wheel.raycastInfo.wheelDirectionWS);

			Vector3 chassisVelocityAtContactPoint = Vector3.zero();
			Vector3 relpos = Vector3.zero();
			relpos.sub2(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(Vector3.zero()));

			getRigidBody().getVelocityInLocalPoint(relpos, chassisVelocityAtContactPoint);

			double projVel = wheel.raycastInfo.contactNormalWS.dot(chassisVelocityAtContactPoint);

			if (denominator >= -0.1) {
				wheel.suspensionRelativeVelocity = 0;
				wheel.clippedInvContactDotSuspension = 1 / 0.1;
			}
			else {
				double inv = -1 / denominator;
				wheel.suspensionRelativeVelocity = projVel * inv;
				wheel.clippedInvContactDotSuspension = inv;
			}

		}
		else {
			// put wheel info as in rest position
			wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestlength;
			wheel.suspensionRelativeVelocity = 0;
			wheel.raycastInfo.contactNormalWS.negateFrom(wheel.raycastInfo.wheelDirectionWS);
			wheel.clippedInvContactDotSuspension = 1;
		}

		return depth;
	}
	
	Transform getChassisWorldTransform(Transform out) {
		/*
		if (getRigidBody()->getMotionState())
		{
			btTransform chassisWorldTrans;
			getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
			return chassisWorldTrans;
		}
		*/

		return getRigidBody().getCenterOfMassTransform(out);
	}
	
	void updateVehicle(double step) {
		for (int i = 0; i < getNumWheels(); i++) {
			updateWheelTransform(i, false);
		}
		
		Vector3 tmp = Vector3.zero();

		_currentVehicleSpeedKmHour = 3.6 * getRigidBody().getLinearVelocity(tmp).length;

		Transform chassisTrans = getChassisWorldTransform(Transform());

		Vector3 forwardW = Vector3.zero();
		forwardW.setValues(
				chassisTrans.basis.getElement(0, _indexForwardAxis),
				chassisTrans.basis.getElement(1, _indexForwardAxis),
				chassisTrans.basis.getElement(2, _indexForwardAxis));

		if (forwardW.dot(getRigidBody().getLinearVelocity(tmp)) < 0) {
			_currentVehicleSpeedKmHour *= -1;
		}

		//
		// simulate suspension
		//

		int i = 0;
		// for (i = 0; i < wheelInfo.length; i++) {
		// 	double depth;
		// 	depth = rayCast(wheelInfo.getQuick(i));
		// }

		updateSuspension(step);

		for (i = 0; i < wheelInfo.length; i++) {
			// apply suspension force
			WheelInfo wheel = wheelInfo.getQuick(i);

			double suspensionForce = wheel.wheelsSuspensionForce;

			if (suspensionForce > wheel.maxSuspensionForce) {
				suspensionForce = wheel.maxSuspensionForce;
			}
			Vector3 impulse = Vector3.zero();
			impulse.scaleFrom(suspensionForce * step, wheel.raycastInfo.contactNormalWS);
			Vector3 relpos = Vector3.zero();
			relpos.sub2(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(tmp));

			getRigidBody().applyImpulse(impulse, relpos);
		}

		updateFriction(step);

		for (i = 0; i < wheelInfo.length; i++) {
			WheelInfo wheel = wheelInfo.getQuick(i);
			Vector3 relpos = Vector3.zero();
			relpos.sub2(wheel.raycastInfo.hardPointWS, getRigidBody().getCenterOfMassPosition(tmp));
			Vector3 vel = getRigidBody().getVelocityInLocalPoint(relpos, Vector3.zero());

			if (wheel.raycastInfo.isInContact) {
				Transform chassisWorldTransform = getChassisWorldTransform(Transform());

				Vector3 fwd = Vector3.zero();
				fwd.setValues(
						chassisWorldTransform.basis.getElement(0, _indexForwardAxis),
						chassisWorldTransform.basis.getElement(1, _indexForwardAxis),
						chassisWorldTransform.basis.getElement(2, _indexForwardAxis));

				double proj = fwd.dot(wheel.raycastInfo.contactNormalWS);
				tmp.scaleFrom(proj, wheel.raycastInfo.contactNormalWS);
				fwd.sub(tmp);

				double proj2 = fwd.dot(vel);

				wheel.deltaRotation = (proj2 * step) / (wheel.wheelsRadius);
				wheel.rotation += wheel.deltaRotation;

			}
			else {
				wheel.rotation += wheel.deltaRotation;
			}

			wheel.deltaRotation *= 0.99; // damping of rotation when not in contact
		}
	}

	void setSteeringValue(double steering, int wheel) {
		assert (wheel >= 0 && wheel < getNumWheels());

		WheelInfo wheelInfo = getWheelInfo(wheel);
		wheelInfo.steering = steering;
	}
	
	double getSteeringValue(int wheel) {
		return getWheelInfo(wheel).steering;
	}

	void applyEngineForce(double force, int wheel) {
		assert (wheel >= 0 && wheel < getNumWheels());
		WheelInfo wheelInfo = getWheelInfo(wheel);
		wheelInfo.engineForce = force;
	}

	WheelInfo getWheelInfo(int index) {
		assert ((index >= 0) && (index < getNumWheels()));

		return wheelInfo.getQuick(index);
	}

	void setBrake(double brake, int wheelIndex) {
		assert ((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
		getWheelInfo(wheelIndex).brake = brake;
	}

	void updateSuspension(double deltaTime) {
		double chassisMass = 1 / _chassisBody.getInvMass();

		for (int wIt = 0; wIt < getNumWheels(); wIt++) {
			WheelInfo wheelInfo = this.wheelInfo.getQuick(wIt);

			if (wheelInfo.raycastInfo.isInContact) {
				double force;
				//	Spring
				{
					double suspLength = wheelInfo.getSuspensionRestlength;
					double currentLength = wheelInfo.raycastInfo.suspensionLength;

					double lengthDiff = (suspLength - currentLength);

					force = wheelInfo.suspensionStiffness * lengthDiff * wheelInfo.clippedInvContactDotSuspension;
				}

				// Damper
				{
					double projectedRelVel = wheelInfo.suspensionRelativeVelocity;
					{
						double suspDamping;
						if (projectedRelVel < 0) {
							suspDamping = wheelInfo.wheelsDampingCompression;
						}
						else {
							suspDamping = wheelInfo.wheelsDampingRelaxation;
						}
						force -= suspDamping * projectedRelVel;
					}
				}

				// RESULT
				wheelInfo.wheelsSuspensionForce = force * chassisMass;
				if (wheelInfo.wheelsSuspensionForce < 0) {
					wheelInfo.wheelsSuspensionForce = 0;
				}
			}
			else {
				wheelInfo.wheelsSuspensionForce = 0;
			}
		}
	}
	
	double _calcRollingFriction(_WheelContactPoint contactPoint) {
		Vector3 tmp = Vector3.zero();
		
		double j1 = 0;
		
		Vector3 contactPosWorld = contactPoint.frictionPositionWorld;

		Vector3 relPos1 = Vector3.zero();
		relPos1.sub2(contactPosWorld, contactPoint.body0.getCenterOfMassPosition(tmp));
		Vector3 relPos2 = Vector3.zero();
		relPos2.sub2(contactPosWorld, contactPoint.body1.getCenterOfMassPosition(tmp));

		double maxImpulse = contactPoint.maxImpulse;

		Vector3 vel1 = contactPoint.body0.getVelocityInLocalPoint(relPos1, Vector3.zero());
		Vector3 vel2 = contactPoint.body1.getVelocityInLocalPoint(relPos2, Vector3.zero());
		Vector3 vel = Vector3.zero();
		vel.sub2(vel1, vel2);

		double vrel = contactPoint.frictionDirectionWorld.dot(vel);

		// calculate j that moves us to zero relative velocity
		j1 = -vrel * contactPoint.jacDiagABInv;
		j1 = min(j1, maxImpulse);
		j1 = max(j1, -maxImpulse);

		return j1;
	}
	
	void updateFriction(double timeStep) {
		// calculate the impulse, so that the wheels don't move sidewards
		int numWheel = getNumWheels();
		if (numWheel == 0) {
			return;
		}

		MiscUtil.resizeObjectArray(forwardWS, numWheel, Vector3.zero());
		MiscUtil.resizeObjectArray(axle, numWheel, Vector3.zero());
		MiscUtil.resize(forwardImpulse, numWheel, 0);
		MiscUtil.resize(sideImpulse, numWheel, 0);

		Vector3 tmp = Vector3.zero();

		//int numWheelsOnGround = 0;

		// collapse all those loops into one!
		for (int i = 0; i < getNumWheels(); i++) {
			WheelInfo wheelInfo = this.wheelInfo.getQuick(i);
			RigidBody? groundObject = wheelInfo.raycastInfo.groundObject as RigidBody?;
			if (groundObject != null) {
				//numWheelsOnGround++;
			}
			sideImpulse.set(i, 0);
			forwardImpulse.set(i, 0);
		}

		{
			Transform wheelTrans = Transform();
			for (int i = 0; i < getNumWheels(); i++) {

				WheelInfo wheelInfo = this.wheelInfo.getQuick(i);

				RigidBody? groundObject = wheelInfo.raycastInfo.groundObject as RigidBody?;

				if (groundObject != null) {
					getWheelTransformWS(i, wheelTrans);

					Matrix3 wheelBasis0 = Matrix3.copy(wheelTrans.basis);
					axle.getQuick(i)?.setValues(
							wheelBasis0.getElement(0, _indexRightAxis),
							wheelBasis0.getElement(1, _indexRightAxis),
							wheelBasis0.getElement(2, _indexRightAxis));

					Vector3 surfNormalWS = wheelInfo.raycastInfo.contactNormalWS;
					double proj = axle.getQuick(i)?.dot(surfNormalWS) ?? 0;
					tmp.scaleFrom(proj, surfNormalWS);
					axle.getQuick(i)?.sub(tmp);
					axle.getQuick(i)?.normalize();

					forwardWS.getQuick(i)?.cross2(surfNormalWS, axle.getQuick(i));
					forwardWS.getQuick(i)?.normalize();

					double doublePtr = 0;
					ContactConstraint.resolveSingleBilateral(_chassisBody, wheelInfo.raycastInfo.contactPointWS,
							groundObject, wheelInfo.raycastInfo.contactPointWS,
							0, axle.getQuick(i) ?? Vector3.zero(), doublePtr, timeStep);
					sideImpulse.set(i, doublePtr);

					sideImpulse.set(i, sideImpulse.get(i) * _sideFrictionStiffness2);
				}
			}
		}

		double sideFactor = 1;
		double fwdFactor = 0.5;

		bool sliding = false;
		{
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				WheelInfo wheelInfo = this.wheelInfo.getQuick(wheel);
				RigidBody? groundObject = wheelInfo.raycastInfo.groundObject as RigidBody?;

				double rollingFriction = 0;

				if (groundObject != null) {
					if (wheelInfo.engineForce != 0) {
						rollingFriction = wheelInfo.engineForce * timeStep;
					}
					else { 
						double defaultRollingFrictionImpulse = 0;
						double maxImpulse = wheelInfo.brake != 0 ? wheelInfo.brake : defaultRollingFrictionImpulse;
						_WheelContactPoint contactPt = _WheelContactPoint(_chassisBody, groundObject, wheelInfo.raycastInfo.contactPointWS, forwardWS.getQuick(wheel) ?? Vector3.zero(), maxImpulse);
						rollingFriction = _calcRollingFriction(contactPt);
					}
				}

				// switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

				forwardImpulse.set(wheel, 0);
				this.wheelInfo.getQuick(wheel).skidInfo = 1;

				if (groundObject != null) {
					this.wheelInfo.getQuick(wheel).skidInfo = 1;

					double maximp = wheelInfo.wheelsSuspensionForce * timeStep * wheelInfo.frictionSlip;
					double maximpSide = maximp;

					double maximpSquared = maximp * maximpSide;

					forwardImpulse.set(wheel, rollingFriction); //wheelInfo.m_engineForce* timeStep;

					double x = (forwardImpulse.get(wheel)) * fwdFactor;
					double y = (sideImpulse.get(wheel)) * sideFactor;

					double impulseSquared = (x * x + y * y);

					if (impulseSquared > maximpSquared) {
						sliding = true;

						double factor = maximp / sqrt(impulseSquared);

						this.wheelInfo.getQuick(wheel).skidInfo *= factor;
					}
				}

			}
		}

		if (sliding) {
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				if (sideImpulse.get(wheel) != 0) {
					if (wheelInfo.getQuick(wheel).skidInfo < 1) {
						forwardImpulse.set(wheel, forwardImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
						sideImpulse.set(wheel, sideImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
					}
				}
			}
		}

		// apply the impulses
		{
			for (int wheel = 0; wheel < getNumWheels(); wheel++) {
				WheelInfo wheelInfo = this.wheelInfo.getQuick(wheel);

				Vector3 relPos = Vector3.zero();
				relPos.sub2(wheelInfo.raycastInfo.contactPointWS, _chassisBody.getCenterOfMassPosition(tmp));

				if (forwardImpulse.get(wheel) != 0) {
					tmp.scaleFrom(forwardImpulse.get(wheel), forwardWS.getQuick(wheel));
					_chassisBody.applyImpulse(tmp, relPos);
				}
				if (sideImpulse.get(wheel) != 0) {
					RigidBody groundObject = this.wheelInfo.getQuick(wheel).raycastInfo.groundObject as RigidBody;

					Vector3 relPos2 = Vector3.zero();
					relPos2.sub2(wheelInfo.raycastInfo.contactPointWS, groundObject.getCenterOfMassPosition(tmp));

					Vector3 sideImp = Vector3.zero();
					sideImp.scaleFrom(sideImpulse.get(wheel), axle.getQuick(wheel));

					relPos.z *= wheelInfo.rollInfluence;
					_chassisBody.applyImpulse(sideImp, relPos);

					// apply friction impulse on the ground
					tmp.negateFrom(sideImp);
					groundObject.applyImpulse(tmp, relPos2);
				}
			}
		}
	}
	
	@override
	void buildJacobian() {
		// not yet
	}

	@override
	void solveConstraint(double timeStep) {
		// not yet
	}
	
	int getNumWheels() {
		return wheelInfo.length;
	}

	void setPitchControl(double pitch) {
		_pitchControl = pitch;
	}

	RigidBody getRigidBody() {
		return _chassisBody;
	}

	int getRightAxis() {
		return _indexRightAxis;
	}

	int getUpAxis() {
		return _indexUpAxis;
	}

	int getForwardAxis() {
		return _indexForwardAxis;
	}

	/**
	 * Worldspace forward vector.
	 */
	Vector3 getForwardVector(Vector3 out) {
		Transform chassisTrans = getChassisWorldTransform(Transform());

		out.setValues(
				chassisTrans.basis.getElement(0, _indexForwardAxis),
				chassisTrans.basis.getElement(1, _indexForwardAxis),
				chassisTrans.basis.getElement(2, _indexForwardAxis));

		return out;
	}

	/**
	 * Velocity of vehicle (positive if velocity vector has same direction as foward vector).
	 */
	double getCurrentSpeedKmHour() {
		return _currentVehicleSpeedKmHour;
	}

	void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex) {
		_indexRightAxis = rightIndex;
		_indexUpAxis = upIndex;
		_indexForwardAxis = forwardIndex;
	}
}

////////////////////////////////////////////////////////////////////////////

class _WheelContactPoint {
  RigidBody body0;
  RigidBody body1;
  final Vector3 frictionPositionWorld = Vector3.zero();
  final Vector3 frictionDirectionWorld = Vector3.zero();
  double jacDiagABInv = 0;
  double maxImpulse;

  _WheelContactPoint(this.body0, this.body1, Vector3 frictionPosWorld, Vector3 frictionDirectionWorld, this.maxImpulse) {
    frictionPositionWorld.setFrom(frictionPosWorld);
    this.frictionDirectionWorld.setFrom(frictionDirectionWorld);

    double denom0 = body0.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
    double denom1 = body1.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
    double relaxation = 1;
    jacDiagABInv = relaxation / (denom0 + denom1);
  }
}
