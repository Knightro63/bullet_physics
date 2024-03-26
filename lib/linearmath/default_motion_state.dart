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
import 'package:bullet_physics/linearmath/motion_state.dart';
import 'package:bullet_physics/linearmath/transform.dart';

class DefaultMotionState extends MotionState {

	/** Current interpolated world transform, used to draw object. */
	final Transform graphicsWorldTrans = Transform();
	
	/** Center of mass offset transform, used to adjust graphics world transform. */
	final Transform centerOfMassOffset = Transform();
	
	/** Initial world transform. */
	final Transform startWorldTrans = Transform();
	
	/**
	 * Creates a DefaultMotionState with all transforms set to identity.
	 */
	DefaultMotionState([Transform? startTrans, Transform? centerOfMassOffset]) {
    if(startTrans != null){
		  graphicsWorldTrans.copy(startTrans);
      startWorldTrans.copy(startTrans);
    }
    else{
      graphicsWorldTrans.setIdentity();
      startWorldTrans.setIdentity();
    }

    if(centerOfMassOffset != null){
		  this.centerOfMassOffset.copy(centerOfMassOffset);
    }
    else{
      this.centerOfMassOffset.setIdentity();
    }
		
	}
	
  @override
	Transform getWorldTransform(Transform out) {
		out.inverse(centerOfMassOffset);
		out.mul(graphicsWorldTrans);
		return out;
	}

  @override
	void setWorldTransform(Transform centerOfMassWorldTrans) {
		graphicsWorldTrans.copy(centerOfMassWorldTrans);
		graphicsWorldTrans.mul(centerOfMassOffset);
	}
}
