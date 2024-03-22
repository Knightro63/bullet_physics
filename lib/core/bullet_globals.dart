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

import './contact_destroyed_callback.dart';
import './contact_added_callback.dart';
import './contact_processed_callback.dart';
/**
 * Bullet global settings and constants.
 * 
 * @author jezek2
 */
class BulletGlobals{
	
	static const bool debug = false;
	
	static const double convexDistanceMargin = 0.04;
	static const double fltEpsilon = 1.19209290e-07;
	static const double simdEpsilon = fltEpsilon;
	
	static const double simd2pi = 6.283185307179586232;
	static const double simdPi = simd2pi * 0.5;
	static const double simdHalfPi = simd2pi * 0.25;
	static const double simdRadsPerDeg = simd2pi / 360;
	static const double simdDegsPerRad = 360 / simd2pi;
	static const double simdInfinity = double.infinity;

  static final BulletGlobals _threadLocal = BulletGlobals();

	////////////////////////////////////////////////////////////////////////////

	// static ThreadLocal<BulletGlobals> _threadLocal = ThreadLocal<BulletGlobals>() {
	// 	@override
	// 	BulletGlobals initialValue() {
	// 		return BulletGlobals();
	// 	}
	// };

	ContactDestroyedCallback? _gContactDestroyedCallback;
	ContactAddedCallback? _gContactAddedCallback;
	ContactProcessedCallback? _gContactProcessedCallback;

	double _contactBreakingThreshold = 0.02;
	// // RigidBody
	double _deactivationTime = 2;
	bool _disableDeactivation = false;
	
	static ContactAddedCallback? getContactAddedCallback() {
		return _threadLocal._gContactAddedCallback;
	}

	static void setContactAddedCallback(ContactAddedCallback callback) {
		_threadLocal._gContactAddedCallback = callback;
	}

	static ContactDestroyedCallback? getContactDestroyedCallback() {
		return _threadLocal._gContactDestroyedCallback;
	}

	static void setContactDestroyedCallback(ContactDestroyedCallback callback) {
		_threadLocal._gContactDestroyedCallback = callback;
	}

	static ContactProcessedCallback? getContactProcessedCallback() {
		return _threadLocal._gContactProcessedCallback;
	}

	static void setContactProcessedCallback(ContactProcessedCallback callback) {
		_threadLocal._gContactProcessedCallback = callback;
	}
	
	////////////////////////////////////////////////////////////////////////////

	static double getContactBreakingThreshold() {
		return _threadLocal._contactBreakingThreshold;
	}

	static void setContactBreakingThreshold(double threshold) {
		_threadLocal._contactBreakingThreshold = threshold;
	}

	static double getDeactivationTime() {
		return _threadLocal._deactivationTime;
	}

	static void setDeactivationTime(double time) {
		_threadLocal._deactivationTime = time;
	}

	static bool isDeactivationDisabled() {
		return _threadLocal._disableDeactivation;
	}

	static void setDeactivationDisabled(bool disable) {
		_threadLocal._disableDeactivation = disable;
	}
  void remove(){}

	////////////////////////////////////////////////////////////////////////////

	/**
	 * Cleans all current thread specific settings and caches.
	 */
	static void cleanCurrentThread() {
		_threadLocal.remove();
		//ArrayPool.cleanCurrentThread();
	}
}
