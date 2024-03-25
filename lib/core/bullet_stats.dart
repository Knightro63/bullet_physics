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

import '../linearmath/c_profile_manager.dart';
import '../linearmath/clock.dart';
import 'package:vector_math/vector_math.dart';

/**
 * Bullet statistics and profile support.
 * 
 * @author jezek2
 */
class BulletStats {
  static int gTotalContactPoints = 0;
	// GjkPairDetector
	// temp globals, to improve GJK/EPA/penetration calculations
	static int gNumDeepPenetrationChecks = 0;
	static int gNumGjkChecks = 0;
	static int gNumSplitImpulseRecoveries = 0;

	static int gNumAlignedAllocs = 0;
	static int gNumAlignedFree = 0;
	static int gTotalBytesAlignedAllocs = 0;	
	// 	// JAVA NOTE: added for statistics in applet demo
	static int stepSimulationTime = 0;
	static int updateTime = 0;

	static int gPickingConstraintId = 0;
	static final Vector3 gOldPickingPos = Vector3.zero();
	static double gOldPickingDist = 0;
	
	static int gOverlappingPairs = 0;
	static int gRemovePairs = 0;
	static int gAddedPairs = 0;
	static int gFindPairs = 0;
	
	static final Clock gProfileClock = Clock();

	// DiscreteDynamicsWorld:
	static int gNumClampedCcdMotions = 0;


	
	static bool _enableProfile = false;
	
	////////////////////////////////////////////////////////////////////////////
	
	static bool isProfileEnabled() {
		return _enableProfile;
	}

	static void setProfileEnabled(bool b) {
		_enableProfile = b;
	}
	
	static int profileGetTicks() {
		int ticks = gProfileClock.getTimeMicroseconds();
		return ticks;
	}

	static double profileGetTickRate() {
		//return 1000000;
		return 1000;
	}
	
	/**
	 * Pushes profile node. Use try/finally block to call {@link #popProfile} method.
	 * 
	 * @param name must be {@link String#intern interned} String (not needed for String literals)
	 */
	static void pushProfile(String name) {
		if (_enableProfile) {
			CProfileManager.startProfile(name);
		}
	}
	
	/**
	 * Pops profile node.
	 */
	static void popProfile() {
		if (_enableProfile) {
			CProfileManager.stopProfile();
		}
	}
}
