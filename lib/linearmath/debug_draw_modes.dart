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
/**
 * Debug draw modes, used by demo framework.
 * 
 * @author jezek2
 */
class DebugDrawModes {
	static const int noDebug              = 0;
	static const int drawWireframe        = 1;
	static const int drawAABB             = 2;
	static const int drawFeaturesText     = 4;
	static const int drawContactPoints    = 8;
	static const int noDeactivation       = 16;
	static const int noHelpText           = 32;
	static const int drawText             = 64;
	static const int profileTimings       = 128;
	static const int enableSatComparison  = 256;
	static const int disableBulletLCP     = 512;
	static const int enableCCD            = 1024;

  // PORT_ISSUE: added these modes here, but not yet supported
  static const int drawConstraints = (1 << 11);
  static const int drawConstraintLimits = (1 << 12);

	static const int maxDebugDrawMode   = 1025;
}
