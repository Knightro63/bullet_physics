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

/***************************************************************************************************
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

import '../core/bullet_stats.dart';
import './c_profile_node.dart';
import './c_profile_iterator.dart';

class CProfileManager{

	static final CProfileNode _root = CProfileNode("Root", null);
	static CProfileNode _currentNode = _root;
	static int _frameCounter = 0;
	static int _resetTime = 0;

	/**
	 * @param name must be {@link String#intern interned} String (not needed for String literals)
	 */
	static void startProfile(String name) {
		if (name != _currentNode.getName()) {
			_currentNode = _currentNode.getSubNode(name);
		}

		_currentNode.call();
	}
	
	static void stopProfile() {
		// Return will indicate whether we should back up to our parent (we may
		// be profiling a recursive function)
		if (_currentNode.Return()) {
			_currentNode = _currentNode.getParent()!;
		}
	}

	static void cleanupMemory() {
		_root.cleanupMemory();
	}

	static void reset() {
		_root.reset();
		_root.call();
		_frameCounter = 0;
		_resetTime = BulletStats.profileGetTicks();
	}
	
	static void incrementFrameCounter() {
		_frameCounter++;
	}
	
	static int getFrameCountSinceReset() {
		return _frameCounter;
	}
	
	static double getTimeSinceReset() {
		int time = BulletStats.profileGetTicks();
		time -= _resetTime;
		return time / BulletStats.profileGetTickRate();
	}

	static CProfileIterator getIterator() {
		return CProfileIterator(_root);
	}
	
	static void releaseIterator(CProfileIterator iterator) {
		/*delete ( iterator);*/
	}
	
}
