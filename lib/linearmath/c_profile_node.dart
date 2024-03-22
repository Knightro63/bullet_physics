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

/***************************************************************************************************
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

import '../core/bullet_stats.dart';

/**
 * A node in the Profile Hierarchy Tree.
 * 
 * @author jezek2
 */
class CProfileNode {
	String name;
	int totalCalls = 0;
	double totalTime = 0;
	int startTime = 0;
	int recursionCounter = 0;
	
	CProfileNode? parent;
	CProfileNode? child;
	CProfileNode? sibling;

	CProfileNode(this.name, this.parent) {
		child = null;
		sibling = null;
		reset();
	}

	CProfileNode getSubNode(String name) {
		// Try to find this sub node
		CProfileNode? child = this.child;
		while (child != null) {
			if (child.name == name) {
				return child;
			}
			child = child.sibling;
		}

		// We didn't find it, so add it

		CProfileNode node = CProfileNode(name, this);
		node.sibling = this.child;
		this.child = node;
		return node;
	}

	CProfileNode? getParent() {
		return parent;
	}

	CProfileNode? getSibling() {
		return sibling;
	}

	CProfileNode? getChild() {
		return child;
	}

	void cleanupMemory() {
		child = null;
		sibling = null;
	}
	
	void reset() {
		totalCalls = 0;
		totalTime = 0.0;
		BulletStats.gProfileClock.reset();
    child?.reset();
    sibling?.reset();
	}
	
	void call() {
		totalCalls++;
		if (recursionCounter++ == 0) {
			startTime = BulletStats.profileGetTicks();
		}
	}
	
	bool Return() {
		if (--recursionCounter == 0 && totalCalls != 0) {
			int time = BulletStats.profileGetTicks();
			time -= startTime;
			totalTime += time / BulletStats.profileGetTickRate();
		}
		return (recursionCounter == 0);
	}

	String getName() {
		return name;
	}
	
	int getTotalCalls() {
		return totalCalls;
	}

	double getTotalTime() {
		return totalTime;
	}
	
}
