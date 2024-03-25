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

import './c_profile_node.dart';

/**
 * Iterator to navigate through profile tree.
 * 
 * @author jezek2
 */
class CProfileIterator {

	CProfileNode? _currentParent;
	CProfileNode? _currentChild;

	CProfileIterator(CProfileNode start) {
		_currentParent = start;
		_currentChild = _currentParent?.getChild();
	}
	
	// Access all the children of the current parent
	
	void first() {
		_currentChild = _currentParent?.getChild();
	}
	
	void next() {
		_currentChild = _currentChild?.getSibling();
	}
	
	bool isDone() {
		return (_currentChild == null);
	}
	
	bool isRoot() {
		return (_currentParent?.getParent() == null);
	}

	/**
	 * Make the given child the parent.
	 */
	void enterChild(int index) {
		_currentChild = _currentParent?.getChild();
		while ((_currentChild != null) && (index != 0)) {
			index--;
			_currentChild = _currentChild?.getSibling();
		}

		if (_currentChild != null) {
			_currentParent = _currentChild;
			_currentChild = _currentParent?.getChild();
		}
	}
	
	//void enterLargestChild(); // Make the largest child the parent
	
	/**
	 * Make the current parent's parent the parent.
	 */
	void enterParent() {
		if (_currentParent?.getParent() != null) {
			_currentParent = _currentParent?.getParent();
		}
		_currentChild = _currentParent?.getChild();
	}

	// Access the current child
	
	String getCurrentName() {
		return _currentChild?.getName() ?? '';
	}

	int getCurrentTotalCalls() {
		return _currentChild?.getTotalCalls() ?? 0;
	}

	double getCurrentTotalTime() {
		return _currentChild?.getTotalTime() ?? 0;
	}

	// Access the current parent
	
	String getCurrentParentName() {
		return _currentParent?.getName() ?? '';
	}

	int getCurrentParentTotalCalls() {
		return _currentParent?.getTotalCalls() ?? 0;
	}

	double getCurrentParentTotalTime() {
		return _currentParent?.getTotalTime() ?? 0;
	}
}
