/* Copyright (c) 2007 Scott Lembcke
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
module chipmunk.cpSpace;

import chipmunk.chipmunk_types;
import chipmunk.cpArbiter;
import chipmunk.cpBody;
import chipmunk.cpShape;
import chipmunk.cpSpaceHash;
import chipmunk.cpHashSet;
import chipmunk.cpArray;
import chipmunk.cpBB;

import chipmunk.constraints.cpConstraint;

// Number of frames that contact information should persist.
extern(C) cpTimestamp cp_contact_persistence;

extern(C):

alias cpBool function(cpArbiter *arb, cpSpace *space, void *data) cpCollisionBeginFunc;
alias cpBool function(cpArbiter *arb, cpSpace *space, void *data) cpCollisionPreSolveFunc;
alias void function(cpArbiter *arb, cpSpace *space, void *data) cpCollisionPostSolveFunc;
alias void function(cpArbiter *arb, cpSpace *space, void *data) cpCollisionSeparateFunc;

// Structure for holding collision pair function information.
// Used internally.
struct cpCollisionHandler {
	cpCollisionType a;
	cpCollisionType b;
	cpCollisionBeginFunc begin;
	cpCollisionPreSolveFunc preSolve;
	cpCollisionPostSolveFunc postSolve;
	cpCollisionSeparateFunc separate;
	void *data;
}

struct cpContactBufferHeader {
	cpTimestamp stamp;
	cpContactBufferHeader *next;
	uint numContacts;
}

struct cpSpace
{
	// *** User definable fields
	
	// Number of iterations to use in the impulse solver to solve contacts.
	int iterations;
	
	// Number of iterations to use in the impulse solver to solve elastic collisions.
	int elasticIterations;
	
	// Default gravity to supply when integrating rigid body motions.
	cpVect gravity;
	
	// Default damping to supply when integrating rigid body motions.
	cpFloat damping;
	
	// Speed threshold for a body to be considered idle.
	// The default value of 0 means to let the space guess a good threshold based on gravity.
	cpFloat idleSpeedThreshold;
	
	// Time a group of bodies must remain idle in order to fall asleep
	// The default value of INFINITY disables the sleeping algorithm.
	cpFloat sleepTimeThreshold;
	
	// *** Internally Used Fields
	// private:
	
	// When the space lock count is non zero you cannot add or remove objects
	int locked;
	
	// Time stamp. Is incremented on every call to cpSpaceStep().
	cpTimestamp stamp;

	// The static and active shape spatial hashes.
	cpSpaceHash *staticShapes;
	cpSpaceHash *activeShapes;
	
	// List of bodies in the system.
	cpArray *bodies;
	
	// List of groups of sleeping bodies.
	cpArray *sleepingComponents;
	
	// List of bodies that have been flagged to be awoken.
	cpArray *rousedBodies;
	
	// List of active arbiters for the impulse solver.
	cpArray *arbiters;
	cpArray *pooledArbiters;
	
	// Linked list ring of contact buffers.
	// Head is the newest buffer, and each buffer points to a newer buffer.
	// Head wraps around and points to the oldest (tail) buffer.
	cpContactBufferHeader *contactBuffersHead;
	cpContactBufferHeader *_contactBuffersTail_Deprecated;
	
	// List of buffers to be free()ed when destroying the space.
	cpArray *allocatedBuffers;
	
	// Persistant contact set.
	cpHashSet *contactSet;
	
	// List of constraints in the system.
	cpArray *constraints;
	
	// Set of collisionpair functions.
	cpHashSet *collFuncSet;
	// Default collision handler.
	cpCollisionHandler defaultHandler;
	
	cpHashSet *postStepCallbacks;
	
	cpBody staticBody;
}

// Basic allocation/destruction functions.
cpSpace* cpSpaceAlloc();
cpSpace* cpSpaceInit(cpSpace *space);
cpSpace* cpSpaceNew();

void cpSpaceDestroy(cpSpace *space);
void cpSpaceFree(cpSpace *space);

// Convenience function. Frees all referenced entities. (bodies, shapes and constraints)
void cpSpaceFreeChildren(cpSpace *space);

// Collision handler management functions.
void cpSpaceSetDefaultCollisionHandler(
	cpSpace *space,
	cpCollisionBeginFunc begin,
	cpCollisionPreSolveFunc preSolve,
	cpCollisionPostSolveFunc postSolve,
	cpCollisionSeparateFunc separate,
	void *data
);
void cpSpaceAddCollisionHandler(
	cpSpace *space,
	cpCollisionType a, cpCollisionType b,
	cpCollisionBeginFunc begin,
	cpCollisionPreSolveFunc preSolve,
	cpCollisionPostSolveFunc postSolve,
	cpCollisionSeparateFunc separate,
	void *data
);
void cpSpaceRemoveCollisionHandler(cpSpace *space, cpCollisionType a, cpCollisionType b);

// Add and remove entities from the system.
cpShape *cpSpaceAddShape(cpSpace *space, cpShape *shape);
cpShape *cpSpaceAddStaticShape(cpSpace *space, cpShape *shape);
cpBody *cpSpaceAddBody(cpSpace *space, cpBody *b);
cpConstraint *cpSpaceAddConstraint(cpSpace *space, cpConstraint *constraint);

void cpSpaceRemoveShape(cpSpace *space, cpShape *shape);
void cpSpaceRemoveStaticShape(cpSpace *space, cpShape *shape);
void cpSpaceRemoveBody(cpSpace *space, cpBody *b);
void cpSpaceRemoveConstraint(cpSpace *space, cpConstraint *constraint);

// Post Step function definition
alias void function(cpSpace *space, void *obj, void *data) cpPostStepFunc;
// Register a post step function to be called after cpSpaceStep() has finished.
// obj is used a key, you can only register one callback per unique value for obj
void cpSpaceAddPostStepCallback(cpSpace *space, cpPostStepFunc func, void *obj, void *data);

// Point query callback function
alias void function(cpShape *shape, void *data) cpSpacePointQueryFunc;
void cpSpacePointQuery(cpSpace *space, cpVect point, cpLayers layers, cpGroup group, cpSpacePointQueryFunc func, void *data);
cpShape *cpSpacePointQueryFirst(cpSpace *space, cpVect point, cpLayers layers, cpGroup group);

// Segment query callback function
alias void function(cpShape *shape, cpFloat t, cpVect n, void *data) cpSpaceSegmentQueryFunc;
void cpSpaceSegmentQuery(cpSpace *space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSpaceSegmentQueryFunc func, void *data);
cpShape *cpSpaceSegmentQueryFirst(cpSpace *space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSegmentQueryInfo *_out);

// BB query callback function
alias void function(cpShape *shape, void *data) cpSpaceBBQueryFunc;
void cpSpaceBBQuery(cpSpace *space, cpBB bb, cpLayers layers, cpGroup group, cpSpaceBBQueryFunc func, void *data);

// Shape query callback function
alias void function(cpShape *shape, cpContactPointSet *points, void *data) cpSpaceShapeQueryFunc;
cpBool cpSpaceShapeQuery(cpSpace *space, cpShape *shape, cpSpaceShapeQueryFunc func, void *data);


void cpSpaceActivateShapesTouchingShape(cpSpace *space, cpShape *shape);


// Iterator function for iterating the bodies in a space.
alias void function(cpBody *b, void *data) cpSpaceBodyIterator;

void cpSpaceEachBody(cpSpace *space, cpSpaceBodyIterator func, void *data);

// Spatial hash management functions.
void cpSpaceResizeStaticHash(cpSpace *space, cpFloat dim, int count);
void cpSpaceResizeActiveHash(cpSpace *space, cpFloat dim, int count);
void cpSpaceRehashStatic(cpSpace *space);

void cpSpaceRehashShape(cpSpace *space, cpShape *shape);

// Update the space.
void cpSpaceStep(cpSpace *space, cpFloat dt);
