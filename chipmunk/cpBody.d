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
module chipmunk.cpBody;

import chipmunk.chipmunk_types;

import chipmunk.cpShape;
import chipmunk.cpSpace;
import chipmunk.cpVect;

extern(C):

alias void function(cpBody *b, cpVect gravity, cpFloat damping, cpFloat dt) cpBodyVelocityFunc;
alias void function(cpBody *b, cpFloat dt) cpBodyPositionFunc;

cpBodyVelocityFunc cpBodyUpdateVelocityDefault;
cpBodyPositionFunc cpBodyUpdatePositionDefault;

// Structure to hold information about the contact graph components
// when putting groups of objects to sleep.
// No interesting user accessible fields.
struct cpComponentNode {
	cpBody *parent;
	cpBody *next;
	int rank;
	cpFloat idleTime;
}

struct cpBody{
	// *** Integration Functions.

	// Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity)
	cpBodyVelocityFunc velocity_func;
	
	// Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition)
	cpBodyPositionFunc position_func;
	
	// *** Mass Properties
	
	// Mass and it's inverse.
	// Always use cpBodySetMass() whenever changing the mass as these values must agree.
	cpFloat m, m_inv;
	
	// Moment of inertia and it's inverse.
	// Always use cpBodySetMoment() whenever changing the moment as these values must agree.
	cpFloat i, i_inv;
	
	// *** Positional Properties
	
	// Linear components of motion (position, velocity, and force)
	cpVect p, v, f;
	
	// Angular components of motion (angle, angular velocity, and torque)
	// Always use cpBodySetAngle() to set the angle of the body as a and rot must agree.
	cpFloat a, w, t;
	
	// Cached unit length vector representing the angle of the body.
	// Used for fast vector rotation using cpvrotate().
	cpVect rot;
	
	// *** User Definable Fields
	
	// User defined data pointer.
	cpDataPointer data;
	
	// *** Other Fields
	
	// Maximum velocities this body can move at after integrating velocity
	cpFloat v_limit, w_limit;
	
	// *** Internally Used Fields
	//private:
	
	// Velocity bias values used when solving penetrations and correcting constraints.
	cpVect v_bias;
	cpFloat w_bias;
	
	// Space this body has been added to
	cpSpace *space;
	
	// Pointer to the shape list.
	// Shapes form a linked list using cpShape.next when added to a space.
	cpShape *shapesList;
	
	// Used by cpSpaceStep() to store contact graph information.
	cpComponentNode node;
}

// Basic allocation/destruction functions
cpBody *cpBodyAlloc();
cpBody *cpBodyInit(cpBody *b, cpFloat m, cpFloat i);
cpBody *cpBodyNew(cpFloat m, cpFloat i);

cpBody *cpBodyInitStatic(cpBody *b);
cpBody *cpBodyNewStatic();

void cpBodyDestroy(cpBody *b);
void cpBodyFree(cpBody *b);

// Wake up a sleeping or idle body. (defined in cpSpace.c)
void cpBodyActivate(cpBody *b);

// Force a body to sleep;
// defined in cpSpaceComponent.c
void cpBodySleep(cpBody *b);
void cpBodySleepWithGroup(cpBody *b, cpBody *group);

cpBool cpBodyIsSleeping(const cpBody *b)
{
	return (b.node.next != null);
}

cpBool cpBodyIsStatic(const cpBody *b)
{
	return b.node.idleTime == cpFloat.infinity;
}

cpBool cpBodyIsRogue(const cpBody *b)
{
	return (b.space == null);
}

/+
#define CP_DefineBodyGetter(type, member, name) \
type cpBodyGet##name(const cpBody *b){return b.member;}

#define CP_DefineBodySetter(type, member, name) \
void cpBodySet##name(cpBody *b, const type value){ \
	cpBodyActivate(b); \
	b.member = value; \
} \

#define CP_DefineBodyProperty(type, member, name) \
CP_DefineBodyGetter(type, member, name) \
CP_DefineBodySetter(type, member, name)


// Accessors for cpBody struct members
CP_DefineBodyGetter(cpFloat, m, Mass);
void cpBodySetMass(cpBody *b, cpFloat m);

CP_DefineBodyGetter(cpFloat, i, Moment);
void cpBodySetMoment(cpBody *b, cpFloat i);


CP_DefineBodyProperty(cpVect, p, Pos);
CP_DefineBodyProperty(cpVect, v, Vel);
CP_DefineBodyProperty(cpVect, f, Force);
CP_DefineBodyGetter(cpFloat, a, Angle);
void cpBodySetAngle(cpBody *b, cpFloat a);
CP_DefineBodyProperty(cpFloat, w, AngVel);
CP_DefineBodyProperty(cpFloat, t, Torque);
CP_DefineBodyGetter(cpVect, rot, Rot);
CP_DefineBodyProperty(cpFloat, v_limit, VelLimit);
CP_DefineBodyProperty(cpFloat, w_limit, AngVelLimit);
+/

//  Modify the velocity of the body so that it will move to the specified absolute coordinates in the next timestep.
// Intended for objects that are moved manually with a custom velocity integration function.
void cpBodySlew(cpBody *b, cpVect pos, cpFloat dt);

// Default Integration functions.
void cpBodyUpdateVelocity(cpBody *b, cpVect gravity, cpFloat damping, cpFloat dt);
void cpBodyUpdatePosition(cpBody *b, cpFloat dt);

// Convert body local to world coordinates
cpVect cpBodyLocal2World(const cpBody *b, const cpVect v)
{
	return cpvadd(b.p, cpvrotate(v, b.rot));
}

// Convert world to body local coordinates
cpVect cpBodyWorld2Local(const cpBody *b, const cpVect v)
{
	return cpvunrotate(cpvsub(v, b.p), b.rot);
}

// Apply an impulse (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
void cpBodyApplyImpulse(cpBody *b, const cpVect j, const cpVect r)
{
	b.v = cpvadd(b.v, cpvmult(j, b.m_inv));
	b.w += b.i_inv*cpvcross(r, j);
}

// Zero the forces on a body.
void cpBodyResetForces(cpBody *b);
// Apply a force (in world coordinates) to a body at a point relative to the center of gravity (also in world coordinates).
void cpBodyApplyForce(cpBody *b, const cpVect f, const cpVect r);

cpFloat cpBodyKineticEnergy(const cpBody *b)
{
	// Need to do some fudging to avoid NaNs
	cpFloat vsq = cpvdot(b.v, b.v);
	cpFloat wsq = b.w*b.w;
	return (vsq ? vsq*b.m : 0.0f) + (wsq ? wsq*b.i : 0.0f);
}

// Apply a damped spring force between two bodies.
// Warning: Large damping values can be unstable. Use a cpDampedSpring constraint for this instead.
void cpApplyDampedSpring(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat rlen, cpFloat k, cpFloat dmp, cpFloat dt);
