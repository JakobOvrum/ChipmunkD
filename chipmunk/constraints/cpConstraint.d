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
module chipmunk.constraints.cpConstraint;

import chipmunk.chipmunk_types;
import chipmunk.cpBody;

// TODO: Comment me!

extern(C):
	
extern(C) cpFloat cp_constraint_bias_coef;


alias void function(cpConstraint *constraint, cpFloat dt, cpFloat dt_inv) cpConstraintPreStepFunction;
alias void function(cpConstraint *constraint) cpConstraintApplyImpulseFunction;
alias cpFloat function(cpConstraint *constraint) cpConstraintGetImpulseFunction;

struct cpConstraintClass {
	cpConstraintPreStepFunction preStep;
	cpConstraintApplyImpulseFunction applyImpulse;
	cpConstraintGetImpulseFunction getImpulse;
}

struct cpConstraint {
	const cpConstraintClass *klass; // private
	
	cpBody* a, b;
	cpFloat maxForce;
	cpFloat biasCoef;
	cpFloat maxBias;
	
	cpDataPointer data;
}

//#ifdef CP_USE_DEPRECATED_API_4
//typedef cpConstraint cpJoint;
//#endif

void cpConstraintDestroy(cpConstraint *constraint);
void cpConstraintFree(cpConstraint *constraint);

void cpConstraintActivateBodies(cpConstraint *constraint)
{
	cpBody *a = constraint.a; if(a) cpBodyActivate(a);
	cpBody *b = constraint.b; if(b) cpBodyActivate(b);
}

cpFloat cpConstraintGetImpulse(cpConstraint *constraint)
{
	return constraint.klass.getImpulse(constraint);
}

/+
#define cpConstraintCheckCast(constraint, struct) \
	cpAssert(constraint->klass == struct##GetClass(), "Constraint is not a "#struct);


#define CP_DefineConstraintGetter(struct, type, member, name) \
static inline type \
struct##Get##name(const cpConstraint *constraint){ \
	cpConstraintCheckCast(constraint, struct); \
	return ((struct *)constraint)->member; \
} \

#define CP_DefineConstraintSetter(struct, type, member, name) \
static inline void \
struct##Set##name(cpConstraint *constraint, type value){ \
	cpConstraintCheckCast(constraint, struct); \
	cpConstraintActivateBodies(constraint); \
	((struct *)constraint)->member = value; \
} \

#define CP_DefineConstraintProperty(struct, type, member, name) \
CP_DefineConstraintGetter(struct, type, member, name) \
CP_DefineConstraintSetter(struct, type, member, name)
+/

// Built in Joint types
/+
#include "cpPinJoint.h"
#include "cpSlideJoint.h"
#include "cpPivotJoint.h"
#include "cpGrooveJoint.h"
#include "cpDampedSpring.h"
#include "cpDampedRotarySpring.h"
#include "cpRotaryLimitJoint.h"
#include "cpRatchetJoint.h"
#include "cpGearJoint.h"
#include "cpSimpleMotor.h"
+/