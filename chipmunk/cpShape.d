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
module chipmunk.cpShape;

import chipmunk.chipmunk_types;
import chipmunk.cpBB;
import chipmunk.cpBody;
import chipmunk.cpVect;

extern(C):

struct cpSegmentQueryInfo {
	cpShape *shape; // shape that was hit, NULL if no collision
	cpFloat t; // Distance along query segment, will always be in the range [0, 1].
	cpVect n; // normal of hit surface
}

// Enumeration of shape types.
enum cpShapeType{
	CP_CIRCLE_SHAPE,
	CP_SEGMENT_SHAPE,
	CP_POLY_SHAPE,
	CP_NUM_SHAPES
}

// Shape class. Holds function pointers and type data.
struct cpShapeClass {
	cpShapeType type;
	
	// Called by cpShapeCacheBB().
	extern(C):
	cpBB function(cpShape *shape, cpVect p, cpVect rot) cacheData;
	
	// Called to by cpShapeDestroy().
	void function(cpShape *shape) destroy;
	
	// called by cpShapePointQuery().
	cpBool function(cpShape *shape, cpVect p) pointQuery;
	
	// called by cpShapeSegmentQuery()
	void function(cpShape *shape, cpVect a, cpVect b, cpSegmentQueryInfo *info) segmentQuery;
}

// Basic shape struct that the others inherit from.
struct cpShape{
	// The "class" of a shape as defined above // PRIVATE
	const cpShapeClass *klass;
	
	// cpBody that the shape is attached to.
	cpBody* _body;

	// Cached BBox for the shape.
	cpBB bb;
	
	// Sensors invoke callbacks, but do not generate collisions
	cpBool sensor;
	
	// *** Surface properties.
	
	// Coefficient of restitution. (elasticity)
	cpFloat e;
	// Coefficient of friction.
	cpFloat u;
	// Surface velocity used when solving for friction.
	cpVect surface_v;

	// *** User Definable Fields

	// User defined data pointer for the shape.
	cpDataPointer data;
	
	// User defined collision type for the shape.
	cpCollisionType collision_type;
	// User defined collision group for the shape.
	cpGroup group;
	// User defined layer bitmask for the shape.
	cpLayers layers;
	
	// *** Internally Used Fields
	//private:
	
	// Shapes form a linked list when added to space on a non-NULL body
	cpShape *next;
	
	// Unique id used as the hash value.
	cpHashValue hashid;
}

// Low level shape initialization func.
cpShape* cpShapeInit(cpShape *shape, const cpShapeClass *klass, cpBody *b);

// Basic destructor functions. (allocation functions are not shared)
void cpShapeDestroy(cpShape *shape);
void cpShapeFree(cpShape *shape);

// Cache the BBox of the shape.
cpBB cpShapeCacheBB(cpShape *shape);

// Test if a point lies within a shape.
cpBool cpShapePointQuery(cpShape *shape, cpVect p);

//#define CP_DeclareShapeGetter(struct, type, name) type struct##Get##name(cpShape *shape)

// Circle shape structure.
struct cpCircleShape{
	//private:
	cpShape shape;
	
	// Center in body space coordinates
	cpVect c;
	// Radius.
	cpFloat r;
	
	// Transformed center. (world space coordinates)
	cpVect tc;
}

// Basic allocation functions for cpCircleShape.
cpCircleShape *cpCircleShapeAlloc();
cpCircleShape *cpCircleShapeInit(cpCircleShape *circle, cpBody *b, cpFloat radius, cpVect offset);
cpShape *cpCircleShapeNew(cpBody *b, cpFloat radius, cpVect offset);

cpVect cpCircleShapeGetOffset(cpShape* shape);
cpFloat cpCircleShapeGetRadius(cpShape* shape);
//CP_DeclareShapeGetter(cpCircleShape, cpVect, Offset);
//CP_DeclareShapeGetter(cpCircleShape, cpFloat, Radius);

// Segment shape structure.
struct cpSegmentShape{
	//private:
	cpShape shape;
	
	// Endpoints and normal of the segment. (body space coordinates)
	cpVect a, b, n;
	// Radius of the segment. (Thickness)
	cpFloat r;

	// Transformed endpoints and normal. (world space coordinates)
	cpVect ta, tb, tn;
}

// Basic allocation functions for cpSegmentShape.
cpSegmentShape* cpSegmentShapeAlloc();
cpSegmentShape* cpSegmentShapeInit(cpSegmentShape *seg, cpBody *b, cpVect a, cpVect b, cpFloat radius);
cpShape* cpSegmentShapeNew(cpBody *b, cpVect a, cpVect b, cpFloat radius);

cpVect cpSegmentShapeGetA(cpShape* shape);
cpVect cpSegmentShapeGetB(cpShape* shape);
cpVect cpSegmentShapeGetNormal(cpShape* shape);
cpFloat cpSegmentShapeGetRadius(cpShape* shape);

/+
CP_DeclareShapeGetter(cpSegmentShape, cpVect, A);
CP_DeclareShapeGetter(cpSegmentShape, cpVect, B);
CP_DeclareShapeGetter(cpSegmentShape, cpVect, Normal);
CP_DeclareShapeGetter(cpSegmentShape, cpFloat, Radius);
+/

// For determinism, you can reset the shape id counter.
void cpResetShapeIdCounter();

// Directed segment queries against individual shapes.
void cpSegmentQueryInfoPrint(cpSegmentQueryInfo *info);

cpBool cpShapeSegmentQuery(cpShape *shape, cpVect a, cpVect b, cpSegmentQueryInfo *info);

cpVect cpSegmentQueryHitPoint(const cpVect start, const cpVect end, const cpSegmentQueryInfo info)
{
	return cpvlerp(start, end, info.t);
}

cpFloat cpSegmentQueryHitDist(const cpVect start, const cpVect end, const cpSegmentQueryInfo info)
{
	return cpvdist(start, end)*info.t;
}
