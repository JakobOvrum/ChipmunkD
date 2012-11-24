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
module chipmunk.chipmunk;

public import chipmunk.chipmunk_types,
			  chipmunk.cpVect,
			  chipmunk.cpBB,
			  chipmunk.cpArray,
			  chipmunk.cpHashSet,
			  chipmunk.cpSpaceHash,
			  
			  chipmunk.cpBody,
			  chipmunk.cpShape,
			  chipmunk.cpPolyShape,
			  
			  chipmunk.cpArbiter,
			  chipmunk.cpCollision,
			  
			  chipmunk.constraints.cpConstraint,
			  
			  chipmunk.cpSpace;

extern(C):

void cpMessage(const char *message, const char *condition, const char *file, int line, int isError);
/+#ifdef NDEBUG
	#define	cpAssertWarn(condition, message)
#else
	#define cpAssertWarn(condition, message) if(!(condition)) cpMessage(message, #condition, __FILE__, __LINE__, 0)
#endif

#ifdef NDEBUG
	#define	cpAssert(condition, message)
#else
	#define cpAssert(condition, message) if(!(condition)) cpMessage(message, #condition, __FILE__, __LINE__, 1)
#endif+/

/+
#ifndef INFINITY
	#ifdef _MSC_VER
		union MSVC_EVIL_FLOAT_HACK
		{
			unsigned __int8 Bytes[4];
			float Value;
		};
		static union MSVC_EVIL_FLOAT_HACK INFINITY_HACK = {{0x00, 0x00, 0x80, 0x7F}};
		#define INFINITY (INFINITY_HACK.Value)
	#endif
	
	#ifdef __GNUC__
		#define INFINITY (__builtin_inf())
	#endif
	
	#ifndef INFINITY
		#define INFINITY (1e1000)
	#endif
#endif
+/

// Maximum allocated size for various Chipmunk buffers
enum CP_BUFFER_BYTES = (32*1024);

import std.c.stdlib;

alias malloc cpmalloc;
alias calloc cpcalloc;
alias realloc cprealloc;
alias free cpfree;

enum CP_HASH_COEF = 3344921057uL;

cpHashValue CP_HASH_PAIR(cpHashValue a, cpHashValue b)
{
	return cast(cpHashValue)((a*CP_HASH_COEF) ^ (b*CP_HASH_COEF));
}

extern(C) const char *cpVersionString;

void cpInitChipmunk();

/**
	Calculate the moment of inertia for a circle.
	r1 and r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
*/
cpFloat cpMomentForCircle(cpFloat m, cpFloat r1, cpFloat r2, cpVect offset);

/**
	Calculate area of a hollow circle.
*/
cpFloat cpAreaForCircle(cpFloat r1, cpFloat r2);

/**
	Calculate the moment of inertia for a line segment.
	Beveling radius is not supported.
*/
cpFloat cpMomentForSegment(cpFloat m, cpVect a, cpVect b);

/**
	Calculate the area of a fattened (capsule shaped) line segment.
*/
cpFloat cpAreaForSegment(cpVect a, cpVect b, cpFloat r);

/**
	Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity is at it's centroid. The offset is added to each vertex.
*/
cpFloat cpMomentForPoly(cpFloat m, int numVerts, const cpVect *verts, cpVect offset);

/**
	Calculate the signed area of a polygon.
*/
cpFloat cpAreaForPoly(const int numVerts, const cpVect *verts);

/**
	Calculate the natural centroid of a polygon.
*/
cpVect cpCentroidForPoly(const int numVerts, const cpVect *verts);

/**
	Center the polygon on the origin. (Subtracts the centroid of the polygon from each vertex)
*/
void cpRecenterPoly(const int numVerts, cpVect *verts);

/**
	Calculate the moment of inertia for a solid box.
*/
cpFloat cpMomentForBox(cpFloat m, cpFloat width, cpFloat height);

