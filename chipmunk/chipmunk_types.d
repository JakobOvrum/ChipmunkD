module chipmunk.chipmunk_types;

import std.math;

version = CP_USE_DOUBLES;

version(CP_USE_DOUBLES)
{
	alias double cpFloat;
}
else
{
	alias float cpFloat;
}

alias sqrt cpfsqrt;
alias sin cpfsin;
alias cos cpfcos;
alias acos cpfacos;
alias atan2 cpfatan2;
alias modf cpfmod;
alias exp cpfexp;
alias pow cpfpow;
alias floor cpffloor;
alias ceil cpfceil;

cpFloat cpfmax(cpFloat a, cpFloat b)
{
	return (a > b) ? a : b;
}

cpFloat cpfmin(cpFloat a, cpFloat b)
{
	return (a < b) ? a : b;
}

cpFloat cpfabs(cpFloat n)
{
	return (n < 0) ? -n : n;
}

cpFloat cpfclamp(cpFloat f, cpFloat min, cpFloat max)
{
	return cpfmin(cpfmax(f, min), max);
}

cpFloat cpflerp(cpFloat f1, cpFloat f2, cpFloat t)
{
	return f1*(1.0f - t) + f2*t;
}

cpFloat cpflerpconst(cpFloat f1, cpFloat f2, cpFloat d)
{
	return f1 + cpfclamp(f2 - f1, -d, d);
}

struct cpVect
{
	cpFloat x,y;
}

alias uint cpHashValue;

// Oh C, how we love to define our own boolean types to get compiler compatibility
alias int cpBool;
enum cpTrue = 1;
enum cpFalse = 0;

alias void* cpDataPointer;

alias uint cpCollisionType;

alias uint cpGroup;

alias uint cpLayers;

alias uint cpTimestamp;

enum CP_NO_GROUP = cast(cpGroup)0;

enum CP_ALL_LAYERS = cast(cpLayers)0;
