/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

// import { B2_lengthUnitsPerMeter } from "./b2_settings.js";

export function B2Assert(condition: boolean, ...args: any[]): asserts condition {
  if (!condition) {
    // debugger;
    throw new Error(...args);
  }
}

export function B2Maybe<T>(value: T | undefined, def: T): T {
  return value !== undefined ? value : def;
}

export const B2_maxFloat: number = 1E+37; // FLT_MAX instead of Number.MAX_VALUE;
export const B2_epsilon: number = 1E-5; // FLT_EPSILON instead of Number.EPSILON;
export const B2_epsilon_sq: number = (B2_epsilon * B2_epsilon);
export const B2_pi: number = 3.14159265359; // Math.PI;

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Tunable Constants

/// You can use this to change the length scale used by your game.
/// For example for inches you could use 39.4.
export const B2_lengthUnitsPerMeter: number = 1.0;

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because B2BlockAllocator has a maximum object size.
export const B2_maxPolygonVertices: number = 8;

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
export const B2_maxManifoldPoints: number = 2;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
export const B2_aabbExtension: number = 0.1 * B2_lengthUnitsPerMeter;

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
export const B2_aabbMultiplier: number = 4;

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
export const B2_linearSlop: number = 0.005 * B2_lengthUnitsPerMeter;

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
export const B2_angularSlop: number = 2 / 180 * B2_pi;

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
export const B2_polygonRadius: number = 2 * B2_linearSlop;

/// Maximum number of sub-steps per contact in continuous physics simulation.
export const B2_maxSubSteps: number = 8;

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
export const B2_maxTOIContacts: number = 32;

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
export const B2_maxLinearCorrection: number = 0.2 * B2_lengthUnitsPerMeter;

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
export const B2_maxAngularCorrection: number = 8 / 180 * B2_pi;

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
export const B2_maxTranslation: number = 2 * B2_lengthUnitsPerMeter;
export const B2_maxTranslationSquared: number = B2_maxTranslation * B2_maxTranslation;

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
export const B2_maxRotation: number = 0.5 * B2_pi;
export const B2_maxRotationSquared: number = B2_maxRotation * B2_maxRotation;

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
export const B2_baumgarte: number = 0.2;
export const B2_toiBaumgarte: number = 0.75;

// #if B2_ENABLE_PARTICLE

// Particle

/// A symbolic constant that stands for particle allocation error.
export const B2_invalidParticleIndex: number = -1;

export const B2_maxParticleIndex: number = 0x7FFFFFFF;

/// The default distance between particles, multiplied by the particle diameter.
export const B2_particleStride: number = 0.75;

/// The minimum particle weight that produces pressure.
export const B2_minParticleWeight: number = 1.0;

/// The upper limit for particle pressure.
export const B2_maxParticlePressure: number = 0.25;

/// The upper limit for force between particles.
export const B2_maxParticleForce: number = 0.5;

/// The maximum distance between particles in a triad, multiplied by the particle diameter.
export const B2_maxTriadDistance: number = 2.0 * B2_lengthUnitsPerMeter;
export const B2_maxTriadDistanceSquared: number = (B2_maxTriadDistance * B2_maxTriadDistance);

/// The initial size of particle data buffers.
export const B2_minParticleSystemBufferCapacity: number = 256;

/// The time into the future that collisions against barrier particles will be detected.
export const B2_barrierCollisionTime: number = 2.5;

// #endif

// Sleep

/// The time that a body must be still before it will go to sleep.
export const B2_timeToSleep: number = 0.5;

/// A body cannot sleep if its linear velocity is above this tolerance.
export const B2_linearSleepTolerance: number = 0.01 * B2_lengthUnitsPerMeter;

/// A body cannot sleep if its angular velocity is above this tolerance.
export const B2_angularSleepTolerance: number = 2 / 180 * B2_pi;

// FILE* B2_dumpFile = nullptr;

// void B2OpenDump(const char* fileName)
// {
// 	B2Assert(B2_dumpFile == nullptr);
// 	B2_dumpFile = fopen(fileName, "w");
// }

// void B2Dump(const char* string, ...)
// {
// 	if (B2_dumpFile == nullptr)
// 	{
// 		return;
// 	}

// 	va_list args;
// 	va_start(args, string);
// 	vfprintf(B2_dumpFile, string, args);
// 	va_end(args);
// }

// void B2CloseDump()
// {
// 	fclose(B2_dumpFile);
// 	B2_dumpFile = nullptr;
// }

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
export class B2Version {
  public major: number = 0; ///< significant changes
  public minor: number = 0; ///< incremental changes
  public revision: number = 0; ///< bug fixes

  constructor(major: number = 0, minor: number = 0, revision: number = 0) {
    this.major = major;
    this.minor = minor;
    this.revision = revision;
  }

  public toString(): string {
    return this.major + "." + this.minor + "." + this.revision;
  }
}

/// Current version.
export const B2_version: B2Version = new B2Version(2, 4, 1);

export const B2_branch: string = "master";
export const B2_commit: string = "9ebbbcd960ad424e03e5de6e66a40764c16f51bc";

export function B2ParseInt(v: string): number {
  return parseInt(v, 10);
}

export function B2ParseUInt(v: string): number {
  return Math.abs(parseInt(v, 10));
}

export function B2MakeArray<T>(length: number, init: (i: number) => T): T[] {
  const a: T[] = new Array<T>(length);
  for (let i: number = 0; i < length; ++i) {
    a[i] = init(i);
  }
  return a;
}

export function B2MakeNullArray<T>(length: number): Array<T | null> {
  const a: Array<T | null> = new Array<T | null>(length);
  for (let i: number = 0; i < length; ++i) {
    a[i] = null;
  }
  return a;
}

export function B2MakeNumberArray(length: number, init: number = 0): number[] {
  const a: number[] = new Array<number>(length);
  for (let i: number = 0; i < length; ++i) {
    a[i] = init;
  }
  return a;
}
