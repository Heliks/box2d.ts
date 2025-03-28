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

// DEBUG: import { B2Assert } from "../common/b2_settings.js";
import { B2_epsilon, B2_epsilon_sq, B2_polygonRadius, B2_linearSlop } from "../common/b2_settings.js";
import { B2Max, B2Vec2, B2Rot, B2Transform } from "../common/b2_math.js";
import { B2Shape } from "./b2_shape.js";

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
export class B2DistanceProxy {
  public readonly m_buffer: B2Vec2[] = B2Vec2.MakeArray(2);
  public m_vertices: B2Vec2[] = this.m_buffer;
  public m_count: number = 0;
  public m_radius: number = 0;

  public Copy(other: Readonly<B2DistanceProxy>): this {
    if (other.m_vertices === other.m_buffer) {
      this.m_vertices = this.m_buffer;
      this.m_buffer[0].Copy(other.m_buffer[0]);
      this.m_buffer[1].Copy(other.m_buffer[1]);
    } else {
      this.m_vertices = other.m_vertices;
    }
    this.m_count = other.m_count;
    this.m_radius = other.m_radius;
    return this;
  }

  public Reset(): B2DistanceProxy {
    this.m_vertices = this.m_buffer;
    this.m_count = 0;
    this.m_radius = 0;
    return this;
  }

  public SetShape(shape: B2Shape, index: number): void {
    shape.SetupDistanceProxy(this, index);
  }

  public SetVerticesRadius(vertices: B2Vec2[], count: number, radius: number): void {
    this.m_vertices = vertices;
    this.m_count = count;
    this.m_radius = radius;
  }

  public GetSupport(d: B2Vec2): number {
    let bestIndex: number = 0;
    let bestValue: number = B2Vec2.DotVV(this.m_vertices[0], d);
    for (let i: number = 1; i < this.m_count; ++i) {
      const value: number = B2Vec2.DotVV(this.m_vertices[i], d);
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
    }

    return bestIndex;
  }

  public GetSupportVertex(d: B2Vec2): B2Vec2 {
    let bestIndex: number = 0;
    let bestValue: number = B2Vec2.DotVV(this.m_vertices[0], d);
    for (let i: number = 1; i < this.m_count; ++i) {
      const value: number = B2Vec2.DotVV(this.m_vertices[i], d);
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
    }

    return this.m_vertices[bestIndex];
  }

  public GetVertexCount(): number {
    return this.m_count;
  }

  public GetVertex(index: number): B2Vec2 {
    // DEBUG: B2Assert(0 <= index && index < this.m_count);
    return this.m_vertices[index];
  }
}

export class B2SimplexCache {
  public metric: number = 0;
  public count: number = 0;
  public readonly indexA: [number, number, number] = [0, 0, 0];
  public readonly indexB: [number, number, number] = [0, 0, 0];

  public Reset(): B2SimplexCache {
    this.metric = 0;
    this.count = 0;
    return this;
  }
}

export class B2DistanceInput {
  public readonly proxyA: B2DistanceProxy = new B2DistanceProxy();
  public readonly proxyB: B2DistanceProxy = new B2DistanceProxy();
  public readonly transformA: B2Transform = new B2Transform();
  public readonly transformB: B2Transform = new B2Transform();
  public useRadii: boolean = false;

  public Reset(): B2DistanceInput {
    this.proxyA.Reset();
    this.proxyB.Reset();
    this.transformA.SetIdentity();
    this.transformB.SetIdentity();
    this.useRadii = false;
    return this;
  }
}

export class B2DistanceOutput {
  public readonly pointA: B2Vec2 = new B2Vec2();
  public readonly pointB: B2Vec2 = new B2Vec2();
  public distance: number = 0;
  public iterations: number = 0; ///< number of GJK iterations used

  public Reset(): B2DistanceOutput {
    this.pointA.SetZero();
    this.pointB.SetZero();
    this.distance = 0;
    this.iterations = 0;
    return this;
  }
}

/// Input parameters for B2ShapeCast
export class B2ShapeCastInput {
  public readonly proxyA: B2DistanceProxy = new B2DistanceProxy();
  public readonly proxyB: B2DistanceProxy = new B2DistanceProxy();
  public readonly transformA: B2Transform = new B2Transform();
  public readonly transformB: B2Transform = new B2Transform();
  public readonly translationB: B2Vec2 = new B2Vec2();
}

/// Output results for B2ShapeCast
export class B2ShapeCastOutput {
  public readonly point: B2Vec2 = new B2Vec2();
  public readonly normal: B2Vec2 = new B2Vec2();
  public lambda: number = 0.0;
  public iterations: number = 0;
}

export let B2_gjkCalls: number = 0;
export let B2_gjkIters: number = 0;
export let B2_gjkMaxIters: number = 0;
export function B2_gjk_reset(): void {
  B2_gjkCalls = 0;
  B2_gjkIters = 0;
  B2_gjkMaxIters = 0;
}

export class B2SimplexVertex {
  public readonly wA: B2Vec2 = new B2Vec2(); // support point in proxyA
  public readonly wB: B2Vec2 = new B2Vec2(); // support point in proxyB
  public readonly w: B2Vec2 = new B2Vec2(); // wB - wA
  public a: number = 0; // barycentric coordinate for closest point
  public indexA: number = 0; // wA index
  public indexB: number = 0; // wB index

  public Copy(other: B2SimplexVertex): B2SimplexVertex {
    this.wA.Copy(other.wA);     // support point in proxyA
    this.wB.Copy(other.wB);     // support point in proxyB
    this.w.Copy(other.w);       // wB - wA
    this.a = other.a;           // barycentric coordinate for closest point
    this.indexA = other.indexA; // wA index
    this.indexB = other.indexB; // wB index
    return this;
  }
}

export class B2Simplex {
  public readonly m_v1: B2SimplexVertex = new B2SimplexVertex();
  public readonly m_v2: B2SimplexVertex = new B2SimplexVertex();
  public readonly m_v3: B2SimplexVertex = new B2SimplexVertex();
  public readonly m_vertices: B2SimplexVertex[] = [/*3*/];
  public m_count: number = 0;

  constructor() {
    this.m_vertices[0] = this.m_v1;
    this.m_vertices[1] = this.m_v2;
    this.m_vertices[2] = this.m_v3;
  }

  public ReadCache(cache: B2SimplexCache, proxyA: B2DistanceProxy, transformA: B2Transform, proxyB: B2DistanceProxy, transformB: B2Transform): void {
    // DEBUG: B2Assert(0 <= cache.count && cache.count <= 3);

    // Copy data from cache.
    this.m_count = cache.count;
    const vertices: B2SimplexVertex[] = this.m_vertices;
    for (let i: number = 0; i < this.m_count; ++i) {
      const v: B2SimplexVertex = vertices[i];
      v.indexA = cache.indexA[i];
      v.indexB = cache.indexB[i];
      const wALocal: B2Vec2 = proxyA.GetVertex(v.indexA);
      const wBLocal: B2Vec2 = proxyB.GetVertex(v.indexB);
      B2Transform.MulXV(transformA, wALocal, v.wA);
      B2Transform.MulXV(transformB, wBLocal, v.wB);
      B2Vec2.SubVV(v.wB, v.wA, v.w);
      v.a = 0;
    }

    // Compute the new simplex metric, if it is substantially different than
    // old metric then flush the simplex.
    if (this.m_count > 1) {
      const metric1: number = cache.metric;
      const metric2: number = this.GetMetric();
      if (metric2 < 0.5 * metric1 || 2 * metric1 < metric2 || metric2 < B2_epsilon) {
        // Reset the simplex.
        this.m_count = 0;
      }
    }

    // If the cache is empty or invalid ...
    if (this.m_count === 0) {
      const v: B2SimplexVertex = vertices[0];
      v.indexA = 0;
      v.indexB = 0;
      const wALocal: B2Vec2 = proxyA.GetVertex(0);
      const wBLocal: B2Vec2 = proxyB.GetVertex(0);
      B2Transform.MulXV(transformA, wALocal, v.wA);
      B2Transform.MulXV(transformB, wBLocal, v.wB);
      B2Vec2.SubVV(v.wB, v.wA, v.w);
      v.a = 1;
      this.m_count = 1;
    }
  }

  public WriteCache(cache: B2SimplexCache): void {
    cache.metric = this.GetMetric();
    cache.count = this.m_count;
    const vertices: B2SimplexVertex[] = this.m_vertices;
    for (let i: number = 0; i < this.m_count; ++i) {
      cache.indexA[i] = vertices[i].indexA;
      cache.indexB[i] = vertices[i].indexB;
    }
  }

  public GetSearchDirection(out: B2Vec2): B2Vec2 {
    switch (this.m_count) {
      case 1:
        return B2Vec2.NegV(this.m_v1.w, out);

      case 2: {
        const e12: B2Vec2 = B2Vec2.SubVV(this.m_v2.w, this.m_v1.w, out);
        const sgn: number = B2Vec2.CrossVV(e12, B2Vec2.NegV(this.m_v1.w, B2Vec2.s_t0));
        if (sgn > 0) {
          // Origin is left of e12.
          return B2Vec2.CrossOneV(e12, out);
        } else {
          // Origin is right of e12.
          return B2Vec2.CrossVOne(e12, out);
        }
      }

      default:
        // DEBUG: B2Assert(false);
        return out.SetZero();
    }
  }

  public GetClosestPoint(out: B2Vec2): B2Vec2 {
    switch (this.m_count) {
      case 0:
        // DEBUG: B2Assert(false);
        return out.SetZero();

      case 1:
        return out.Copy(this.m_v1.w);

      case 2:
        return out.Set(
          this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x,
          this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);

      case 3:
        return out.SetZero();

      default:
        // DEBUG: B2Assert(false);
        return out.SetZero();
    }
  }

  public GetWitnessPoints(pA: B2Vec2, pB: B2Vec2): void {
    switch (this.m_count) {
      case 0:
        // DEBUG: B2Assert(false);
        break;

      case 1:
        pA.Copy(this.m_v1.wA);
        pB.Copy(this.m_v1.wB);
        break;

      case 2:
        pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
        pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
        pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
        pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
        break;

      case 3:
        pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
        pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
        break;

      default:
        // DEBUG: B2Assert(false);
        break;
    }
  }

  public GetMetric(): number {
    switch (this.m_count) {
      case 0:
        // DEBUG: B2Assert(false);
        return 0;

      case 1:
        return 0;

      case 2:
        return B2Vec2.DistanceVV(this.m_v1.w, this.m_v2.w);

      case 3:
        return B2Vec2.CrossVV(B2Vec2.SubVV(this.m_v2.w, this.m_v1.w, B2Vec2.s_t0), B2Vec2.SubVV(this.m_v3.w, this.m_v1.w, B2Vec2.s_t1));

      default:
        // DEBUG: B2Assert(false);
        return 0;
    }
  }

  public Solve2(): void {
    const w1: B2Vec2 = this.m_v1.w;
    const w2: B2Vec2 = this.m_v2.w;
    const e12: B2Vec2 = B2Vec2.SubVV(w2, w1, B2Simplex.s_e12);

    // w1 region
    const d12_2: number = (-B2Vec2.DotVV(w1, e12));
    if (d12_2 <= 0) {
      // a2 <= 0, so we clamp it to 0
      this.m_v1.a = 1;
      this.m_count = 1;
      return;
    }

    // w2 region
    const d12_1: number = B2Vec2.DotVV(w2, e12);
    if (d12_1 <= 0) {
      // a1 <= 0, so we clamp it to 0
      this.m_v2.a = 1;
      this.m_count = 1;
      this.m_v1.Copy(this.m_v2);
      return;
    }

    // Must be in e12 region.
    const inv_d12: number = 1 / (d12_1 + d12_2);
    this.m_v1.a = d12_1 * inv_d12;
    this.m_v2.a = d12_2 * inv_d12;
    this.m_count = 2;
  }

  public Solve3(): void {
    const w1: B2Vec2 = this.m_v1.w;
    const w2: B2Vec2 = this.m_v2.w;
    const w3: B2Vec2 = this.m_v3.w;

    // Edge12
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    // a3 = 0
    const e12: B2Vec2 = B2Vec2.SubVV(w2, w1, B2Simplex.s_e12);
    const w1e12: number = B2Vec2.DotVV(w1, e12);
    const w2e12: number = B2Vec2.DotVV(w2, e12);
    const d12_1: number = w2e12;
    const d12_2: number = (-w1e12);

    // Edge13
    // [1      1     ][a1] = [1]
    // [w1.e13 w3.e13][a3] = [0]
    // a2 = 0
    const e13: B2Vec2 = B2Vec2.SubVV(w3, w1, B2Simplex.s_e13);
    const w1e13: number = B2Vec2.DotVV(w1, e13);
    const w3e13: number = B2Vec2.DotVV(w3, e13);
    const d13_1: number = w3e13;
    const d13_2: number = (-w1e13);

    // Edge23
    // [1      1     ][a2] = [1]
    // [w2.e23 w3.e23][a3] = [0]
    // a1 = 0
    const e23: B2Vec2 = B2Vec2.SubVV(w3, w2, B2Simplex.s_e23);
    const w2e23: number = B2Vec2.DotVV(w2, e23);
    const w3e23: number = B2Vec2.DotVV(w3, e23);
    const d23_1: number = w3e23;
    const d23_2: number = (-w2e23);

    // Triangle123
    const n123: number = B2Vec2.CrossVV(e12, e13);

    const d123_1: number = n123 * B2Vec2.CrossVV(w2, w3);
    const d123_2: number = n123 * B2Vec2.CrossVV(w3, w1);
    const d123_3: number = n123 * B2Vec2.CrossVV(w1, w2);

    // w1 region
    if (d12_2 <= 0 && d13_2 <= 0) {
      this.m_v1.a = 1;
      this.m_count = 1;
      return;
    }

    // e12
    if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
      const inv_d12: number = 1 / (d12_1 + d12_2);
      this.m_v1.a = d12_1 * inv_d12;
      this.m_v2.a = d12_2 * inv_d12;
      this.m_count = 2;
      return;
    }

    // e13
    if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
      const inv_d13: number = 1 / (d13_1 + d13_2);
      this.m_v1.a = d13_1 * inv_d13;
      this.m_v3.a = d13_2 * inv_d13;
      this.m_count = 2;
      this.m_v2.Copy(this.m_v3);
      return;
    }

    // w2 region
    if (d12_1 <= 0 && d23_2 <= 0) {
      this.m_v2.a = 1;
      this.m_count = 1;
      this.m_v1.Copy(this.m_v2);
      return;
    }

    // w3 region
    if (d13_1 <= 0 && d23_1 <= 0) {
      this.m_v3.a = 1;
      this.m_count = 1;
      this.m_v1.Copy(this.m_v3);
      return;
    }

    // e23
    if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
      const inv_d23: number = 1 / (d23_1 + d23_2);
      this.m_v2.a = d23_1 * inv_d23;
      this.m_v3.a = d23_2 * inv_d23;
      this.m_count = 2;
      this.m_v1.Copy(this.m_v3);
      return;
    }

    // Must be in triangle123
    const inv_d123: number = 1 / (d123_1 + d123_2 + d123_3);
    this.m_v1.a = d123_1 * inv_d123;
    this.m_v2.a = d123_2 * inv_d123;
    this.m_v3.a = d123_3 * inv_d123;
    this.m_count = 3;
  }
  private static s_e12: B2Vec2 = new B2Vec2();
  private static s_e13: B2Vec2 = new B2Vec2();
  private static s_e23: B2Vec2 = new B2Vec2();
}

const B2Distance_s_simplex: B2Simplex = new B2Simplex();
const B2Distance_s_saveA: [number, number, number] = [0, 0, 0];
const B2Distance_s_saveB: [number, number, number] = [0, 0, 0];
const B2Distance_s_p: B2Vec2 = new B2Vec2();
const B2Distance_s_d: B2Vec2 = new B2Vec2();
const B2Distance_s_normal: B2Vec2 = new B2Vec2();
const B2Distance_s_supportA: B2Vec2 = new B2Vec2();
const B2Distance_s_supportB: B2Vec2 = new B2Vec2();
export function B2Distance(output: B2DistanceOutput, cache: B2SimplexCache, input: B2DistanceInput): void {
  ++B2_gjkCalls;

  const proxyA: B2DistanceProxy = input.proxyA;
  const proxyB: B2DistanceProxy = input.proxyB;

  const transformA: B2Transform = input.transformA;
  const transformB: B2Transform = input.transformB;

  // Initialize the simplex.
  const simplex: B2Simplex = B2Distance_s_simplex;
  simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

  // Get simplex vertices as an array.
  const vertices: B2SimplexVertex[] = simplex.m_vertices;
  const k_maxIters: number = 20;

  // These store the vertices of the last simplex so that we
  // can check for duplicates and prevent cycling.
  const saveA: [number, number, number] = B2Distance_s_saveA;
  const saveB: [number, number, number] = B2Distance_s_saveB;
  let saveCount: number = 0;

  // Main iteration loop.
  let iter: number = 0;
  while (iter < k_maxIters) {
    // Copy simplex so we can identify duplicates.
    saveCount = simplex.m_count;
    for (let i: number = 0; i < saveCount; ++i) {
      saveA[i] = vertices[i].indexA;
      saveB[i] = vertices[i].indexB;
    }

    switch (simplex.m_count) {
      case 1:
        break;

      case 2:
        simplex.Solve2();
        break;

      case 3:
        simplex.Solve3();
        break;

      default:
        // DEBUG: B2Assert(false);
        break;
    }

    // If we have 3 points, then the origin is in the corresponding triangle.
    if (simplex.m_count === 3) {
      break;
    }

    // Get search direction.
    const d: B2Vec2 = simplex.GetSearchDirection(B2Distance_s_d);

    // Ensure the search direction is numerically fit.
    if (d.LengthSquared() < B2_epsilon_sq) {
      // The origin is probably contained by a line segment
      // or triangle. Thus the shapes are overlapped.

      // We can't return zero here even though there may be overlap.
      // In case the simplex is a point, segment, or triangle it is difficult
      // to determine if the origin is contained in the CSO or very close to it.
      break;
    }

    // Compute a tentative new simplex vertex using support points.
    const vertex: B2SimplexVertex = vertices[simplex.m_count];
    vertex.indexA = proxyA.GetSupport(B2Rot.MulTRV(transformA.q, B2Vec2.NegV(d, B2Vec2.s_t0), B2Distance_s_supportA));
    B2Transform.MulXV(transformA, proxyA.GetVertex(vertex.indexA), vertex.wA);
    vertex.indexB = proxyB.GetSupport(B2Rot.MulTRV(transformB.q, d, B2Distance_s_supportB));
    B2Transform.MulXV(transformB, proxyB.GetVertex(vertex.indexB), vertex.wB);
    B2Vec2.SubVV(vertex.wB, vertex.wA, vertex.w);

    // Iteration count is equated to the number of support point calls.
    ++iter;
    ++B2_gjkIters;

    // Check for duplicate support points. This is the main termination criteria.
    let duplicate: boolean = false;
    for (let i: number = 0; i < saveCount; ++i) {
      if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
        duplicate = true;
        break;
      }
    }

    // If we found a duplicate support point we must exit to avoid cycling.
    if (duplicate) {
      break;
    }

    // New vertex is ok and needed.
    ++simplex.m_count;
  }

  B2_gjkMaxIters = B2Max(B2_gjkMaxIters, iter);

  // Prepare output.
  simplex.GetWitnessPoints(output.pointA, output.pointB);
  output.distance = B2Vec2.DistanceVV(output.pointA, output.pointB);
  output.iterations = iter;

  // Cache the simplex.
  simplex.WriteCache(cache);

  // Apply radii if requested.
  if (input.useRadii) {
    const rA: number = proxyA.m_radius;
    const rB: number = proxyB.m_radius;

    if (output.distance > (rA + rB) && output.distance > B2_epsilon) {
      // Shapes are still no overlapped.
      // Move the witness points to the outer surface.
      output.distance -= rA + rB;
      const normal: B2Vec2 = B2Vec2.SubVV(output.pointB, output.pointA, B2Distance_s_normal);
      normal.Normalize();
      output.pointA.SelfMulAdd(rA, normal);
      output.pointB.SelfMulSub(rB, normal);
    } else {
      // Shapes are overlapped when radii are considered.
      // Move the witness points to the middle.
      const p: B2Vec2 = B2Vec2.MidVV(output.pointA, output.pointB, B2Distance_s_p);
      output.pointA.Copy(p);
      output.pointB.Copy(p);
      output.distance = 0;
    }
  }
}

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
// bool B2ShapeCast(B2ShapeCastOutput* output, const B2ShapeCastInput* input);
const B2ShapeCast_s_n = new B2Vec2();
const B2ShapeCast_s_simplex = new B2Simplex();
const B2ShapeCast_s_wA = new B2Vec2();
const B2ShapeCast_s_wB = new B2Vec2();
const B2ShapeCast_s_v = new B2Vec2();
const B2ShapeCast_s_p = new B2Vec2();
const B2ShapeCast_s_pointA = new B2Vec2();
const B2ShapeCast_s_pointB = new B2Vec2();
export function B2ShapeCast(output: B2ShapeCastOutput, input: B2ShapeCastInput): boolean {
  output.iterations = 0;
  output.lambda = 1.0;
  output.normal.SetZero();
  output.point.SetZero();

  // const B2DistanceProxy* proxyA = &input.proxyA;
  const proxyA = input.proxyA;
  // const B2DistanceProxy* proxyB = &input.proxyB;
  const proxyB = input.proxyB;

  // float32 radiusA = B2Max(proxyA.m_radius, B2_polygonRadius);
  const radiusA = B2Max(proxyA.m_radius, B2_polygonRadius);
  // float32 radiusB = B2Max(proxyB.m_radius, B2_polygonRadius);
  const radiusB = B2Max(proxyB.m_radius, B2_polygonRadius);
  // float32 radius = radiusA + radiusB;
  const radius = radiusA + radiusB;

  // B2Transform xfA = input.transformA;
  const xfA = input.transformA;
  // B2Transform xfB = input.transformB;
  const xfB = input.transformB;

  // B2Vec2 r = input.translationB;
  const r = input.translationB;
  // B2Vec2 n(0.0f, 0.0f);
  const n = B2ShapeCast_s_n.Set(0.0, 0.0);
  // float32 lambda = 0.0f;
  let lambda = 0.0;

  // Initial simplex
  const simplex = B2ShapeCast_s_simplex;
  simplex.m_count = 0;

  // Get simplex vertices as an array.
  // B2SimplexVertex* vertices = &simplex.m_v1;
  const vertices = simplex.m_vertices;

  // Get support point in -r direction
  // int32 indexA = proxyA.GetSupport(B2MulT(xfA.q, -r));
  let indexA = proxyA.GetSupport(B2Rot.MulTRV(xfA.q, B2Vec2.NegV(r, B2Vec2.s_t1), B2Vec2.s_t0));
  // B2Vec2 wA = B2Mul(xfA, proxyA.GetVertex(indexA));
  let wA = B2Transform.MulXV(xfA, proxyA.GetVertex(indexA), B2ShapeCast_s_wA);
  // int32 indexB = proxyB.GetSupport(B2MulT(xfB.q, r));
  let indexB = proxyB.GetSupport(B2Rot.MulTRV(xfB.q, r, B2Vec2.s_t0));
  // B2Vec2 wB = B2Mul(xfB, proxyB.GetVertex(indexB));
  let wB = B2Transform.MulXV(xfB, proxyB.GetVertex(indexB), B2ShapeCast_s_wB);
  // B2Vec2 v = wA - wB;
  const v = B2Vec2.SubVV(wA, wB, B2ShapeCast_s_v);

  // Sigma is the target distance between polygons
  // float32 sigma = B2Max(B2_polygonRadius, radius - B2_polygonRadius);
  const sigma = B2Max(B2_polygonRadius, radius - B2_polygonRadius);
  // const float32 tolerance = 0.5f * B2_linearSlop;
  const tolerance = 0.5 * B2_linearSlop;

  // Main iteration loop.
  // const int32 k_maxIters = 20;
  const k_maxIters = 20;
  // int32 iter = 0;
  let iter = 0;
  // while (iter < k_maxIters && v.Length() - sigma > tolerance)
  while (iter < k_maxIters && v.Length() - sigma > tolerance) {
    // DEBUG: B2Assert(simplex.m_count < 3);

    output.iterations += 1;

    // Support in direction -v (A - B)
    // indexA = proxyA.GetSupport(B2MulT(xfA.q, -v));
    indexA = proxyA.GetSupport(B2Rot.MulTRV(xfA.q, B2Vec2.NegV(v, B2Vec2.s_t1), B2Vec2.s_t0));
    // wA = B2Mul(xfA, proxyA.GetVertex(indexA));
    wA = B2Transform.MulXV(xfA, proxyA.GetVertex(indexA), B2ShapeCast_s_wA);
    // indexB = proxyB.GetSupport(B2MulT(xfB.q, v));
    indexB = proxyB.GetSupport(B2Rot.MulTRV(xfB.q, v, B2Vec2.s_t0));
    // wB = B2Mul(xfB, proxyB.GetVertex(indexB));
    wB = B2Transform.MulXV(xfB, proxyB.GetVertex(indexB), B2ShapeCast_s_wB);
    // B2Vec2 p = wA - wB;
    const p = B2Vec2.SubVV(wA, wB, B2ShapeCast_s_p);

    // -v is a normal at p
    v.Normalize();

    // Intersect ray with plane
    const vp = B2Vec2.DotVV(v, p);
    const vr = B2Vec2.DotVV(v, r);
    if (vp - sigma > lambda * vr) {
      if (vr <= 0.0) {
        return false;
      }

      lambda = (vp - sigma) / vr;
      if (lambda > 1.0) {
        return false;
      }

      // n = -v;
      n.Copy(v).SelfNeg();
      simplex.m_count = 0;
    }

    // Reverse simplex since it works with B - A.
    // Shift by lambda * r because we want the closest point to the current clip point.
    // Note that the support point p is not shifted because we want the plane equation
    // to be formed in unshifted space.
    // B2SimplexVertex* vertex = vertices + simplex.m_count;
    const vertex: B2SimplexVertex = vertices[simplex.m_count];
    vertex.indexA = indexB;
    // vertex.wA = wB + lambda * r;
    vertex.wA.Copy(wB).SelfMulAdd(lambda, r);
    vertex.indexB = indexA;
    // vertex.wB = wA;
    vertex.wB.Copy(wA);
    // vertex.w = vertex.wB - vertex.wA;
    vertex.w.Copy(vertex.wB).SelfSub(vertex.wA);
    vertex.a = 1.0;
    simplex.m_count += 1;

    switch (simplex.m_count) {
      case 1:
        break;

      case 2:
        simplex.Solve2();
        break;

      case 3:
        simplex.Solve3();
        break;

      default:
      // DEBUG: B2Assert(false);
    }

    // If we have 3 points, then the origin is in the corresponding triangle.
    if (simplex.m_count === 3) {
      // Overlap
      return false;
    }

    // Get search direction.
    // v = simplex.GetClosestPoint();
    simplex.GetClosestPoint(v);

    // Iteration count is equated to the number of support point calls.
    ++iter;
  }

  if (iter === 0) {
    // Initial overlap
    return false;
  }

  // Prepare output.
  const pointA = B2ShapeCast_s_pointA;
  const pointB = B2ShapeCast_s_pointB;
  simplex.GetWitnessPoints(pointA, pointB);

  if (v.LengthSquared() > 0.0) {
    // n = -v;
    n.Copy(v).SelfNeg();
    n.Normalize();
  }

  // output.point = pointA + radiusA * n;
  output.normal.Copy(n);
  output.lambda = lambda;
  output.iterations = iter;
  return true;
}
