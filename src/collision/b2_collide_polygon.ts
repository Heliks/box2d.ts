// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import { B2_linearSlop, B2_maxFloat, B2_maxManifoldPoints } from "../common/b2_settings.js";
import { B2Vec2, B2Transform, B2Rot } from "../common/b2_math.js";
import { B2Manifold, B2ClipVertex, B2ContactFeature, B2ContactFeatureType, B2ManifoldType, B2ClipSegmentToLine, B2ManifoldPoint } from "./b2_collision.js";
import { B2PolygonShape } from "./b2_polygon_shape.js";

// Find the max separation between poly1 and poly2 using edge normals from poly1.
const B2FindMaxSeparation_s_xf: B2Transform = new B2Transform();
const B2FindMaxSeparation_s_n: B2Vec2 = new B2Vec2();
const B2FindMaxSeparation_s_v1: B2Vec2 = new B2Vec2();
function B2FindMaxSeparation(edgeIndex: [number], poly1: B2PolygonShape, xf1: B2Transform, poly2: B2PolygonShape, xf2: B2Transform): number {
  const count1: number = poly1.m_count;
  const count2: number = poly2.m_count;
  const n1s: B2Vec2[] = poly1.m_normals;
  const v1s: B2Vec2[] = poly1.m_vertices;
  const v2s: B2Vec2[] = poly2.m_vertices;
  const xf: B2Transform = B2Transform.MulTXX(xf2, xf1, B2FindMaxSeparation_s_xf);

  let bestIndex: number = 0;
  let maxSeparation: number = -B2_maxFloat;

  for (let i: number = 0; i < count1; ++i) {
    // Get poly1 normal in frame2.
    const n: B2Vec2 = B2Rot.MulRV(xf.q, n1s[i], B2FindMaxSeparation_s_n);
    const v1: B2Vec2 = B2Transform.MulXV(xf, v1s[i], B2FindMaxSeparation_s_v1);

    // Find deepest point for normal i.
    let si: number = B2_maxFloat;
    for (let j: number = 0; j < count2; ++j) {
      const sij = B2Vec2.DotVV(n, B2Vec2.SubVV(v2s[j], v1, B2Vec2.s_t0));
      if (sij < si) {
        si = sij;
      }
    }

    if (si > maxSeparation) {
      maxSeparation = si;
      bestIndex = i;
    }
  }

  edgeIndex[0] = bestIndex;
  return maxSeparation;
}

const B2FindIncidentEdge_s_normal1: B2Vec2 = new B2Vec2();
function B2FindIncidentEdge(c: [B2ClipVertex, B2ClipVertex], poly1: B2PolygonShape, xf1: B2Transform, edge1: number, poly2: B2PolygonShape, xf2: B2Transform): void {
  const normals1: B2Vec2[] = poly1.m_normals;

  const count2: number = poly2.m_count;
  const vertices2: B2Vec2[] = poly2.m_vertices;
  const normals2: B2Vec2[] = poly2.m_normals;

  // DEBUG: B2Assert(0 <= edge1 && edge1 < poly1.m_count);

  // Get the normal of the reference edge in poly2's frame.
  const normal1: B2Vec2 = B2Rot.MulTRV(xf2.q, B2Rot.MulRV(xf1.q, normals1[edge1], B2Vec2.s_t0), B2FindIncidentEdge_s_normal1);

  // Find the incident edge on poly2.
  let index: number = 0;
  let minDot: number = B2_maxFloat;
  for (let i: number = 0; i < count2; ++i) {
    const dot: number = B2Vec2.DotVV(normal1, normals2[i]);
    if (dot < minDot) {
      minDot = dot;
      index = i;
    }
  }

  // Build the clip vertices for the incident edge.
  const i1: number = index;
  const i2: number = i1 + 1 < count2 ? i1 + 1 : 0;

  const c0: B2ClipVertex = c[0];
  B2Transform.MulXV(xf2, vertices2[i1], c0.v);
  const cf0: B2ContactFeature = c0.id.cf;
  cf0.indexA = edge1;
  cf0.indexB = i1;
  cf0.typeA = B2ContactFeatureType.e_face;
  cf0.typeB = B2ContactFeatureType.e_vertex;

  const c1: B2ClipVertex = c[1];
  B2Transform.MulXV(xf2, vertices2[i2], c1.v);
  const cf1: B2ContactFeature = c1.id.cf;
  cf1.indexA = edge1;
  cf1.indexB = i2;
  cf1.typeA = B2ContactFeatureType.e_face;
  cf1.typeB = B2ContactFeatureType.e_vertex;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
const B2CollidePolygons_s_incidentEdge: [B2ClipVertex, B2ClipVertex] = [ new B2ClipVertex(), new B2ClipVertex() ];
const B2CollidePolygons_s_clipPoints1: [B2ClipVertex, B2ClipVertex] = [ new B2ClipVertex(), new B2ClipVertex() ];
const B2CollidePolygons_s_clipPoints2: [B2ClipVertex, B2ClipVertex] = [ new B2ClipVertex(), new B2ClipVertex() ];
const B2CollidePolygons_s_edgeA: [number] = [ 0 ];
const B2CollidePolygons_s_edgeB: [number] = [ 0 ];
const B2CollidePolygons_s_localTangent: B2Vec2 = new B2Vec2();
const B2CollidePolygons_s_localNormal: B2Vec2 = new B2Vec2();
const B2CollidePolygons_s_planePoint: B2Vec2 = new B2Vec2();
const B2CollidePolygons_s_normal: B2Vec2 = new B2Vec2();
const B2CollidePolygons_s_tangent: B2Vec2 = new B2Vec2();
const B2CollidePolygons_s_ntangent: B2Vec2 = new B2Vec2();
const B2CollidePolygons_s_v11: B2Vec2 = new B2Vec2();
const B2CollidePolygons_s_v12: B2Vec2 = new B2Vec2();
export function B2CollidePolygons(manifold: B2Manifold, polyA: B2PolygonShape, xfA: B2Transform, polyB: B2PolygonShape, xfB: B2Transform): void {
  manifold.pointCount = 0;
  const totalRadius: number = polyA.m_radius + polyB.m_radius;

  const edgeA: [number] = B2CollidePolygons_s_edgeA; edgeA[0] = 0;
  const separationA: number = B2FindMaxSeparation(edgeA, polyA, xfA, polyB, xfB);
  if (separationA > totalRadius) {
    return;
  }

  const edgeB: [number] = B2CollidePolygons_s_edgeB; edgeB[0] = 0;
  const separationB: number = B2FindMaxSeparation(edgeB, polyB, xfB, polyA, xfA);
  if (separationB > totalRadius) {
    return;
  }

  let poly1: B2PolygonShape; // reference polygon
  let poly2: B2PolygonShape; // incident polygon
  let xf1: B2Transform, xf2: B2Transform;
  let edge1: number = 0; // reference edge
  let flip: number = 0;
  const k_tol: number = 0.1 * B2_linearSlop;

  if (separationB > separationA + k_tol) {
    poly1 = polyB;
    poly2 = polyA;
    xf1 = xfB;
    xf2 = xfA;
    edge1 = edgeB[0];
    manifold.type = B2ManifoldType.e_faceB;
    flip = 1;
  } else {
    poly1 = polyA;
    poly2 = polyB;
    xf1 = xfA;
    xf2 = xfB;
    edge1 = edgeA[0];
    manifold.type = B2ManifoldType.e_faceA;
    flip = 0;
  }

  const incidentEdge: [B2ClipVertex, B2ClipVertex] = B2CollidePolygons_s_incidentEdge;
  B2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

  const count1: number = poly1.m_count;
  const vertices1: B2Vec2[] = poly1.m_vertices;

  const iv1: number = edge1;
  const iv2: number = edge1 + 1 < count1 ? edge1 + 1 : 0;

  const local_v11: B2Vec2 = vertices1[iv1];
  const local_v12: B2Vec2 = vertices1[iv2];

  const localTangent: B2Vec2 = B2Vec2.SubVV(local_v12, local_v11, B2CollidePolygons_s_localTangent);
  localTangent.Normalize();

  const localNormal: B2Vec2 = B2Vec2.CrossVOne(localTangent, B2CollidePolygons_s_localNormal);
  const planePoint: B2Vec2 = B2Vec2.MidVV(local_v11, local_v12, B2CollidePolygons_s_planePoint);

  const tangent: B2Vec2 = B2Rot.MulRV(xf1.q, localTangent, B2CollidePolygons_s_tangent);
  const normal: B2Vec2 = B2Vec2.CrossVOne(tangent, B2CollidePolygons_s_normal);

  const v11: B2Vec2 = B2Transform.MulXV(xf1, local_v11, B2CollidePolygons_s_v11);
  const v12: B2Vec2 = B2Transform.MulXV(xf1, local_v12, B2CollidePolygons_s_v12);

  // Face offset.
  const frontOffset: number = B2Vec2.DotVV(normal, v11);

  // Side offsets, extended by polytope skin thickness.
  const sideOffset1: number = -B2Vec2.DotVV(tangent, v11) + totalRadius;
  const sideOffset2: number = B2Vec2.DotVV(tangent, v12) + totalRadius;

  // Clip incident edge against extruded edge1 side edges.
  const clipPoints1: [B2ClipVertex, B2ClipVertex] = B2CollidePolygons_s_clipPoints1;
  const clipPoints2: [B2ClipVertex, B2ClipVertex] = B2CollidePolygons_s_clipPoints2;
  let np: number;

  // Clip to box side 1
  const ntangent: B2Vec2 = B2Vec2.NegV(tangent, B2CollidePolygons_s_ntangent);
  np = B2ClipSegmentToLine(clipPoints1, incidentEdge, ntangent, sideOffset1, iv1);

  if (np < 2) {
    return;
  }

  // Clip to negative box side 1
  np = B2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

  if (np < 2) {
    return;
  }

  // Now clipPoints2 contains the clipped points.
  manifold.localNormal.Copy(localNormal);
  manifold.localPoint.Copy(planePoint);

  let pointCount: number = 0;
  for (let i: number = 0; i < B2_maxManifoldPoints; ++i) {
    const cv: B2ClipVertex = clipPoints2[i];
    const separation: number = B2Vec2.DotVV(normal, cv.v) - frontOffset;

    if (separation <= totalRadius) {
      const cp: B2ManifoldPoint = manifold.points[pointCount];
      B2Transform.MulTXV(xf2, cv.v, cp.localPoint);
      cp.id.Copy(cv.id);
      if (flip) {
        // Swap features
        const cf: B2ContactFeature = cp.id.cf;
        cp.id.cf.indexA = cf.indexB;
        cp.id.cf.indexB = cf.indexA;
        cp.id.cf.typeA = cf.typeB;
        cp.id.cf.typeB = cf.typeA;
      }
      ++pointCount;
    }
  }

  manifold.pointCount = pointCount;
}
