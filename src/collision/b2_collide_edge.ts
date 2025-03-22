// DEBUG: import { B2Assert } from "../common/b2_settings.js";
import { B2_maxManifoldPoints } from "../common/b2_settings.js";
import { B2Min, B2Vec2, B2Rot, B2Transform } from "../common/b2_math.js";
import { B2ContactFeatureType, B2ContactID } from "./b2_collision.js";
import { B2Manifold, B2ManifoldType, B2ManifoldPoint, B2ClipVertex, B2ClipSegmentToLine } from "./b2_collision.js";
import { B2CircleShape } from "./b2_circle_shape.js";
import { B2PolygonShape } from "./b2_polygon_shape.js";
import { B2EdgeShape } from "./b2_edge_shape.js";

const B2CollideEdgeAndCircle_s_Q: B2Vec2 = new B2Vec2();
const B2CollideEdgeAndCircle_s_e: B2Vec2 = new B2Vec2();
const B2CollideEdgeAndCircle_s_d: B2Vec2 = new B2Vec2();
const B2CollideEdgeAndCircle_s_e1: B2Vec2 = new B2Vec2();
const B2CollideEdgeAndCircle_s_e2: B2Vec2 = new B2Vec2();
const B2CollideEdgeAndCircle_s_P: B2Vec2 = new B2Vec2();
const B2CollideEdgeAndCircle_s_n: B2Vec2 = new B2Vec2();
const B2CollideEdgeAndCircle_s_id: B2ContactID = new B2ContactID();
export function B2CollideEdgeAndCircle(manifold: B2Manifold, edgeA: B2EdgeShape, xfA: B2Transform, circleB: B2CircleShape, xfB: B2Transform): void {
  manifold.pointCount = 0;

  // Compute circle in frame of edge
  const Q: B2Vec2 = B2Transform.MulTXV(xfA, B2Transform.MulXV(xfB, circleB.m_p, B2Vec2.s_t0), B2CollideEdgeAndCircle_s_Q);

  const A: B2Vec2 = edgeA.m_vertex1;
  const B: B2Vec2 = edgeA.m_vertex2;
  const e: B2Vec2 = B2Vec2.SubVV(B, A, B2CollideEdgeAndCircle_s_e);

  // Normal points to the right for a CCW winding
  // B2Vec2 n(e.y, -e.x);
  // const n: B2Vec2 = B2CollideEdgeAndCircle_s_n.Set(-e.y, e.x);
  const n: B2Vec2 = B2CollideEdgeAndCircle_s_n.Set(e.y, -e.x);
  // float offset = B2Dot(n, Q - A);
  const offset: number = B2Vec2.DotVV(n, B2Vec2.SubVV(Q, A, B2Vec2.s_t0));

  const oneSided: boolean = edgeA.m_oneSided;
  if (oneSided && offset < 0.0) {
    return;
  }

  // Barycentric coordinates
  const u: number = B2Vec2.DotVV(e, B2Vec2.SubVV(B, Q, B2Vec2.s_t0));
  const v: number = B2Vec2.DotVV(e, B2Vec2.SubVV(Q, A, B2Vec2.s_t0));

  const radius: number = edgeA.m_radius + circleB.m_radius;

  // const cf: B2ContactFeature = new B2ContactFeature();
  const id: B2ContactID = B2CollideEdgeAndCircle_s_id;
  id.cf.indexB = 0;
  id.cf.typeB = B2ContactFeatureType.e_vertex;

  // Region A
  if (v <= 0) {
    const P: B2Vec2 = A;
    const d: B2Vec2 = B2Vec2.SubVV(Q, P, B2CollideEdgeAndCircle_s_d);
    const dd: number = B2Vec2.DotVV(d, d);
    if (dd > radius * radius) {
      return;
    }

    // Is there an edge connected to A?
    if (edgeA.m_oneSided) {
      const A1: B2Vec2 = edgeA.m_vertex0;
      const B1: B2Vec2 = A;
      const e1: B2Vec2 = B2Vec2.SubVV(B1, A1, B2CollideEdgeAndCircle_s_e1);
      const u1: number = B2Vec2.DotVV(e1, B2Vec2.SubVV(B1, Q, B2Vec2.s_t0));

      // Is the circle in Region AB of the previous edge?
      if (u1 > 0) {
        return;
      }
    }

    id.cf.indexA = 0;
    id.cf.typeA = B2ContactFeatureType.e_vertex;
    manifold.pointCount = 1;
    manifold.type = B2ManifoldType.e_circles;
    manifold.localNormal.SetZero();
    manifold.localPoint.Copy(P);
    manifold.points[0].id.Copy(id);
    // manifold.points[0].id.key = 0;
    // manifold.points[0].id.cf = cf;
    manifold.points[0].localPoint.Copy(circleB.m_p);
    return;
  }

  // Region B
  if (u <= 0) {
    const P: B2Vec2 = B;
    const d: B2Vec2 = B2Vec2.SubVV(Q, P, B2CollideEdgeAndCircle_s_d);
    const dd: number = B2Vec2.DotVV(d, d);
    if (dd > radius * radius) {
      return;
    }

    // Is there an edge connected to B?
    if (edgeA.m_oneSided) {
      const B2: B2Vec2 = edgeA.m_vertex3;
      const A2: B2Vec2 = B;
      const e2: B2Vec2 = B2Vec2.SubVV(B2, A2, B2CollideEdgeAndCircle_s_e2);
      const v2: number = B2Vec2.DotVV(e2, B2Vec2.SubVV(Q, A2, B2Vec2.s_t0));

      // Is the circle in Region AB of the next edge?
      if (v2 > 0) {
        return;
      }
    }

    id.cf.indexA = 1;
    id.cf.typeA = B2ContactFeatureType.e_vertex;
    manifold.pointCount = 1;
    manifold.type = B2ManifoldType.e_circles;
    manifold.localNormal.SetZero();
    manifold.localPoint.Copy(P);
    manifold.points[0].id.Copy(id);
    // manifold.points[0].id.key = 0;
    // manifold.points[0].id.cf = cf;
    manifold.points[0].localPoint.Copy(circleB.m_p);
    return;
  }

  // Region AB
  const den: number = B2Vec2.DotVV(e, e);
  // DEBUG: B2Assert(den > 0);
  const P: B2Vec2 = B2CollideEdgeAndCircle_s_P;
  P.x = (1 / den) * (u * A.x + v * B.x);
  P.y = (1 / den) * (u * A.y + v * B.y);
  const d: B2Vec2 = B2Vec2.SubVV(Q, P, B2CollideEdgeAndCircle_s_d);
  const dd: number = B2Vec2.DotVV(d, d);
  if (dd > radius * radius) {
    return;
  }

  if (offset < 0) {
    n.Set(-n.x, -n.y);
  }
  n.Normalize();

  id.cf.indexA = 0;
  id.cf.typeA = B2ContactFeatureType.e_face;
  manifold.pointCount = 1;
  manifold.type = B2ManifoldType.e_faceA;
  manifold.localNormal.Copy(n);
  manifold.localPoint.Copy(A);
  manifold.points[0].id.Copy(id);
  // manifold.points[0].id.key = 0;
  // manifold.points[0].id.cf = cf;
  manifold.points[0].localPoint.Copy(circleB.m_p);
}

enum B2EPAxisType {
  e_unknown = 0,
  e_edgeA = 1,
  e_edgeB = 2,
}

class B2EPAxis {
  public normal: B2Vec2 = new B2Vec2();
  public type: B2EPAxisType = B2EPAxisType.e_unknown;
  public index: number = 0;
  public separation: number = 0;
}

class B2TempPolygon {
  public vertices: B2Vec2[] = [];
  public normals: B2Vec2[] = [];
  public count: number = 0;
}

class B2ReferenceFace {
  public i1: number = 0;
  public i2: number = 0;
  public readonly v1: B2Vec2 = new B2Vec2();
  public readonly v2: B2Vec2 = new B2Vec2();
  public readonly normal: B2Vec2 = new B2Vec2();
  public readonly sideNormal1: B2Vec2 = new B2Vec2();
  public sideOffset1: number = 0;
  public readonly sideNormal2: B2Vec2 = new B2Vec2();
  public sideOffset2: number = 0;
}

// static B2EPAxis B2ComputeEdgeSeparation(const B2TempPolygon& polygonB, const B2Vec2& v1, const B2Vec2& normal1)
const B2ComputeEdgeSeparation_s_axis = new B2EPAxis();
const B2ComputeEdgeSeparation_s_axes: [ B2Vec2, B2Vec2 ] = [ new B2Vec2(), new B2Vec2() ];
function B2ComputeEdgeSeparation(polygonB: Readonly<B2TempPolygon>, v1: Readonly<B2Vec2>, normal1: Readonly<B2Vec2>): B2EPAxis {
  // B2EPAxis axis;
  const axis: B2EPAxis = B2ComputeEdgeSeparation_s_axis;
  axis.type = B2EPAxisType.e_edgeA;
  axis.index = -1;
  axis.separation = -Number.MAX_VALUE; // -FLT_MAX;
  axis.normal.SetZero();

  // B2Vec2 axes[2] = { normal1, -normal1 };
  const axes: [B2Vec2, B2Vec2] = B2ComputeEdgeSeparation_s_axes;
  axes[0].Copy(normal1);
  axes[1].Copy(normal1).SelfNeg();

  // Find axis with least overlap (min-max problem)
  for (let j = 0; j < 2; ++j) {
    let sj: number = Number.MAX_VALUE; // FLT_MAX;

    // Find deepest polygon vertex along axis j
    for (let i = 0; i < polygonB.count; ++i) {
      // float si = B2Dot(axes[j], polygonB.vertices[i] - v1);
      const si: number = B2Vec2.DotVV(axes[j], B2Vec2.SubVV(polygonB.vertices[i], v1, B2Vec2.s_t0));
      if (si < sj) {
        sj = si;
      }
    }

    if (sj > axis.separation) {
      axis.index = j;
      axis.separation = sj;
      axis.normal.Copy(axes[j]);
    }
  }

  return axis;
}

// static B2EPAxis B2ComputePolygonSeparation(const B2TempPolygon& polygonB, const B2Vec2& v1, const B2Vec2& v2)
const B2ComputePolygonSeparation_s_axis = new B2EPAxis();
const B2ComputePolygonSeparation_s_n = new B2Vec2();
function B2ComputePolygonSeparation(polygonB: Readonly<B2TempPolygon>, v1: Readonly<B2Vec2>, v2: Readonly<B2Vec2>): B2EPAxis {
  const axis: B2EPAxis = B2ComputePolygonSeparation_s_axis;
  axis.type = B2EPAxisType.e_unknown;
  axis.index = -1;
  axis.separation = -Number.MAX_VALUE; // -FLT_MAX;
  axis.normal.SetZero();

  for (let i = 0; i < polygonB.count; ++i) {
    // B2Vec2 n = -polygonB.normals[i];
    const n: B2Vec2 = B2Vec2.NegV(polygonB.normals[i], B2ComputePolygonSeparation_s_n);

    // float s1 = B2Dot(n, polygonB.vertices[i] - v1);
    const s1: number = B2Vec2.DotVV(n, B2Vec2.SubVV(polygonB.vertices[i], v1, B2Vec2.s_t0));
    // float s2 = B2Dot(n, polygonB.vertices[i] - v2);
    const s2: number = B2Vec2.DotVV(n, B2Vec2.SubVV(polygonB.vertices[i], v2, B2Vec2.s_t0));
    // float s = B2Min(s1, s2);
    const s: number = B2Min(s1, s2);

    if (s > axis.separation) {
      axis.type = B2EPAxisType.e_edgeB;
      axis.index = i;
      axis.separation = s;
      axis.normal.Copy(n);
    }
  }

  return axis;
}

const B2CollideEdgeAndPolygon_s_xf = new B2Transform();
const B2CollideEdgeAndPolygon_s_centroidB = new B2Vec2();
const B2CollideEdgeAndPolygon_s_edge1 = new B2Vec2();
const B2CollideEdgeAndPolygon_s_normal1 = new B2Vec2();
const B2CollideEdgeAndPolygon_s_edge0 = new B2Vec2();
const B2CollideEdgeAndPolygon_s_normal0 = new B2Vec2();
const B2CollideEdgeAndPolygon_s_edge2 = new B2Vec2();
const B2CollideEdgeAndPolygon_s_normal2 = new B2Vec2();
const B2CollideEdgeAndPolygon_s_tempPolygonB = new B2TempPolygon();
const B2CollideEdgeAndPolygon_s_ref = new B2ReferenceFace();
const B2CollideEdgeAndPolygon_s_clipPoints: [B2ClipVertex, B2ClipVertex] = [ new B2ClipVertex(), new B2ClipVertex() ];
const B2CollideEdgeAndPolygon_s_clipPoints1: [B2ClipVertex, B2ClipVertex] = [ new B2ClipVertex(), new B2ClipVertex() ];
const B2CollideEdgeAndPolygon_s_clipPoints2: [B2ClipVertex, B2ClipVertex] = [ new B2ClipVertex(), new B2ClipVertex() ];
export function B2CollideEdgeAndPolygon(manifold: B2Manifold, edgeA: B2EdgeShape, xfA: B2Transform, polygonB: B2PolygonShape, xfB: B2Transform): void {
  manifold.pointCount = 0;

  // B2Transform xf = B2MulT(xfA, xfB);
  const xf = B2Transform.MulTXX(xfA, xfB, B2CollideEdgeAndPolygon_s_xf);

  // B2Vec2 centroidB = B2Mul(xf, polygonB.m_centroid);
  const centroidB: B2Vec2 = B2Transform.MulXV(xf, polygonB.m_centroid, B2CollideEdgeAndPolygon_s_centroidB);

  // B2Vec2 v1 = edgeA.m_vertex1;
  const v1: B2Vec2 = edgeA.m_vertex1;
  // B2Vec2 v2 = edgeA.m_vertex2;
  const v2: B2Vec2 = edgeA.m_vertex2;

  // B2Vec2 edge1 = v2 - v1;
  const edge1: B2Vec2 = B2Vec2.SubVV(v2, v1, B2CollideEdgeAndPolygon_s_edge1);
  edge1.Normalize();

  // Normal points to the right for a CCW winding
  // B2Vec2 normal1(edge1.y, -edge1.x);
  const normal1 = B2CollideEdgeAndPolygon_s_normal1.Set(edge1.y, -edge1.x);
  // float offset1 = B2Dot(normal1, centroidB - v1);
  const offset1: number = B2Vec2.DotVV(normal1, B2Vec2.SubVV(centroidB, v1, B2Vec2.s_t0));

  const oneSided: boolean = edgeA.m_oneSided;
  if (oneSided && offset1 < 0.0) {
    return;
  }

  // Get polygonB in frameA
  // B2TempPolygon tempPolygonB;
  const tempPolygonB: B2TempPolygon = B2CollideEdgeAndPolygon_s_tempPolygonB;
  tempPolygonB.count = polygonB.m_count;
  for (let i = 0; i < polygonB.m_count; ++i) {
    if (tempPolygonB.vertices.length <= i) { tempPolygonB.vertices.push(new B2Vec2()); }
    if (tempPolygonB.normals.length <= i) { tempPolygonB.normals.push(new B2Vec2()); }
    // tempPolygonB.vertices[i] = B2Mul(xf, polygonB.m_vertices[i]);
    B2Transform.MulXV(xf, polygonB.m_vertices[i], tempPolygonB.vertices[i]);
    // tempPolygonB.normals[i] = B2Mul(xf.q, polygonB.m_normals[i]);
    B2Rot.MulRV(xf.q, polygonB.m_normals[i], tempPolygonB.normals[i]);
  }

  const radius: number = polygonB.m_radius + edgeA.m_radius;

  // B2EPAxis edgeAxis = B2ComputeEdgeSeparation(tempPolygonB, v1, normal1);
  const edgeAxis: B2EPAxis = B2ComputeEdgeSeparation(tempPolygonB, v1, normal1);
  if (edgeAxis.separation > radius) {
    return;
  }

  // B2EPAxis polygonAxis = B2ComputePolygonSeparation(tedge0.y, -edge0.xempPolygonB, v1, v2);
  const polygonAxis: B2EPAxis = B2ComputePolygonSeparation(tempPolygonB, v1, v2);
  if (polygonAxis.separation > radius) {
    return;
  }

  // Use hysteresis for jitter reduction.
  const k_relativeTol: number = 0.98;
  const k_absoluteTol: number = 0.001;

  // B2EPAxis primaryAxis;
  let primaryAxis: B2EPAxis;
  if (polygonAxis.separation - radius > k_relativeTol * (edgeAxis.separation - radius) + k_absoluteTol) {
    primaryAxis = polygonAxis;
  } else {
    primaryAxis = edgeAxis;
  }

  if (oneSided) {
    // Smooth collision
    // See https://box2d.org/posts/2020/06/ghost-collisions/

    // B2Vec2 edge0 = v1 - edgeA.m_vertex0;
    const edge0: B2Vec2 = B2Vec2.SubVV(v1, edgeA.m_vertex0, B2CollideEdgeAndPolygon_s_edge0);
    edge0.Normalize();
    // B2Vec2 normal0(edge0.y, -edge0.x);
    const normal0: B2Vec2 = B2CollideEdgeAndPolygon_s_normal0.Set(edge0.y, -edge0.x);
    const convex1: boolean = B2Vec2.CrossVV(edge0, edge1) >= 0.0;

    // B2Vec2 edge2 = edgeA.m_vertex3 - v2;
    const edge2: B2Vec2 = B2Vec2.SubVV(edgeA.m_vertex3, v2, B2CollideEdgeAndPolygon_s_edge2);
    edge2.Normalize();
    // B2Vec2 normal2(edge2.y, -edge2.x);
    const normal2: B2Vec2 = B2CollideEdgeAndPolygon_s_normal2.Set(edge2.y, -edge2.x);
    const convex2: boolean = B2Vec2.CrossVV(edge1, edge2) >= 0.0;

    const sinTol: number = 0.1;
    const side1: boolean = B2Vec2.DotVV(primaryAxis.normal, edge1) <= 0.0;

    // Check Gauss Map
    if (side1) {
      if (convex1) {
        if (B2Vec2.CrossVV(primaryAxis.normal, normal0) > sinTol) {
          // Skip region
          return;
        }

        // Admit region
      } else {
        // Snap region
        primaryAxis = edgeAxis;
      }
    } else {
      if (convex2) {
        if (B2Vec2.CrossVV(normal2, primaryAxis.normal) > sinTol) {
          // Skip region
          return;
        }

        // Admit region
      } else {
        // Snap region
        primaryAxis = edgeAxis;
      }
    }
  }

  // B2ClipVertex clipPoints[2];
  const clipPoints: [B2ClipVertex, B2ClipVertex] = B2CollideEdgeAndPolygon_s_clipPoints;
  // B2ReferenceFace ref;
  const ref: B2ReferenceFace = B2CollideEdgeAndPolygon_s_ref;
  if (primaryAxis.type === B2EPAxisType.e_edgeA) {
    manifold.type = B2ManifoldType.e_faceA;

    // Search for the polygon normal that is most anti-parallel to the edge normal.
    let bestIndex: number = 0;
    let bestValue: number = B2Vec2.DotVV(primaryAxis.normal, tempPolygonB.normals[0]);
    for (let i = 1; i < tempPolygonB.count; ++i) {
      const value: number = B2Vec2.DotVV(primaryAxis.normal, tempPolygonB.normals[i]);
      if (value < bestValue) {
        bestValue = value;
        bestIndex = i;
      }
    }

    const i1: number = bestIndex;
    const i2: number = i1 + 1 < tempPolygonB.count ? i1 + 1 : 0;

    clipPoints[0].v.Copy(tempPolygonB.vertices[i1]);
    clipPoints[0].id.cf.indexA = 0;
    clipPoints[0].id.cf.indexB = i1;
    clipPoints[0].id.cf.typeA = B2ContactFeatureType.e_face;
    clipPoints[0].id.cf.typeB = B2ContactFeatureType.e_vertex;

    clipPoints[1].v.Copy(tempPolygonB.vertices[i2]);
    clipPoints[1].id.cf.indexA = 0;
    clipPoints[1].id.cf.indexB = i2;
    clipPoints[1].id.cf.typeA = B2ContactFeatureType.e_face;
    clipPoints[1].id.cf.typeB = B2ContactFeatureType.e_vertex;

    ref.i1 = 0;
    ref.i2 = 1;
    ref.v1.Copy(v1);
    ref.v2.Copy(v2);
    ref.normal.Copy(primaryAxis.normal);
    ref.sideNormal1.Copy(edge1).SelfNeg(); // ref.sideNormal1 = -edge1;
    ref.sideNormal2.Copy(edge1);
  } else {
    manifold.type = B2ManifoldType.e_faceB;

    clipPoints[0].v.Copy(v2);
    clipPoints[0].id.cf.indexA = 1;
    clipPoints[0].id.cf.indexB = primaryAxis.index;
    clipPoints[0].id.cf.typeA = B2ContactFeatureType.e_vertex;
    clipPoints[0].id.cf.typeB = B2ContactFeatureType.e_face;

    clipPoints[1].v.Copy(v1);
    clipPoints[1].id.cf.indexA = 0;
    clipPoints[1].id.cf.indexB = primaryAxis.index;
    clipPoints[1].id.cf.typeA = B2ContactFeatureType.e_vertex;
    clipPoints[1].id.cf.typeB = B2ContactFeatureType.e_face;

    ref.i1 = primaryAxis.index;
    ref.i2 = ref.i1 + 1 < tempPolygonB.count ? ref.i1 + 1 : 0;
    ref.v1.Copy(tempPolygonB.vertices[ref.i1]);
    ref.v2.Copy(tempPolygonB.vertices[ref.i2]);
    ref.normal.Copy(tempPolygonB.normals[ref.i1]);

    // CCW winding
    ref.sideNormal1.Set(ref.normal.y, -ref.normal.x);
    ref.sideNormal2.Copy(ref.sideNormal1).SelfNeg(); // ref.sideNormal2 = -ref.sideNormal1;
  }

  ref.sideOffset1 = B2Vec2.DotVV(ref.sideNormal1, ref.v1);
  ref.sideOffset2 = B2Vec2.DotVV(ref.sideNormal2, ref.v2);

  // Clip incident edge against reference face side planes
  // B2ClipVertex clipPoints1[2];
  const clipPoints1: [B2ClipVertex, B2ClipVertex] = B2CollideEdgeAndPolygon_s_clipPoints1; // [new B2ClipVertex(), new B2ClipVertex()];
  // B2ClipVertex clipPoints2[2];
  const clipPoints2: [B2ClipVertex, B2ClipVertex] = B2CollideEdgeAndPolygon_s_clipPoints2; // [new B2ClipVertex(), new B2ClipVertex()];
  // int32 np;
  let np: number;

  // Clip to side 1
  np = B2ClipSegmentToLine(clipPoints1, clipPoints, ref.sideNormal1, ref.sideOffset1, ref.i1);

  if (np < B2_maxManifoldPoints) {
    return;
  }

  // Clip to side 2
  np = B2ClipSegmentToLine(clipPoints2, clipPoints1, ref.sideNormal2, ref.sideOffset2, ref.i2);

  if (np < B2_maxManifoldPoints) {
    return;
  }

  // Now clipPoints2 contains the clipped points.
  if (primaryAxis.type === B2EPAxisType.e_edgeA) {
    manifold.localNormal.Copy(ref.normal);
    manifold.localPoint.Copy(ref.v1);
  } else {
    manifold.localNormal.Copy(polygonB.m_normals[ref.i1]);
    manifold.localPoint.Copy(polygonB.m_vertices[ref.i1]);
  }

  let pointCount = 0;
  for (let i = 0; i < B2_maxManifoldPoints; ++i) {
    const separation: number = B2Vec2.DotVV(ref.normal, B2Vec2.SubVV(clipPoints2[i].v, ref.v1, B2Vec2.s_t0));

    if (separation <= radius) {
      const cp: B2ManifoldPoint = manifold.points[pointCount];

      if (primaryAxis.type === B2EPAxisType.e_edgeA) {
        B2Transform.MulTXV(xf, clipPoints2[i].v, cp.localPoint); // cp.localPoint = B2MulT(xf, clipPoints2[i].v);
        cp.id.Copy(clipPoints2[i].id);
      } else {
        cp.localPoint.Copy(clipPoints2[i].v);
        cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
        cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
        cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
        cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
      }

      ++pointCount;
    }
  }

  manifold.pointCount = pointCount;
}
