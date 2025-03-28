/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

import { B2_linearSlop, B2Maybe, B2_maxFloat } from "../common/b2_settings.js";
import { B2Abs, B2Clamp, B2Vec2, B2Rot, XY, B2Max, B2Transform } from "../common/b2_math.js";
import { B2Joint, B2JointDef, B2JointType, B2IJointDef } from "./b2_joint.js";
import { B2SolverData } from "./b2_time_step.js";
import { B2Body } from "./b2_body.js";
import { B2Color, B2Draw } from "../common/b2_draw.js";

export interface B2IDistanceJointDef extends B2IJointDef {
  localAnchorA?: XY;
  localAnchorB?: XY;
  length?: number;
  minLength?: number;
  maxLength?: number;
  stiffness?: number;
  damping?: number;
}

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
export class B2DistanceJointDef extends B2JointDef implements B2IDistanceJointDef {
  public readonly localAnchorA: B2Vec2 = new B2Vec2();
  public readonly localAnchorB: B2Vec2 = new B2Vec2();
  public length: number = 1;
  public minLength: number = 0;
  public maxLength: number = B2_maxFloat; // FLT_MAX;
  public stiffness: number = 0;
  public damping: number = 0;

  constructor() {
    super(B2JointType.e_distanceJoint);
  }

  public Initialize(b1: B2Body, B2: B2Body, anchor1: XY, anchor2: XY): void {
    this.bodyA = b1;
    this.bodyB = B2;
    this.bodyA.GetLocalPoint(anchor1, this.localAnchorA);
    this.bodyB.GetLocalPoint(anchor2, this.localAnchorB);
    this.length = B2Max(B2Vec2.DistanceVV(anchor1, anchor2), B2_linearSlop);
    this.minLength = this.length;
    this.maxLength = this.length;
  }
}

export class B2DistanceJoint extends B2Joint {
  public m_stiffness: number = 0;
  public m_damping: number = 0;
  public m_bias: number = 0;
  public m_length: number = 0;
  public m_minLength: number = 0;
  public m_maxLength: number = 0;

  // Solver shared
  public readonly m_localAnchorA: B2Vec2 = new B2Vec2();
  public readonly m_localAnchorB: B2Vec2 = new B2Vec2();
  public m_gamma: number = 0;
  public m_impulse: number = 0;
  public m_lowerImpulse: number = 0;
  public m_upperImpulse: number = 0;

  // Solver temp
  public m_indexA: number = 0;
  public m_indexB: number = 0;
  public readonly m_u: B2Vec2 = new B2Vec2();
  public readonly m_rA: B2Vec2 = new B2Vec2();
  public readonly m_rB: B2Vec2 = new B2Vec2();
  public readonly m_localCenterA: B2Vec2 = new B2Vec2();
  public readonly m_localCenterB: B2Vec2 = new B2Vec2();
  public m_currentLength: number = 0;
  public m_invMassA: number = 0;
  public m_invMassB: number = 0;
  public m_invIA: number = 0;
  public m_invIB: number = 0;
  public m_softMass: number = 0;
  public m_mass: number = 0;

  public readonly m_qA: B2Rot = new B2Rot();
  public readonly m_qB: B2Rot = new B2Rot();
  public readonly m_lalcA: B2Vec2 = new B2Vec2();
  public readonly m_lalcB: B2Vec2 = new B2Vec2();

  constructor(def: B2IDistanceJointDef) {
    super(def);

    this.m_localAnchorA.Copy(B2Maybe(def.localAnchorA, B2Vec2.ZERO));
    this.m_localAnchorB.Copy(B2Maybe(def.localAnchorB, B2Vec2.ZERO));
    this.m_length = B2Max(B2Maybe(def.length, this.GetCurrentLength()), B2_linearSlop);
    this.m_minLength = B2Max(B2Maybe(def.minLength, this.m_length), B2_linearSlop);
    this.m_maxLength = B2Max(B2Maybe(def.maxLength, this.m_length), this.m_minLength);
    this.m_stiffness = B2Maybe(def.stiffness, 0);
    this.m_damping = B2Maybe(def.damping, 0);
  }

  public GetAnchorA<T extends XY>(out: T): T {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
  }

  public GetAnchorB<T extends XY>(out: T): T {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
  }

  public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    // B2Vec2 F = inv_dt * (m_impulse + m_lowerImpulse - m_upperImpulse) * m_u;
    out.x = inv_dt * (this.m_impulse + this.m_lowerImpulse - this.m_upperImpulse) * this.m_u.x;
    out.y = inv_dt * (this.m_impulse + this.m_lowerImpulse - this.m_upperImpulse) * this.m_u.y;
    return out;
  }

  public GetReactionTorque(inv_dt: number): number {
    return 0;
  }

  public GetLocalAnchorA(): Readonly<B2Vec2> { return this.m_localAnchorA; }

  public GetLocalAnchorB(): Readonly<B2Vec2> { return this.m_localAnchorB; }

  public SetLength(length: number): number {
    this.m_impulse = 0;
    this.m_length = B2Max(B2_linearSlop, length);
    return this.m_length;
  }

  public GetLength(): number {
    return this.m_length;
  }

  public SetMinLength(minLength: number): number {
    this.m_lowerImpulse = 0;
    this.m_minLength = B2Clamp(minLength, B2_linearSlop, this.m_maxLength);
    return this.m_minLength;
  }

  public SetMaxLength(maxLength: number): number {
    this.m_upperImpulse = 0;
    this.m_maxLength = B2Max(maxLength, this.m_minLength);
    return this.m_maxLength;
  }

  public GetCurrentLength(): number {
    const pA: B2Vec2 = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, new B2Vec2());
    const pB: B2Vec2 = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, new B2Vec2());
    return B2Vec2.DistanceVV(pA, pB);
  }

  public SetStiffness(stiffness: number): void {
    this.m_stiffness = stiffness;
  }

  public GetStiffness() {
    return this.m_stiffness;
  }

  public SetDamping(damping: number): void {
    this.m_damping = damping;
  }

  public GetDamping() {
    return this.m_damping;
  }

  public override Dump(log: (format: string, ...args: any[]) => void) {
    const indexA: number = this.m_bodyA.m_islandIndex;
    const indexB: number = this.m_bodyB.m_islandIndex;

    log("  const jd: B2DistanceJointDef = new B2DistanceJointDef();\n");
    log("  jd.bodyA = bodies[%d];\n", indexA);
    log("  jd.bodyB = bodies[%d];\n", indexB);
    log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
    log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
    log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
    log("  jd.length = %.15f;\n", this.m_length);
    log("  jd.minLength = %.15f;\n", this.m_minLength);
    log("  jd.maxLength = %.15f;\n", this.m_maxLength);
    log("  jd.stiffness = %.15f;\n", this.m_stiffness);
    log("  jd.damping = %.15f;\n", this.m_damping);
    log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
  }

  private static InitVelocityConstraints_s_P = new B2Vec2();
  public InitVelocityConstraints(data: B2SolverData): void {
    this.m_indexA = this.m_bodyA.m_islandIndex;
    this.m_indexB = this.m_bodyB.m_islandIndex;
    this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
    this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
    this.m_invMassA = this.m_bodyA.m_invMass;
    this.m_invMassB = this.m_bodyB.m_invMass;
    this.m_invIA = this.m_bodyA.m_invI;
    this.m_invIB = this.m_bodyB.m_invI;

    const cA: B2Vec2 = data.positions[this.m_indexA].c;
    const aA: number = data.positions[this.m_indexA].a;
    const vA: B2Vec2 = data.velocities[this.m_indexA].v;
    let wA: number = data.velocities[this.m_indexA].w;

    const cB: B2Vec2 = data.positions[this.m_indexB].c;
    const aB: number = data.positions[this.m_indexB].a;
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    // const qA: B2Rot = new B2Rot(aA), qB: B2Rot = new B2Rot(aB);
    const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

    // m_rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
    B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
    B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
    // m_rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
    B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
    B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
    // m_u = cB + m_rB - cA - m_rA;
    this.m_u.x = cB.x + this.m_rB.x - cA.x - this.m_rA.x;
    this.m_u.y = cB.y + this.m_rB.y - cA.y - this.m_rA.y;

    // Handle singularity.
    this.m_currentLength = this.m_u.Length();
    if (this.m_currentLength > B2_linearSlop) {
      this.m_u.SelfMul(1 / this.m_currentLength);
    } else {
      this.m_u.SetZero();
      this.m_mass = 0;
      this.m_impulse = 0;
      this.m_lowerImpulse = 0;
      this.m_upperImpulse = 0;
    }

    // float32 crAu = B2Cross(m_rA, m_u);
    const crAu: number = B2Vec2.CrossVV(this.m_rA, this.m_u);
    // float32 crBu = B2Cross(m_rB, m_u);
    const crBu: number = B2Vec2.CrossVV(this.m_rB, this.m_u);
    // float32 invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;
    let invMass: number = this.m_invMassA + this.m_invIA * crAu * crAu + this.m_invMassB + this.m_invIB * crBu * crBu;
    this.m_mass = invMass !== 0 ? 1 / invMass : 0;

    if (this.m_stiffness > 0 && this.m_minLength < this.m_maxLength) {
      // soft
      const C: number = this.m_currentLength - this.m_length;

      const d: number = this.m_damping;
      const k: number = this.m_stiffness;

      // magic formulas
      const h: number = data.step.dt;

      // gamma = 1 / (h * (d + h * k))
      // the extra factor of h in the denominator is since the lambda is an impulse, not a force
      this.m_gamma = h * (d + h * k);
      this.m_gamma = this.m_gamma !== 0 ? 1 / this.m_gamma : 0;
      this.m_bias = C * h * k * this.m_gamma;

      invMass += this.m_gamma;
      this.m_softMass = invMass !== 0 ? 1 / invMass : 0;
    }
    else {
      // rigid
      this.m_gamma = 0;
      this.m_bias = 0;
      this.m_softMass = this.m_mass;
    }

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      this.m_impulse *= data.step.dtRatio;
      this.m_lowerImpulse *= data.step.dtRatio;
      this.m_upperImpulse *= data.step.dtRatio;

      const P: B2Vec2 = B2Vec2.MulSV(this.m_impulse + this.m_lowerImpulse - this.m_upperImpulse, this.m_u, B2DistanceJoint.InitVelocityConstraints_s_P);
      vA.SelfMulSub(this.m_invMassA, P);
      wA -= this.m_invIA * B2Vec2.CrossVV(this.m_rA, P);
      vB.SelfMulAdd(this.m_invMassB, P);
      wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, P);
    }
    else {
      this.m_impulse = 0;
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA;
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  private static SolveVelocityConstraints_s_vpA = new B2Vec2();
  private static SolveVelocityConstraints_s_vpB = new B2Vec2();
  private static SolveVelocityConstraints_s_P = new B2Vec2();
  public SolveVelocityConstraints(data: B2SolverData): void {
    const vA: B2Vec2 = data.velocities[this.m_indexA].v;
    let wA: number = data.velocities[this.m_indexA].w;
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    if (this.m_minLength < this.m_maxLength) {
      if (this.m_stiffness > 0) {
        // Cdot = dot(u, v + cross(w, r))
        const vpA: B2Vec2 = B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2DistanceJoint.SolveVelocityConstraints_s_vpA);
        const vpB: B2Vec2 = B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2DistanceJoint.SolveVelocityConstraints_s_vpB);
        const Cdot: number = B2Vec2.DotVV(this.m_u, B2Vec2.SubVV(vpB, vpA, B2Vec2.s_t0));

        const impulse: number = -this.m_softMass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
        this.m_impulse += impulse;

        const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_u, B2DistanceJoint.SolveVelocityConstraints_s_P);
        vA.SelfMulSub(this.m_invMassA, P);
        wA -= this.m_invIA * B2Vec2.CrossVV(this.m_rA, P);
        vB.SelfMulAdd(this.m_invMassB, P);
        wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, P);
      }

      // lower
      {
        const C: number = this.m_currentLength - this.m_minLength;
        const bias: number = B2Max(0, C) * data.step.inv_dt;

        const vpA: B2Vec2 = B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2DistanceJoint.SolveVelocityConstraints_s_vpA);
        const vpB: B2Vec2 = B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2DistanceJoint.SolveVelocityConstraints_s_vpB);
        const Cdot: number = B2Vec2.DotVV(this.m_u, B2Vec2.SubVV(vpB, vpA, B2Vec2.s_t0));

        let impulse: number = -this.m_mass * (Cdot + bias);
        const oldImpulse: number = this.m_lowerImpulse;
        this.m_lowerImpulse = B2Max(0, this.m_lowerImpulse + impulse);
        impulse = this.m_lowerImpulse - oldImpulse;
        const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_u, B2DistanceJoint.SolveVelocityConstraints_s_P);

        vA.SelfMulSub(this.m_invMassA, P);
        wA -= this.m_invIA * B2Vec2.CrossVV(this.m_rA, P);
        vB.SelfMulAdd(this.m_invMassB, P);
        wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, P);
      }

      // upper
      {
        const C: number = this.m_maxLength - this.m_currentLength;
        const bias: number = B2Max(0, C) * data.step.inv_dt;

        const vpA: B2Vec2 = B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2DistanceJoint.SolveVelocityConstraints_s_vpA);
        const vpB: B2Vec2 = B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2DistanceJoint.SolveVelocityConstraints_s_vpB);
        const Cdot: number = B2Vec2.DotVV(this.m_u, B2Vec2.SubVV(vpA, vpB, B2Vec2.s_t0));

        let impulse: number = -this.m_mass * (Cdot + bias);
        const oldImpulse: number = this.m_upperImpulse;
        this.m_upperImpulse = B2Max(0, this.m_upperImpulse + impulse);
        impulse = this.m_upperImpulse - oldImpulse;
        const P: B2Vec2 = B2Vec2.MulSV(-impulse, this.m_u, B2DistanceJoint.SolveVelocityConstraints_s_P);

        vA.SelfMulSub(this.m_invMassA, P);
        wA -= this.m_invIA * B2Vec2.CrossVV(this.m_rA, P);
        vB.SelfMulAdd(this.m_invMassB, P);
        wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, P);
      }
    }
    else {
      // Equal limits

      // Cdot = dot(u, v + cross(w, r))
      const vpA: B2Vec2 = B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2DistanceJoint.SolveVelocityConstraints_s_vpA);
      const vpB: B2Vec2 = B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2DistanceJoint.SolveVelocityConstraints_s_vpB);
      const Cdot: number = B2Vec2.DotVV(this.m_u, B2Vec2.SubVV(vpB, vpA, B2Vec2.s_t0));

      const impulse: number = -this.m_mass * Cdot;
      this.m_impulse += impulse;

      const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_u, B2DistanceJoint.SolveVelocityConstraints_s_P);
      vA.SelfMulSub(this.m_invMassA, P);
      wA -= this.m_invIA * B2Vec2.CrossVV(this.m_rA, P);
      vB.SelfMulAdd(this.m_invMassB, P);
      wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, P);
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA;
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  private static SolvePositionConstraints_s_P = new B2Vec2();
  public SolvePositionConstraints(data: B2SolverData): boolean {
    const cA: B2Vec2 = data.positions[this.m_indexA].c;
    let aA: number = data.positions[this.m_indexA].a;
    const cB: B2Vec2 = data.positions[this.m_indexB].c;
    let aB: number = data.positions[this.m_indexB].a;

    // const qA: B2Rot = new B2Rot(aA), qB: B2Rot = new B2Rot(aB);
    const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

    // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
    const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA); // use m_rA
    // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
    const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB); // use m_rB
    // B2Vec2 u = cB + rB - cA - rA;
    const u: B2Vec2 = this.m_u; // use m_u
    u.x = cB.x + rB.x - cA.x - rA.x;
    u.y = cB.y + rB.y - cA.y - rA.y;

    const length: number = this.m_u.Normalize();
    let C: number;
    if (this.m_minLength == this.m_maxLength)
    {
      C = length - this.m_minLength;
    }
    else if (length < this.m_minLength)
    {
      C = length - this.m_minLength;
    }
    else if (this.m_maxLength < length)
    {
      C = length - this.m_maxLength;
    }
    else
    {
      return true;
    }

    const impulse: number = -this.m_mass * C;
    const P: B2Vec2 = B2Vec2.MulSV(impulse, u, B2DistanceJoint.SolvePositionConstraints_s_P);

    cA.SelfMulSub(this.m_invMassA, P);
    aA -= this.m_invIA * B2Vec2.CrossVV(rA, P);
    cB.SelfMulAdd(this.m_invMassB, P);
    aB += this.m_invIB * B2Vec2.CrossVV(rB, P);

    // data.positions[this.m_indexA].c = cA;
    data.positions[this.m_indexA].a = aA;
    // data.positions[this.m_indexB].c = cB;
    data.positions[this.m_indexB].a = aB;

    return B2Abs(C) < B2_linearSlop;
  }

  private static Draw_s_pA = new B2Vec2();
  private static Draw_s_pB = new B2Vec2();
  private static Draw_s_axis = new B2Vec2();
  private static Draw_s_c1 = new B2Color(0.7, 0.7, 0.7);
  private static Draw_s_c2 = new B2Color(0.3, 0.9, 0.3);
  private static Draw_s_c3 = new B2Color(0.9, 0.3, 0.3);
  private static Draw_s_c4 = new B2Color(0.4, 0.4, 0.4);
  private static Draw_s_pRest = new B2Vec2();
  private static Draw_s_pMin = new B2Vec2();
  private static Draw_s_pMax = new B2Vec2();
  public override Draw(draw: B2Draw): void {
    const xfA: Readonly<B2Transform> = this.m_bodyA.GetTransform();
    const xfB: Readonly<B2Transform> = this.m_bodyB.GetTransform();
    const pA = B2Transform.MulXV(xfA, this.m_localAnchorA, B2DistanceJoint.Draw_s_pA);
    const pB = B2Transform.MulXV(xfB, this.m_localAnchorB, B2DistanceJoint.Draw_s_pB);

    const axis: B2Vec2 = B2Vec2.SubVV(pB, pA, B2DistanceJoint.Draw_s_axis);
    axis.Normalize();
  
    const c1 = B2DistanceJoint.Draw_s_c1; // B2Color c1(0.7f, 0.7f, 0.7f);
    const c2 = B2DistanceJoint.Draw_s_c2; // B2Color c2(0.3f, 0.9f, 0.3f);
    const c3 = B2DistanceJoint.Draw_s_c3; // B2Color c3(0.9f, 0.3f, 0.3f);
    const c4 = B2DistanceJoint.Draw_s_c4; // B2Color c4(0.4f, 0.4f, 0.4f);
  
    draw.DrawSegment(pA, pB, c4);
    
    // B2Vec2 pRest = pA + this.m_length * axis;
    const pRest: B2Vec2 = B2Vec2.AddVMulSV(pA, this.m_length, axis, B2DistanceJoint.Draw_s_pRest);
    draw.DrawPoint(pRest, 8.0, c1);
  
    if (this.m_minLength != this.m_maxLength) {
      if (this.m_minLength > B2_linearSlop) {
        // B2Vec2 pMin = pA + this.m_minLength * axis;
        const pMin: B2Vec2 = B2Vec2.AddVMulSV(pA, this.m_minLength, axis, B2DistanceJoint.Draw_s_pMin);
        draw.DrawPoint(pMin, 4.0, c2);
      }
  
      if (this.m_maxLength < B2_maxFloat) {
        // B2Vec2 pMax = pA + this.m_maxLength * axis;
        const pMax: B2Vec2 = B2Vec2.AddVMulSV(pA, this.m_maxLength, axis, B2DistanceJoint.Draw_s_pMax);
        draw.DrawPoint(pMax, 4.0, c3);
      }
    }
  }
}
