/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
import { B2_linearSlop, B2Maybe } from "../common/b2_settings.js";
import { B2Abs, B2Clamp, B2Vec2, B2Rot, XY, B2Max, B2Min, B2Transform } from "../common/b2_math.js";
import { B2Joint, B2JointDef, B2JointType, B2IJointDef } from "./b2_joint.js";
import { B2SolverData } from "./b2_time_step.js";
import { B2Body } from "./b2_body.js";
import { B2Draw, B2Color } from "../common/b2_draw.js";

export interface B2IWheelJointDef extends B2IJointDef {
  /// The local anchor point relative to bodyA's origin.
  localAnchorA?: XY;

  /// The local anchor point relative to bodyB's origin.
  localAnchorB?: XY;

  /// The local translation axis in bodyA.
  localAxisA?: XY;

  /// Enable/disable the joint limit.
  enableLimit?: boolean;

  /// The lower translation limit, usually in meters.
  lowerTranslation?: number;

  /// The upper translation limit, usually in meters.
  upperTranslation?: number;

  /// Enable/disable the joint motor.
  enableMotor?: boolean;

  /// The maximum motor torque, usually in N-m.
  maxMotorTorque?: number;

  /// The desired motor speed in radians per second.
  motorSpeed?: number;

  /// Suspension stiffness. Typically in units N/m.
  stiffness?: number;

  /// Suspension damping. Typically in units of N*s/m.
  damping?: number;
}

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
export class B2WheelJointDef extends B2JointDef implements B2IWheelJointDef {
  public readonly localAnchorA: B2Vec2 = new B2Vec2(0, 0);

  public readonly localAnchorB: B2Vec2 = new B2Vec2(0, 0);

  public readonly localAxisA: B2Vec2 = new B2Vec2(1, 0);

  public enableLimit: boolean = false;

  public lowerTranslation: number = 0;

  public upperTranslation: number = 0;

  public enableMotor = false;

  public maxMotorTorque: number = 0;

  public motorSpeed: number = 0;

  public stiffness: number = 0;

  public damping: number = 0;

  constructor() {
    super(B2JointType.e_wheelJoint);
  }

  public Initialize(bA: B2Body, bB: B2Body, anchor: XY, axis: XY): void {
    this.bodyA = bA;
    this.bodyB = bB;
    this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
    this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
    this.bodyA.GetLocalVector(axis, this.localAxisA);
  }
}

export class B2WheelJoint extends B2Joint {
  public readonly m_localAnchorA: B2Vec2 = new B2Vec2();
  public readonly m_localAnchorB: B2Vec2 = new B2Vec2();
  public readonly m_localXAxisA: B2Vec2 = new B2Vec2();
  public readonly m_localYAxisA: B2Vec2 = new B2Vec2();

  public m_impulse: number = 0;
  public m_motorImpulse: number = 0;
  public m_springImpulse: number = 0;

  public m_lowerImpulse: number = 0;
  public m_upperImpulse: number = 0;
  public m_translation: number = 0;
  public m_lowerTranslation: number = 0;
  public m_upperTranslation: number = 0;

  public m_maxMotorTorque: number = 0;
  public m_motorSpeed: number = 0;

  public m_enableLimit = false;
  public m_enableMotor = false;

  public m_stiffness: number = 0;
  public m_damping: number = 0;

  // Solver temp
  public m_indexA: number = 0;
  public m_indexB: number = 0;
  public readonly m_localCenterA: B2Vec2 = new B2Vec2();
  public readonly m_localCenterB: B2Vec2 = new B2Vec2();
  public m_invMassA: number = 0;
  public m_invMassB: number = 0;
  public m_invIA: number = 0;
  public m_invIB: number = 0;

  public readonly m_ax: B2Vec2 = new B2Vec2();
  public readonly m_ay: B2Vec2 = new B2Vec2();
  public m_sAx: number = 0;
  public m_sBx: number = 0;
  public m_sAy: number = 0;
  public m_sBy: number = 0;

  public m_mass: number = 0;
  public m_motorMass: number = 0;
  public m_axialMass: number = 0;
  public m_springMass: number = 0;

  public m_bias: number = 0;
  public m_gamma: number = 0;

  public readonly m_qA: B2Rot = new B2Rot();
  public readonly m_qB: B2Rot = new B2Rot();
  public readonly m_lalcA: B2Vec2 = new B2Vec2();
  public readonly m_lalcB: B2Vec2 = new B2Vec2();
  public readonly m_rA: B2Vec2 = new B2Vec2();
  public readonly m_rB: B2Vec2 = new B2Vec2();

  constructor(def: B2IWheelJointDef) {
    super(def);

    this.m_localAnchorA.Copy(B2Maybe(def.localAnchorA, B2Vec2.ZERO));
    this.m_localAnchorB.Copy(B2Maybe(def.localAnchorB, B2Vec2.ZERO));
    this.m_localXAxisA.Copy(B2Maybe(def.localAxisA, B2Vec2.UNITX));
    B2Vec2.CrossOneV(this.m_localXAxisA, this.m_localYAxisA);

    this.m_lowerTranslation = B2Maybe(def.lowerTranslation, 0);
    this.m_upperTranslation = B2Maybe(def.upperTranslation, 0);
    this.m_enableLimit = B2Maybe(def.enableLimit, false);

    this.m_maxMotorTorque = B2Maybe(def.maxMotorTorque, 0);
    this.m_motorSpeed = B2Maybe(def.motorSpeed, 0);
    this.m_enableMotor = B2Maybe(def.enableMotor, false);

    this.m_ax.SetZero();
    this.m_ay.SetZero();

    this.m_stiffness = B2Maybe(def.stiffness, 0);
    this.m_damping = B2Maybe(def.damping, 0);
  }

  public GetMotorSpeed(): number {
    return this.m_motorSpeed;
  }

  public GetMaxMotorTorque(): number {
    return this.m_maxMotorTorque;
  }

  public SetSpringFrequencyHz(hz: number): void {
    this.m_stiffness = hz;
  }

  public GetSpringFrequencyHz(): number {
    return this.m_stiffness;
  }

  public SetSpringDampingRatio(ratio: number): void {
    this.m_damping = ratio;
  }

  public GetSpringDampingRatio(): number {
    return this.m_damping;
  }

  private static InitVelocityConstraints_s_d = new B2Vec2();
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

    const mA: number = this.m_invMassA, mB: number = this.m_invMassB;
    const iA: number = this.m_invIA, iB: number = this.m_invIB;

    const cA: B2Vec2 = data.positions[this.m_indexA].c;
    const aA: number = data.positions[this.m_indexA].a;
    const vA: B2Vec2 = data.velocities[this.m_indexA].v;
    let wA: number = data.velocities[this.m_indexA].w;

    const cB: B2Vec2 = data.positions[this.m_indexB].c;
    const aB: number = data.positions[this.m_indexB].a;
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

    // Compute the effective masses.
    // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
    B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
    const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
    // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
    B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
    const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
    // B2Vec2 d = cB + rB - cA - rA;
    const d: B2Vec2 = B2Vec2.SubVV(
      B2Vec2.AddVV(cB, rB, B2Vec2.s_t0),
      B2Vec2.AddVV(cA, rA, B2Vec2.s_t1),
      B2WheelJoint.InitVelocityConstraints_s_d);

    // Point to line constraint
    {
      // m_ay = B2Mul(qA, m_localYAxisA);
      B2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);
      // m_sAy = B2Cross(d + rA, m_ay);
      this.m_sAy = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), this.m_ay);
      // m_sBy = B2Cross(rB, m_ay);
      this.m_sBy = B2Vec2.CrossVV(rB, this.m_ay);

      this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;

      if (this.m_mass > 0) {
        this.m_mass = 1 / this.m_mass;
      }
    }

    // Spring constraint
    B2Rot.MulRV(qA, this.m_localXAxisA, this.m_ax); // m_ax = B2Mul(qA, m_localXAxisA);
    this.m_sAx = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), this.m_ax);
    this.m_sBx = B2Vec2.CrossVV(rB, this.m_ax);

    const invMass: number = mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx;
    if (invMass > 0.0) {
      this.m_axialMass = 1.0 / invMass;
    } else {
      this.m_axialMass = 0.0;
    }

    this.m_springMass = 0;
    this.m_bias = 0;
    this.m_gamma = 0;

    if (this.m_stiffness > 0.0 && invMass > 0.0) {
      this.m_springMass = 1.0 / invMass;

      const C: number = B2Vec2.DotVV(d, this.m_ax);

      // magic formulas
      const h: number = data.step.dt;
      this.m_gamma = h * (this.m_damping + h * this.m_stiffness);
      if (this.m_gamma > 0.0) {
        this.m_gamma = 1.0 / this.m_gamma;
      }

      this.m_bias = C * h * this.m_stiffness * this.m_gamma;

      this.m_springMass = invMass + this.m_gamma;
      if (this.m_springMass > 0.0) {
        this.m_springMass = 1.0 / this.m_springMass;
      }
    } else {
      this.m_springImpulse = 0.0;
    }

    if (this.m_enableLimit) {
      this.m_translation = B2Vec2.DotVV(this.m_ax, d);
    } else {
      this.m_lowerImpulse = 0.0;
      this.m_upperImpulse = 0.0;
    }

    if (this.m_enableMotor) {
      this.m_motorMass = iA + iB;
      if (this.m_motorMass > 0) {
        this.m_motorMass = 1 / this.m_motorMass;
      }
    } else {
      this.m_motorMass = 0;
      this.m_motorImpulse = 0;
    }

    if (data.step.warmStarting) {
      // Account for variable time step.
      this.m_impulse *= data.step.dtRatio;
      this.m_springImpulse *= data.step.dtRatio;
      this.m_motorImpulse *= data.step.dtRatio;

      const axialImpulse: number = this.m_springImpulse + this.m_lowerImpulse - this.m_upperImpulse;
      // B2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
      const P: B2Vec2 = B2Vec2.AddVV(
        B2Vec2.MulSV(this.m_impulse, this.m_ay, B2Vec2.s_t0),
        B2Vec2.MulSV(axialImpulse, this.m_ax, B2Vec2.s_t1),
        B2WheelJoint.InitVelocityConstraints_s_P);
      // float32 LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
      const LA: number = this.m_impulse * this.m_sAy + axialImpulse * this.m_sAx + this.m_motorImpulse;
      // float32 LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;
      const LB: number = this.m_impulse * this.m_sBy + axialImpulse * this.m_sBx + this.m_motorImpulse;

      // vA -= m_invMassA * P;
      vA.SelfMulSub(this.m_invMassA, P);
      wA -= this.m_invIA * LA;

      // vB += m_invMassB * P;
      vB.SelfMulAdd(this.m_invMassB, P);
      wB += this.m_invIB * LB;
    } else {
      this.m_impulse = 0;
      this.m_springImpulse = 0;
      this.m_motorImpulse = 0;
      this.m_lowerImpulse = 0;
      this.m_upperImpulse = 0;
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA;
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  private static SolveVelocityConstraints_s_P = new B2Vec2();
  public SolveVelocityConstraints(data: B2SolverData): void {
    const mA: number = this.m_invMassA, mB: number = this.m_invMassB;
    const iA: number = this.m_invIA, iB: number = this.m_invIB;

    const vA: B2Vec2 = data.velocities[this.m_indexA].v;
    let wA: number = data.velocities[this.m_indexA].w;
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    // Solve spring constraint
    {
      const Cdot: number = B2Vec2.DotVV(this.m_ax, B2Vec2.SubVV(vB, vA, B2Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
      const impulse: number = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
      this.m_springImpulse += impulse;

      // B2Vec2 P = impulse * m_ax;
      const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_ax, B2WheelJoint.SolveVelocityConstraints_s_P);
      const LA: number = impulse * this.m_sAx;
      const LB: number = impulse * this.m_sBx;

      // vA -= mA * P;
      vA.SelfMulSub(mA, P);
      wA -= iA * LA;

      // vB += mB * P;
      vB.SelfMulAdd(mB, P);
      wB += iB * LB;
    }

    // Solve rotational motor constraint
    {
      const Cdot: number = wB - wA - this.m_motorSpeed;
      let impulse: number = -this.m_motorMass * Cdot;

      const oldImpulse: number = this.m_motorImpulse;
      const maxImpulse: number = data.step.dt * this.m_maxMotorTorque;
      this.m_motorImpulse = B2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = this.m_motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    if (this.m_enableLimit) {
      // Lower limit
      {
        const C: number = this.m_translation - this.m_lowerTranslation;
        const Cdot: number = B2Vec2.DotVV(this.m_ax, B2Vec2.SubVV(vB, vA, B2Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
        let impulse: number = -this.m_axialMass * (Cdot + B2Max(C, 0.0) * data.step.inv_dt);
        const oldImpulse: number = this.m_lowerImpulse;
        this.m_lowerImpulse = B2Max(this.m_lowerImpulse + impulse, 0.0);
        impulse = this.m_lowerImpulse - oldImpulse;

        // B2Vec2 P = impulse * this.m_ax;
        const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_ax, B2WheelJoint.SolveVelocityConstraints_s_P);
        const LA: number = impulse * this.m_sAx;
        const LB: number = impulse * this.m_sBx;

        // vA -= mA * P;
        vA.SelfMulSub(mA, P);
        wA -= iA * LA;
        // vB += mB * P;
        vB.SelfMulAdd(mB, P);
        wB += iB * LB;
      }

      // Upper limit
      // Note: signs are flipped to keep C positive when the constraint is satisfied.
      // This also keeps the impulse positive when the limit is active.
      {
        const C: number = this.m_upperTranslation - this.m_translation;
        const Cdot: number = B2Vec2.DotVV(this.m_ax, B2Vec2.SubVV(vA, vB, B2Vec2.s_t0)) + this.m_sAx * wA - this.m_sBx * wB;
        let impulse: number = -this.m_axialMass * (Cdot + B2Max(C, 0.0) * data.step.inv_dt);
        const oldImpulse: number = this.m_upperImpulse;
        this.m_upperImpulse = B2Max(this.m_upperImpulse + impulse, 0.0);
        impulse = this.m_upperImpulse - oldImpulse;

        // B2Vec2 P = impulse * this.m_ax;
        const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_ax, B2WheelJoint.SolveVelocityConstraints_s_P);
        const LA: number = impulse * this.m_sAx;
        const LB: number = impulse * this.m_sBx;

        // vA += mA * P;
        vA.SelfMulAdd(mA, P);
        wA += iA * LA;
        // vB -= mB * P;
        vB.SelfMulSub(mB, P);
        wB -= iB * LB;
      }
    }

    // Solve point to line constraint
    {
      const Cdot: number = B2Vec2.DotVV(this.m_ay, B2Vec2.SubVV(vB, vA, B2Vec2.s_t0)) + this.m_sBy * wB - this.m_sAy * wA;
      const impulse: number = -this.m_mass * Cdot;
      this.m_impulse += impulse;

      // B2Vec2 P = impulse * m_ay;
      const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_ay, B2WheelJoint.SolveVelocityConstraints_s_P);
      const LA: number = impulse * this.m_sAy;
      const LB: number = impulse * this.m_sBy;

      // vA -= mA * P;
      vA.SelfMulSub(mA, P);
      wA -= iA * LA;

      // vB += mB * P;
      vB.SelfMulAdd(mB, P);
      wB += iB * LB;
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA;
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  private static SolvePositionConstraints_s_d = new B2Vec2();
  private static SolvePositionConstraints_s_P = new B2Vec2();
  public SolvePositionConstraints(data: B2SolverData): boolean {
    const cA: B2Vec2 = data.positions[this.m_indexA].c;
    let aA: number = data.positions[this.m_indexA].a;
    const cB: B2Vec2 = data.positions[this.m_indexB].c;
    let aB: number = data.positions[this.m_indexB].a;

    // const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

    // // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
    // B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
    // const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
    // // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
    // B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
    // const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
    // // B2Vec2 d = (cB - cA) + rB - rA;
    // const d: B2Vec2 = B2Vec2.AddVV(
    //   B2Vec2.SubVV(cB, cA, B2Vec2.s_t0),
    //   B2Vec2.SubVV(rB, rA, B2Vec2.s_t1),
    //   B2WheelJoint.SolvePositionConstraints_s_d);

    // // B2Vec2 ay = B2Mul(qA, m_localYAxisA);
    // const ay: B2Vec2 = B2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);

    // // float32 sAy = B2Cross(d + rA, ay);
    // const sAy = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), ay);
    // // float32 sBy = B2Cross(rB, ay);
    // const sBy = B2Vec2.CrossVV(rB, ay);

    // // float32 C = B2Dot(d, ay);
    // const C: number = B2Vec2.DotVV(d, this.m_ay);

    // const k: number = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_sAy * this.m_sAy + this.m_invIB * this.m_sBy * this.m_sBy;

    // let impulse: number;
    // if (k !== 0) {
    //   impulse = - C / k;
    // } else {
    //   impulse = 0;
    // }

    // // B2Vec2 P = impulse * ay;
    // const P: B2Vec2 = B2Vec2.MulSV(impulse, ay, B2WheelJoint.SolvePositionConstraints_s_P);
    // const LA: number = impulse * sAy;
    // const LB: number = impulse * sBy;

    // // cA -= m_invMassA * P;
    // cA.SelfMulSub(this.m_invMassA, P);
    // aA -= this.m_invIA * LA;
    // // cB += m_invMassB * P;
    // cB.SelfMulAdd(this.m_invMassB, P);
    // aB += this.m_invIB * LB;

    let linearError: number = 0.0;

    if (this.m_enableLimit) {
      // B2Rot qA(aA), qB(aB);
      const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

      // B2Vec2 rA = B2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
      // B2Vec2 rB = B2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
      // B2Vec2 d = (cB - cA) + rB - rA;

      // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
      B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
      const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
      // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
      B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
      const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
      // B2Vec2 d = (cB - cA) + rB - rA;
      const d: B2Vec2 = B2Vec2.AddVV(
        B2Vec2.SubVV(cB, cA, B2Vec2.s_t0),
        B2Vec2.SubVV(rB, rA, B2Vec2.s_t1),
        B2WheelJoint.SolvePositionConstraints_s_d);

      // B2Vec2 ax = B2Mul(qA, this.m_localXAxisA);
      const ax: B2Vec2 = B2Rot.MulRV(qA, this.m_localXAxisA, this.m_ax);
      // float sAx = B2Cross(d + rA, this.m_ax);
      const sAx = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), this.m_ax);
      // float sBx = B2Cross(rB, this.m_ax);
      const sBx = B2Vec2.CrossVV(rB, this.m_ax);

      let C: number = 0.0;
      const translation: number = B2Vec2.DotVV(ax, d);
      if (B2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * B2_linearSlop) {
        C = translation;
      } else if (translation <= this.m_lowerTranslation) {
        C = B2Min(translation - this.m_lowerTranslation, 0.0);
      } else if (translation >= this.m_upperTranslation) {
        C = B2Max(translation - this.m_upperTranslation, 0.0);
      }

      if (C !== 0.0) {

        const invMass: number = this.m_invMassA + this.m_invMassB + this.m_invIA * sAx * sAx + this.m_invIB * sBx * sBx;
        let impulse: number = 0.0;
        if (invMass !== 0.0) {
          impulse = -C / invMass;
        }

        const P: B2Vec2 = B2Vec2.MulSV(impulse, ax, B2WheelJoint.SolvePositionConstraints_s_P);
        const LA: number = impulse * sAx;
        const LB: number = impulse * sBx;

        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        aA -= this.m_invIA * LA;
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        // aB += m_invIB * LB;
        aB += this.m_invIB * LB;

        linearError = B2Abs(C);
      }
    }

    // Solve perpendicular constraint
    {
      // B2Rot qA(aA), qB(aB);
      const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

      // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
      // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
      // B2Vec2 d = (cB - cA) + rB - rA;

      // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
      B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
      const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
      // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
      B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
      const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
      // B2Vec2 d = (cB - cA) + rB - rA;
      const d: B2Vec2 = B2Vec2.AddVV(
        B2Vec2.SubVV(cB, cA, B2Vec2.s_t0),
        B2Vec2.SubVV(rB, rA, B2Vec2.s_t1),
        B2WheelJoint.SolvePositionConstraints_s_d);

      // B2Vec2 ay = B2Mul(qA, m_localYAxisA);
      const ay: B2Vec2 = B2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);

      // float sAy = B2Cross(d + rA, ay);
      const sAy = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), ay);
      // float sBy = B2Cross(rB, ay);
      const sBy = B2Vec2.CrossVV(rB, ay);

      // float C = B2Dot(d, ay);
      const C: number = B2Vec2.DotVV(d, ay);

      const invMass: number = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_sAy * this.m_sAy + this.m_invIB * this.m_sBy * this.m_sBy;

      let impulse: number = 0.0;
      if (invMass !== 0.0) {
        impulse = - C / invMass;
      }

      // B2Vec2 P = impulse * ay;
      // const LA: number = impulse * sAy;
      // const LB: number = impulse * sBy;
      const P: B2Vec2 = B2Vec2.MulSV(impulse, ay, B2WheelJoint.SolvePositionConstraints_s_P);
      const LA: number = impulse * sAy;
      const LB: number = impulse * sBy;

      // cA -= m_invMassA * P;
      cA.SelfMulSub(this.m_invMassA, P);
      aA -= this.m_invIA * LA;
      // cB += m_invMassB * P;
      cB.SelfMulAdd(this.m_invMassB, P);
      aB += this.m_invIB * LB;

      linearError = B2Max(linearError, B2Abs(C));
    }

    // data.positions[this.m_indexA].c = cA;
    data.positions[this.m_indexA].a = aA;
    // data.positions[this.m_indexB].c = cB;
    data.positions[this.m_indexB].a = aB;

    return linearError <= B2_linearSlop;
  }

  public GetDefinition(def: B2WheelJointDef): B2WheelJointDef {
    // DEBUG: B2Assert(false); // TODO
    return def;
  }

  public GetAnchorA<T extends XY>(out: T): T {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
  }

  public GetAnchorB<T extends XY>(out: T): T {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
  }

  public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    out.x = inv_dt * (this.m_impulse * this.m_ay.x + (this.m_springImpulse + this.m_lowerImpulse - this.m_upperImpulse) * this.m_ax.x);
    out.y = inv_dt * (this.m_impulse * this.m_ay.y + (this.m_springImpulse + this.m_lowerImpulse - this.m_upperImpulse) * this.m_ax.y);
    return out;
  }

  public GetReactionTorque(inv_dt: number): number {
    return inv_dt * this.m_motorImpulse;
  }

  public GetLocalAnchorA(): Readonly<B2Vec2> { return this.m_localAnchorA; }

  public GetLocalAnchorB(): Readonly<B2Vec2> { return this.m_localAnchorB; }

  public GetLocalAxisA(): Readonly<B2Vec2> { return this.m_localXAxisA; }

  public GetJointTranslation(): number {
    return this.GetPrismaticJointTranslation();
  }

  public GetJointLinearSpeed(): number {
    return this.GetPrismaticJointSpeed();
  }

  public GetJointAngle(): number {
    return this.GetRevoluteJointAngle();
  }

  public GetJointAngularSpeed(): number {
    return this.GetRevoluteJointSpeed();
  }

  public GetPrismaticJointTranslation(): number {
    const bA: B2Body = this.m_bodyA;
    const bB: B2Body = this.m_bodyB;

    const pA: B2Vec2 = bA.GetWorldPoint(this.m_localAnchorA, new B2Vec2());
    const pB: B2Vec2 = bB.GetWorldPoint(this.m_localAnchorB, new B2Vec2());
    const d: B2Vec2 = B2Vec2.SubVV(pB, pA, new B2Vec2());
    const axis: B2Vec2 = bA.GetWorldVector(this.m_localXAxisA, new B2Vec2());

    const translation: number = B2Vec2.DotVV(d, axis);
    return translation;
  }

  public GetPrismaticJointSpeed(): number {
    const bA: B2Body = this.m_bodyA;
    const bB: B2Body = this.m_bodyB;

    // B2Vec2 rA = B2Mul(bA.m_xf.q, m_localAnchorA - bA.m_sweep.localCenter);
    B2Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA);
    const rA = B2Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA);
    // B2Vec2 rB = B2Mul(bB.m_xf.q, m_localAnchorB - bB.m_sweep.localCenter);
    B2Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB);
    const rB = B2Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB);
    // B2Vec2 pA = bA.m_sweep.c + rA;
    const pA = B2Vec2.AddVV(bA.m_sweep.c, rA, B2Vec2.s_t0); // pA uses s_t0
    // B2Vec2 pB = bB.m_sweep.c + rB;
    const pB = B2Vec2.AddVV(bB.m_sweep.c, rB, B2Vec2.s_t1); // pB uses s_t1
    // B2Vec2 d = pB - pA;
    const d = B2Vec2.SubVV(pB, pA, B2Vec2.s_t2); // d uses s_t2
    // B2Vec2 axis = B2Mul(bA.m_xf.q, m_localXAxisA);
    const axis = bA.GetWorldVector(this.m_localXAxisA, new B2Vec2());

    const vA = bA.m_linearVelocity;
    const vB = bB.m_linearVelocity;
    const wA = bA.m_angularVelocity;
    const wB = bB.m_angularVelocity;

    // float32 speed = B2Dot(d, B2Cross(wA, axis)) + B2Dot(axis, vB + B2Cross(wB, rB) - vA - B2Cross(wA, rA));
    const speed =
      B2Vec2.DotVV(d, B2Vec2.CrossSV(wA, axis, B2Vec2.s_t0)) +
      B2Vec2.DotVV(
        axis,
        B2Vec2.SubVV(
          B2Vec2.AddVCrossSV(vB, wB, rB, B2Vec2.s_t0),
          B2Vec2.AddVCrossSV(vA, wA, rA, B2Vec2.s_t1),
          B2Vec2.s_t0));
    return speed;
  }

  public GetRevoluteJointAngle(): number {
    // B2Body* bA = this.m_bodyA;
    // B2Body* bB = this.m_bodyB;
    // return bB.this.m_sweep.a - bA.this.m_sweep.a;
    return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a;
  }

  public GetRevoluteJointSpeed(): number {
    const wA: number = this.m_bodyA.m_angularVelocity;
    const wB: number = this.m_bodyB.m_angularVelocity;
    return wB - wA;
  }

  public IsMotorEnabled(): boolean {
    return this.m_enableMotor;
  }

  public EnableMotor(flag: boolean): void {
    if (flag !== this.m_enableMotor) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableMotor = flag;
    }
  }

  public SetMotorSpeed(speed: number): void {
    if (speed !== this.m_motorSpeed) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_motorSpeed = speed;
    }
  }

  public SetMaxMotorTorque(force: number): void {
    if (force !== this.m_maxMotorTorque) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_maxMotorTorque = force;
    }
  }

  public GetMotorTorque(inv_dt: number): number {
    return inv_dt * this.m_motorImpulse;
  }

  /// Is the joint limit enabled?
  public IsLimitEnabled(): boolean {
    return this.m_enableLimit;
  }

  /// Enable/disable the joint translation limit.
  public EnableLimit(flag: boolean): void {
    if (flag !== this.m_enableLimit) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableLimit = flag;
      this.m_lowerImpulse = 0.0;
      this.m_upperImpulse = 0.0;
    }
  }

  /// Get the lower joint translation limit, usually in meters.
  public GetLowerLimit(): number {
    return this.m_lowerTranslation;
  }

  /// Get the upper joint translation limit, usually in meters.
  public GetUpperLimit(): number {
    return this.m_upperTranslation;
  }

  /// Set the joint translation limits, usually in meters.
  public SetLimits(lower: number, upper: number): void {
    // B2Assert(lower <= upper);
    if (lower !== this.m_lowerTranslation || upper !== this.m_upperTranslation) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_lowerTranslation = lower;
      this.m_upperTranslation = upper;
      this.m_lowerImpulse = 0.0;
      this.m_upperImpulse = 0.0;
    }
  }

  public override Dump(log: (format: string, ...args: any[]) => void): void {
    const indexA = this.m_bodyA.m_islandIndex;
    const indexB = this.m_bodyB.m_islandIndex;

    log("  const jd: B2WheelJointDef = new B2WheelJointDef();\n");
    log("  jd.bodyA = bodies[%d];\n", indexA);
    log("  jd.bodyB = bodies[%d];\n", indexB);
    log("  jd.collideConnected = %s;\n", (this.m_collideConnected) ? ("true") : ("false"));
    log("  jd.localAnchorA.Set(%.15f, %.15f);\n", this.m_localAnchorA.x, this.m_localAnchorA.y);
    log("  jd.localAnchorB.Set(%.15f, %.15f);\n", this.m_localAnchorB.x, this.m_localAnchorB.y);
    log("  jd.localAxisA.Set(%.15f, %.15f);\n", this.m_localXAxisA.x, this.m_localXAxisA.y);
    log("  jd.enableMotor = %s;\n", (this.m_enableMotor) ? ("true") : ("false"));
    log("  jd.motorSpeed = %.15f;\n", this.m_motorSpeed);
    log("  jd.maxMotorTorque = %.15f;\n", this.m_maxMotorTorque);
    log("  jd.stiffness = %.15f;\n", this.m_stiffness);
    log("  jd.damping = %.15f;\n", this.m_damping);
    log("  joints[%d] = this.m_world.CreateJoint(jd);\n", this.m_index);
  }

  ///
  private static Draw_s_pA = new B2Vec2();
  private static Draw_s_pB = new B2Vec2();
  private static Draw_s_axis = new B2Vec2();
  private static Draw_s_c1 = new B2Color(0.7, 0.7, 0.7);
  private static Draw_s_c2 = new B2Color(0.3, 0.9, 0.3);
  private static Draw_s_c3 = new B2Color(0.9, 0.3, 0.3);
  private static Draw_s_c4 = new B2Color(0.3, 0.3, 0.9);
  private static Draw_s_c5 = new B2Color(0.4, 0.4, 0.4);
  private static Draw_s_lower = new B2Vec2();
  private static Draw_s_upper = new B2Vec2();
  private static Draw_s_perp = new B2Vec2();
  public override Draw(draw: B2Draw): void {
    const xfA: Readonly<B2Transform> = this.m_bodyA.GetTransform();
    const xfB: Readonly<B2Transform> = this.m_bodyB.GetTransform();
    const pA = B2Transform.MulXV(xfA, this.m_localAnchorA, B2WheelJoint.Draw_s_pA);
    const pB = B2Transform.MulXV(xfB, this.m_localAnchorB, B2WheelJoint.Draw_s_pB);

    // B2Vec2 axis = B2Mul(xfA.q, m_localXAxisA);
    const axis: B2Vec2 = B2Rot.MulRV(xfA.q, this.m_localXAxisA, B2WheelJoint.Draw_s_axis);

    const c1 = B2WheelJoint.Draw_s_c1; // B2Color c1(0.7f, 0.7f, 0.7f);
    const c2 = B2WheelJoint.Draw_s_c2; // B2Color c2(0.3f, 0.9f, 0.3f);
    const c3 = B2WheelJoint.Draw_s_c3; // B2Color c3(0.9f, 0.3f, 0.3f);
    const c4 = B2WheelJoint.Draw_s_c4; // B2Color c4(0.3f, 0.3f, 0.9f);
    const c5 = B2WheelJoint.Draw_s_c5; // B2Color c5(0.4f, 0.4f, 0.4f);

    draw.DrawSegment(pA, pB, c5);

    if (this.m_enableLimit) {
      // B2Vec2 lower = pA + m_lowerTranslation * axis;
      const lower = B2Vec2.AddVMulSV(pA, this.m_lowerTranslation, axis, B2WheelJoint.Draw_s_lower);
      // B2Vec2 upper = pA + m_upperTranslation * axis;
      const upper = B2Vec2.AddVMulSV(pA, this.m_upperTranslation, axis, B2WheelJoint.Draw_s_upper);
      // B2Vec2 perp = B2Mul(xfA.q, m_localYAxisA);
      const perp = B2Rot.MulRV(xfA.q, this.m_localYAxisA, B2WheelJoint.Draw_s_perp);
      // draw.DrawSegment(lower, upper, c1);
      draw.DrawSegment(lower, upper, c1);
      // draw.DrawSegment(lower - 0.5f * perp, lower + 0.5f * perp, c2);
      draw.DrawSegment(B2Vec2.AddVMulSV(lower, -0.5, perp, B2Vec2.s_t0), B2Vec2.AddVMulSV(lower, 0.5, perp, B2Vec2.s_t1), c2);
      // draw.DrawSegment(upper - 0.5f * perp, upper + 0.5f * perp, c3);
      draw.DrawSegment(B2Vec2.AddVMulSV(upper, -0.5, perp, B2Vec2.s_t0), B2Vec2.AddVMulSV(upper, 0.5, perp, B2Vec2.s_t1), c3);
    } else {
      // draw.DrawSegment(pA - 1.0f * axis, pA + 1.0f * axis, c1);
      draw.DrawSegment(B2Vec2.AddVMulSV(pA, -1.0, axis, B2Vec2.s_t0), B2Vec2.AddVMulSV(pA, 1.0, axis, B2Vec2.s_t1), c1);
    }

    draw.DrawPoint(pA, 5.0, c1);
    draw.DrawPoint(pB, 5.0, c4);
  }
}
