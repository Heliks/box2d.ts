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

// DEBUG: import { B2Assert } from "../common/b2_settings.js";
import { B2Maybe, B2_pi } from "../common/b2_settings.js";
import { B2Vec2, XY, B2Transform } from "../common/b2_math.js";
import { B2Body } from "./b2_body.js";
import { B2SolverData } from "./b2_time_step.js";
import { B2Draw, B2Color } from "../common/b2_draw.js";
import { B2PulleyJoint } from "./b2_pulley_joint.js";

export enum B2JointType {
  e_unknownJoint = 0,
  e_revoluteJoint = 1,
  e_prismaticJoint = 2,
  e_distanceJoint = 3,
  e_pulleyJoint = 4,
  e_mouseJoint = 5,
  e_gearJoint = 6,
  e_wheelJoint = 7,
  e_weldJoint = 8,
  e_frictionJoint = 9,
  e_ropeJoint = 10,
  e_motorJoint = 11,
  e_areaJoint = 12,
}

export class B2Jacobian {
  public readonly linear: B2Vec2 = new B2Vec2();
  public angularA: number = 0;
  public angularB: number = 0;

  public SetZero(): B2Jacobian {
    this.linear.SetZero();
    this.angularA = 0;
    this.angularB = 0;
    return this;
  }

  public Set(x: XY, a1: number, a2: number): B2Jacobian {
    this.linear.Copy(x);
    this.angularA = a1;
    this.angularB = a2;
    return this;
  }
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
export class B2JointEdge {
  private _other: B2Body | null = null; ///< provides quick access to the other body attached.
  public get other(): B2Body {
    if (this._other === null) { throw new Error(); }
    return this._other;
  }
  public set other(value: B2Body) {
    if (this._other !== null) { throw new Error(); }
    this._other = value;
  }
  public readonly joint: B2Joint;    ///< the joint
  public prev: B2JointEdge | null = null;  ///< the previous joint edge in the body's joint list
  public next: B2JointEdge | null = null;  ///< the next joint edge in the body's joint list
  constructor(joint: B2Joint) {
    this.joint = joint;
  }
  public Reset(): void {
    this._other = null;
    this.prev = null;
    this.next = null;
  }
}

/// Joint definitions are used to construct joints.
export interface B2IJointDef {
  /// The joint type is set automatically for concrete joint types.
  type: B2JointType;

  /// Use this to attach application specific data to your joints.
  userData?: any;

  /// The first attached body.
  bodyA: B2Body;

  /// The second attached body.
  bodyB: B2Body;

  /// Set this flag to true if the attached bodies should collide.
  collideConnected?: boolean;
}

/// Joint definitions are used to construct joints.
export abstract class B2JointDef implements B2IJointDef {
  /// The joint type is set automatically for concrete joint types.
  public readonly type: B2JointType = B2JointType.e_unknownJoint;

  /// Use this to attach application specific data to your joints.
  public userData: any = null;

  /// The first attached body.
  public bodyA!: B2Body;

  /// The second attached body.
  public bodyB!: B2Body;

  /// Set this flag to true if the attached bodies should collide.
  public collideConnected: boolean = false;

  constructor(type: B2JointType) {
    this.type = type;
  }
}

/// Utility to compute linear stiffness values from frequency and damping ratio
// void B2LinearStiffness(float& stiffness, float& damping,
// 	float frequencyHertz, float dampingRatio,
// 	const B2Body* bodyA, const B2Body* bodyB);
export function B2LinearStiffness(def: { stiffness: number, damping: number }, frequencyHertz: number, dampingRatio: number, bodyA: B2Body, bodyB: B2Body): void {
  const massA: number = bodyA.GetMass();
  const massB: number = bodyB.GetMass();
  let mass: number;
  if (massA > 0.0 && massB > 0.0) {
    mass = massA * massB / (massA + massB);
  } else if (massA > 0.0) {
    mass = massA;
  } else {
    mass = massB;
  }

  const omega: number = 2.0 * B2_pi * frequencyHertz;
  def.stiffness = mass * omega * omega;
  def.damping = 2.0 * mass * dampingRatio * omega;
}

/// Utility to compute rotational stiffness values frequency and damping ratio
// void B2AngularStiffness(float& stiffness, float& damping,
// 	float frequencyHertz, float dampingRatio,
// 	const B2Body* bodyA, const B2Body* bodyB);
export function B2AngularStiffness(def: { stiffness: number, damping: number }, frequencyHertz: number, dampingRatio: number, bodyA: B2Body, bodyB: B2Body): void {
  const IA: number = bodyA.GetInertia();
  const IB: number = bodyB.GetInertia();
  let I: number;
  if (IA > 0.0 && IB > 0.0) {
    I = IA * IB / (IA + IB);
  } else if (IA > 0.0) {
    I = IA;
  } else {
    I = IB;
  }

  const omega: number = 2.0 * B2_pi * frequencyHertz;
  def.stiffness = I * omega * omega;
  def.damping = 2.0 * I * dampingRatio * omega;
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
export abstract class B2Joint {
  public readonly m_type: B2JointType = B2JointType.e_unknownJoint;
  public m_prev: B2Joint | null = null;
  public m_next: B2Joint | null = null;
  public readonly m_edgeA: B2JointEdge = new B2JointEdge(this);
  public readonly m_edgeB: B2JointEdge = new B2JointEdge(this);
  public m_bodyA: B2Body;
  public m_bodyB: B2Body;

  public m_index: number = 0;

  public m_islandFlag: boolean = false;
  public m_collideConnected: boolean = false;

  public m_userData: any = null;

  constructor(def: B2IJointDef) {
    // DEBUG: B2Assert(def.bodyA !== def.bodyB);

    this.m_type = def.type;
    this.m_edgeA.other = def.bodyB;
    this.m_edgeB.other = def.bodyA;
    this.m_bodyA = def.bodyA;
    this.m_bodyB = def.bodyB;

    this.m_collideConnected = B2Maybe(def.collideConnected, false);

    this.m_userData = B2Maybe(def.userData, null);
  }

  /// Get the type of the concrete joint.
  public GetType(): B2JointType {
    return this.m_type;
  }

  /// Get the first body attached to this joint.
  public GetBodyA(): B2Body {
    return this.m_bodyA;
  }

  /// Get the second body attached to this joint.
  public GetBodyB(): B2Body {
    return this.m_bodyB;
  }

  /// Get the anchor point on bodyA in world coordinates.
  public abstract GetAnchorA<T extends XY>(out: T): T;

  /// Get the anchor point on bodyB in world coordinates.
  public abstract GetAnchorB<T extends XY>(out: T): T;

  /// Get the reaction force on bodyB at the joint anchor in Newtons.
  public abstract GetReactionForce<T extends XY>(inv_dt: number, out: T): T;

  /// Get the reaction torque on bodyB in N*m.
  public abstract GetReactionTorque(inv_dt: number): number;

  /// Get the next joint the world joint list.
  public GetNext(): B2Joint | null {
    return this.m_next;
  }

  /// Get the user data pointer.
  public GetUserData(): any {
    return this.m_userData;
  }

  /// Set the user data pointer.
  public SetUserData(data: any): void {
    this.m_userData = data;
  }

  /// Short-cut function to determine if either body is inactive.
  public IsEnabled(): boolean {
    return this.m_bodyA.IsEnabled() && this.m_bodyB.IsEnabled();
  }

  /// Get collide connected.
  /// Note: modifying the collide connect flag won't work correctly because
  /// the flag is only checked when fixture AABBs begin to overlap.
  public GetCollideConnected(): boolean {
    return this.m_collideConnected;
  }

  /// Dump this joint to the log file.
  public Dump(log: (format: string, ...args: any[]) => void): void {
    log("// Dump is not supported for this joint type.\n");
  }

  /// Shift the origin for any points stored in world coordinates.
  public ShiftOrigin(newOrigin: XY): void { }

  /// Debug draw this joint
  private static Draw_s_p1: B2Vec2 = new B2Vec2();
  private static Draw_s_p2: B2Vec2 = new B2Vec2();
  private static Draw_s_color: B2Color = new B2Color(0.5, 0.8, 0.8);
  private static Draw_s_c: B2Color = new B2Color();
  public Draw(draw: B2Draw): void {
    const xf1: B2Transform = this.m_bodyA.GetTransform();
    const xf2: B2Transform = this.m_bodyB.GetTransform();
    const x1: B2Vec2 = xf1.p;
    const x2: B2Vec2 = xf2.p;
    const p1: B2Vec2 = this.GetAnchorA(B2Joint.Draw_s_p1);
    const p2: B2Vec2 = this.GetAnchorB(B2Joint.Draw_s_p2);

    const color: B2Color = B2Joint.Draw_s_color.SetRGB(0.5, 0.8, 0.8);

    switch (this.m_type) {
      case B2JointType.e_distanceJoint:
        draw.DrawSegment(p1, p2, color);
        break;

      case B2JointType.e_pulleyJoint:
        {
          const pulley: B2PulleyJoint = this as unknown as B2PulleyJoint;
          const s1: B2Vec2 = pulley.GetGroundAnchorA();
          const s2: B2Vec2 = pulley.GetGroundAnchorB();
          draw.DrawSegment(s1, p1, color);
          draw.DrawSegment(s2, p2, color);
          draw.DrawSegment(s1, s2, color);
        }
        break;

      case B2JointType.e_mouseJoint:
        {
          const c = B2Joint.Draw_s_c;
          c.Set(0.0, 1.0, 0.0);
          draw.DrawPoint(p1, 4.0, c);
          draw.DrawPoint(p2, 4.0, c);

          c.Set(0.8, 0.8, 0.8);
          draw.DrawSegment(p1, p2, c);
        }
        break;

      default:
        draw.DrawSegment(x1, p1, color);
        draw.DrawSegment(p1, p2, color);
        draw.DrawSegment(x2, p2, color);
      }
    }

  public abstract InitVelocityConstraints(data: B2SolverData): void;

  public abstract SolveVelocityConstraints(data: B2SolverData): void;

  // This returns true if the position errors are within tolerance.
  public abstract SolvePositionConstraints(data: B2SolverData): boolean;
}
