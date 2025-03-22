// DEBUG: import { B2Assert } from "../common/b2_settings.js";
import { B2_epsilon, B2_linearSlop, B2_maxLinearCorrection, B2MakeNumberArray, B2Maybe } from "../common/b2_settings.js";
import { B2Sq, B2Sqrt, B2Vec2, XY } from "../common/b2_math.js";
import { B2Joint, B2JointDef, B2JointType, B2IJointDef } from "./b2_joint.js";
import { B2DistanceJoint, B2DistanceJointDef } from "./b2_distance_joint.js";
import { B2SolverData } from "./b2_time_step.js";
import { B2Body } from "./b2_body.js";

export interface B2IAreaJointDef extends B2IJointDef {
  // world: B2World;

  bodies: B2Body[];

  stiffness?: number;

  damping?: number;
}

export class B2AreaJointDef extends B2JointDef implements B2IAreaJointDef {
  public bodies: B2Body[] = [];

  public stiffness: number = 0;

  public damping: number = 0;

  constructor() {
    super(B2JointType.e_areaJoint);
  }

  public AddBody(body: B2Body): void {
    this.bodies.push(body);

    if (this.bodies.length === 1) {
      this.bodyA = body;
    } else if (this.bodies.length === 2) {
      this.bodyB = body;
    }
  }
}

export class B2AreaJoint extends B2Joint {
  public m_bodies: B2Body[];
  public m_stiffness: number = 0;
  public m_damping: number = 0;

  // Solver shared
  public m_impulse: number = 0;

  // Solver temp
  public readonly m_targetLengths: number[];
  public m_targetArea: number = 0;
  public readonly m_normals: B2Vec2[];
  public readonly m_joints: B2DistanceJoint[];
  public readonly m_deltas: B2Vec2[];
  public readonly m_delta: B2Vec2 = new B2Vec2();

  constructor(def: B2IAreaJointDef) {
    super(def);

    // DEBUG: B2Assert(def.bodies.length >= 3, "You cannot create an area joint with less than three bodies.");

    this.m_bodies = def.bodies;
    this.m_stiffness = B2Maybe(def.stiffness, 0);
    this.m_damping = B2Maybe(def.damping, 0);

    this.m_targetLengths = B2MakeNumberArray(def.bodies.length);
    this.m_normals = B2Vec2.MakeArray(def.bodies.length);
    this.m_joints = []; // B2MakeNullArray(def.bodies.length);
    this.m_deltas = B2Vec2.MakeArray(def.bodies.length);

    const djd: B2DistanceJointDef = new B2DistanceJointDef();
    djd.stiffness = this.m_stiffness;
    djd.damping = this.m_damping;

    this.m_targetArea = 0;

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: B2Body = this.m_bodies[i];
      const next: B2Body = this.m_bodies[(i + 1) % this.m_bodies.length];

      const body_c: B2Vec2 = body.GetWorldCenter();
      const next_c: B2Vec2 = next.GetWorldCenter();

      this.m_targetLengths[i] = B2Vec2.DistanceVV(body_c, next_c);

      this.m_targetArea += B2Vec2.CrossVV(body_c, next_c);

      djd.Initialize(body, next, body_c, next_c);
      this.m_joints[i] = body.GetWorld().CreateJoint(djd);
    }

    this.m_targetArea *= 0.5;
  }

  public GetAnchorA<T extends XY>(out: T): T {
    return out;
  }

  public GetAnchorB<T extends XY>(out: T): T {
    return out;
  }

  public GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    return out;
  }

  public GetReactionTorque(inv_dt: number): number {
    return 0;
  }

  public SetStiffness(stiffness: number): void {
    this.m_stiffness = stiffness;

    for (let i: number = 0; i < this.m_joints.length; ++i) {
      this.m_joints[i].SetStiffness(stiffness);
    }
  }

  public GetStiffness() {
    return this.m_stiffness;
  }

  public SetDamping(damping: number): void {
    this.m_damping = damping;

    for (let i: number = 0; i < this.m_joints.length; ++i) {
      this.m_joints[i].SetDamping(damping);
    }
  }

  public GetDamping() {
    return this.m_damping;
  }

  public override Dump(log: (format: string, ...args: any[]) => void) {
    log("Area joint dumping is not supported.\n");
  }

  public InitVelocityConstraints(data: B2SolverData): void {
    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const prev: B2Body = this.m_bodies[(i + this.m_bodies.length - 1) % this.m_bodies.length];
      const next: B2Body = this.m_bodies[(i + 1) % this.m_bodies.length];
      const prev_c: B2Vec2 = data.positions[prev.m_islandIndex].c;
      const next_c: B2Vec2 = data.positions[next.m_islandIndex].c;
      const delta: B2Vec2 = this.m_deltas[i];

      B2Vec2.SubVV(next_c, prev_c, delta);
    }

    if (data.step.warmStarting) {
      this.m_impulse *= data.step.dtRatio;

      for (let i: number = 0; i < this.m_bodies.length; ++i) {
        const body: B2Body = this.m_bodies[i];
        const body_v: B2Vec2 = data.velocities[body.m_islandIndex].v;
        const delta: B2Vec2 = this.m_deltas[i];

        body_v.x += body.m_invMass *  delta.y * 0.5 * this.m_impulse;
        body_v.y += body.m_invMass * -delta.x * 0.5 * this.m_impulse;
      }
    } else {
      this.m_impulse = 0;
    }
  }

  public SolveVelocityConstraints(data: B2SolverData): void {
    let dotMassSum: number = 0;
    let crossMassSum: number = 0;

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: B2Body = this.m_bodies[i];
      const body_v: B2Vec2 = data.velocities[body.m_islandIndex].v;
      const delta: B2Vec2 = this.m_deltas[i];

      dotMassSum += delta.LengthSquared() / body.GetMass();
      crossMassSum += B2Vec2.CrossVV(body_v, delta);
    }

    const lambda: number = -2 * crossMassSum / dotMassSum;
    // lambda = B2Clamp(lambda, -B2_maxLinearCorrection, B2_maxLinearCorrection);

    this.m_impulse += lambda;

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: B2Body = this.m_bodies[i];
      const body_v: B2Vec2 = data.velocities[body.m_islandIndex].v;
      const delta: B2Vec2 = this.m_deltas[i];

      body_v.x += body.m_invMass *  delta.y * 0.5 * lambda;
      body_v.y += body.m_invMass * -delta.x * 0.5 * lambda;
    }
  }

  public SolvePositionConstraints(data: B2SolverData): boolean {
    let perimeter: number = 0;
    let area: number = 0;

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: B2Body = this.m_bodies[i];
      const next: B2Body = this.m_bodies[(i + 1) % this.m_bodies.length];
      const body_c: B2Vec2 = data.positions[body.m_islandIndex].c;
      const next_c: B2Vec2 = data.positions[next.m_islandIndex].c;

      const delta: B2Vec2 = B2Vec2.SubVV(next_c, body_c, this.m_delta);

      let dist: number = delta.Length();
      if (dist < B2_epsilon) {
        dist = 1;
      }

      this.m_normals[i].x =  delta.y / dist;
      this.m_normals[i].y = -delta.x / dist;

      perimeter += dist;

      area += B2Vec2.CrossVV(body_c, next_c);
    }

    area *= 0.5;

    const deltaArea: number = this.m_targetArea - area;
    const toExtrude: number = 0.5 * deltaArea / perimeter;
    let done: boolean = true;

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: B2Body = this.m_bodies[i];
      const body_c: B2Vec2 = data.positions[body.m_islandIndex].c;
      const next_i: number = (i + 1) % this.m_bodies.length;

      const delta: B2Vec2 = B2Vec2.AddVV(this.m_normals[i], this.m_normals[next_i], this.m_delta);
      delta.SelfMul(toExtrude);

      const norm_sq: number = delta.LengthSquared();
      if (norm_sq > B2Sq(B2_maxLinearCorrection)) {
        delta.SelfMul(B2_maxLinearCorrection / B2Sqrt(norm_sq));
      }
      if (norm_sq > B2Sq(B2_linearSlop)) {
        done = false;
      }

      body_c.x += delta.x;
      body_c.y += delta.y;
    }

    return done;
  }
}
