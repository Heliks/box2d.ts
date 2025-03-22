// DEBUG: import { B2Assert } from "../common/b2_settings.js";
import { B2ShapeType } from "../collision/b2_shape.js";
import { B2Contact } from "./b2_contact.js";
import { B2CircleContact } from "./b2_circle_contact.js";
import { B2PolygonContact } from "./b2_polygon_contact.js";
import { B2PolygonAndCircleContact } from "./b2_polygon_circle_contact.js";
import { B2EdgeAndCircleContact } from "./b2_edge_circle_contact.js";
import { B2EdgeAndPolygonContact } from "./b2_edge_polygon_contact.js";
import { B2ChainAndCircleContact } from "./b2_chain_circle_contact.js";
import { B2ChainAndPolygonContact } from "./b2_chain_polygon_contact.js";
import { B2Fixture } from "./b2_fixture.js";

export class B2ContactRegister {
  public pool: B2Contact[] = [];
  public createFcn: (() => B2Contact) | null = null;
  public destroyFcn: ((contact: B2Contact) => void) | null = null;
  public primary: boolean = false;
}

export class B2ContactFactory {
  public readonly m_registers: B2ContactRegister[][] = [];

  constructor() {
    this.InitializeRegisters();
  }

  private AddType(createFcn: () => B2Contact, destroyFcn: (contact: B2Contact) => void, typeA: B2ShapeType, typeB: B2ShapeType): void {
    const pool: B2Contact[] = [];

    function poolCreateFcn(): B2Contact {
      return pool.pop() || createFcn();
    }

    function poolDestroyFcn(contact: B2Contact): void {
      pool.push(contact);
    }

    this.m_registers[typeA][typeB].pool = pool;
    this.m_registers[typeA][typeB].createFcn = poolCreateFcn; // createFcn;
    this.m_registers[typeA][typeB].destroyFcn = poolDestroyFcn; // destroyFcn;
    this.m_registers[typeA][typeB].primary = true;

    if (typeA !== typeB) {
      this.m_registers[typeB][typeA].pool = pool;
      this.m_registers[typeB][typeA].createFcn = poolCreateFcn; // createFcn;
      this.m_registers[typeB][typeA].destroyFcn = poolDestroyFcn; // destroyFcn;
      this.m_registers[typeB][typeA].primary = false;
    }
  }

  private InitializeRegisters(): void {
    for (let i: number = 0; i < B2ShapeType.e_shapeTypeCount; i++) {
      this.m_registers[i] = [];
      for (let j: number = 0; j < B2ShapeType.e_shapeTypeCount; j++) {
        this.m_registers[i][j] = new B2ContactRegister();
      }
    }

    this.AddType(          B2CircleContact.Create,           B2CircleContact.Destroy, B2ShapeType.e_circleShape,  B2ShapeType.e_circleShape);
    this.AddType(B2PolygonAndCircleContact.Create, B2PolygonAndCircleContact.Destroy, B2ShapeType.e_polygonShape, B2ShapeType.e_circleShape);
    this.AddType(         B2PolygonContact.Create,          B2PolygonContact.Destroy, B2ShapeType.e_polygonShape, B2ShapeType.e_polygonShape);
    this.AddType(   B2EdgeAndCircleContact.Create,    B2EdgeAndCircleContact.Destroy, B2ShapeType.e_edgeShape,    B2ShapeType.e_circleShape);
    this.AddType(  B2EdgeAndPolygonContact.Create,   B2EdgeAndPolygonContact.Destroy, B2ShapeType.e_edgeShape,    B2ShapeType.e_polygonShape);
    this.AddType(  B2ChainAndCircleContact.Create,   B2ChainAndCircleContact.Destroy, B2ShapeType.e_chainShape,   B2ShapeType.e_circleShape);
    this.AddType( B2ChainAndPolygonContact.Create,  B2ChainAndPolygonContact.Destroy, B2ShapeType.e_chainShape,   B2ShapeType.e_polygonShape);
  }

  public Create(fixtureA: B2Fixture, indexA: number, fixtureB: B2Fixture, indexB: number): B2Contact | null {
    const typeA: B2ShapeType = fixtureA.GetType();
    const typeB: B2ShapeType = fixtureB.GetType();

    // DEBUG: B2Assert(0 <= typeA && typeA < B2ShapeType.e_shapeTypeCount);
    // DEBUG: B2Assert(0 <= typeB && typeB < B2ShapeType.e_shapeTypeCount);

    const reg: B2ContactRegister = this.m_registers[typeA][typeB];
    if (reg.createFcn) {
      const c: B2Contact = reg.createFcn();
      if (reg.primary) {
        c.Reset(fixtureA, indexA, fixtureB, indexB);
      } else {
        c.Reset(fixtureB, indexB, fixtureA, indexA);
      }
      return c;
    } else {
      return null;
    }
  }

  public Destroy(contact: B2Contact): void {
    const typeA: B2ShapeType = contact.m_fixtureA.GetType();
    const typeB: B2ShapeType = contact.m_fixtureB.GetType();

    // DEBUG: B2Assert(0 <= typeA && typeB < B2ShapeType.e_shapeTypeCount);
    // DEBUG: B2Assert(0 <= typeA && typeB < B2ShapeType.e_shapeTypeCount);

    const reg: B2ContactRegister = this.m_registers[typeA][typeB];
    if (reg.destroyFcn) {
      reg.destroyFcn(contact);
    }
  }
}
