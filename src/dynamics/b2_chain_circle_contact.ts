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

import { B2Transform } from "../common/b2_math.js";
import { B2CollideEdgeAndCircle } from "../collision/b2_collide_edge.js";
import { B2Manifold } from "../collision/b2_collision.js";
import { B2ChainShape } from "../collision/b2_chain_shape.js";
import { B2CircleShape } from "../collision/b2_circle_shape.js";
import { B2EdgeShape } from "../collision/b2_edge_shape.js";
import { B2Contact } from "./b2_contact.js";

export class B2ChainAndCircleContact extends B2Contact<B2ChainShape, B2CircleShape> {
  public static Create(): B2Contact {
    return new B2ChainAndCircleContact();
  }

  public static Destroy(contact: B2Contact): void {
  }

  private static Evaluate_s_edge = new B2EdgeShape();
  public Evaluate(manifold: B2Manifold, xfA: B2Transform, xfB: B2Transform): void {
    const edge: B2EdgeShape = B2ChainAndCircleContact.Evaluate_s_edge;
    this.GetShapeA().GetChildEdge(edge, this.m_indexA);
    B2CollideEdgeAndCircle(manifold, edge, xfA, this.GetShapeB(), xfB);
  }
}
