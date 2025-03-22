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
import { B2CollidePolygonAndCircle } from "../collision/b2_collide_circle.js";
import { B2Manifold } from "../collision/b2_collision.js";
import { B2CircleShape } from "../collision/b2_circle_shape.js";
import { B2PolygonShape } from "../collision/b2_polygon_shape.js";
import { B2Contact } from "./b2_contact.js";

export class B2PolygonAndCircleContact extends B2Contact<B2PolygonShape, B2CircleShape> {
  public static Create(): B2Contact {
    return new B2PolygonAndCircleContact();
  }

  public static Destroy(contact: B2Contact): void {
  }

  public Evaluate(manifold: B2Manifold, xfA: B2Transform, xfB: B2Transform): void {
    B2CollidePolygonAndCircle(manifold, this.GetShapeA(), xfA, this.GetShapeB(), xfB);
  }
}
