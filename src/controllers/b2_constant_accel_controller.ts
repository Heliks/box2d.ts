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

// #if B2_ENABLE_CONTROLLER

import { B2Controller } from "./b2_controller.js";
import { B2Vec2 } from "../common/b2_math.js";
import { B2TimeStep } from "../dynamics/b2_time_step.js";
import { B2Draw } from "../common/b2_draw.js";

/**
 * Applies a force every frame
 */
export class B2ConstantAccelController extends B2Controller {
  /**
   * The acceleration to apply
   */
  public readonly A = new B2Vec2(0, 0);

  public Step(step: B2TimeStep) {
    const dtA = B2Vec2.MulSV(step.dt, this.A, B2ConstantAccelController.Step_s_dtA);
    for (let i = this.m_bodyList; i; i = i.nextBody) {
      const body = i.body;
      if (!body.IsAwake()) {
        continue;
      }
      body.SetLinearVelocity(B2Vec2.AddVV(body.GetLinearVelocity(), dtA, B2Vec2.s_t0));
    }
  }
  private static Step_s_dtA = new B2Vec2();

  public Draw(draw: B2Draw) {}
}

// #endif
