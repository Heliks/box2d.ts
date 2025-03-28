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

// #if B2_ENABLE_CONTROLLER

import { B2Controller } from "./b2_controller.js";
import { B2Mat22, B2Vec2, B2Max } from "../common/b2_math.js";
import { B2TimeStep } from "../dynamics/b2_time_step.js";
import { B2_epsilon } from "../common/b2_settings.js";
import { B2Draw } from "../common/b2_draw.js";

/**
 * Applies top down linear damping to the controlled bodies
 * The damping is calculated by multiplying velocity by a matrix
 * in local co-ordinates.
 */
export class B2TensorDampingController extends B2Controller {
    /// Tensor to use in damping model
    public readonly T = new B2Mat22();
    /*Some examples (matrixes in format (row1; row2))
    (-a 0; 0 -a)    Standard isotropic damping with strength a
    ( 0 a; -a 0)    Electron in fixed field - a force at right angles to velocity with proportional magnitude
    (-a 0; 0 -b)    Differing x and y damping. Useful e.g. for top-down wheels.
    */
    //By the way, tensor in this case just means matrix, don't let the terminology get you down.

    /// Set this to a positive number to clamp the maximum amount of damping done.
    public maxTimestep = 0;
    // Typically one wants maxTimestep to be 1/(max eigenvalue of T), so that damping will never cause something to reverse direction

    /**
     * @see B2Controller::Step
     */
    public Step(step: B2TimeStep) {
        let timestep = step.dt;
        if (timestep <= B2_epsilon) {
            return;
        }
        if (timestep > this.maxTimestep && this.maxTimestep > 0) {
            timestep = this.maxTimestep;
        }
        for (let i = this.m_bodyList; i; i = i.nextBody) {
            const body = i.body;
            if (!body.IsAwake()) {
                continue;
            }
            const damping = body.GetWorldVector(
            B2Mat22.MulMV(
                this.T,
                body.GetLocalVector(
                body.GetLinearVelocity(),
                B2Vec2.s_t0),
                B2Vec2.s_t1),
            B2TensorDampingController.Step_s_damping);
            //    body->SetLinearVelocity(body->GetLinearVelocity() + timestep * damping);
            body.SetLinearVelocity(B2Vec2.AddVV(body.GetLinearVelocity(), B2Vec2.MulSV(timestep, damping, B2Vec2.s_t0), B2Vec2.s_t1));
        }
    }
    private static Step_s_damping = new B2Vec2();

    public Draw(draw: B2Draw) {}

    /**
     * Sets damping independantly along the x and y axes
     */
    public SetAxisAligned(xDamping: number, yDamping: number) {
      this.T.ex.x = (-xDamping);
      this.T.ex.y = 0;
      this.T.ey.x = 0;
      this.T.ey.y = (-yDamping);
      if (xDamping > 0 || yDamping > 0) {
        this.maxTimestep = 1 / B2Max(xDamping, yDamping);
      } else {
        this.maxTimestep = 0;
      }
    }
}

// #endif
