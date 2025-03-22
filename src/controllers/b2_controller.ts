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

import { B2Body } from "../dynamics/b2_body.js";
import { B2TimeStep } from "../dynamics/b2_time_step.js";
import { B2Draw } from "../common/b2_draw.js";

/**
 * A controller edge is used to connect bodies and controllers
 * together in a bipartite graph.
 */
export class B2ControllerEdge {
  public readonly controller: B2Controller; ///< provides quick access to other end of this edge.
  public readonly body: B2Body; ///< the body
  public prevBody: B2ControllerEdge | null = null; ///< the previous controller edge in the controllers's joint list
  public nextBody: B2ControllerEdge | null = null; ///< the next controller edge in the controllers's joint list
  public prevController: B2ControllerEdge | null = null; ///< the previous controller edge in the body's joint list
  public nextController: B2ControllerEdge | null = null; ///< the next controller edge in the body's joint list
  constructor(controller: B2Controller, body: B2Body) {
    this.controller = controller;
    this.body = body;
  }
}

/**
 * Base class for controllers. Controllers are a convience for
 * encapsulating common per-step functionality.
 */
export abstract class B2Controller {
  // m_world: B2World;
  public m_bodyList: B2ControllerEdge | null = null;
  public m_bodyCount: number = 0;
  public m_prev: B2Controller | null = null;
  public m_next: B2Controller | null = null;

  /**
   * Controllers override this to implement per-step functionality.
   */
  public abstract Step(step: B2TimeStep): void;

  /**
   * Controllers override this to provide debug drawing.
   */
  public abstract Draw(debugDraw: B2Draw): void;

  /**
   * Get the next controller in the world's body list.
   */
  public GetNext(): B2Controller | null {
    return this.m_next;
  }

  /**
   * Get the previous controller in the world's body list.
   */
  public GetPrev(): B2Controller | null {
    return this.m_prev;
  }

  /**
   * Get the parent world of this body.
   */
  // GetWorld() {
  //   return this.m_world;
  // }

  /**
   * Get the attached body list
   */
  public GetBodyList(): B2ControllerEdge | null {
    return this.m_bodyList;
  }

  /**
   * Adds a body to the controller list.
   */
  public AddBody(body: B2Body): void {
    const edge = new B2ControllerEdge(this, body);

    //Add edge to controller list
    edge.nextBody = this.m_bodyList;
    edge.prevBody = null;
    if (this.m_bodyList) {
      this.m_bodyList.prevBody = edge;
    }
    this.m_bodyList = edge;
    ++this.m_bodyCount;

    //Add edge to body list
    edge.nextController = body.m_controllerList;
    edge.prevController = null;
    if (body.m_controllerList) {
      body.m_controllerList.prevController = edge;
    }
    body.m_controllerList = edge;
    ++body.m_controllerCount;
  }

  /**
   * Removes a body from the controller list.
   */
  public RemoveBody(body: B2Body): void {
    //Assert that the controller is not empty
    if (this.m_bodyCount <= 0) { throw new Error(); }

    //Find the corresponding edge
    /*B2ControllerEdge*/
    let edge = this.m_bodyList;
    while (edge && edge.body !== body) {
      edge = edge.nextBody;
    }

    //Assert that we are removing a body that is currently attached to the controller
    if (edge === null) { throw new Error(); }

    //Remove edge from controller list
    if (edge.prevBody) {
      edge.prevBody.nextBody = edge.nextBody;
    }
    if (edge.nextBody) {
      edge.nextBody.prevBody = edge.prevBody;
    }
    if (this.m_bodyList === edge) {
      this.m_bodyList = edge.nextBody;
    }
    --this.m_bodyCount;

    //Remove edge from body list
    if (edge.nextController) {
      edge.nextController.prevController = edge.prevController;
    }
    if (edge.prevController) {
      edge.prevController.nextController = edge.nextController;
    }
    if (body.m_controllerList === edge) {
      body.m_controllerList = edge.nextController;
    }
    --body.m_controllerCount;
  }

  /**
   * Removes all bodies from the controller list.
   */
  public Clear(): void {
    while (this.m_bodyList) {
      this.RemoveBody(this.m_bodyList.body);
    }

    this.m_bodyCount = 0;
  }
}

// #endif
