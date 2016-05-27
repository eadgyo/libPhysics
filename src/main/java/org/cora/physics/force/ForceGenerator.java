package org.cora.physics.force;

import org.cora.physics.entities.Particle;

/**
 * Used to create force
 */
public abstract class ForceGenerator
{
    abstract void updateForce(Particle element, float dt);
}
