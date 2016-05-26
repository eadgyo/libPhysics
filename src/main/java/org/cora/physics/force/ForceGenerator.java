package org.cora.physics.force;

import org.cora.physics.entities.Particle;

public abstract class ForceGenerator
{
    abstract void updateForce(Particle element, float dt);
}
