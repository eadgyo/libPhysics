package org.cora.physics.force;

import org.cora.maths.Vector2D;
import org.cora.physics.entities.Particle;

/**
 * Created by ronan-h on 11/06/16.
 */
public class Buoyancy extends ForceGenerator
{
    private float maxDepth;
    private float volume;
    private float waterHeight;
    private float liquidDensity;

    public Buoyancy(float maxDepth, float volume, float waterHeight, float liquidDensity)
    {
        this.maxDepth = maxDepth;
        this.volume = volume;
        this.waterHeight = waterHeight;
        this.liquidDensity = liquidDensity;
    }

    @Override
    public void updateForce(Particle element, float dt)
    {
        float depth = element.getPosition().y;

        if (depth >= waterHeight + maxDepth)
            return;

        Vector2D force = new Vector2D(0, 0);

        if (depth <= waterHeight - maxDepth)
        {
            force.y = liquidDensity*volume;
            element.addForce(force);
            return;
        }

        force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;
        element.addForce(force);
    }
}
