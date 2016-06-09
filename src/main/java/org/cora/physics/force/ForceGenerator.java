package org.cora.physics.force;

import org.cora.physics.entities.Particle;

import java.util.Map;

/**
 * Used to create force
 */
public abstract class ForceGenerator implements Cloneable
{
    public abstract void updateForce(Particle element, float dt);

    public Object clone()
    {
        try
        {
            return super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            return null;
        }
    }

    public Object clone(Map<Particle, Particle> change)
    {
        return clone();
    }
}
