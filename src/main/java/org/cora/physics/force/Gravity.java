package org.cora.physics.force;

import org.cora.physics.entities.Particle;

import org.cora.maths.Vector2D;

public class Gravity extends ForceGenerator
{
    Vector2D gravity;
    
    public Gravity(Vector2D gravity)
    {
	this.gravity = gravity;
    }
    
    @Override
    public void updateForce(Particle p, float dt)
    {
	if (p.getInverseMass() == 0.0f)
	    return;
	
	p.addForce(gravity.multiply(p.getMass()));
    }

    @Override
    public Object clone()
    {
        Gravity g = null;

        g = (Gravity) super.clone();

        if (g == null)
            return g;

        g.gravity = (Vector2D) gravity.clone();

        return g;
    }
}
