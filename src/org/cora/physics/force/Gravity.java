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
}
