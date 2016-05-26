package org.cora.physics.entities;

import org.cora.maths.Vector2D;

public class RigidBody extends Particle
{
    private final static float dampingRotation = 0.95f;
    private float              inverseInertia;
    private float              rotation, torqueAccum;

    @Override
    public void initPhysics()
    {
        super.initPhysics();
        if (form != null)
        {
            computeInertia();
        }
    }

    public float getTorqueAccum()
    {
        return torqueAccum;
    }

    public void setTorqueAccum(float torqueAccum)
    {
        this.torqueAccum = torqueAccum;
    }

    public float getTorque()
    {
        return torqueAccum;
    }

    public void setTorque(float torque)
    {
        this.torqueAccum = torque;
    }

    public RigidBody()
    {
        super();
        inverseInertia = 0;
        torqueAccum = 0;
    }

    public void set(RigidBody rb)
    {
        super.set(rb);
        inverseInertia = rb.getInverseInertia();
        torqueAccum = rb.getTorque();
    }

    public void setInverseInertiaTensor(float inverseInertia)
    {
        this.inverseInertia = inverseInertia;
    }

    public Object clone()
    {
        RigidBody clone = new RigidBody();
        clone.set(this);
        return clone;
    }

    @Override
    public void integrate(float dt)
    {
        if (inverseMass <= 0.0f)
            return;
        
        lastAcceleration.set(acceleration);
        acceleration.reset();

        // Integrate position
        setPosition(position.addScaledVector(velocity, dt));
        rotateRadians(rotation * dt, position);

        // Integrate Velocity
        acceleration.selfAddScaledVector(forceAccum, inverseMass);
        velocity.selfAddScaledVector(acceleration, dt);
        velocity.selfMultiply((float) Math.pow(damping, dt));

        float rotationAcceleration = torqueAccum * inverseInertia;
        rotation += rotationAcceleration * dt;
        rotation *= (float) Math.pow(dampingRotation, dt);

        clearAccumulator();
    }

    public void clearAccumulator()
    {
        super.clearAccumulator();
        torqueAccum = 0;
    }

    public void addForceAtPoint(Vector2D force, Vector2D p)
    {
        forceAccum.selfAdd(force);
        torqueAccum += (p.sub(position)).crossProductZ(force);
    }

    public void computeInertia()
    {
        inverseInertia = form.calculateInertia(inverseMass);
    }
    
    public float getInverseInertia()
    {
        return inverseInertia;
    }

    public void setInverseInertia(float inverseInertia)
    {
        this.inverseInertia = inverseInertia;
    }
    
    public float getRotation()
    {
        return rotation;
    }

    public void setRotation(float rotation)
    {
        this.rotation = rotation;
    }
}
