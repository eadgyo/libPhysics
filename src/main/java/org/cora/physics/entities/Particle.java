package org.cora.physics.entities;

import org.cora.maths.Form;
import org.cora.maths.Vector2D;
import org.cora.physics.collision.ContactInformation;
import org.cora.physics.entities.material.MaterialType;

public class Particle implements Cloneable
{
    protected boolean            isAwake;
    protected final static float damping = 0.999f;

    protected Vector2D           position, velocity, acceleration,
            lastAcceleration;

    protected Vector2D           forceAccum;
    protected float              inverseMass;

    protected Form               form;
    protected MaterialType       materialType;

    public Particle()
    {
        position = new Vector2D();
        velocity = new Vector2D();
        acceleration = new Vector2D();
        lastAcceleration = new Vector2D();
        forceAccum = new Vector2D();

        isAwake = true;
        inverseMass = 0;
        materialType = null;
    }

    public void set(Particle p)
    {
        position.set(p.getPosition());
        velocity.set(p.getVelocity());
        lastAcceleration.set(p.getLastAcceleration());
        acceleration.set(p.getAcceleration());
        forceAccum.set(p.getForceAccum());
        materialType = p.getMaterialType();

        isAwake = p.isAwake();
        inverseMass = p.getInverseMass();
        if (p.getForm() != null)
            setForm((Form) p.getForm().clone());
    }

    public void initPhysics()
    {

    }

    public Object clone()
    {
        Particle clone = new Particle();
        clone.set(this);
        return clone;
    }

    public void integrate(float dt)
    {
        if (inverseMass <= 0.0f)
            return;
        assert (dt > 0.0f);
        lastAcceleration.set(acceleration);
        acceleration.reset();

        // Integrate position
        setPosition(position.addScaledVector(velocity, dt));

        // Integrate Velocity
        acceleration.selfAddScaledVector(forceAccum, inverseMass);
        velocity.selfAddScaledVector(acceleration, dt);
        velocity.selfMultiply((float) Math.pow(damping, dt));

        clearAccumulator();
    }

    public void clearAccumulator()
    {
        forceAccum.reset();
    }

    public Vector2D getForceAccum()
    {
        return forceAccum;
    }
    
    public ContactInformation getContactInformation(Particle B)
    {
        return materialType.getMaterialInformation(B.getMaterialType());
    }

    public MaterialType getMaterialType()
    {
        return materialType;
    }
    
    public void setForceAccum(Vector2D forceAccum)
    {
        this.forceAccum = forceAccum;
    }

    public void addForce(Vector2D force)
    {
        forceAccum.selfAdd(force);
    }

    // Transformations
    public void setDegrees(float degrees)
    {
        setRadians((float) ((degrees * Math.PI) / 180), new Vector2D(1, 0));
    }

    public void setDegrees(float degrees, Vector2D vec)
    {
        setRadians((float) ((degrees * Math.PI) / 180), vec);
    }

    public void setRadians(float radians, Vector2D vec)
    {
        float angle = radians - form.getOmega();
        this.rotateRadians(angle, position);
    }

    public void setRadians(float radians)
    {
        this.setRadians(radians, new Vector2D(1, 0));
    }

    public void setScale(float scale)
    {
        float factor = scale * (1 / this.getScale());
        this.scale(factor, position);
    }
    
    public void setMaterialType(MaterialType materialType)
    {
        this.materialType = materialType;
    }

    // /////////////////////////////
    // Transformations //
    // /////////////////////////////
    public void translate(Vector2D vec)
    {
        form.translate(vec);
        position.set(form.getCenter());
    }

    public void translateX(float vecX)
    {
        position.translateX(vecX);
        position.set(form.getCenter());
    }

    public void translateY(float vecY)
    {
        form.translate(new Vector2D(0, vecY));
        position.set(form.getCenter());
    }

    public void flipH(Vector2D center)
    {
        form.flipH(center);
        position.set(form.getCenter());
    }

    public void flipV(Vector2D center)
    {
        form.flipV(center);
        position.set(form.getCenter());
    }

    public void scale(float factor, Vector2D center)
    {
        if (factor != 0)
            form.scale(factor, center);
        else
            form.scale(0.0001f, center);
        position.set(form.getCenter());
    }

    public void rotateRadians(float radians, Vector2D center)
    {
        form.rotateRadians(radians, center);
        position.set(form.getCenter());
    }

    public void setPositionX(float x)
    {
        translateX(x - this.getX());
        position.set(form.getCenter());
    }

    public void setPositionY(float y)
    {
        translateY(y - this.getY());
        position.set(form.getCenter());
    }

    // getter/setter
    public Vector2D getLastAcceleration()
    {
        return lastAcceleration;
    }

    public void setLastAcceleration(Vector2D lastAcceleration)
    {
        this.lastAcceleration = lastAcceleration;
    }

    public float getScale()
    {
        return form.getScale();
    }

    public boolean isAwake()
    {
        return isAwake;
    }

    public void setAwake(boolean isAwake)
    {
        this.isAwake = isAwake;
    }

    public float getX()
    {
        return position.x;
    }

    public float getY()
    {
        return position.y;
    }

    public Vector2D getPosition()
    {
        return position;
    }

    public void setPosition(Vector2D position)
    {
        this.position.set(position);
        if (form != null)
            form.setPos(position);
    }

    public Vector2D getVelocity()
    {
        return velocity;
    }

    public void setVelocity(Vector2D velocity)
    {
        this.velocity.set(velocity);
    }

    public Vector2D getAcceleration()
    {
        return acceleration;
    }

    public void setAcceleration(Vector2D acceleration)
    {
        this.acceleration.set(acceleration);
    }

    public float getInverseMass()
    {
        return inverseMass;
    }

    public float getMass()
    {
        return 1.0f / inverseMass;
    }

    public void setMass(float mass)
    {
        this.inverseMass = 1.0f / mass;
    }

    public void setInverseMass(float inverseMass)
    {
        this.inverseMass = inverseMass;
    }

    public Form getForm()
    {
        return form;
    }

    public void setForm(Form form)
    {
        this.form = form;
    }
}