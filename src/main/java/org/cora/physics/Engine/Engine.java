package org.cora.physics.Engine;

import java.util.HashSet;
import java.util.Set;

import org.cora.physics.collision.ContactEngine;
import org.cora.physics.entities.Particle;
import org.cora.physics.force.ForceGenerator;
import org.cora.physics.force.ForceRegistry;

/**
 * Engine that handle objects physics
 * Force update
 * Contact resolution
 */
public class Engine
{
    private Set<Particle> elements;
    private ForceRegistry forceRegistry;
    private ContactEngine contactEngine;

    public Engine()
    {
        elements = new HashSet<Particle>();
        forceRegistry = new ForceRegistry();
        contactEngine = new ContactEngine();
    }

    /**
     * Add an element to the engine
     * @param p element added to the engine
     */
    public void addElement(Particle p)
    {
        if (!elements.contains(p))
        {
            elements.add(p);
            contactEngine.add(p);
        }
    }

    /**
     * Add an element to the engine with no contact resolution
     * @param p element added to the engine
     */
    public void addElementNoContact(Particle p)
    {
        if (!elements.contains(p))
        {
            elements.add(p);
        }
    }

    /**
     * Remove an element from this engine
     * @param p element to be removed
     */
    public void removeElement(Particle p)
    {
        elements.remove(p);
        contactEngine.remove(p);
        forceRegistry.removeAll(p);
    }

    /**
     * Add a force to an existing element in the engine
     * The element will be updated automatically with engine.update
     * @param p element to apply force
     * @param force force to apply
     */
    public void addForce(Particle p, ForceGenerator force)
    {
        forceRegistry.add(p, force);
    }

    /**
     * Remove force linked to an element
     * @param p linked element
     * @param force force to remove
     */
    public void removeForce(Particle p, ForceGenerator force)
    {
        forceRegistry.remove(p, force);
    }

    /**
     * Remove all forces on one element
     * @param p linked element
     */
    public void removeAllForcesOn(Particle p)
    {
        forceRegistry.removeAll(p);
    }

    /**
     * Update the engine
     * 2 steps
     * 1) force
     * 2) contacts resolution
     * @param dt time between since last update
     */
    public void update(float dt)
    {
        forceRegistry.update(dt);
        for (Particle p : elements)
        {
            p.integrate(dt);
        }
        contactEngine.update(dt);
    }

    /**
     *
     * @return all element stored
     */
    public Set<Particle> getAllElements()
    {
        return new HashSet<Particle>(elements);
    }
}
