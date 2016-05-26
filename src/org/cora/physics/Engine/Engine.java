package org.cora.physics.Engine;

import java.util.HashSet;
import java.util.Set;

import org.cora.physics.collision.ContactEngine;
import org.cora.physics.entities.Particle;
import org.cora.physics.force.ForceGenerator;
import org.cora.physics.force.ForceRegistry;

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

    public void addElement(Particle p)
    {
        if (!elements.contains(p))
        {
            elements.add(p);
            contactEngine.add(p);
        }
    }

    public void addElementNoContact(Particle p)
    {
        if (!elements.contains(p))
        {
            elements.add(p);
        }
    }

    public void removeElement(Particle p)
    {
        elements.remove(p);
        contactEngine.remove(p);
        forceRegistry.removeAll(p);
    }

    public void addForce(Particle p, ForceGenerator force)
    {
        forceRegistry.add(p, force);
    }

    public void remove(Particle p, ForceGenerator force) throws Exception
    {
        forceRegistry.remove(p, force);
    }

    public void update(float dt)
    {
        forceRegistry.update(dt);
        for (Particle p : elements)
        {
            p.integrate(dt);
        }
        contactEngine.update(dt);
    }

    public Set<Particle> getAllElements()
    {
        return new HashSet<Particle>(elements);
    }
}
