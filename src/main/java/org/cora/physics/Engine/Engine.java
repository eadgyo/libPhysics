package org.cora.physics.Engine;

import org.cora.physics.collision.Contact;
import org.cora.physics.collision.ContactEngine;
import org.cora.physics.collision.ContactGenerator;
import org.cora.physics.entities.Particle;
import org.cora.physics.force.ForceGenerator;
import org.cora.physics.force.ForceRegistry;

import java.util.HashSet;
import java.util.Set;

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
     * 1) apply force
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

    /**
     * Set threshold value to accept side collision
     * Low threshold will make resting contacts unstable
     * High thresold will make strange results
     * @param threshold value used in contacts resolution
     */
    public void setThresholdSideDetection(float threshold) { ContactGenerator.THRESHOLD_SIDE_DETECTION = threshold; }

    /**
     * Default paramater for non material type collision
     * @param defaultFriction between 0 and 1.0
     */
    public void setDefaultFriction(float defaultFriction)
    {
        Contact.DEFAULT_FRICTION = defaultFriction;
    }

    /**
     * Default paramater for non material type collision
     * @param defaultRestitution between 0 and 1.0
     */
    public void setDefaultRestitution(float defaultRestitution)
    {
        Contact.DEFAULT_RESTITUTION = defaultRestitution;
    }

    /**
     * Default paramater for non material type collision
     * @param defaultSeparation between 0 and 1.0
     */
    public void setDefaultSeparation(float defaultSeparation)
    {
        Contact.DEFAULT_SEP = defaultSeparation;
    }

    /**
     * Correction take into account last acceleration
     * @param correction active or desactive feature
     */
    public void setRestitutionCorrection(boolean correction)
    {
        Contact.ACTIVE_RESTITUTION_CORRECTION = correction;
    }

    /**
     * Set threshold value to accept side collision
     * Low threshold will make resting contacts unstable
     * High thresold will make strange results
     * @return threshold value used in contacts resolution
     */
    public float getThresholdSideDetection() { return ContactGenerator.THRESHOLD_SIDE_DETECTION; }

    /**
     * Default paramater for non material type collision
     * @return defaultFriction between 0 and 1.0
     */
    public float getDefaultFriction()
    {
        return Contact.DEFAULT_FRICTION;
    }

    /**
     * Default paramater for non material type collision
     * @return defaultRestitution between 0 and 1.0
     */
    public float setDefaultRestitution()
    {
        return Contact.DEFAULT_RESTITUTION;
    }

    /**
     * Default paramater for non material type collision
     * @return defaultSeparation between 0 and 1.0
     */
    public float setDefaultSeparation()
    {
        return Contact.DEFAULT_SEP;
    }

    /**
     * Correction take into account last acceleration
     * @return correction active or desactive feature
     */
    public boolean setRestitutionCorrection()
    {
        return Contact.ACTIVE_RESTITUTION_CORRECTION;
    }
}
