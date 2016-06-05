package org.cora.physics.Engine;

import org.cora.maths.Form;
import org.cora.maths.sRectangle;
import org.cora.physics.collision.Contact;
import org.cora.physics.collision.ContactEngine;
import org.cora.physics.collision.ContactGenerator;
import org.cora.physics.entities.Particle;
import org.cora.physics.force.ForceGenerator;
import org.cora.physics.force.ForceRegistry;

import java.util.ArrayList;
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
    private float minDt = 0.02f;

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
        if (dt > minDt)
            dt = minDt;

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

    /**
     * Get quadtree used for collision detection
     * @return quadtree
     */
    public final QuadTree getQuadtree()
    {
        return contactEngine.getQuadTree();
    }

    /**
     * Change minDt engine update
     * If minDt is too high, slow pc will have strange collisions responses
     * If minDt is too low, physics reactions will be slow down
     * @param minDt value
     */
    public void setMinDt(float minDt) { this.minDt = minDt; }

    /**
     * Get minDt engine update
     * @return minDt value
     */
    public float getMinDt() { return minDt; }

    /**
     * Get elements colliding form QT
     * @param A element
     * @return collidings element int set
     */
    public Set<Particle> getCollisionsQTSet(Particle A)
    {
        return contactEngine.getCollisionsQTSet(A);
    }

    /**
     * Get elements colliding form QT
     * @param A element
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollisionsQTList(Particle A)
    {
        return contactEngine.getCollisionsQTList(A);
    }

    /**
     * Get elements colliding form QT
     * @param rec box
     * @return collidings element int set
     */
    public Set<Particle> getCollisionsQTSet(sRectangle rec)
    {
        return contactEngine.getCollisionsQTSet(rec);
    }

    /**
     * Get elements colliding form QT
     * @param rec box
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollisionsQTList(sRectangle rec)
    {
        return contactEngine.getCollisionsQTList(rec);
    }

    /**
     * Know if two elements were colliding during last contact resolution
     * @param A first element
     * @param B second element
     * @return collision result
     */
    public boolean wasColliding(Particle A, Particle B)
    {
        return contactEngine.wasColliding(A, B);
    }

    /**
     * Get the rectangle of quadtree
     * All elements are inside this rect
     * @return bound
     */
    public sRectangle getQTBound()
    {
        return contactEngine.getQTBound();
    }

    /**
     * Get min x of quatree's rect
     * @return min x
     */
    public float getMinXQT()
    {
        return contactEngine.getMinXQT();
    }

    /**
     * Get max x of quadtree's rect
     * @return max x
     */
    public float getMaxXQT()
    {
        return contactEngine.getMaxXQT();
    }

    /**
     * Get min y of quadtree's rect
     * @return min y
     */
    public float getMinYQT()
    {
        return contactEngine.getMinYQT();
    }

    /**
     * Get max y of quadtree's rect
     * @return max y
     */
    public float getMaxYQT()
    {
        return contactEngine.getMaxYQT();
    }

    /**
     * Get colliding elements
     * @param f form
     * @return collidings element int set
     */
    public Set<Particle> getCollidingsSet(Form f)
    {
        return contactEngine.getCollidingsSet(f);
    }

    /**
     * Get colliding elements
     * @param f form
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollidingsList(Form f)
    {
        return contactEngine.getCollidingsList(f);
    }

    /**
     * Get colliding elements
     * @param A element
     * @return collidings element int set
     */
    public Set<Particle> getCollidingsSet(Particle A)
    {
        return contactEngine.getCollidingsSet(A);
    }

    /**
     * Get colliding elements
     * @param A element
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollidingsList(Particle A)
    {
        return contactEngine.getCollidingsList(A);
    }
}
