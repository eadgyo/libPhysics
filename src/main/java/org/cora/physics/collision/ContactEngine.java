package org.cora.physics.collision;

import org.cora.maths.Circle;
import org.cora.maths.sRectangle;
import org.cora.physics.Engine.QuadTree;
import org.cora.physics.entities.Particle;
import org.cora.physics.entities.RigidBody;

import java.util.*;

/**
 * Engine that handle and resolve objects collisions
 */
public class ContactEngine
{
    private ArrayList<Particle> elements;
    private ArrayList<Contact>  contacts;
    private QuadTree            quadTree;
    private boolean             saveCollision;
    private Map<Particle, Set<Particle>> savedCollisions;

    public ContactEngine()
    {
        elements = new ArrayList<Particle>();
        contacts = new ArrayList<Contact>();
        savedCollisions = new HashMap<Particle, Set<Particle>>();
        quadTree = new QuadTree();
        saveCollision = true;
    }

    public void add(Particle p)
    {
        elements.add(p);
    }

    public void remove(Particle p)
    {
        elements.remove(p);
    }

    public void findContacts(float dt)
    {
        contacts.clear();
        Particle A, B;
        for (int i = 0; i < elements.size() - 1; i++)
        {
            A = elements.get(i);
            for (int j = i + 1; j < elements.size(); j++)
            {
                B = elements.get(j);
                ContactGenerator.generateContacts(A, B, contacts, dt);
            }
        }
    }

    public void findAndResolveContacts(float dt)
    {
        float minX = Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;
        float maxX = - Float.MAX_VALUE;
        float maxY = - Float.MAX_VALUE;

        for (int i = 0; i < elements.size(); i++)
        {
            elements.get(i).computeStoredBounds();
            Circle circle = elements.get(i).getSavedCircleBound();

            if (minX > circle.getMinX())
            {
                minX = circle.getMinX();
            }

            if (maxX < circle.getMaxX())
            {
                maxX = circle.getMaxX();
            }

            if (minY > circle.getMinY())
            {
                minY = circle.getMinY();
            }

            if (maxY < circle.getMaxY())
            {
                maxY = circle.getMaxY();
            }
        }

        quadTree.clear();
        quadTree.init(new sRectangle(minX, minY, maxX - minX, maxY - minY));
        quadTree.inserts(elements);
        contacts.clear();
        savedCollisions.clear();

        boolean isCollision = true;
        ArrayList<Particle> collidings = new ArrayList<Particle>();
        Particle A, B;

        int test = 0;
        while (isCollision && test < elements.size() * 2)
        {
            isCollision = false;
            for (int i = 0; i < elements.size(); i++)
            {
                A = elements.get(i);
                quadTree.retrieve(A, collidings);

                for (int j = 0; j < collidings.size(); j++)
                {
                    B = collidings.get(j);
                    if (B == A)
                        continue;

                    if (ContactGenerator.generateContacts(A, B, contacts, dt))
                    {
                        addSavedCollision(A, B);
                        for (int w = 0; w < contacts.size(); w++)
                        {
                            contacts.get(w).resolve(dt);
                            isCollision = true;
                        }
                    }
                    contacts.clear();
                }
                collidings.clear();
            }
            test++;
        }
    }

    private void addSavedCollision(Particle A, Particle B)
    {
        if (saveCollision)
        {
            addSavedCollisionFree(A, B);
            addSavedCollisionFree(B, A);
        }
    }

    private void addSavedCollisionFree(Particle A, Particle B)
    {
        Set<Particle> ps = savedCollisions.get(A);
        if (ps == null)
        {
            ps = new HashSet<Particle>();
            savedCollisions.put(A, ps);
        }
        ps.add(B);
    }

    public void update(float dt)
    {
        //resolveAllContactsDebug(dt);
        //findContacts(dt);
        //ContactResolver.resolveContacts(contacts, dt);
        findAndResolveContacts(dt);
    }

    public void resolveAllContactsDebug(float dt)
    {
        Particle A, B;
        for (int i = 0; i < elements.size() - 1; i++)
        {
            A = elements.get(i);
            for (int j = i + 1; j < elements.size(); j++)
            {
                RigidBody rA = (RigidBody) A;
                RigidBody rB = (RigidBody) elements.get(j);

                ContactTest.testCollision(rA, rB, dt);
            }
        }
    }

    /**
     * Get quadtree used for collision detection
     * @return quadtree
     */
    public final QuadTree getQuadTree()
    {
        return quadTree;
    }

    /**
     * Get elements colliding form QT
     * @param A element
     * @return collidings element int set
     */
    public Set<Particle> getCollisionsQTSet(Particle A)
    {
        Set<Particle> collidings = new HashSet<Particle>();
        quadTree.retrieve(A, collidings);
        return collidings;
    }

    /**
     * Get elements colliding form QT
     * @param A element
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollisionsQTList(Particle A)
    {
        ArrayList<Particle> collidings = new ArrayList<Particle>();
        quadTree.retrieve(A, collidings);
        return collidings;
    }

    /**
     * Get elements colliding form QT
     * @param rec box
     * @return collidings element int set
     */
    public Set<Particle> getCollisionsQTSet(sRectangle rec)
    {
        Set<Particle> collidings = new HashSet<Particle>();
        quadTree.retrieve(rec, collidings);
        return collidings;
    }

    /**
     * Get elements colliding form QT
     * @param rec box
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollisionsQTList(sRectangle rec)
    {
        ArrayList<Particle> collidings = new ArrayList<Particle>();
        quadTree.retrieve(rec, collidings);
        return collidings;
    }

    /**
     * Know if two elements were colliding during last contact resolution
     * @param A first element
     * @param B second element
     * @return collision result
     */
    public boolean wasColliding(Particle A, Particle B)
    {
        Set<Particle> ps = savedCollisions.get(A);
        return ps != null && ps.contains(B);
    }
}
