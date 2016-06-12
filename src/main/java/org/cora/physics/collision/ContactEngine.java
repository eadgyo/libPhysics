package org.cora.physics.collision;

import org.cora.maths.Circle;
import org.cora.maths.Form;
import org.cora.maths.collision.CollisionDetectorNoT;
import org.cora.maths.sRectangle;
import org.cora.physics.Engine.QuadTree;
import org.cora.physics.entities.Particle;
import org.cora.physics.entities.RigidBody;

import java.util.*;

/**
 * Engine that handle and resolve objects collisions
 */
public class ContactEngine implements Cloneable
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

    @Override
    public Object clone()
    {
        ContactEngine ce = null;

        try
        {
            ce = (ContactEngine) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            return ce;
        }

        ce.elements = new ArrayList<>(elements);
        ce.contacts = new ArrayList<>();

        for (int i = 0; i < contacts.size(); i++)
        {
            Contact contact =  contacts.get(i);
            ce.contacts.add((Contact) contact.clone());
        }

        ce.quadTree = (QuadTree) quadTree.clone();
        ce.savedCollisions = new HashMap<Particle, Set<Particle>>();

        savedCollisions.entrySet();
        for (Iterator<Map.Entry<Particle, Set<Particle>>> iterator = savedCollisions.entrySet().iterator(); iterator.hasNext(); )
        {
            Map.Entry<Particle, Set<Particle>> next =  iterator.next();
            Particle p = next.getKey();
            Set<Particle> ps = next.getValue();
            ce.savedCollisions.put(p, new HashSet<>(ps));
        }
        return ce;
    }

    public Object clone(Map<Particle, Particle> change)
    {
        ContactEngine ce = null;

        try
        {
            ce = (ContactEngine) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            return ce;
        }

        ce.elements = new ArrayList<>();
        for (int i = 0; i < elements.size(); i++)
        {
            Particle particle =  elements.get(i);
            ce.elements.add(change.get(particle));
        }

        ce.contacts = new ArrayList<>();

        for (int i = 0; i < contacts.size(); i++)
        {
            Contact contact =  contacts.get(i);
            ce.contacts.add((Contact) contact.clone(change));
        }

        ce.quadTree = (QuadTree) quadTree.clone(change);
        ce.savedCollisions = new HashMap<Particle, Set<Particle>>();

        savedCollisions.entrySet();
        for (Iterator<Map.Entry<Particle, Set<Particle>>> iterator = savedCollisions.entrySet().iterator(); iterator.hasNext(); )
        {
            Map.Entry<Particle, Set<Particle>> next =  iterator.next();
            Particle p = next.getKey();
            Set<Particle> ps = next.getValue();

            Set<Particle> psC = new HashSet<>();
            for (Iterator<Particle> particleIterator = ps.iterator(); particleIterator.hasNext(); )
            {
                Particle particle =  particleIterator.next();
                psC.add(change.get(particle));
            }
            ce.savedCollisions.put(change.get(p), psC);
        }
        return ce;
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

    public void initQT(int x, int y, int width)
    {
        quadTree.init(x, y, width, width);
    }

    public void findAndResolveContacts(float dt)
    {
        quadTree.clear();
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
     * Get elements may colliding form QT
     * @param A element
     * @return collidings element int set
     */
    public Set<Particle> getCollisionsQTSet(Particle A)
    {
        Set<Particle> mayCollidings = new HashSet<Particle>();
        quadTree.retrieve(A, mayCollidings);
        return mayCollidings;
    }

    /**
     * Get elements may colliding form QT
     * @param A element
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollisionsQTList(Particle A)
    {
        ArrayList<Particle> mayCollidings = new ArrayList<Particle>();
        quadTree.retrieve(A, mayCollidings);
        return mayCollidings;
    }

    /**
     * Get elements may colliding form QT
     * @param rec box
     * @return collidings element int set
     */
    public Set<Particle> getCollisionsQTSet(sRectangle rec)
    {
        Set<Particle> mayCollidings = new HashSet<Particle>();
        quadTree.retrieve(rec, mayCollidings);
        return mayCollidings;
    }

    /**
     * Get elements may colliding form QT
     * @param rec box
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollisionsQTList(sRectangle rec)
    {
        ArrayList<Particle> mayCollidings = new ArrayList<Particle>();
        quadTree.retrieve(rec, mayCollidings);
        return mayCollidings;
    }

    /**
     * Get colliding elements
     * @param f form
     * @return collidings element int set
     */
    public Set<Particle> getCollidingsSet(Form f)
    {
        ArrayList<Particle> mayCollidings = getCollisionsQTList(f.getSRectangleBound());
        Set<Particle> collidings = new HashSet<Particle>();

        Circle cF = f.getCircleBound();

        for (int i = 0; i < mayCollidings.size(); i++)
        {
            if (CollisionDetectorNoT.isCollidingOptimised(f, mayCollidings.get(i).getForm(), f.getCircleBound(), cF))
                collidings.add(mayCollidings.get(i));
        }

        return collidings;
    }

    /**
     * Get colliding elements
     * @param f form
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollidingsList(Form f)
    {
        ArrayList<Particle> mayCollidings = getCollisionsQTList(f.getSRectangleBound());
        ArrayList<Particle> collidings = new ArrayList<Particle>();

        Circle cF = f.getCircleBound();

        for (int i = 0; i < mayCollidings.size(); i++)
        {
            if (CollisionDetectorNoT.isCollidingOptimised(f, mayCollidings.get(i).getForm(), f.getCircleBound(), cF))
            collidings.add(mayCollidings.get(i));
        }

        return collidings;
    }

    /**
     * Get colliding elements
     * @param A element
     * @return collidings element int set
     */
    public Set<Particle> getCollidingsSet(Particle A)
    {
        ArrayList<Particle> mayCollidings = getCollisionsQTList(A.getSavedSRectangleBound());
        Set<Particle> collidings = new HashSet<>();

        for (int i = 0; i < mayCollidings.size(); i++)
        {
            if (CollisionDetectorNoT.isCollidingOptimised(A.getForm(), mayCollidings.get(i).getForm(), A.getSavedCircleBound(), mayCollidings.get(i).getSavedCircleBound()))
            collidings.add(mayCollidings.get(i));
        }

        return collidings;
    }

    /**
     * Get colliding elements
     * @param A element
     * @return collidings element int List
     */
    public ArrayList<Particle> getCollidingsList(Particle A)
    {
        ArrayList<Particle> mayCollidings = getCollisionsQTList(A.getSavedSRectangleBound());
        ArrayList<Particle> collidings = new ArrayList<Particle>();

        for (int i = 0; i < mayCollidings.size(); i++)
        {
            if (CollisionDetectorNoT.isCollidingOptimised(A.getForm(), mayCollidings.get(i).getForm(), A.getSavedCircleBound(), mayCollidings.get(i).getSavedCircleBound()))
            collidings.add(mayCollidings.get(i));
        }

        return collidings;
    }


    /**
     * Test if an element is colliding
     * @param f form
     * @return collidings element int List
     */
    public boolean isColliding(Form f)
    {
        ArrayList<Particle> mayCollidings = getCollisionsQTList(f.getSRectangleBound());
        ArrayList<Particle> collidings = new ArrayList<Particle>();

        Circle cF = f.getCircleBound();

        for (int i = 0; i < mayCollidings.size(); i++)
        {
            if (CollisionDetectorNoT.isCollidingOptimised(f, mayCollidings.get(i).getForm(), f.getCircleBound(), cF))
                return true;
        }

        return false;
    }

    /**
     * Test if an element is colliding
     * @param A element
     * @return collidings element int List
     */
    public boolean isColliding(Particle A)
    {
        ArrayList<Particle> mayCollidings = getCollisionsQTList(A.getSavedSRectangleBound());
        ArrayList<Particle> collidings = new ArrayList<Particle>();

        for (int i = 0; i < mayCollidings.size(); i++)
        {
            if (CollisionDetectorNoT.isCollidingOptimised(A.getForm(), mayCollidings.get(i).getForm(), A.getSavedCircleBound(), mayCollidings.get(i).getSavedCircleBound()))
                return true;
        }

        return false;
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

    /**
     * Know if an element was colliding during last contact resolution
     * @param A element
     * @return collision result
     */
    public boolean wasColliding(Particle A)
    {
        Set<Particle> ps = savedCollisions.get(A);
        return ps != null && ps.size() != 0;
    }

    /**
     * Get all elements that were in collision with A during last contact resolution
     * @param A element
     * @return all elements that were in collision or null if no collision
     */
    public Set<Particle> getWereCollidingWith(Particle A)
    {
        return savedCollisions.get(A);
    }

    /**
     * Get the rectangle of quadtree
     * All elements are inside this rect
     * @return bound
     */
    public sRectangle getQTBound()
    {
        return quadTree.getRect();
    }


    /**
     * Get min x of quatree's rect
     * @return min x
     */
    public float getMinXQT()
    {
        return quadTree.getMinX();
    }

    /**
     * Get max x of quadtree's rect
     * @return max x
     */
    public float getMaxXQT()
    {
        return quadTree.getMaxX();
    }

    /**
     * Get min y of quadtree's rect
     * @return min y
     */
    public float getMinYQT()
    {
        return quadTree.getMinY();
    }

    /**
     * Get max y of quadtree's rect
     * @return max y
     */
    public float getMaxYQT()
    {
        return quadTree.getMaxY();
    }
}
