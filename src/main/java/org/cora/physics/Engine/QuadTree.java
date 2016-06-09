package org.cora.physics.Engine;


import org.cora.maths.Vector2D;
import org.cora.maths.sRectangle;
import org.cora.physics.entities.Particle;

import java.util.*;

public class QuadTree implements Cloneable
{
    private final static int MAX_OBJECTS = 5;
    private final static int MAX_LEVELS = 50;

    private int level;
    private Set<Particle> particles;
    private sRectangle rect;
    private QuadTree[] nodes;

    public QuadTree()
    {
        level = 0;
        rect = null;

        particles = new HashSet<Particle>();
        rect = new sRectangle();
        nodes = new QuadTree[4];
    }

    public QuadTree(int level, sRectangle rect)
    {
        this.level = level;
        this.rect = rect;

        this.particles = new HashSet<Particle>();
        nodes = new QuadTree[4];
    }

    @Override
    public Object clone()
    {
        QuadTree q = null;

        try
        {
            q = (QuadTree) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            e.printStackTrace();
        }

        q.particles = new HashSet<Particle>(particles);
        q.rect = (sRectangle) rect.clone();
        q.nodes = new QuadTree[4];

        for (int i = 0; i < nodes.length; i++)
        {
            q.nodes[i] = (QuadTree) nodes[i].clone();
        }

        return q;
    }

    /**
     * Copy an object and change each particle with a given one
     * @param change map
     * @return saved quadTree
     */
    public Object clone(Map<Particle, Particle> change)
    {
        QuadTree q = null;

        try
        {
            q = (QuadTree) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            e.printStackTrace();
        }

        q.particles = new HashSet<Particle>();
        for (Iterator<Particle> iterator = particles.iterator(); iterator.hasNext(); )
        {
            Particle next =  iterator.next();
            q.particles.add(change.get(next));
        }

        q.rect = (sRectangle) rect.clone();
        q.nodes = new QuadTree[4];

        if (nodes[0] != null)
        {
            for (int i = 0; i < nodes.length; i++)
            {
                q.nodes[i] = (QuadTree) nodes[i].clone(change);
            }
        }

        return q;
    }

    /**
     * Change particle with given one
     * @param change map of particles
     */
    public void changeParticles(Map<Particle, Particle> change)
    {
        Set<Particle> newOne = new HashSet<Particle>();
        for (Iterator<Particle> iterator = particles.iterator(); iterator.hasNext(); )
        {
            Particle next =  iterator.next();
            newOne.add(change.get(next));
        }

        particles = newOne;
        if (nodes[0] != null)
        {
            for (int i = 0; i < nodes.length; i++)
            {
                nodes[i].changeParticles(change);
            }
        }
    }

    public void init(sRectangle rect)
    {
        level = 0;
        this.rect.set(rect);
    }

    public void init(float x, float y, float width, float height)
    {
        level = 0;
        this.rect.set(x, y, width, height);
    }

    public void setRect(sRectangle rect)
    {
        this.rect = rect;
    }

    public void clear()
    {
        particles.clear();
        if (nodes[0] != null)
        {
            for (int i = 0; i < nodes.length; i++)
            {
                nodes[i].clear();
                nodes[i] = null;
            }
        }
    }

    public void split()
    {
        int subWidth = (int) (rect.getWidth() / 2);
        int subHeight = (int) (rect.getHeight() / 2);
        int x = (int) rect.getLeftX();
        int y = (int) rect.getLeftY();
        nodes[0] = new QuadTree(level + 1, new sRectangle(x + subWidth, y, subWidth, subHeight));
        nodes[1] = new QuadTree(level + 1, new sRectangle(x, y, subWidth, subHeight));
        nodes[2] = new QuadTree(level + 1, new sRectangle(x, y + subHeight, subWidth, subHeight));
        nodes[3] = new QuadTree(level + 1, new sRectangle(x + subWidth, y + subHeight, subWidth, subHeight));
    }

    public int getIndex(sRectangle rect)
    {
        int index = -1;
        Vector2D l_rectCenter = new Vector2D((float) (this.rect.getCenterX()), (float) (this.rect.getCenterY()));
        if (rect.getX(2) < l_rectCenter.x)
        {
            if (rect.getY(2) < l_rectCenter.y)
                index = 1;
            else if (rect.getY(0) > l_rectCenter.y)
                index = 2;
        }
        else if (rect.getX(0) > l_rectCenter.x)
        {
            if (rect.getY(2) < l_rectCenter.y)
                index = 0;
            else if (rect.getY(0) > l_rectCenter.y)
                index = 3;
        }
        return index;
    }

    public void insert(Particle particle)
    {
        if (nodes[0] != null)
        {
            int index = getIndex(particle.getSavedSRectangleBound());
            if (index != -1) // Si le rectangle rentre dans l'une des quatres cases
            {
                nodes[index].insert(particle);
                return;
            }
        }

        if (particles.size() + 1 > MAX_OBJECTS && level + 1 < MAX_LEVELS)
        {
            if (nodes[0] == null)
            {
                split();
                Iterator<Particle> it = particles.iterator();
                while (it.hasNext())
                {
                    Particle p = it.next();
                    int index = getIndex(p.getSavedSRectangleBound());
                    if (index != -1)
                    {
                        it.remove();
                        nodes[index].insert(p);
                    }
                }
            }
            int index = getIndex(particle.getSavedSRectangleBound());
            if (index != -1)
                nodes[index].insert(particle);
            else
                particles.add(particle);
        }
        else
            particles.add(particle);
    }

    public void inserts(ArrayList<Particle> particles)
    {
        for (int i = 0; i < particles.size(); i++)
        {
            this.insert(particles.get(i));
        }
    }

    public void retrieve(Particle particle, Collection<Particle> resParticles)
    {
        retrieve(particle.getSavedSRectangleBound(), resParticles);
    }

    public boolean isValid(sRectangle rec)
    {
        return rec.getX(0) > rect.getX(0) && rec.getX(2) < rect.getX(2)
                && rec.getY(0) > rect.getY(0) && rec.getY(2) < rect.getY(2);
    }

    public void retrieve(sRectangle rect, Collection<Particle> resParticles)
    {
        int index = getIndex(rect);
        if (index != -1 && nodes[0] != null)
            nodes[index].retrieve(rect, resParticles);
        else
            addEntitiesChild(resParticles);
        resParticles.addAll(this.particles);
    }

    public void addEntities(Collection<Particle> resParticles)
    {
        resParticles.addAll(particles);
        if (nodes[0] == null)
            return;
        for (int i = 0; i < 4; i++)
        {
            nodes[i].addEntities(resParticles);
        }
    }

    public void addEntitiesChild(Collection<Particle> resParticles)
    {
        if (nodes[0] == null)
            return;
        for (int i = 0; i < 4; i++)
        {
            nodes[i].addEntities(resParticles);
        }
    }

    public final QuadTree[] getNodes()
    {
        return nodes;
    }

    public final sRectangle getRect()
    {
        return rect;
    }

    /**
     * Get min x of rect
     * @return min x
     */
    public float getMinX()
    {
        return rect.getMinX();
    }

    /**
     * Get max x of rect
     * @return max x
     */
    public float getMaxX()
    {
        return rect.getMaxX();
    }

    /**
     * Get min y of rect
     * @return min y
     */
    public float getMinY()
    {
        return rect.getMinY();
    }

    /**
     * Get max y of rect
     * @return max y
     */
    public float getMaxY()
    {
        return rect.getMaxY();
    }
}
