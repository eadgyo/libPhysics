package org.cora.physics.Engine;


import org.cora.graphics.graphics.Graphics;
import org.cora.maths.Vector2D;
import org.cora.maths.sRectangle;
import org.cora.physics.entities.Particle;

import java.util.ArrayList;

public class QuadTree
{
    private final static int MAX_OBJECTS = 5;
    private final static int MAX_LEVELS = 50;

    private int level;
    private ArrayList<Particle> particles;
    private sRectangle rect;
    private QuadTree[] nodes;

    public QuadTree()
    {
        level = 0;
        rect = null;

        this.particles = new ArrayList<Particle>();
        nodes = new QuadTree[4];
    }

    public QuadTree(int level, sRectangle rect)
    {
        this.level = level;
        this.rect = rect;

        this.particles = new ArrayList<Particle>();
        nodes = new QuadTree[4];
    }

    public void init(sRectangle rect)
    {
        level = 0;
        this.rect = rect;
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

                int i = 0;
                while (i < particles.size())
                {
                    int index = getIndex(particles.get(i).getSavedSRectangleBound());
                    if (index != -1)
                        nodes[index].insert(particles.remove(i));
                    else
                        i++;
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

    public void retrieve(Particle particle, ArrayList<Particle> resParticles)
    {
        retrieve(particle.getSavedSRectangleBound(), resParticles);
    }

    public boolean isValid(sRectangle rec)
    {
        return rec.getX(0) > rect.getX(0) && rec.getX(2) < rect.getX(2)
                && rec.getY(0) > rect.getY(0) && rec.getY(2) < rect.getY(2);
    }

    public void retrieve(sRectangle rect, ArrayList<Particle> resParticles)
    {
        int index = getIndex(rect);
        if (index != -1 && nodes[0] != null)
            nodes[index].retrieve(rect, resParticles);
        else
            addEntities(resParticles);
        resParticles.addAll(this.particles);
    }

    public void addEntities(ArrayList<Particle> resParticles)
    {
        resParticles.addAll(particles);
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

    public void draw(Graphics g)
    {
        g.drawForm(rect);

        if (nodes[0] != null)
        {
            for (int i = 0; i < nodes.length; i++)
            {
                nodes[i].draw(g);
            }
        }
    }
}
