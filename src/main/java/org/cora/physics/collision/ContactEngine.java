package org.cora.physics.collision;

import java.util.ArrayList;

import org.cora.physics.entities.Particle;

/**
 * Engine that handle and resolve objects collisions
 */
public class ContactEngine
{
    private ArrayList<Particle> elements;
    private ArrayList<Contact>  contacts;

    public ContactEngine()
    {
        elements = new ArrayList<Particle>();
        contacts = new ArrayList<Contact>();
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

    public void update(float dt)
    {
        findContacts(dt);
        ContactResolver.resolveContacts(contacts, dt);
    }
}
