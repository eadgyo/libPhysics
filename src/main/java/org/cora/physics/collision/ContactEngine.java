package org.cora.physics.collision;

import org.cora.physics.entities.Particle;
import org.cora.physics.entities.RigidBody;

import java.util.ArrayList;

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

    public void findAndResolveContacts(float dt)
    {
        contacts.clear();
        Particle A, B;
        boolean isCollision = true;

        int test = 0;

        for (int i = 0; i < elements.size(); i++)
        {
            elements.get(i).contact.clear();
        }

        while (isCollision && test < elements.size() * 2)
        {
            isCollision = false;
            for (int i = 0; i < elements.size() - 1; i++)
            {
                A = elements.get(i);
                for (int j = i + 1; j < elements.size(); j++)
                {
                    B = elements.get(j);
                    ContactGenerator.generateContacts(A, B, contacts, dt);
                    for (int w = 0; w < contacts.size(); w++)
                    {
                        contacts.get(w).resolve(dt);
                        /*
                        if (!A.contact.contains(B))
                            contacts.get(w).resolveVelocity(dt);
                        */
                        A.contact.add(B);
                        B.contact.add(A);
                        isCollision = true;
                    }
                    contacts.clear();
                }
            }
            test++;
        }
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
}
