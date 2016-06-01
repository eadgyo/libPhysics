package org.cora.physics.collision;

import org.cora.maths.*;
import org.cora.maths.collision.CollisionDetector;
import org.cora.physics.entities.Particle;

import java.util.ArrayList;

public class ContactGenerator
{
    public static boolean generateContacts(Particle A, Particle B,
            ArrayList<Contact> contacts, float dt)
    {
        Vector2D push = new Vector2D();
        FloatA t = new FloatA(dt);

        if (isColliding(A, B, push, t))
        {
            // Find contacts
            ArrayList<Vector2D> contactsA = findContacts(A, push, t.v);
            ArrayList<Vector2D> contactsB = findContacts(B, push.multiply(-1),
                    t.v);

            if (!analyseContacts(A, B, contactsA, contactsB, contacts))
                return false;

            Vector2D n = push.multiply(-1);
            for (int i = 0; i < contacts.size(); i++)
            {
                contacts.get(i).setContactNormal(n);
                contacts.get(i).setPenetration(t.v);
            }

            ContactInformation information = A.getContactInformation(B);
            if (information != null)
            {
                for (int i = 0; i < contacts.size(); i++)
                {
                    contacts.get(i).set(information);
                }
            }
            return true;
        }
        return false;
    }

    public static boolean isColliding(Particle A, Particle B, Vector2D push,
            FloatA t)
    {
        return CollisionDetector.isColliding(A.getForm(), B.getForm(),
                A.getVelocity(), B.getVelocity(), push, t);
    }

    public static boolean analyseContacts(Particle A, Particle B,
            ArrayList<Vector2D> contactsA, ArrayList<Vector2D> contactsB,
            ArrayList<Contact> contacts)
    {
        if (contactsA.size() == 0 || contactsB.size() == 0)
            return false;
        if (contactsA.size() == 1 && contactsB.size() == 2)
        {
            Vector2D PA = contactsA.get(0);
            Vector2D PB1 = contactsB.get(0);
            Vector2D PB2 = contactsB.get(1);

            Vector2D projection = new Vector2D();
            if (!projectOnSegment(PA, PB1, PB2, projection))
                return false;

            Contact contact = new Contact(A, B);
            contact.setC(contactsA.get(0), projection);
            contacts.add(contact);

        }
        else if (contactsA.size() == 2 && contactsB.size() == 1)
        {
            Vector2D PB = contactsB.get(0);
            Vector2D PA1 = contactsA.get(0);
            Vector2D PA2 = contactsA.get(1);

            Vector2D projection = new Vector2D();
            if (!projectOnSegment(PB, PA1, PA2, projection))
                return false;

            Contact contact = new Contact(A, B);
            contact.setC(projection, contactsB.get(0));
            contacts.add(contact);

        }
        else if (contactsA.size() == 2 && contactsB.size() == 2)
        {
            Vector2D edgeA = new Vector2D(contactsA.get(0), contactsA.get(1));

            if (!handleEdgeToEdge(A, B, edgeA, contactsA, contactsB, contacts))
            {
                return false;
            }
        }
        return true;
    }

    public static boolean handleEdgeToEdge(Particle A, Particle B,
            Vector2D edge, ArrayList<Vector2D> contactsA,
            ArrayList<Vector2D> contactsB, ArrayList<Contact> contacts)
    {
        float min0 = 0;
        float max0 = edge.getSqMagnitude();

        if (min0 > max0)
        {
            float temp = max0;
            max0 = min0;
            min0 = temp;

            Vector2D tempA = contactsA.get(0);
            contactsA.set(0, contactsA.get(1));
            contactsA.set(1, tempA);
        }

        float min1 = (contactsB.get(0).sub(contactsA.get(0)))
                .scalarProduct(edge);
        float max1 = (contactsB.get(1).sub(contactsA.get(0)))
                .scalarProduct(edge);

        if (min1 > max1)
        {
            float temp = max1;
            max1 = min1;
            min1 = temp;

            Vector2D tempB = contactsB.get(0);
            contactsB.set(0, contactsB.get(1));
            contactsB.set(1, tempB);
        }

        if (min0 > max1 || min1 > max0)
            return false;

        // On est sûr qu'il y a une collision
        if (min0 > min1)
        {
            // Le point en collision est le point contactsA_0
            // Projection du point sur le coté opposé
            Vector2D projection = new Vector2D();
            if (projectOnSegment(contactsA.get(0), contactsB.get(0),
                    contactsB.get(1), projection))
            {
                // On ajoute le point projeter et sa projection
                Contact contact = new Contact(A, B);
                contact.setC(contactsA.get(0), projection);
                contacts.add(contact);
            }
        }
        else
        {
            // Le point en collision est le point contactsB_0
            // Projection du point sur le coté opposé
            Vector2D projection = new Vector2D();
            if (projectOnSegment(contactsB.get(0), contactsA.get(0),
                    contactsA.get(1), projection))
            {
                // On ajoute le point projeter et sa projection
                Contact contact = new Contact(A, B);
                contact.setC(projection, contactsB.get(0));
                contacts.add(contact);
            }
        }

        if (max0 != min0 && max1 != min1)
        {
            if (max0 < max1)
            {
                // Le point en collision est le point contactsA_1
                // Projection du point sur le coté opposé
                Vector2D projection = new Vector2D();
                if (projectOnSegment(contactsA.get(1), contactsB.get(0),
                        contactsB.get(1), projection))
                {
                    // On ajoute le point projeter et sa projection
                    Contact contact = new Contact(A, B);
                    contact.setC(contactsA.get(1), projection);
                    contacts.add(contact);
                }
            }
            else
            {
                // Le point en collision est le point contactsB_1
                // Projection du point sur le coté opposé
                Vector2D projection = new Vector2D();
                if (projectOnSegment(contactsB.get(1), contactsA.get(0),
                        contactsA.get(1), projection))
                {
                    // On ajoute le point projeter et sa projection
                    Contact contact = new Contact(A, B);
                    contact.setC(projection, contactsB.get(1));
                    contacts.add(contact);
                }
            }
        }
        return true;
    }

    public static ArrayList<Vector2D> findContacts(Particle pA, Vector2D push,
            float t)
    {
        Form A = pA.getForm();

        if (!(A instanceof RoundForm))
        {
            return findSupportPointsForm(A, push, t, pA.getVelocity());
        }

        if (A instanceof Circle)
        {
            Circle cA = (Circle) A;
            return findSupportPointsCircle(cA, push, t, pA.getVelocity());
        }
        assert (false);
        return null;
    }

    public static ArrayList<Vector2D> findSupportPointsCircle(Circle A,
            Vector2D push, float t, Vector2D VA)
    {
        ArrayList<Vector2D> S = new ArrayList<Vector2D>();

        // Point le plus bas simple à trouver, il faut juste prendre le point +
        // le rayon
        Vector2D contact = A.getCenter().add(push.multiply(-A.getRadius()));

        if (t > 0)
            contact.add(VA.multiply(t));

        S.add(contact);
        return S;
    }

    public static ArrayList<Vector2D> findSupportPointsForm(Form A,
            Vector2D push, float t, Vector2D VA)
    {
        ArrayList<Vector2D> S = new ArrayList<Vector2D>();

        //Conversion
        Matrix2 orientation = A.getOrientation().convertMatrix2();
        Vector2D pushOA = orientation.inverse().multiply(push);
        ArrayList<Float> scalar = new ArrayList<Float>(A.size());
        float dmin;

        //On cherche le point minimum par rapport au vecteur
        scalar.add(A.getLocal(0).scalarProduct(pushOA));
        dmin = scalar.get(0);

        for(int i=1; i<A.size(); i++)
        {
            scalar.add(A.getLocal(i).scalarProduct(pushOA));

            if(scalar.get(i) < dmin)
            {
                dmin = scalar.get(i);
            }
        }

        float threshold = 0.1f;
        ArrayList<Float> s = new ArrayList<Float>(2);
        Vector2D perp = pushOA.getPerpendicular();

        //On regarde s'il y a deux points a peu près au meme niveau
        for(int i=0; i<A.size(); i++)
        {
            if(scalar.get(i) < dmin + threshold)
            {
                Vector2D contact = transform(A.getLocal(i), A.getCenter(), VA, orientation, t);
                float fScalar = contact.scalarProduct(perp);

                //On prend les deux points les plus éloignés
                if(s.size() < 2)
                {
                    s.add(fScalar);
                    S.add(contact);

                    if(s.size() > 1)
                    {
                        if(s.get(0) > s.get(1))
                        {
                            float temp = s.get(0);
                            s.set(0, s.get(1));
                            s.set(1, temp);

                            Vector2D tempV = S.get(0);
                            S.set(0, S.get(1));
                            S.set(1, tempV);
                        }
                    }
                }
                else
                {
                    if(fScalar < s.get(0)) //< min
                    {
                        s.set(0, fScalar);
                        S.set(0, contact);
                    }
                    else if(fScalar > s.get(1)) //> max
                    {
                        s.set(1, fScalar);
                        S.set(1, contact);
                    }
                }
            }
        }

        return S;
    }

    // Tools
    public static boolean projectOnSegment(Vector2D PA, Vector2D PB1,
            Vector2D PB2, Vector2D project)
    {
        Vector2D edgeB = new Vector2D(PB1, PB2);
        Vector2D projection = new Vector2D(PB1, PA);
        float magnitude = edgeB.normalize();
        float fProjection = edgeB.scalarProduct(projection);

        if (fProjection < 0)
            fProjection = 0;
        if (fProjection > magnitude)
            fProjection = magnitude;
        project.set(edgeB.multiply(fProjection).add(PB1));
        return true;
    }

    public static Vector2D transform(Vector2D vertex, Vector2D p, Vector2D v,
            Matrix2 orientation, float t)
    {
        Vector2D T = p.add(orientation.multiply(vertex));

        // Si la collision est future
        if (t > 0)
            T.add(v.multiply(t));

        return T;
    }
}
