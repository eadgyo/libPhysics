package org.cora.physics.collision;

import org.cora.maths.Vector2D;

import java.util.ArrayList;

public class ContactResolver
{
    public static void resolveContacts(ArrayList<Contact> c, float dt)
    {
        resolveVelocity(c, dt);
        resolvePenetration(c, dt);
    }

    public static void resolveVelocity(ArrayList<Contact> c, float dt)
    {
        int i = 0;
        for (i = 0; i < c.size(); i++)
        {
            c.get(i).resolveVelocity(dt);
        }
    }

    public static void resolvePenetration(ArrayList<Contact> c, float dt)
    {
        int iterationsUsed = 0;
        int iterations = c.size() * 2;

        int i, index;
        Vector2D linearChange[] = new Vector2D[]{new Vector2D(), new Vector2D()};
        float angularChange[] = new float[]{0, 0};
        float max;


        /*for (i = 0; i < c.size(); i++)
        {
            if (c.get(i).getPenetration() < 0)
                c.get(i).resolvePenetration();
        }*/

        // Init
        while (iterationsUsed < iterations)
        {
            max = -Float.MAX_VALUE;
            index = c.size();

            // Find biggest penetration
            for (i = 0; i < c.size(); i++)
            {
                if (c.get(i).getPenetration() < 0
                        && c.get(i).getPenetration() > max)
                {
                    max = c.get(i).getPenetration();
                    index = i;
                }
            }

            if (index == c.size())
                break;

            linearChange = c.get(index).resolvePenetration();

            //c.get(index).applyPositionChange(linearChange, angularChange);

            for (i = 0; i < c.size(); i++)
            {
                for (int b = 0; b < 2; b++)
                {
                    for (int d = 0; d < 2; d++)
                    {
                        if (c.get(i).get(b) == c.get(index).get(d))
                        {
                            Vector2D deltaPosition = new Vector2D(-angularChange[d] * c.get(i).getRPY(b),
                                    angularChange[d] * c.get(i).getRPX(b));

                            deltaPosition.selfAdd(linearChange[d]);

                            c.get(i).setPenetration(c.get(i).getPenetration() +
                                    deltaPosition.scalarProduct(c.get(i).getContactNormal()) * (b != 0 ? 1 : -1));
                        }
                    }
                }

            }
            iterationsUsed++;
        }

    }
}
