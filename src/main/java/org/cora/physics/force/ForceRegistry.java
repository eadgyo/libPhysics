package org.cora.physics.force;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.cora.physics.entities.Particle;

/**
 * Registry that holds force applied on element
 */
public class ForceRegistry
{
    private class Registration
    {
        public ForceGenerator force;

        public Registration(ForceGenerator force)
        {
            this.force = force;
        }
    }

    Map<Particle, ArrayList<Registration>> registrations;

    public ForceRegistry()
    {
        registrations = new HashMap<Particle, ArrayList<Registration>>();
    }

    public void clear()
    {
        for (Entry<Particle, ArrayList<Registration>> entry : registrations
                .entrySet())
        {
            entry.getValue().clear();
        }
        registrations.clear();
    }

    public ArrayList<Registration> getOrCreate(Particle p)
    {
        ArrayList<Registration> tmpRegistrations = registrations.get(p);

        if (tmpRegistrations == null)
        {
            tmpRegistrations = new ArrayList<Registration>();
            registrations.put(p, tmpRegistrations);
        }

        return tmpRegistrations;
    }

    public void add(Particle p, ForceGenerator force)
    {
        ArrayList<Registration> tmpRegistrations = getOrCreate(p);

        Registration registration = new Registration(force);
        tmpRegistrations.add(registration);
    }

    public void remove(Particle p, ForceGenerator force)
    {
        ArrayList<Registration> tmpRegistrations = registrations.get(p);

        if (tmpRegistrations == null)
            return;

        for (int i = 0; i < tmpRegistrations.size(); i++)
        {
            if (tmpRegistrations.get(i).force == force)
            {
                tmpRegistrations.remove(i);
                i--;
            }
        }
    }

    public void removeAll(Particle p)
    {
        ArrayList<Registration> tmpRegistrations = registrations.get(p);

        if (tmpRegistrations == null)
            return;

        tmpRegistrations.clear();
        registrations.remove(p);
    }

    public void update(float dt)
    {
        for (Entry<Particle, ArrayList<Registration>> entry : registrations
                .entrySet())
        {
            Particle p = entry.getKey();
            ArrayList<Registration> tmpRegistrations = entry.getValue();
            for (int i = 0; i < tmpRegistrations.size(); i++)
            {
                tmpRegistrations.get(i).force.updateForce(p, dt);
            }
        }
    }
}
