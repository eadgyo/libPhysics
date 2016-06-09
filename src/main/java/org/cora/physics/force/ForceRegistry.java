package org.cora.physics.force;

import org.cora.physics.entities.Particle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

/**
 * Registry that holds force applied on element
 */
public class ForceRegistry implements Cloneable
{
    private class Registration implements Cloneable
    {
        public ForceGenerator force;

        public Registration(ForceGenerator force)
        {
            this.force = force;
        }

        @Override
        public Object clone()
        {
            Registration r = null;

            try
            {
                r = (Registration) super.clone();
            }
            catch (CloneNotSupportedException e)
            {
                e.printStackTrace();
            }

            r.force = (ForceGenerator) force.clone();
            return force;
        }

        public Object clone(Map<Particle, Particle> change)
        {
            Registration r = null;

            try
            {
                r = (Registration) super.clone();
            }
            catch (CloneNotSupportedException e)
            {
                e.printStackTrace();
            }

            r.force = (ForceGenerator) force.clone(change);
            return r;
        }
    }

    Map<Particle, ArrayList<Registration>> registrations;

    public ForceRegistry()
    {
        registrations = new HashMap<Particle, ArrayList<Registration>>();
    }

    @Override
    public Object clone()
    {
        ForceRegistry registry = null;
        try
        {
            registry = (ForceRegistry) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
        }

        registry.registrations = new HashMap<>();

        for (Iterator<Entry<Particle, ArrayList<Registration>>> it = registrations.entrySet().iterator(); it.hasNext(); )
        {
            Entry<Particle, ArrayList<Registration>> e = it.next();
            ArrayList<Registration> registrations1 = e.getValue();
            for (int i = 0; i < registrations1.size(); i++)
            {
                Registration registration = (Registration) registrations1.get(i).clone();
            }

            registry.registrations.put(e.getKey(), new ArrayList<>(e.getValue()));
        }

        return registry;
    }

    public Object clone(Map<Particle, Particle> change)
    {
        ForceRegistry registry = null;
        try
        {
            registry = (ForceRegistry) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
        }

        registry.registrations = new HashMap<>();

        for (Iterator<Entry<Particle, ArrayList<Registration>>> it = registrations.entrySet().iterator(); it.hasNext(); )
        {
            Entry<Particle, ArrayList<Registration>> e = it.next();
            ArrayList<Registration> registrations1 = e.getValue();
            for (int i = 0; i < registrations1.size(); i++)
            {
                Registration registration = (Registration) registrations1.get(i).clone(change);
            }

            registry.registrations.put(e.getKey(), new ArrayList<>(e.getValue()));
        }

        return registry;
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
