package org.cora.physics.collision;

import org.cora.physics.entities.Particle;
import org.cora.physics.entities.RigidBody;

import org.cora.maths.FloatA;
import org.cora.maths.Vector2D;

/**
 * Class holding collision data of two objects
 */
public class Contact
{
    private Particle A, B;
    private float    coefRestitution;
    private float    penetration;
    private float    coefFriction;
    private Vector2D contactNormal;
    private Vector2D CA, CB;
    private float    sep;

    public Contact(Particle A, Particle B)
    {
        this(A, B, 0.3f, 0.5f);
    }
    
    public Contact(Particle A, Particle B, float coefRestitution, float coefFriction)
    {
        this(A, B, coefRestitution, coefFriction, 1.0f);
    }
    
    public Contact(Particle A, Particle B, float coefRestitution, float coefFriction, float sep)
    {
        this.coefRestitution = coefRestitution;
        this.coefFriction = coefFriction;
        this.sep = sep;
        CA = new Vector2D();
        CB = new Vector2D();
        contactNormal = new Vector2D();
        this.A = A;
        this.B = B;
    }
    
    public Particle get(int i)
    {
        if (i == 0)
            return A;
        else if (i == 1)
            return B;
        return null;
    }

    public Particle getA()
    {
        return A;
    }

    public void setA(Particle a)
    {
        A = a;
    }

    public Particle getB()
    {
        return B;
    }

    public void setB(Particle b)
    {
        B = b;
    }

    public float getRestitution()
    {
        return coefRestitution;
    }

    public void setRestitution(float restitution)
    {
        this.coefRestitution = restitution;
    }

    public float getPenetration()
    {
        return penetration;
    }

    public void setPenetration(float penetration)
    {
        this.penetration = penetration;
    }

    public float getCoefFriction()
    {
        return coefFriction;
    }

    public void setCoefFriction(float coefFriction)
    {
        this.coefFriction = coefFriction;
    }

    public Vector2D getContactNormal()
    {
        return contactNormal;
    }

    public void setContactNormal(Vector2D contactNormal)
    {
        this.contactNormal.set(contactNormal);
    }

    public void setC(Vector2D CA, Vector2D CB)
    {
        this.CA.set(CA);
        this.CB.set(CB);
    }

    public Vector2D getCA()
    {
        return CA;
    }

    public void setCA(Vector2D cA)
    {
        CA = cA;
    }

    public Vector2D getCB()
    {
        return CB;
    }

    public void setCB(Vector2D cB)
    {
        CB = cB;
    }

    public float getSep()
    {
        return sep;
    }

    public void setSep(float sep)
    {
        this.sep = sep;
    }
    
    public void set(ContactInformation information)
    {
        set(information.coefRestitution, information.coefFriction, information.sep);
    }
    
    public void set(float coefRestitution, float coefFriction, float sep)
    {
        this.coefRestitution = coefRestitution;
        this.coefFriction = coefFriction;
        this.sep = sep;
    }

    public void resolve(float dt)
    {
        resolvePenetration(dt);
        resolveVelocity(dt);
    }

    public void resolveVelocity(float dt)
    {
        /*
         * Calcul de j
         */

        contactNormal.normalize();
        float tColl = (penetration > 0f) ? penetration : 0f;

        Vector2D PA = A.getPosition();
        Vector2D PB = B.getPosition();
        Vector2D VA = A.getVelocity();
        Vector2D VB = B.getVelocity();

        RigidBody rA = (RigidBody) A;
        RigidBody rB = (RigidBody) B;

        float rotA = (rA == null) ? 0f : rA.getRotation();
        float rotB = (rB == null) ? 0f : rB.getRotation();

        // Calcul en fonction du temps
        Vector2D QA = PA.add(VA.multiply(tColl));
        Vector2D QB = PB.add(VB.multiply(tColl));
        Vector2D rAP = (CA.sub(QA));
        Vector2D rBP = (CB.sub(QB));
        Vector2D TA = rAP.getPerpendicular();
        Vector2D TB = rBP.getPerpendicular();
        Vector2D VPA = VA.sub(TA.multiply(-rotA));
        Vector2D VPB = VB.sub(TB.multiply(-rotB));

        Vector2D relVel = VPB.sub(VPA);
        FloatA vAcc = new FloatA(0);

        float vn = desiredVel(relVel, vAcc, dt);

        Vector2D N = contactNormal.getPerpendicular();
        float vt = N.scalarProduct(relVel);
        Vector2D Vt = N.multiply((vt < 0) ? -1 : 1);

        if (vn > 0.0f)
        {
            // Separating collision
            // Les deux entités s'éloignent
            return;
        }

        Vector2D J;
        float t0 = 0;
        float t1 = 0;

        if (rA != null)
        {
            t0 = (rAP.crossProductZ(contactNormal))
                    * (rAP.crossProductZ(contactNormal))
                    * rA.getInverseInertia();
        }

        if (rB != null)
        {
            t1 = (rBP.crossProductZ(contactNormal))
                    * (rBP.crossProductZ(contactNormal))
                    * rB.getInverseInertia();
        }

        float m = A.getInverseMass() + B.getInverseMass();

        float denom = m + t0 + t1;
        float jn = (vn - vAcc.v) / denom;

        J = contactNormal.multiply(vAcc.v + ((1f + coefRestitution) * -jn));
        // Dynamic friction

        J.selfAdd(Vt.multiply((coefFriction * jn)));

        Vector2D VA1 = VA.add(J.multiply(-A.getInverseMass()));
        Vector2D VB1 = VB.add(J.multiply(B.getInverseMass()));
        A.setVelocity(VA1);
        B.setVelocity(VB1);

        // Angular Responce
        if (rA != null)
        {
            float rotA1 = rotA + -rA.getInverseInertia() * rAP.crossProductZ(J);
            rA.setRotation(rotA1);
        }

        if (rB != null)
        {
            float rotB1 = rotB + rB.getInverseInertia() * rBP.crossProductZ(J);
            rB.setRotation(rotB1);
        }

        // Static friction
        if (coefFriction > 0.0f && vt > vn * coefFriction)
        {
            Contact frictionContact = new Contact(A, B, coefFriction, -1);
            frictionContact.setContactNormal(Vt);
            frictionContact.setPenetration(0);
            frictionContact.setC(CA, CB);

            frictionContact.resolveVelocity(dt);
        }
    }

    public Vector2D[] resolvePenetration(float dt)
    {
        float iMassA = A.getInverseMass();
        float iMassB = B.getInverseMass();
        float total = (iMassA + iMassB);

        Vector2D linearChange[] = new Vector2D[2];

        linearChange[0] = new Vector2D();
        linearChange[1] = new Vector2D();

        Vector2D D = new Vector2D(CA, CB);
        D.selfMultiply(sep);

        if (iMassA == 0 && iMassB == 0)
            return linearChange;
        if (iMassA > 0)
        {
            Vector2D D0 = D.multiply(iMassA / total);
            A.translate(D0);
            linearChange[0].set(D0);
        }
        if (iMassB > 0)
        {
            Vector2D D0 = D.multiply(-iMassB / total);
            B.translate(D0);
            linearChange[1].set(D0);
        }
        return linearChange;
    }

    public float calculateSeparatingVelocity()
    {
        return 0;
    }

    public float desiredVel(Vector2D relVel, FloatA vAcc, float dt)
    {
        vAcc.v = -A.getLastAcceleration().scalarProduct(contactNormal) * dt;
        vAcc.v += B.getLastAcceleration().scalarProduct(contactNormal) * dt;

        float vn = relVel.scalarProduct(contactNormal);

        if (Math.abs(vn) < 0.25f)
        {
            coefRestitution = 0;
        }

        return vn;
    }
}
