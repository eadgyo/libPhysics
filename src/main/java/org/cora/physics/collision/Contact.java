package org.cora.physics.collision;

import org.cora.maths.FloatA;
import org.cora.maths.Vector2D;
import org.cora.physics.entities.Particle;
import org.cora.physics.entities.RigidBody;

import java.util.Map;

/**
 * Class holding collision data of two objects
 */
public class Contact implements Cloneable
{
    private Particle A, B;
    private float    coefRestitution;
    private float    penetration;
    private float    coefFriction;
    private Vector2D contactNormal;
    private Vector2D CA, CB;
    private Vector2D rP[];
    private float    sep;
    private boolean isNotSide;

    public static float DEFAULT_FRICTION = 0.5f;
    public static float DEFAULT_RESTITUTION = 0.0f;
    public static float DEFAULT_SEP = 1.0f;
    public static boolean ACTIVE_RESTITUTION_CORRECTION = true;

    public Contact(Particle A, Particle B)
    {
        this(A, B, DEFAULT_RESTITUTION, DEFAULT_FRICTION);
    }
    
    public Contact(Particle A, Particle B, float coefRestitution, float coefFriction)
    {
        this(A, B, coefRestitution, coefFriction, DEFAULT_SEP);
    }
    
    public Contact(Particle A, Particle B, float coefRestitution, float coefFriction, float sep)
    {
        this.coefRestitution = coefRestitution;
        this.coefFriction = coefFriction;
        this.sep = sep;
        CA = new Vector2D();
        CB = new Vector2D();
        rP = new Vector2D[] { new Vector2D(), new Vector2D() };
        contactNormal = new Vector2D();
        this.A = A;
        this.B = B;
        isNotSide = true;
    }
    
    public Particle get(int i)
    {
        if (i == 0)
            return A;
        else if (i == 1)
            return B;
        return null;
    }

    @Override
    public Object clone()
    {
        Contact c = null;
        try
        {
            c = (Contact) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            return c;
        }

        c.contactNormal = (Vector2D) c.contactNormal.clone();
        c.CA = (Vector2D) CA.clone();
        c.CB = (Vector2D) CB.clone();
        c.rP = new Vector2D[rP.length];
        for (int i = 0; i < rP.length; i++)
        {
            Vector2D vector2D = (Vector2D) rP[i].clone();
            c.rP[i] = vector2D;
        }
        c.A = A;
        c.B = B;
        return c;
    }

    public Object clone(Map<Particle, Particle> change)
    {
        Contact c = null;
        try
        {
            c = (Contact) super.clone();
        }
        catch (CloneNotSupportedException e)
        {
            return c;
        }

        c.contactNormal = (Vector2D) c.contactNormal.clone();
        c.CA = (Vector2D) CA.clone();
        c.CB = (Vector2D) CB.clone();
        c.rP = new Vector2D[rP.length];
        for (int i = 0; i < rP.length; i++)
        {
            Vector2D vector2D = (Vector2D) rP[i].clone();
            c.rP[i] = vector2D;
        }
        c.A = change.get(A);
        c.B = change.get(B);
        return c;
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
        setCA(CA);
        setCB(CB);
    }

    public Vector2D getCA()
    {
        return CA;
    }

    public void setCA(Vector2D cA)
    {
        CA = cA;
        rP[0].set(CA.sub(A.getPosition()));
    }

    public Vector2D getCB()
    {
        return CB;
    }

    public void setCB(Vector2D cB)
    {
        CB = cB;
        rP[1].set(CB.sub(B.getPosition()));
    }

    public boolean getIsNotSide()
    {
        return isNotSide;
    }

    public void setIsNotSide(boolean isNotSide)
    {
        this.isNotSide = isNotSide;
    }

    public Vector2D getRelativePenetration(int i)
    {
        return rP[i];
    }

    public float getRPX(int i)
    {
        return rP[i].x;
    }

    public float getRPY(int i)
    {
        return rP[i].y;
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
        /*float angularChange[] = new float[] {0, 0};
        Vector2D linearChange[] = new Vector2D[] { new Vector2D(), new Vector2D() };

        applyPositionChange(linearChange, angularChange);*/
        resolveVelocity(dt);
        resolvePenetration();
    }

    public void resolveVelocity(float dt)
    {
        /*
         * Calcul de j
         */
        float tColl = (penetration > 0f) ? penetration : 0f;

        Vector2D PA = A.getPosition();
        Vector2D PB = B.getPosition();
        Vector2D VA = A.getVelocity();
        Vector2D VB = B.getVelocity();

        RigidBody rA;
        if (A instanceof RigidBody)
            rA = (RigidBody) A;
        else
            rA = null;

        RigidBody rB;
        if (B instanceof RigidBody)
            rB = (RigidBody) B;
        else
            rB = null;

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

        FloatA vAcc = new FloatA(0);

        Vector2D relVel = VPB.sub(VPA);

        float restitution = (relVel.x < 0.5f) ? 0 : coefRestitution;
        if (ACTIVE_RESTITUTION_CORRECTION && restitution != 0)
        {
            Vector2D scaledContact = contactNormal.multiply(dt);
            float velocityFromAcc = B.getLastAcceleration().scalarProduct(scaledContact) - A.getLastAcceleration().scalarProduct(scaledContact);
            relVel.x = relVel.x - restitution * (relVel.x - velocityFromAcc);;
        }

        float vn = contactNormal.scalarProduct(relVel);
        Vector2D Vn = contactNormal.multiply(vn);

        if (vn > 0.0f)
        {
            // Separating collision
            // Les deux entités s'éloignent
            return;
        }

        Vector2D Vt = relVel.sub(Vn);
        float vt = Vt.normalize();

        Vector2D J, Jn, Jt;
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
        float jn = vn / denom;


        Jn = contactNormal.multiply(-((1.0f + coefRestitution) * jn));
        if (coefFriction > 0)
        {
            Jt = Vt.multiply(coefFriction * jn);
            J = Jn.add(Jt);
        }
        else
        {
            J = Jn;
        }

        // Dynamic friction
        Vector2D VA1 = VA.add(J.multiply(-A.getInverseMass()));
        Vector2D VB1 = VB.add(J.multiply(B.getInverseMass()));
        A.setVelocity(VA1);
        B.setVelocity(VB1);

        // Angular Responce
        if (rA != null)
        {
            float rotA1 = -rA.getInverseInertia() * rAP.crossProductZ(J);
            rA.setRotation(rotA + rotA1);

        }

        if (rB != null)
        {
            float rotB1 = rB.getInverseInertia() * rBP.crossProductZ(J);
            rB.setRotation(rotB + rotB1);

        }


        // Static friction
        if (coefFriction > 0.0f && vn < 0.0f)
        {
            float cone = -vt / vn;

            if (cone < coefFriction)
            {
                Contact frictionContact = new Contact(A, B, coefFriction, 0);
                frictionContact.setContactNormal(Vt);
                frictionContact.setPenetration(0);
                frictionContact.setC(CA, CB);

                frictionContact.resolveVelocity(dt);
            }
        }
    }

    public void applyPositionChange(Vector2D linearChange[], float angularChange[])
    {
        if (penetration >= 0)
            return;

        float angularLimit = 0.2f;
        float angularMove[] = new float[2];
        float linearMove[] = new float[2];

        float totalInertia = 0;
        float linearInertia[] = new float[2];
        float angularInertia[] = new float[2];

        Particle p[] = new Particle[] { A, B };
        RigidBody rA = (RigidBody) A;
        RigidBody rB = (RigidBody) B;
        RigidBody r[] = new RigidBody[] { rA, rB };

        Vector2D rP[] = new Vector2D[] { CA.sub(A.getPosition()), CB.sub(B.getPosition())};

        for (int i = 0; i < 2; i++)
        {
            angularInertia[i] = 0;
            if (r[i] != null)
            {
                angularInertia[i] = (rP[i].crossProductZ(contactNormal))
                        * (rP[i].crossProductZ(contactNormal))
                        * r[i].getInverseInertia();
            }

            linearInertia[i] = p[i].getInverseMass();

            totalInertia += linearInertia[i] + angularInertia[i];
        }

        for (int i = 0; i < 2; i++)
        {
            float sign = (i == 0) ? 1 : -1;

            angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
            linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

            /*
            // Limit the angular move
            Vector2D projection = rP[i].addScaledVector(contactNormal, -(rP[i].scalarProduct(contactNormal)));

            float maxMagnitude = angularLimit * projection.getMagnitude();

            if (angularMove[i] < - maxMagnitude)
            {
                float totalMove = angularMove[i] + linearMove[i];
                angularMove[i] = -maxMagnitude;
                linearMove[i] = totalMove - angularMove[i];
            }
            else if (angularMove[i] > maxMagnitude)
            {
                float totalMove = angularMove[i] + linearMove[i];
                angularMove[i] = maxMagnitude;
                linearMove[i] = totalMove - angularMove[i];
            }*/

            if (angularMove[i] == 0)
            {
                angularChange[i] = 0;
            }
            else
            {
                float targetAngularDirection = rP[i].crossProductZ(contactNormal);

                float inverseInertiaTensor = 0;

                if (r[i] != null)
                {
                    inverseInertiaTensor = r[i].getInverseInertia();
                }

                //angularChange[i] = inverseInertiaTensor * targetAngularDirection * ( angularMove[i] / angularInertia[i] );
            }

            linearChange[i].set(contactNormal.multiply(linearMove[i]));

            Vector2D pos = p[i].getPosition().addScaledVector(contactNormal, linearMove[i]);
            p[i].setPosition(pos);

            if (r[i] != null)
            {
                float rad = r[i].getOrientation();
                rad += angularChange[i];
                r[i].setOrientation(rad);
            }

            if (angularChange[0] != 0)
            {
                int z = 0;
            }
        }

    }

    public Vector2D[] resolvePenetration()
    {
        Vector2D linearChange[] = new Vector2D[2];

        linearChange[0] = new Vector2D();
        linearChange[1] = new Vector2D();

        if ( penetration >= 0 )
            return linearChange;

        float iMassA = A.getInverseMass();
        float iMassB = B.getInverseMass();
        float total = (iMassA + iMassB);

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

    public float desiredVel(Vector2D relVel, FloatA vAcc, float dt)
    {
        vAcc.v = -A.getLastAcceleration().scalarProduct(contactNormal) * dt;
        vAcc.v += B.getLastAcceleration().scalarProduct(contactNormal) * dt;

        float vn = relVel.scalarProduct(contactNormal);

        if (Math.abs(vn) < 0.50f)
        {
            coefRestitution = 0;
            relVel.set(0, 0);
        }



        return vn;
    }
}
