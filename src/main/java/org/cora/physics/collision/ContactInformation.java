package org.cora.physics.collision;

public class ContactInformation
{
    public float coefRestitution;
    public float coefFriction;
    public float sep;
    
    public ContactInformation()
    {     
    }
    
    public ContactInformation(float coefRestitution, float coefFriction, float sep)
    {
        this.coefRestitution = coefRestitution;
        this.coefFriction = coefFriction;
        this.sep = sep;
    }
    
    static ContactInformation createDefault()
    {
        ContactInformation information = new ContactInformation(0.2f, 0.5f, 1.0f);;
        return information;
    }
}
