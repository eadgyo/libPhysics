package org.cora.physics.entities.material;

import java.util.HashMap;
import java.util.Map;

import org.cora.physics.collision.ContactInformation;

public class MaterialType
{
    private Map<MaterialType, ContactInformation> collisionMaterials;
    
    public MaterialType()
    {
        collisionMaterials = new HashMap<MaterialType, ContactInformation>();
    }
    
    public ContactInformation getMaterialInformation(MaterialType material)
    {
        return collisionMaterials.get(material);
    }
    
    public boolean isPresent(MaterialType material)
    {
        return collisionMaterials.containsKey(material);
    }
    
    public void addMaterialInformation(MaterialType material, ContactInformation information)
    {
        collisionMaterials.put(material, information);
        
        if (this != material)
            material.addMaterialInformationFree(this, information);
    }
    
    public void addMaterialInformationFree(MaterialType material, ContactInformation information)
    {
        collisionMaterials.put(material, information);
    }
    
    public void addMaterialInformation(MaterialType material, float coefRestitution, float coefFriction, float sep)
    {
        ContactInformation information = new ContactInformation(coefRestitution, coefFriction, sep);
        addMaterialInformation(material, information);
    }
    
    public ContactInformation removeMaterialInformation(MaterialType material)
    {
        if (this == material)
            return collisionMaterials.remove(material);
        
        collisionMaterials.remove(material);
        return removeMaterialInformationFree(this);
    }
    
    public ContactInformation removeMaterialInformationFree(MaterialType material)
    {
        return collisionMaterials.remove(material);
    }
}
