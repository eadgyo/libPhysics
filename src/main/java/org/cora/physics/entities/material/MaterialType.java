package org.cora.physics.entities.material;

import java.util.HashMap;
import java.util.Map;

import org.cora.physics.collision.ContactInformation;
/**
 * Material that define contact behavior
 * Exemple: Collision between wood and glass
 */
public class MaterialType
{
    private Map<MaterialType, ContactInformation> collisionMaterials;
    public final static float DEFAULT_DENSITY = 0.05f;
    private float density;

    public MaterialType(float density)
    {
        this.density = density;
        collisionMaterials = new HashMap<MaterialType, ContactInformation>();
    }

    public MaterialType()
    {
        this(DEFAULT_DENSITY);
    }

    /**
     * Get information related to a material
     * @param material colliding material
     * @return information or null if not material not present
     */
    public ContactInformation getMaterialInformation(MaterialType material)
    {
        return collisionMaterials.get(material);
    }
    
    public boolean isPresent(MaterialType material)
    {
        return collisionMaterials.containsKey(material);
    }

    /**
     *
     * @param material other materialType
     * @param information behavior information
     */
    public void addMaterialInformation(MaterialType material, ContactInformation information)
    {
        collisionMaterials.put(material, information);
        
        if (this != material)
            material.addMaterialInformationFree(this, information);
    }

    /**
     * Do not use
     */
    public void addMaterialInformationFree(MaterialType material, ContactInformation information)
    {
        collisionMaterials.put(material, information);
    }
    
    public void addMaterialInformation(MaterialType material, float coefRestitution, float coefFriction, float sep)
    {
        ContactInformation information = new ContactInformation(coefRestitution, coefFriction, sep);
        addMaterialInformation(material, information);
    }

    /**
     * Remove information linked to a material type
     * @param material linked material
     * @return information
     */
    public ContactInformation removeMaterialInformation(MaterialType material)
    {
        if (this == material)
            return collisionMaterials.remove(material);
        
        collisionMaterials.remove(material);
        return removeMaterialInformationFree(this);
    }

    /**
     * Do not use
     */
    public ContactInformation removeMaterialInformationFree(MaterialType material)
    {
        return collisionMaterials.remove(material);
    }

    /**
     * Get material density
     * @return material density
     */
    public float getDensity()
    {
        return density;
    }

    /**
     * Set material density
     */
    public void setDensity(float density)
    {
        this.density = density;
    }
}
