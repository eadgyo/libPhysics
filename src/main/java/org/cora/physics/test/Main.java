package org.cora.physics.test;

import org.cora.physics.Engine.Engine;
import org.cora.physics.entities.RigidBody;
import org.cora.physics.force.Gravity;

import org.cora.maths.Vector2D;
import org.cora.maths.Form;

/**
 * Created by ronan-h on 27/05/16.
 */
public class Main
{
    public void main(String args[])
    {
        Engine engine = new Engine();

        RigidBody A = new RigidBody();
        Form formA = new Form();
        formA.addPoint(new Vector2D(-50.208333f, -9.208335f));
        formA.addPoint(new Vector2D(-90.208333f, 7.7916675f));
        formA.addPoint(new Vector2D(50.916666f, 9.916668f));
        formA.addPoint(new Vector2D(7.7916675f, -9.208334f));
        formA.endForm();

        A.setForm(formA);
        A.setPosition(new Vector2D(160.25064f, 201.08408f));
        A.initPhysics();

        engine.addElement(A);

        RigidBody B = new RigidBody();
        B.setMass(289.00000f);
        Form formB = new Form();
        formB.addPoint(new Vector2D(-8.5f, 8.5f));
        formB.addPoint(new Vector2D(-8.5f, -8.5f));
        formB.addPoint(new Vector2D(8.5f, -8.5f));
        formB.addPoint(new Vector2D(8.5f, 8.5f));

        //Form formB = new Circle(20.0f);
        formB.endForm();

        B.setForm(formB);
        B.setPosition(new Vector2D(180, 400));
        B.initPhysics();


        // Ajout de la force de gravité sur l'élément B
        engine.addElement(B);
        engine.addForce(B, new Gravity(new Vector2D(0, -10.0f)));

        // boucle -->
        while (true)
        {
            // Actualisation du moteur
            engine.update(0.01f);
        }
    }
}
