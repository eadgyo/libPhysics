package org.cora.physics.test;

/*import org.cora.graphics.graphics.Graphics;
import org.cora.graphics.graphics.myColor;
import org.cora.graphics.input.Input;
import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;
import static org.lwjgl.opengl.GL11.GL_FALSE;
*/
import org.cora.maths.Form;
import org.cora.maths.Vector2D;
import org.cora.physics.Engine.Engine;
import org.cora.physics.entities.RigidBody;
import org.cora.physics.force.Gravity;

/**
 * Created by ronan-h on 28/05/16.
 */
public class Main
{
    public static void main(String[] args)
    {
        int WINDOW_WIDTH = 800;
        int WINDOW_HEIGHT = 600;

        /*Graphics g = new Graphics();
        g.init("Test", WINDOW_WIDTH, WINDOW_HEIGHT);
        g.initGL(WINDOW_WIDTH, WINDOW_HEIGHT);

        Input input = new Input();
        input.initGL(g.getScreen());*/

        Engine engine = new Engine();

        RigidBody A = new RigidBody();
        //Form formA =// new Rectangle(new Vector2D(100, 200), new Vector2D(200, 100), 0);
        Form formA = new Form();
        formA.addPoint(new Vector2D(-35, -30));
        formA.addPoint(new Vector2D(50, -30));
        formA.addPoint(new Vector2D(50, 30));
        formA.addPoint(new Vector2D(-40, 30));
        formA.updateCenter();

        A.setForm(formA);
        A.setPosition(new Vector2D(new Vector2D(150, 500)));
        A.initPhysics();

        engine.addElement(A);

        RigidBody B = new RigidBody();
        B.setMass(289.00000f);
        //Form formB = new Rectangle(new Vector2D(15, 70), new Vector2D(150, 500), 0);
        Form formB = new Form();
        formB.addPoint(new Vector2D(-50.208333f, -9.208335f));
        formB.addPoint(new Vector2D(-90.208333f, 7.7916675f));
        formB.addPoint(new Vector2D(50.916666f, 9.916668f));
        formB.addPoint(new Vector2D(7.7916675f, -9.208334f));
        formB.setRadians(2.12f);
        //Form formB = new Circle(20.0f);
        formB.updateCenter();

        B.setForm(formB);
        B.setPosition(new Vector2D(150, 150));
        B.initPhysics();


        // Ajout de la force de gravité sur l'élément B
        engine.addElement(B);
        engine.addForce(B, new Gravity(new Vector2D(0, 100.0f)));

        // boucle -->

        /*while (glfwWindowShouldClose(g.getScreen()) == GL_FALSE)
        {
            g.clear();

            g.setColor(myColor.RED());
            g.drawForm(formA);

            g.setColor(myColor.GREEN());
            g.drawForm(formB);

            engine.update(0.016f);

            input.update();

            g.swapGL();
        }*/
    }

}
