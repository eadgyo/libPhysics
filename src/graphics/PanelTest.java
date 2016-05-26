package graphics;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Polygon;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;

import javax.swing.JPanel;

import cora.addons.RigidBody;
import cora.collision.ContactGenerator;
import cora.collision.Engine;
import cora.force.Gravity;
import cora.maths.Circle;
import cora.maths.FloatA;
import cora.maths.Form;
import cora.maths.RoundForm;
import cora.maths.Vector2D;
import cora.maths.collision.CollisionDetector;

public class PanelTest extends JPanel implements Runnable
{
    private Thread runner;
    public boolean ok;
    private Engine engine;
    private RigidBody A, B;
    private boolean mov = true;

    public PanelTest() throws Exception
    {
        runner = null;
        engine = new Engine();
        initEngine();
        
    }

    private void update(float dt)
    {
        engine.update(dt);
    }

    private void initEngine() throws Exception
    {
        A = new RigidBody();

        Form formA = new Form();
        formA.addPoint(new Vector2D(-50.208333f, -9.208335f));
        formA.addPoint(new Vector2D(-90.208333f, 7.7916675f));
        formA.addPoint(new Vector2D(50.916666f, 9.916668f));
        formA.addPoint(new Vector2D(7.7916675f, -9.208334f));
        formA.endForm();

        A.setForm(formA);
        A.setPosition(new Vector2D(160.25064f, 201.08408f));
        A.initPhysics();

        engine.addBody("A", A);

        B = new RigidBody();
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

        engine.addBody("B", B);
        engine.addForce("B", new Gravity(new Vector2D(0, -10.0f)));
    }

    public void start()
    {
        ok = true;
        runner = null;
        if (runner == null)
        {
            runner = new Thread(this);
            runner.start();
        }
    }

    public void stop()
    {
        ok = false;
        if (runner != null)
        {
            try
            {
                runner.join();
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            runner = null;
        }
    }

    @Override
    public void paintComponent(Graphics g)
    {
        g.setColor(Color.GREEN);
        g.fillRect(0, 0, 800, 600);

        g.setColor(Color.RED);
        ArrayList<RigidBody> bodies = engine.getAllBodies();
        for (int i = 0; i < bodies.size(); i++)
        {
            Form form = bodies.get(i).getForm();
            
            if (! (form instanceof RoundForm))
            {
                Polygon polygon = form.getPolygon();
                g.drawPolygon(polygon);
            }
            else if (form instanceof Circle)
            {
                Circle rA = (Circle) form;
                g.drawOval((int) (rA.getCenterX() - rA.getRadius()), (int) (rA.getCenterY() - rA.getRadius()), (int) rA.getRadius()*2, (int) rA.getRadius()*2);
            }
        }
    }

    @Override
    public void run()
    {   
        //ok = false;
        while (ok)
        {
            this.repaint();
            float dt = 0.008f;
            int wait = (int) (dt * 1000);

            engine.update(dt);
            
            try
            {
                runner.sleep(4);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }
}
