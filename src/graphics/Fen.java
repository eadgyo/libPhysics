package graphics;

import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import javax.swing.JFrame;

public class Fen
{
    PanelTest panel;

    public Fen()
    {
        JFrame frame = new JFrame("Test Engine");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setResizable(false);
        frame.setSize(800, 600);
        frame.setLocationRelativeTo(null);

        try
        {
            panel = new PanelTest();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        panel.setDoubleBuffered(true);
        panel.setBackground(Color.green);
        frame.setContentPane(panel);

        frame.addKeyListener(new KeyUpdater());

        panel.repaint();
        panel.start();

        frame.addWindowListener(new java.awt.event.WindowAdapter()
        {
            @Override
            public void windowClosing(java.awt.event.WindowEvent windowEvent)
            {
                panel.stop();
            }
        });
        
        frame.setVisible(true);
    }

    private class KeyUpdater implements KeyListener
    {
        @Override
        public void keyTyped(KeyEvent e)
        {

        }

        @Override
        public void keyPressed(KeyEvent e)
        {

        }

        @Override
        public void keyReleased(KeyEvent e)
        {

        }

    }
}
