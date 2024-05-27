import java.awt.*;

import javax.swing.JComponent;


public class Painter extends JComponent{
	/**
	 * 
	 */
	private static final long serialVersionUID = 2413542956407597691L;
	AutoAlgo1 algo;
	
	public Painter(AutoAlgo1 algo) {
		this.algo = algo;
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		algo.paint(g);
	}
}
