import java.awt.EventQueue;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.swing.*;
import javax.swing.Timer;
import javax.swing.filechooser.FileNameExtensionFilter;

public class SimulationWindow {

	private JFrame frame;
	private Timer timer;
	private int second;
	private static int battery_charge;
	public static String reason_to_stop;
	public static String none_reason = "None";
	public static String crash_reason = "Crash";
	public static String target_reason = "reached the target";
	public static String battery_reason = "the battery is low";

	public static void main(String[] args)
	{
		battery_charge = 100;
		reason_to_stop = none_reason;
		EventQueue.invokeLater(new Runnable()
		{
			public void run()
			{
				try
				{
					SimulationWindow window = new SimulationWindow();
					window.frame.setVisible(true);
					window.second = 0;
					window.simpleTimer();
					window.timer.start();
				}
				catch (Exception e)
				{
					e.printStackTrace();
				}
			}
		});
	}

	public SimulationWindow() {
		initialize();
	}
	public static JLabel info_label;
	public static boolean return_home = false;
	boolean toogleStop = true;
	static JButton stopBtn = new JButton("Play/Pause");
	
	private void initialize() {
		frame = new JFrame();
		frame.setSize(1800,760);
		frame.setTitle("Drone Simulator");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.getContentPane().setLayout(null);
		stopBtn.addActionListener(new ActionListener(){
			  public void actionPerformed(ActionEvent e)
			  {
				  if(toogleStop) {
					  CPU.stopAllCPUS();
				  } else {
					  CPU.resumeAllCPUS();
				  }
				  toogleStop = !toogleStop;
			  }
		});
		stopBtn.setBounds(1450, 10, 120, 50);
		frame.getContentPane().add(stopBtn);
		JButton toogleMapBtn = new JButton("toogle Map");
		toogleMapBtn.addActionListener(new ActionListener(){
			  public void actionPerformed(ActionEvent e){
				  toogleRealMap = !toogleRealMap;
			  }
		});
		toogleMapBtn.setBounds(1450, 100, 120, 50);
		frame.getContentPane().add(toogleMapBtn);
		JButton Graph = new JButton("Open Graph");
		Graph.addActionListener(new ActionListener(){
			  public void actionPerformed(ActionEvent e){
				  algo1.mGraph.drawGraph();
			  }
		});
		Graph.setBounds(1450, 200, 120, 50);
		frame.getContentPane().add(Graph);
		info_label = new JLabel();
		info_label.setBounds(1450, 350, 300, 200);
		frame.getContentPane().add(info_label);
		info_label2 = new JLabel();
		info_label2.setBounds(1450, 250, 300, 200);
		frame.getContentPane().add(info_label2);
		main();
	}
	public JLabel info_label2;
	public static boolean toogleRealMap = true;
	public static AutoAlgo1 algo1;

	public void main() {
		final String regex = "(\\d+)";
		final Pattern pattern = Pattern.compile(regex, Pattern.MULTILINE);
		JFileChooser chooser = new JFileChooser();
		FileNameExtensionFilter filter = new FileNameExtensionFilter("PNG Images", "png");
		
		int map_num = 4;
		Point[] startPoints = {
				new Point(100,50),
				new Point(50,60),
				new Point(73,68),
				new Point(84,73),
				new Point(92,100)};	
		
        chooser.setFileFilter(filter);
        int returnVal = chooser.showOpenDialog(null);
        if(returnVal == JFileChooser.APPROVE_OPTION)
		{
            final String string = chooser.getSelectedFile().getPath();
            final Matcher matcher = pattern.matcher(string);
            if (matcher.find())
			{
                map_num = Integer.parseInt(matcher.group(0))-10;
            }
			else
			{
            	System.exit(0);
            }
        }
		else
		{
        	System.exit(0);
        }
        Map map = new Map(chooser.getSelectedFile().getPath(),startPoints[map_num-1]);
		algo1 = new AutoAlgo1(map);
		Painter painter = new Painter(algo1);
		painter.setBounds(0, 0, 2000, 2000);
		frame.getContentPane().add(painter);
		CPU painterCPU = new CPU(200,"painter"); // 60 FPS painter
		painterCPU.addFunction(frame::repaint);
		painterCPU.play();
		algo1.play();
		CPU updatesCPU = new CPU(60,"updates");
		updatesCPU.addFunction(algo1.drone::update);
		updatesCPU.play();
		CPU infoCPU = new CPU(6,"update_info");
		infoCPU.addFunction(this::updateInfo);
		infoCPU.play();
	}
	
	public void updateInfo(int deltaTime) {
		String text_red = "<span style=\"color:red;font-weight:bold\">"+ String.valueOf(algo1.is_risky) +"</span>";
		String text_green = "<span style=\"color:green;font-weight:bold\">"+ String.valueOf(algo1.is_risky) +"</span>";
		info_label.setText(algo1.drone.getInfoHTML());
		if (algo1.is_risky)
		{
			info_label2.setText("<html><span>isRisky:&nbsp;" + text_red + "</span><br>"
		+ "<span>Battery:&nbsp;<span style=\"color:blue;font-weight:bold\">" + battery_charge + "&#37;</span></span><br><span>Reason to stop:&nbsp;<span style=\"color:blue;font-weight:bold\">" +
			reason_to_stop + "</span></span></html>");
		}
		else
		{
			info_label2.setText("<html><span>isRisky:&nbsp;" + text_green + "</span><br>" + 
		"<span>Battery:&nbsp;<span style=\"color:blue;font-weight:bold\">" + battery_charge + "&#37;</span></span><br><span>Reason to stop:&nbsp;<span style=\"color:blue;font-weight:bold\">" +
			reason_to_stop + "</span></span></html>");
		}
	}
	
	public static void disabledAllButtons() {
		stopBtn.setEnabled(false);
	}
	
	public void simpleTimer() {
		timer = new Timer(1000, new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				if (toogleStop) {
					second++;
					battery_charge = 100 -  (int) Math.round(Double.valueOf(second) / GlobalParams.one_percent_battery);
					if (second == GlobalParams.max_flight_time)
					{
						disabledAllButtons();
						Drone.ArmDisarm();
						reason_to_stop = battery_reason;
						timer.stop();
					}
				}
			}
		});
	}
}
