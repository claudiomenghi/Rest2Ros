/**
 * Copyright (C) 2011-2014 Swinburne University of Technology and University of Gotheborg
 *
 * These file have been developed as a part of the co4robots project.
 * It is a tool
 *
 * These files are based on PSPWizard which was developed at Faculty of Science, Engineering and
 * Technology at Swinburne University of Technology, Australia.
 * The patterns, structured English grammar and mappings are due to
 * Marco Autili, Universita` dell'Aquila
 * Lars Grunske, University of Stuttgart
 * Markus Lumpe, Swinburne University of Technology
 * Patrizio Pelliccione, University of Gothenburg
 * Antony Tang, Swinburne University of Technology
 *
 * Details about the PSP framework can found in
 * "Aligning Qualitative, Real-Time, and Probabilistic
 * Property Specification Patterns Using a Structured
 * English Grammar"
 *
 *
 *
 * PSPWizard is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * PSPWizard is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PSPWizard; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */
package se.got;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.AdjustmentEvent;
import java.awt.event.AdjustmentListener;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.DefaultComboBoxModel;
import javax.swing.GroupLayout;
import javax.swing.GroupLayout.SequentialGroup;
import javax.swing.ImageIcon;
import javax.swing.JComboBox;
import javax.swing.JFormattedTextField;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollBar;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.UIManager;
import javax.swing.border.TitledBorder;
import javax.swing.plaf.ColorUIResource;
import javax.swing.text.DefaultFormatter;

import se.got.engine.EventSelectionValidator;
import se.got.engine.EventStorage;
import se.got.ltl.LTLFormula;
import se.got.ltl.visitors.LTLFormulaToStringVisitor;
import se.got.sel.Event;

public class Co4robotsGUI extends javax.swing.JFrame  {

	private static final String INIT_POSITION_MESSAGE = "Insert the positions to be considered separated by a comma";

	private static final int FRAME_INIT_HEIGTH = 600;

	private static final int FRAME_INIT_WIDTH = 800;

	private JPanel locationPanel;
	private static JTextField ipTextField;
	private static JTextField portTextField;
	
	
	private JPanel remotePanel;
	private static int FORMULA_COUNTER = 1;

	private static Map<String, LTLFormula> formulae=new HashMap<>();

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private final static String TITLE = "co4robots: High Level Specification Panel";
	private final static String EVENTNAMES = "Show service names";
	private final static String EVENTSPECIFICATION = "Show service specifications";
	private final static String EDITEVENT = "Edit service";
	private final static String ADDEVENT = "Add Service";
	private final static String MOVEMENT_PATTERN = "Movement Specification Pattern";
	private final static String SEND_MISSION = "Send mission";
	private final static String LOAD_MISSION = "Load mission";

	private final static String SELECT_PATTERN_CATEGORY = "Select pattern category";

	private static JList<String> propertyList;

	private JComboBox<String> patternCategorySelector;
	private JComboBox<String> patternBoxSelector;
	private JTextArea ltlFormula;

	private JTextArea intentText;
	//private JTextArea variation;
	private JTextArea examples;
	private JTextArea occurences;
	private JComboBox<String> f1;
	private JComboBox<String> f2;
	private final DefaultComboBoxModel<String> patternItems;

	private JTextField locations;
	public final static Color BACKGROUNDCOLOR = Color.WHITE;
	private EventStorage fEvents;


	public Co4robotsGUI(String ip, String port) {
		super();
		
		this.getContentPane().setBackground(BACKGROUNDCOLOR);
		
		UIManager.put("ComboBox.background", new ColorUIResource(Color.WHITE));

		patternItems = new DefaultComboBoxModel<>();

		this.f1 = new JComboBox<String>();
		this.f2 = new JComboBox<String>();
		String[] elements = { "" };
		f1 = new JComboBox<>();

		f1.setBorder(javax.swing.BorderFactory.createTitledBorder("Formula f1"));
		f1.setToolTipText("");

		f2.setBorder(javax.swing.BorderFactory.createTitledBorder("Formula f2"));
		f2.setToolTipText("");

		TitledBorder movementPatternTitle = BorderFactory.createTitledBorder("Property  List");
		movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

		propertyList = new JList<String>(elements);
		propertyList.setBackground(Color.GRAY);

		propertyList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		initComponents(ip,port);
		System.out.println("GUI loaded");

	}
		
	// Entry point

	public static void main(String args[]) {
		/* Create and display the form */
		final String ip=args.length>0 ? args[0] :"127.0.0.1";
		final String port=args.length>0 ? args[1] :"13000";
		java.awt.EventQueue.invokeLater(new Runnable() {
			public void run() {
				(new Co4robotsGUI(ip,port)).showEditor();
			}
		});
	}

	public void showEditor() {
		setLocationRelativeTo(null); // center?
		setVisible(true);
	}








	public boolean isPatternEventSelectionPossible(Event aEvent, Event aAltEvent) {
		return EventSelectionValidator.isPatternEventSelectionPossible(this, aEvent, aAltEvent);
	}





	/**
	 * This method is called from within the constructor to initialize the form.
	 * WARNING: Do NOT modify this code. The content of this method is always
	 * regenerated by the Form Editor.
	 */
	private void initComponents(String ip, String port) {
		setLayout(new BorderLayout( ));
		//setSize(700,200);
		/*JScrollBar hbar=new JScrollBar(JScrollBar.HORIZONTAL, 30, 20, 0, 700);
        JScrollBar vbar=new JScrollBar(JScrollBar.VERTICAL, 30, 40, 0, 700);
        class MyAdjustmentListener implements AdjustmentListener {
        	@Override
            public void adjustmentValueChanged(AdjustmentEvent e) {
                getContentPane().repaint();
            }
        }
 		
        hbar.addAdjustmentListener(new MyAdjustmentListener( ));
        vbar.addAdjustmentListener(new MyAdjustmentListener( ));
        */
	//ending the initials	
		patternsJPanel = new javax.swing.JPanel();
		patternsJPanel.setBackground(BACKGROUNDCOLOR);

		patternJPanel = new javax.swing.JPanel();
		optionJPanel = new javax.swing.JPanel();
		jPanelLogo = new javax.swing.JPanel();
		fEName = new javax.swing.JCheckBox();
		fESpec = new javax.swing.JCheckBox();
		this.sendMission = new javax.swing.JButton();
		this.loadMission = new javax.swing.JButton();
		jPanel5 = new javax.swing.JPanel();
		jScrollPane1 = new javax.swing.JScrollPane();
		fSELP = new javax.swing.JTextPane();
		propertyPanel = new javax.swing.JPanel();
		jScrollPane2 = new javax.swing.JScrollPane();
		fMapping = new javax.swing.JTextArea();
		jLabel1 = new javax.swing.JLabel();
		jLabel2 = new javax.swing.JLabel();
		eventJPanel = new javax.swing.JPanel();
		fNE = new javax.swing.JButton();
		fSEs = new javax.swing.JButton();
		fEE = new javax.swing.JButton();
		javax.swing.GroupLayout jPanel3Layout = new javax.swing.GroupLayout(optionJPanel);

		getContentPane().setBackground(BACKGROUNDCOLOR);
		patternJPanel.setBackground(BACKGROUNDCOLOR);
		optionJPanel.setBackground(BACKGROUNDCOLOR);
		jPanelLogo.setBackground(BACKGROUNDCOLOR);
		fEName.setBackground(BACKGROUNDCOLOR);
		fESpec.setBackground(BACKGROUNDCOLOR);
		jPanel5.setBackground(BACKGROUNDCOLOR);
		jScrollPane1.setBackground(BACKGROUNDCOLOR);
		fSELP.setBackground(BACKGROUNDCOLOR);
		jScrollPane2.setBackground(BACKGROUNDCOLOR);
		fMapping.setBackground(BACKGROUNDCOLOR);
		jLabel1.setBackground(BACKGROUNDCOLOR);
		jLabel2.setBackground(BACKGROUNDCOLOR);
		eventJPanel.setBackground(BACKGROUNDCOLOR);
		fNE.setBackground(BACKGROUNDCOLOR);
		fSEs.setBackground(BACKGROUNDCOLOR);
		fEE.setBackground(BACKGROUNDCOLOR);
		jPanelLogo.setBackground(Color.WHITE);

		setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
		setTitle(TITLE);
		setResizable(true);

		javax.swing.GroupLayout patternSelectionPanel = new javax.swing.GroupLayout(patternsJPanel);

		patternsJPanel.setLayout(patternSelectionPanel);

		// patternSelectionJPanel.se

		patternsJPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(MOVEMENT_PATTERN));
		patternsJPanel.setToolTipText("");

		String[] patternCategories = { "Core Movement", "Triggers", "Avoidance", "Composition" };
		DefaultComboBoxModel<String> patternCategoriestItems = new DefaultComboBoxModel<>();
		
		Arrays.asList(patternCategories).stream().forEach(p -> patternCategoriestItems.addElement(p.toString()));

		patternCategorySelector = new JComboBox<String>(patternCategoriestItems);

		Arrays.asList(CoreMovementPatterns.values()).stream().forEach(p -> patternItems.addElement(p.toString()));

		patternBoxSelector = new JComboBox<String>(patternItems);

		patternBoxSelector.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				String selectedItem = (String) patternBoxSelector.getSelectedItem();
				if (selectedItem != null) {
					switch (selectedItem) {

					case "OR":
						intentText.setText(Composition.OR.getDescription());
						f1.removeAllItems();
						f2.removeAllItems();

						DefaultComboBoxModel<String> formulaeList1 = new DefaultComboBoxModel<String>();
						DefaultComboBoxModel<String> formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "AND":
						intentText.setText(Composition.AND.getDescription());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);

						break;
						//triggers
					case "Wait":
						intentText.setText(Triggers.WAIT.getDescription());
						////variation.setText(Triggers.WAIT.getVariations());
						examples.setText(Triggers.WAIT.getExamples());
						occurences.setText(Triggers.WAIT.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Instantaneous Reaction":
						intentText.setText(Triggers.INSTANTANEOUS_REACTION.getDescription());
						////variation.setText(Triggers.INSTANTANEOUS_REACTION.getVariations());
						examples.setText(Triggers.INSTANTANEOUS_REACTION.getExamples());
						occurences.setText(Triggers.INSTANTANEOUS_REACTION.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);

						break;
					case "Delayed Reaction":
						intentText.setText(Triggers.DELAYED_REACTION.getDescription());
						////variation.setText(Triggers.DELAYED_REACTION.getVariations());
						examples.setText(Triggers.DELAYED_REACTION.getExamples());
						occurences.setText(Triggers.DELAYED_REACTION.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
						//core movements
						//visits
					case "Visit":
						intentText.setText(CoreMovementPatterns.VISIT.getDescription());
						////variation.setText(CoreMovementPatterns.VISIT.getVariations());
						examples.setText(CoreMovementPatterns.VISIT.getExamples());
						occurences.setText(CoreMovementPatterns.VISIT.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Sequenced Visit":
						intentText.setText(CoreMovementPatterns.SEQUENCED_VISIT.getDescription());
						//variation.setText(CoreMovementPatterns.SEQUENCED_VISIT.getVariations());
						examples.setText(CoreMovementPatterns.SEQUENCED_VISIT.getExamples());
						occurences.setText(CoreMovementPatterns.SEQUENCED_VISIT.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Ordered Visit":
						intentText.setText(CoreMovementPatterns.ORDERED_VISIT.getDescription());
						//variation.setText(CoreMovementPatterns.ORDERED_VISIT.getVariations());
						examples.setText(CoreMovementPatterns.ORDERED_VISIT.getExamples());
						occurences.setText(CoreMovementPatterns.ORDERED_VISIT.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Strict Ordered Visit":
						intentText.setText(CoreMovementPatterns.STRICT_ORDERED_VISIT.getDescription());
						//variation.setText(CoreMovementPatterns.STRICT_ORDERED_VISIT.getVariations());
						examples.setText(CoreMovementPatterns.STRICT_ORDERED_VISIT.getExamples());
						occurences.setText(CoreMovementPatterns.STRICT_ORDERED_VISIT.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Fair Visit":
						intentText.setText(CoreMovementPatterns.FAIR_VISIT.getDescription());
						//variation.setText(CoreMovementPatterns.FAIR_VISIT.getVariations());
						examples.setText(CoreMovementPatterns.FAIR_VISIT.getExamples());
						occurences.setText(CoreMovementPatterns.FAIR_VISIT.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
						//patrolling
					case "Patrolling":
						intentText.setText(CoreMovementPatterns.PATROLLING.getDescription());
						//variation.setText(CoreMovementPatterns.PATROLLING.getVariations());
						examples.setText(CoreMovementPatterns.PATROLLING.getExamples());
						occurences.setText(CoreMovementPatterns.PATROLLING.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Sequenced Patrolling":
						intentText.setText(CoreMovementPatterns.SEQUENCED_PATROLLING.getDescription());
						//variation.setText(CoreMovementPatterns.SEQUENCED_PATROLLING.getVariations());
						examples.setText(CoreMovementPatterns.SEQUENCED_PATROLLING.getExamples());
						occurences.setText(CoreMovementPatterns.SEQUENCED_PATROLLING.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Ordered Patrolling":
						intentText.setText(CoreMovementPatterns.ORDERED_PATROLLING.getDescription());
						//variation.setText(CoreMovementPatterns.ORDERED_PATROLLING.getVariations());
						examples.setText(CoreMovementPatterns.ORDERED_PATROLLING.getExamples());
						occurences.setText(CoreMovementPatterns.ORDERED_PATROLLING.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Strict Ordered Patrolling":
						intentText.setText(CoreMovementPatterns.STRICT_ORDERED_PATROLLING.getDescription());
						////variation.setText(CoreMovementPatterns.STRICT_ORDERED_PATROLLING.getVariations());
						examples.setText(CoreMovementPatterns.STRICT_ORDERED_PATROLLING.getExamples());
						occurences.setText(CoreMovementPatterns.STRICT_ORDERED_PATROLLING.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Fair Patrolling":
						intentText.setText(CoreMovementPatterns.FAIR_PATROLLING.getDescription());
						////variation.setText(CoreMovementPatterns.FAIR_PATROLLING.getVariations());
						examples.setText(CoreMovementPatterns.FAIR_PATROLLING.getExamples());
						occurences.setText(CoreMovementPatterns.FAIR_PATROLLING.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
						//avoidance
						//past avoidance
					case "Past Avoidance":
						intentText.setText(Avoidance.PAST_AVOIDANCE.getDescription());
						//variation.setText(Avoidance.PAST_AVOIDANCE.getVariations());
						examples.setText(Avoidance.PAST_AVOIDANCE.getExamples());
						occurences.setText(Avoidance.PAST_AVOIDANCE.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Future Avoidance":
						intentText.setText(Avoidance.FUTURE_AVOIDANCE.getDescription());
						//variation.setText(Avoidance.FUTURE_AVOIDANCE.getVariations());
						examples.setText(Avoidance.FUTURE_AVOIDANCE.getExamples());
						occurences.setText(Avoidance.FUTURE_AVOIDANCE.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
						break;
					case "Global Avoidance":
						intentText.setText(Avoidance.GLOBAL_AVOIDANCE.getDescription());
						//variation.setText(Avoidance.GLOBAL_AVOIDANCE.getVariations());
						examples.setText(Avoidance.GLOBAL_AVOIDANCE.getExamples());
						occurences.setText(Avoidance.GLOBAL_AVOIDANCE.getOccurrences());
						f1.removeAllItems();
						f2.removeAllItems();

						formulaeList1 = new DefaultComboBoxModel<String>();
						formulaeList2 = new DefaultComboBoxModel<String>();

						formulae.keySet().stream().forEach(p -> formulaeList1.addElement(p));
						formulae.keySet().stream().forEach(p -> formulaeList2.addElement(p));

						f1.setModel(formulaeList1);
						f2.setModel(formulaeList2);
					default:
						break;
					}
				}
				
			}
	});
		
		patternCategorySelector.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				String selectedItem = (String) patternCategorySelector.getSelectedItem();
				cleanPanels();
				switch (selectedItem) {
				case "Core Movement":
					TitledBorder movementPatternTitle = BorderFactory.createTitledBorder("Core Movement");
					movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

					patternBoxSelector.setBorder(movementPatternTitle);
					patternItems.removeAllElements();

					Arrays.asList(CoreMovementPatterns.values()).stream()
							.forEach(p -> patternItems.addElement(p.toString()));

					patternBoxSelector.setModel(patternItems);
					locationPanel.setVisible(true);
					remotePanel.setVisible(true);
					ltlFormula.setVisible(true);
					intentText.setVisible(true);
					//variation.setVisible(true);
					examples.setVisible(true);
					occurences.setVisible(true);

					break;
				case "Triggers":
					movementPatternTitle = BorderFactory.createTitledBorder("Triggers");
					movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

					patternItems.removeAllElements();
					Arrays.asList(Triggers.values()).stream().forEach(p -> patternItems.addElement(p.toString()));

					patternBoxSelector.setBorder(movementPatternTitle);
					patternBoxSelector.setModel(patternItems);
					locationPanel.setVisible(false);
					remotePanel.setVisible(true);
					intentText.setVisible(true);
					ltlFormula.setVisible(true);
					//variation.setVisible(true);
					examples.setVisible(true);
					occurences.setVisible(true);

					break;
				case "Avoidance":
					movementPatternTitle = BorderFactory.createTitledBorder("Avoidance");
					movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

					patternItems.removeAllElements();
					Arrays.asList(Avoidance.values()).stream().forEach(p -> patternItems.addElement(p.toString()));

					patternBoxSelector.setBorder(movementPatternTitle);
					patternBoxSelector.setModel(patternItems);
					locationPanel.setVisible(true);
					remotePanel.setVisible(true);
					intentText.setVisible(true);
					ltlFormula.setVisible(true);
					//variation.setVisible(true);
					examples.setVisible(true);
					occurences.setVisible(true);

					break;
				case "Composition":
					movementPatternTitle = BorderFactory.createTitledBorder("Composition");
					movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

					patternItems.removeAllElements();
					Arrays.asList(Composition.values()).stream().forEach(p -> patternItems.addElement(p.toString()));

					patternBoxSelector.setBorder(movementPatternTitle);
					patternBoxSelector.setModel(patternItems);
					locationPanel.setVisible(false);
					remotePanel.setVisible(true);
					intentText.setVisible(true);
					ltlFormula.setVisible(true);//edited
					//variation.setVisible(false);
					examples.setVisible(false);
					occurences.setVisible(false);

					break;
				default:
					break;

				}

				patternBoxSelector.validate();
				patternBoxSelector.updateUI();
				patternBoxSelector.repaint();
				patternBoxSelector.doLayout();
				

			}
		});
		patternSelectionPanel
				.setHorizontalGroup(patternSelectionPanel.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
						.addGroup(patternSelectionPanel.createSequentialGroup()
								.addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)));

		patternSelectionPanel
				.setVerticalGroup(patternSelectionPanel.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
						.addGroup(patternSelectionPanel.createSequentialGroup()
								.addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)));

		optionJPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Options"));

		this.sendMission.setText(SEND_MISSION);
		this.loadMission.setText(LOAD_MISSION);
	
		this.sendMission.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {
				if (locations.getText().equals(INIT_POSITION_MESSAGE)) {
					JOptionPane.showMessageDialog(null, "Insert the set of locations to be considered");
				} else {
					MissionSender sender = new MissionSender();
					try {
						sender.send(loadMission().accept(new LTLFormulaToStringVisitor()), ipTextField.getText(), portTextField.getText());
					} catch (Exception e) {
						e.printStackTrace();
					}
				}

			}
		});
		this.loadMission.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {

				if (locations.getText().equals(INIT_POSITION_MESSAGE)) {
					JOptionPane.showMessageDialog(null, "Insert the set of locations to be considered");
				} else {

					try {
						loadMission();
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}

		});



		optionJPanel.setLayout(jPanel3Layout);

		jPanelLogo.setLayout(new BorderLayout());

		BufferedImage myPicture;
		try {
			myPicture = ImageIO
					.read(Co4robotsGUI.class.getClassLoader().getResourceAsStream("images/co4robotsLogo.png"));

			ImageIcon icon = new ImageIcon(myPicture);
			JLabel picLabel = new JLabel(icon);
			jPanelLogo.add(picLabel);
		} catch (IOException e) {
			e.printStackTrace();
		}
//capturing the main panel
		JPanel mainPanel = new JPanel();
		mainPanel.setBackground(BACKGROUNDCOLOR);
		javax.swing.GroupLayout layout = new javax.swing.GroupLayout(mainPanel);
		mainPanel.setLayout(layout);
		getContentPane().add(mainPanel);
		

        JScrollPane scrollPane = new JScrollPane();
        scrollPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
        getContentPane().add(scrollPane, BorderLayout.CENTER);
        scrollPane.setViewportView(mainPanel);
	//end capturing		

		layout.setAutoCreateGaps(true);
		layout.setAutoCreateContainerGaps(true);

		patternCategorySelector.setBorder(javax.swing.BorderFactory.createTitledBorder(SELECT_PATTERN_CATEGORY));


		patternBoxSelector.setBorder(javax.swing.BorderFactory.createTitledBorder("Movement Pattern"));

		remotePanel = new JPanel();
		
		ipTextField=new JTextField(15);
		ipTextField.setText(ip);

		portTextField=new JTextField(5);
		portTextField.setText(port);
		
		remotePanel.add(ipTextField);
		remotePanel.add(portTextField);
		
		
		//TitledBorder remoteTitle = BorderFactory.createTitledBorder("Robot");
		//remoteTitle.setTitlePosition(TitledBorder.RIGHT);
		remotePanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Robot"));
		remotePanel.setBackground(BACKGROUNDCOLOR);
		locationPanel = new JPanel();

		java.util.regex.Pattern commaseparated = java.util.regex.Pattern
		        .compile("(([a-z][a-z1-9]*),)*.([a-z][a-z1-9]*)*");

		locations = new JTextField(40);
		
		    
		locations.setText(INIT_POSITION_MESSAGE);
		locationPanel.setBackground(BACKGROUNDCOLOR);
		
		locationPanel.add(locations);
		TitledBorder locationsTitle = BorderFactory.createTitledBorder("Locations");
		locationsTitle.setTitlePosition(TitledBorder.RIGHT);
		locationPanel.setBorder(locationsTitle);

		ltlFormula = new JTextArea();
		TitledBorder ltlFormulaTitle = BorderFactory.createTitledBorder("LTL formula associated with the pattern");
		ltlFormulaTitle.setTitlePosition(TitledBorder.RIGHT);
		ltlFormula.setBorder(ltlFormulaTitle);
		
		intentText = new JTextArea();
		intentText.setLineWrap(true);

		TitledBorder intentTitle = BorderFactory.createTitledBorder("Intent");
		intentTitle.setTitlePosition(TitledBorder.RIGHT);
		intentText.setBorder(intentTitle);

		//variation = new JTextArea();
		//variation.setLineWrap(true);
		TitledBorder variationTitle = BorderFactory.createTitledBorder("Variations");
		variationTitle.setTitlePosition(TitledBorder.RIGHT);
		//variation.setBorder(variationTitle);

		examples = new JTextArea();
		examples.setLineWrap(true);
		TitledBorder examplesTitle = BorderFactory.createTitledBorder("Examples and Known Uses");
		examplesTitle.setTitlePosition(TitledBorder.RIGHT);
		examples.setBorder(examplesTitle);
		

		TitledBorder relationshipsTitle = BorderFactory.createTitledBorder("Relationships");
		relationshipsTitle.setTitlePosition(TitledBorder.RIGHT);
		
		occurences = new JTextArea();
		occurences.setLineWrap(true);
		TitledBorder occuttencesTitle = BorderFactory.createTitledBorder("Occurences");
		occuttencesTitle.setTitlePosition(TitledBorder.RIGHT);
		occurences.setBorder(occuttencesTitle);
		

		TitledBorder propertiesTitle = BorderFactory.createTitledBorder("Property Library");
		propertiesTitle.setTitlePosition(TitledBorder.RIGHT);

		JScrollPane p = new JScrollPane(this.propertyList);
		p.setBackground(BACKGROUNDCOLOR);
		p.setBorder(propertiesTitle);
		
		layout.createSequentialGroup();
		
	
		locationPanel.setBackground(BACKGROUNDCOLOR);
		patternCategorySelector.setBackground(BACKGROUNDCOLOR);
		layout.setHorizontalGroup(
				layout.createParallelGroup(GroupLayout.Alignment.LEADING).addComponent(jPanelLogo)
				.addGroup(layout.createSequentialGroup()
				.addGroup(layout.createParallelGroup()
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(remotePanel)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(patternCategorySelector)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(patternBoxSelector)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(locationPanel)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(ltlFormula)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(intentText)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(f1)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(f2)

				).addGroup(layout.createParallelGroup().addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(this.loadMission)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(this.sendMission)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING).addComponent(p)))

		));

	layout.setVerticalGroup(layout.createSequentialGroup().addComponent(jPanelLogo)
				.addGroup(layout.createParallelGroup().addGroup(layout.createSequentialGroup()
						.addComponent(remotePanel)
						.addComponent(patternCategorySelector)
						.addComponent(patternBoxSelector)
						.addComponent(locationPanel)
						.addComponent(ltlFormula)
						.addComponent(intentText)
						.addComponent(f1)
						.addComponent(f2))
						.addGroup(layout.createSequentialGroup()
								.addComponent(this.loadMission)
								.addComponent(this.sendMission)
								.addComponent(p))));

	f1.setVisible(false);
	f2.setVisible(false);
		
		setBounds(0, 0, FRAME_INIT_WIDTH, FRAME_INIT_HEIGTH);
		setVisible(false);
		this.setBackground(BACKGROUNDCOLOR);
		this.getContentPane().setBackground(BACKGROUNDCOLOR);
		this.setResizable(true);
	}

	// Variables declaration - do not modify//GEN-BEGIN:variables
	private javax.swing.JButton sendMission;
	private javax.swing.JButton loadMission;
	private javax.swing.JButton fEE;
	private javax.swing.JCheckBox fEName;
	private javax.swing.JCheckBox fESpec;
	private javax.swing.JTextArea fMapping;
	private javax.swing.JButton fNE;
	private javax.swing.JTextPane fSELP;
	private javax.swing.JButton fSEs;
	private javax.swing.JLabel jLabel1;
	private javax.swing.JLabel jLabel2;
	private javax.swing.JPanel patternsJPanel;
	private javax.swing.JPanel patternJPanel;
	private javax.swing.JPanel optionJPanel;
	private javax.swing.JPanel jPanelLogo;
	private javax.swing.JPanel jPanel5;
	private javax.swing.JPanel propertyPanel;
	private javax.swing.JPanel eventJPanel;
	private javax.swing.JScrollPane jScrollPane1;
	private javax.swing.JScrollPane jScrollPane2;
	// End of variables declaration//GEN-END:variables

	private LTLFormula loadMission() throws Exception {

		String selectedIdem = (String) patternBoxSelector.getSelectedItem();
		String patternCategory = (String) patternCategorySelector.getSelectedItem();

		LTLFormula computedltlformula = LTLFormula.TRUE;

		String locationsText = locations.getText().replaceAll(" ", "");
		String[] selectedLocations = locationsText.split(",");

		switch (patternCategory) {
		case "Triggers":
			Triggers p2 = Triggers.valueOf(selectedIdem.toUpperCase().replaceAll(" ", "_"));
			// computedltlformula = p2.getMission(selectedLocations);
			
			intentText.setText(p2.getDescription());
			//variation.setText(p2.getVariations());
			examples.setText(p2.getExamples());
			occurences.setText(p2.getOccurrences());

			computedltlformula = p2.getMission(formulae.get((String) f1.getSelectedItem()),
					formulae.get((String) f2.getSelectedItem()));
			
			ltlFormula.setText(computedltlformula.accept(new LTLFormulaToStringVisitor()));

			formulae.put(FORMULA_COUNTER + " - " + (String) patternBoxSelector.getSelectedItem() + "(" + f1.getSelectedItem() +", "
					+f2.getSelectedItem()+")", computedltlformula);

			List<String> array1 = new ArrayList<String>(formulae.keySet());
			String[]t = new String[array1.size()];
			array1.toArray(t);
			propertyList.setListData(t);
			
			FORMULA_COUNTER = FORMULA_COUNTER + 1;
			break;
		case "Avoidance":
			Avoidance p = Avoidance.valueOf(selectedIdem.toUpperCase().replaceAll(" ", "_"));
			computedltlformula = p.getMission(selectedLocations);
			intentText.setText(p.getDescription());

			//variation.setText(p.getVariations());

			examples.setText(p.getExamples());

	
			occurences.setText(p.getOccurrences());
			ltlFormula.setText(computedltlformula.accept(new LTLFormulaToStringVisitor()));

			formulae.put(FORMULA_COUNTER + " - " + (String) patternBoxSelector.getSelectedItem() + "(" + locations.getText()
					+ ")", computedltlformula);

			List<String> array = new ArrayList<String>(formulae.keySet());
			String[] d = new String[array.size()];
			array.toArray(d);
			propertyList.setListData(d);
			FORMULA_COUNTER = FORMULA_COUNTER + 1;
			
			break;
		case "Core Movement":
			CoreMovementPatterns p1 = CoreMovementPatterns.valueOf(selectedIdem.toUpperCase().replaceAll(" ", "_"));
			computedltlformula = p1.getMission(selectedLocations);
			intentText.setText(p1.getDescription());
			//variation.setText(p1.getVariations());
			examples.setText(p1.getExamples());
			occurences.setText(p1.getOccurrences());

			ltlFormula.setText(computedltlformula.accept(new LTLFormulaToStringVisitor()));

			//formulae.put(FORMULA_COUNTER + " - " + (String) patternBoxSelector.getSelectedItem() + "(" + locations.getText()
			//		+ ")", computedltlformula);
			array = new ArrayList<String>(formulae.keySet());
			d = new String[array.size()];
			array.toArray(d);
			propertyList.setListData(d);
			FORMULA_COUNTER = FORMULA_COUNTER + 1;
			
			break;
		case "Composition":
			Composition c = Composition.valueOf(selectedIdem.toUpperCase().replaceAll(" ", "_"));
			intentText.setText(c.getDescription());

			computedltlformula = c.getMission(formulae.get((String) f1.getSelectedItem()),
					formulae.get((String) f2.getSelectedItem()));
			
			ltlFormula.setText(computedltlformula.accept(new LTLFormulaToStringVisitor()));

			formulae.put(FORMULA_COUNTER + " - " + (String) patternBoxSelector.getSelectedItem() + "(" + f1.getSelectedItem() +", "
					+f2.getSelectedItem()+")", computedltlformula);

			array = new ArrayList<String>(formulae.keySet());
			d = new String[array.size()];
			array.toArray(d);
			propertyList.setListData(d);
			
			FORMULA_COUNTER = FORMULA_COUNTER + 1;
		default:
			break;
		}

		

		return computedltlformula;
	}
	private void cleanPanels(){
;
		intentText.setText("");
		ltlFormula.setText("");
		//variation.setText("");
		examples.setText("");
		occurences.setText("");
	}
	public class RegexPatternFormatter extends DefaultFormatter {

		  protected java.util.regex.Matcher matcher;

		  public RegexPatternFormatter(java.util.regex.Pattern regex) {
		    setOverwriteMode(false);
		    matcher = regex.matcher(""); // create a Matcher for the regular
		                   // expression
		  }

		  public Object stringToValue(String string) throws java.text.ParseException {
		    if (string == null)
		      return null;
		    matcher.reset(string); // set 'string' as the matcher's input

		    if (!matcher.matches()) // Does 'string' match the regular expression?
		      throw new java.text.ParseException("does not match regex", 0);

		    // If we get this far, then it did match.
		    return super.stringToValue(string); // will honor the 'valueClass'
		                      // property
		  }
	}
}
