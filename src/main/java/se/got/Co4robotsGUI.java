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
import javax.swing.ImageIcon;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.JTextPane;
import javax.swing.ListSelectionModel;
import javax.swing.border.TitledBorder;
import javax.swing.text.AttributeSet;
import javax.swing.text.SimpleAttributeSet;
import javax.swing.text.StyleConstants;
import javax.swing.text.StyleContext;

import se.got.engine.EventSelectionValidator;
import se.got.engine.EventStorage;
import se.got.ltl.LTLFormula;
import se.got.ltl.visitors.LTLFormulaToStringVisitor;
import se.got.sel.Event;
import se.got.sel.patterns.Pattern;
import se.got.sel.scopes.Scope;

public class Co4robotsGUI extends javax.swing.JFrame  {

	private static final String INIT_POSITION_MESSAGE = "Insert the positions to be considered separated by a comma";

	private static final int FRAME_INIT_HEIGTH = 600;

	private static final int FRAME_INIT_WIDTH = 800;

	private JPanel locationPanel;
	private static int FORMULA_COUNTER = 1;

	private static Map<String, LTLFormula> formulae;

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
	private final static String LOAD_PROPERTY = "Load property";

	private final static String SELECT_PATTERN_CATHEGORY = "Select pattern cathegory";

	private static JList<String> propertyList;

	private JComboBox<String> patternCathegorySelector;
	private JComboBox<String> patternBoxSelector;
	private JTextArea ltlFormula;

	private JTextArea intentText;
	private JTextArea variation;
	private JTextArea examples;
	private JTextArea relationships;
	private JTextArea occurences;
	private JComboBox<String> f1;
	private JComboBox<String> f2;
	private final DefaultComboBoxModel<String> patternItems;

	private JTextField locations;
	public final static Color BACKGROUNDCOLOR = Color.WHITE;
	private EventStorage fEvents;

	private void initMappings() {
	}

	public Co4robotsGUI() {

		patternItems = new DefaultComboBoxModel<>();

		this.f1 = new JComboBox<String>();
		this.f2 = new JComboBox<String>();
		this.formulae = new HashMap<>();
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
		initComponents();

		reset();

		initMappings();

		// update initial SEL and mapping
		updateSELandMapping();
	}

	// Entry point

	public static void main(String args[]) {
		/* Create and display the form */
		java.awt.EventQueue.invokeLater(new Runnable() {
			public void run() {
				(new Co4robotsGUI()).showEditor();
			}
		});
	}

	public void showEditor() {
		setLocationRelativeTo(null); // center?
		setVisible(true);
	}

	public JFrame getHostFrame() {
		return this;
	}

	public void reset() {
		Event.reset();
		fEvents = new EventStorage();

		fSelectedScope = null;
		fSelectedPattern = null;
		EventSelectionValidator.clearSelection();
		fSEs.setEnabled(true);
		fESpec.setSelected(false);
		fEName.setSelected(true);
		Event.EventStringMethod = Event.E_Name;
	}

	// event controller facet

	public Event newEvent(String aName) {
		Event Result = fEvents.newEvent(aName);

		return Result;
	}

	public Event newEvent(String aName, String aSpecification) {
		Event Result = fEvents.newEvent(aName, aSpecification);

		return Result;
	}

	public Iterator<Event> iterator() {
		return fEvents.iterator();
	}

	// Scope events

	private Scope fSelectedScope;

	public boolean isScopeEventSelectionPossible(Event aEvent) {
		return EventSelectionValidator.isScopeEventSelectionPossible(this, aEvent);
	}

	public void updateScope() {

		EventSelectionValidator.updateScopeEvents(fSelectedScope);

		// update SEL and mapping
		updateSELandMapping();
	}

	// Pattern events
	private Pattern fSelectedPattern;


	public boolean isPatternEventSelectionPossible(Event aEvent, Event aAltEvent) {
		return EventSelectionValidator.isPatternEventSelectionPossible(this, aEvent, aAltEvent);
	}

	public void updatePattern() {

		EventSelectionValidator.updatePatternEvents(fSelectedPattern);

		// update SEL and mapping
		updateSELandMapping();
	}

	private void appendToPane(JTextPane tp, String msg, Color c) {
		StyleContext sc = StyleContext.getDefaultStyleContext();
		AttributeSet aset = sc.addAttribute(SimpleAttributeSet.EMPTY, StyleConstants.Foreground, c);

		aset = sc.addAttribute(aset, StyleConstants.FontFamily, "Lucida Console");
		aset = sc.addAttribute(aset, StyleConstants.Alignment, StyleConstants.ALIGN_JUSTIFIED);

		int len = tp.getDocument().getLength();
		tp.setCaretPosition(len);
		tp.setCharacterAttributes(aset, false);
		tp.replaceSelection(msg);
	}

	// SEL expansion

	private void updateSELandMapping() {
		// StringBuilder sb = new StringBuilder();

		if (fSelectedScope != null && fSelectedPattern != null) {
			fSELP.setText("");
			appendToPane(fSELP, fSelectedScope.getSpecificationAsSEL(), Color.GRAY);
			appendToPane(fSELP, ", ", Color.RED);
			appendToPane(fSELP, fSelectedPattern.getSpecificationAsSEL(), Color.DARK_GRAY);
			appendToPane(fSELP, ".", Color.RED);

			// fSELP.setText( sb.toString() );

		}
	}

	/**
	 * This method is called from within the constructor to initialize the form.
	 * WARNING: Do NOT modify this code. The content of this method is always
	 * regenerated by the Form Editor.
	 */
	// <editor-fold defaultstate="collapsed" desc="Generated
	// Code">//GEN-BEGIN:initComponents
	private void initComponents() {

		patternsJPanel = new javax.swing.JPanel();
		patternsJPanel.setBackground(BACKGROUNDCOLOR);

		patternJPanel = new javax.swing.JPanel();
		optionJPanel = new javax.swing.JPanel();
		jPanelLogo = new javax.swing.JPanel();
		fEName = new javax.swing.JCheckBox();
		fESpec = new javax.swing.JCheckBox();
		this.sendMission = new javax.swing.JButton();
		this.loadMission = new javax.swing.JButton();
		this.loadProperty = new javax.swing.JButton();
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
		setResizable(false);

		javax.swing.GroupLayout patternSelectionPanel = new javax.swing.GroupLayout(patternsJPanel);

		patternsJPanel.setLayout(patternSelectionPanel);

		// patternSelectionJPanel.se

		patternsJPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(MOVEMENT_PATTERN));
		patternsJPanel.setToolTipText("");

		String[] patternCategories = { "Core Movement", "Triggers", "Avoidance", "Composition" };
		DefaultComboBoxModel<String> patternCathegoriestItems = new DefaultComboBoxModel<>();

		Arrays.asList(patternCategories).stream().forEach(p -> patternCathegoriestItems.addElement(p.toString()));

		patternCathegorySelector = new JComboBox<String>(patternCathegoriestItems);

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
					default:
						break;
					}
				}

			}
		});

		patternCathegorySelector.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				String selectedItem = (String) patternCathegorySelector.getSelectedItem();
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
					ltlFormula.setVisible(true);
					intentText.setVisible(true);
					variation.setVisible(true);
					examples.setVisible(true);
					relationships.setVisible(true);
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
					intentText.setVisible(true);
					ltlFormula.setVisible(true);
					variation.setVisible(true);
					examples.setVisible(true);
					relationships.setVisible(true);
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
					intentText.setVisible(true);
					ltlFormula.setVisible(true);
					variation.setVisible(true);
					examples.setVisible(true);
					relationships.setVisible(true);
					occurences.setVisible(true);

					break;
				case "Composition":
					movementPatternTitle = BorderFactory.createTitledBorder("Composition");
					movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

					patternItems.removeAllElements();
					Arrays.asList(Composition.values()).stream().forEach(p -> patternItems.addElement(p.toString()));

					patternBoxSelector.setBorder(movementPatternTitle);
					patternBoxSelector.setModel(patternItems);
					intentText.setText(Composition.AND.getDescription());
					locationPanel.setVisible(false);
					intentText.setVisible(true);
					ltlFormula.setVisible(false);
					variation.setVisible(false);
					examples.setVisible(false);
					relationships.setVisible(false);
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

		fEName.setSelected(true);
		fEName.setText(EVENTNAMES);
		fEName.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {
				fENameActionPerformed(evt);
			}
		});

		fESpec.setText(EVENTSPECIFICATION);
		fESpec.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {
				fESpecActionPerformed(evt);
			}
		});

		this.sendMission.setText(SEND_MISSION);
		this.loadMission.setText(LOAD_MISSION);
		this.loadProperty.setText(LOAD_PROPERTY);
		this.sendMission.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {
				System.out.println();
				if (locations.getText().equals(INIT_POSITION_MESSAGE)) {
					JOptionPane.showMessageDialog(null, "Insert the set of locations to be considered");
				} else {
					MissionSender sender = new MissionSender();
					try {
						sender.send(loadMission().accept(new LTLFormulaToStringVisitor()));
					} catch (Exception e) {
						e.printStackTrace();
					}
				}

			}
		});
		this.loadMission.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {

				System.out.println();
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

		this.loadProperty.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {

				if (locations.getText().equals(INIT_POSITION_MESSAGE)) {
					JOptionPane.showMessageDialog(null, "Insert the set of locations to be considered");
				} else {

					LTLFormula property;
					try {
						property = loadMission();

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
			System.out.println(Co4robotsGUI.class.getClassLoader().getResourceAsStream("images/co4robotsLogo.png"));
			myPicture = ImageIO
					.read(Co4robotsGUI.class.getClassLoader().getResourceAsStream("images/co4robotsLogo.png"));

			ImageIcon icon = new ImageIcon(myPicture);
			JLabel picLabel = new JLabel(icon);
			jPanelLogo.add(picLabel);
		} catch (IOException e) {
			e.printStackTrace();
		}

		javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
		getContentPane().setLayout(layout);

		layout.setAutoCreateGaps(true);
		layout.setAutoCreateContainerGaps(true);

		TitledBorder patternCathegoryTile = BorderFactory.createTitledBorder(SELECT_PATTERN_CATHEGORY);
		patternCathegoryTile.setTitlePosition(TitledBorder.RIGHT);

		patternCathegorySelector.setBorder(patternCathegoryTile);

		TitledBorder movementPatternTitle = BorderFactory.createTitledBorder("Movement Pattern");
		movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

		patternBoxSelector.setBorder(movementPatternTitle);

		locationPanel = new JPanel();
		locations = new JTextField(40);
		locations.setText(INIT_POSITION_MESSAGE);

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

		variation = new JTextArea();
		variation.setLineWrap(true);
		TitledBorder variationTitle = BorderFactory.createTitledBorder("Variations");
		variationTitle.setTitlePosition(TitledBorder.RIGHT);
		variation.setBorder(variationTitle);

		examples = new JTextArea();
		examples.setLineWrap(true);
		TitledBorder examplesTitle = BorderFactory.createTitledBorder("Examples and Known Uses");
		examplesTitle.setTitlePosition(TitledBorder.RIGHT);
		examples.setBorder(examplesTitle);

		relationships = new JTextArea();
		relationships.setLineWrap(true);
		TitledBorder relationshipsTitle = BorderFactory.createTitledBorder("Relationships");
		relationshipsTitle.setTitlePosition(TitledBorder.RIGHT);
		relationships.setBorder(relationshipsTitle);

		occurences = new JTextArea();
		occurences.setLineWrap(true);
		TitledBorder occuttencesTitle = BorderFactory.createTitledBorder("Occurences");
		occuttencesTitle.setTitlePosition(TitledBorder.RIGHT);
		occurences.setBorder(occuttencesTitle);

		TitledBorder propertiesTitle = BorderFactory.createTitledBorder("Property Library");
		propertiesTitle.setTitlePosition(TitledBorder.RIGHT);

		JScrollPane p = new JScrollPane(this.propertyList);
		p.setBorder(propertiesTitle);
		layout.setHorizontalGroup(layout.createSequentialGroup()
				.addGroup(layout.createParallelGroup()
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(jPanelLogo)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(patternCathegorySelector)

						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(patternBoxSelector)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(locationPanel)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(ltlFormula)

						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(intentText)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(variation)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(examples)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(relationships)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(occurences)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(f1)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(f2)

				).addGroup(layout.createParallelGroup().addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))

						.addComponent(this.loadProperty)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(this.loadMission)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(this.sendMission)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING).addComponent(p)))

		);
		layout.setVerticalGroup(layout.createSequentialGroup()
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(jPanelLogo)
				.addGroup(layout.createParallelGroup().addGroup(layout.createSequentialGroup()

						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(patternCathegorySelector)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
						.addComponent(patternBoxSelector)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(locationPanel)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(ltlFormula)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(intentText)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(variation)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(examples)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(relationships)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(occurences)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(f1)
						.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(f2))

						.addGroup(layout.createSequentialGroup()
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(this.loadMission)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(this.loadProperty)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(this.sendMission)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(p))));

		setBounds(0, 0, FRAME_INIT_WIDTH, FRAME_INIT_HEIGTH);
		setVisible(true);
		this.setResizable(true);
	}

	private void fENameActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fENameActionPerformed
		// TODO add your handling code here:

		if (!fEName.isSelected()) {
			fESpec.setSelected(true);
			Event.EventStringMethod = Event.E_Spec;
		} else {
			if (fESpec.isSelected())
				Event.EventStringMethod = Event.E_NameAndSpec;
			else
				Event.EventStringMethod = Event.E_Name;
		}
		updateSELandMapping();
		repaint();
	}// GEN-LAST:event_fENameActionPerformed

	private void fESpecActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fESpecActionPerformed
		// TODO add your handling code here:

		if (!fESpec.isSelected()) {
			fEName.setSelected(true);
			Event.EventStringMethod = Event.E_Name;
		} else {
			if (fEName.isSelected())
				Event.EventStringMethod = Event.E_NameAndSpec;
			else
				Event.EventStringMethod = Event.E_Spec;
		}
		updateSELandMapping();
		repaint();
	}// GEN-LAST:event_fESpecActionPerformed

	// Variables declaration - do not modify//GEN-BEGIN:variables
	private javax.swing.JButton sendMission;
	private javax.swing.JButton loadMission;
	private javax.swing.JButton loadProperty;
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
		String patternCategory = (String) patternCathegorySelector.getSelectedItem();

		LTLFormula computedltlformula = LTLFormula.TRUE;

		String locationsText = locations.getText().replaceAll(" ", "");
		String[] selectedLocations = locationsText.split(",");

		switch (patternCategory) {
		case "Triggers":
			Triggers p2 = Triggers.valueOf(selectedIdem.toUpperCase().replaceAll(" ", "_"));
			// computedltlformula = p2.getMission(selectedLocations);
			break;
		case "Avoidance":
			Avoidance p = Avoidance.valueOf(selectedIdem.toUpperCase().replaceAll(" ", "_"));
			computedltlformula = p.getMission(selectedLocations);
			intentText.setText(p.getDescription());

			variation.setText(p.getVariations());

			examples.setText(p.getExamples());

			relationships.setText(p.getRelationships());

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
			variation.setText(p1.getVariations());
			examples.setText(p1.getExamples());
			relationships.setText(p1.getRelationships());
			occurences.setText(p1.getOccurrences());

			ltlFormula.setText(computedltlformula.accept(new LTLFormulaToStringVisitor()));

			formulae.put(FORMULA_COUNTER + " - " + (String) patternBoxSelector.getSelectedItem() + "(" + locations.getText()
					+ ")", computedltlformula);
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
		variation.setText("");
		examples.setText("");
		relationships.setText("");
		occurences.setText("");
	}
}
