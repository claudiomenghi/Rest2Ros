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
import java.awt.Dimension;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.Arrays;
import java.util.Iterator;
import java.util.function.BinaryOperator;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.DefaultComboBoxModel;
import javax.swing.GroupLayout;
import javax.swing.ImageIcon;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.JTextPane;
import javax.swing.border.TitledBorder;
import javax.swing.text.AttributeSet;
import javax.swing.text.SimpleAttributeSet;
import javax.swing.text.StyleConstants;
import javax.swing.text.StyleContext;

import se.got.engine.EventSelectionValidator;
import se.got.engine.EventStorage;
import se.got.engine.PSPController;
import se.got.gui.dialogs.EditEventDialog;
import se.got.gui.dialogs.NewEventDialog;
import se.got.ltl.Formula;
import se.got.ltl.LTLConjunction;
import se.got.ltl.LTLFormula;
import se.got.ltl.LTLEventually;
import se.got.ltl.atoms.LTLIPropositionalAtom;
import se.got.ltl.visitors.ToStringVisitor;
import se.got.sel.Event;
import se.got.sel.patterns.Pattern;
import se.got.sel.scopes.Scope;

public class Co4robotsGUI extends javax.swing.JFrame implements PSPController {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public static final BinaryOperator<LTLFormula> conjunctionOperator = (left, right) -> {

		if (left.equals(LTLFormula.TRUE)) {
			return right;
		}

		if (right.equals(LTLFormula.TRUE)) {
			return left;
		}
		return new LTLConjunction(left, right);
	};

	private final static String TITLE = "co4robots: High Level Specification Panel";
	private final static String EVENTNAMES = "Show service names";
	private final static String EVENTSPECIFICATION = "Show service specifications";
	private final static String EDITEVENT = "Edit service";
	private final static String ADDEVENT = "Add Service";
	private final static String MOVEMENT_PATTERN = "Movement Specification Pattern";
	private final static String SEND_MISSION = "Send mission";
	private final static String LOAD_MISSION = "Load mission";
	private final static String[] MOVEMENT_PATTERNS = { "Visit", "Sequenced Visit", "Ordered Visit",
			"Strict Ordered Visit", "Fair Visit", "Patrolling", "Sequenced Patrolling", "Ordered Patrolling",
			"Strict Ordered Patrolling", "Fair Patrolling" };

	private JComboBox<String> patternBoxSelector;
	private JTextArea ltlFormula;

	private JTextArea intentText;
	private JTextArea variation;
	private JTextArea examples;
	private JTextArea relationships;
	private JTextArea occurences;

	private JTextField locations;
	public final static Color BACKGROUNDCOLOR = Color.WHITE;
	private EventStorage fEvents;

	private void initMappings() {
	}

	public Co4robotsGUI() {
		initComponents();

		reset();

		initMappings();

		// connect scopes with controller

		// connect patterns with controller

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

	// event selection validation facet

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

	public boolean isPatternEventSelectionPossible(Event aEvent) {
		return EventSelectionValidator.isPatternEventSelectionPossible(this, aEvent);
	}

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

		scopeJPanel = new javax.swing.JPanel();
		scopeJPanel.setBackground(BACKGROUNDCOLOR);

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
		setResizable(false);

		scopeJPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(MOVEMENT_PATTERN));
		scopeJPanel.setToolTipText("");

		javax.swing.GroupLayout missionPanel = new javax.swing.GroupLayout(scopeJPanel);
		scopeJPanel.setLayout(missionPanel);

		DefaultComboBoxModel<String> patternItems = new DefaultComboBoxModel<>();

		Arrays.asList(MOVEMENT_PATTERNS).stream().forEach(patternItems::addElement);

		patternBoxSelector = new JComboBox<String>(patternItems);

		missionPanel.setHorizontalGroup(
				missionPanel.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING).addGroup(missionPanel
						.createSequentialGroup().addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
				// .addComponent(patternBoxSelector,
				// javax.swing.GroupLayout.PREFERRED_SIZE,
				// javax.swing.GroupLayout.DEFAULT_SIZE,
				// javax.swing.GroupLayout.PREFERRED_SIZE)
				));

		missionPanel.setVerticalGroup(
				missionPanel.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING).addGroup(missionPanel
						.createSequentialGroup().addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
				// .addComponent(patternBoxSelector,
				// javax.swing.GroupLayout.PREFERRED_SIZE,
				// javax.swing.GroupLayout.DEFAULT_SIZE,
				// javax.swing.GroupLayout.PREFERRED_SIZE)
				));

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
		this.sendMission.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {

				MissionSender sender = new MissionSender();
				try {
					sender.send(loadMission());
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			}
		});
		this.loadMission.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {

				loadMission();
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
			Image image = icon.getImage();
			// Image newimg = icon.getScaledInstance(120, 120,
			// java.awt.Image.SCALE_SMOOTH); // scale it the smooth way
			// icon = new ImageIcon(newimg);
			JLabel picLabel = new JLabel(icon);
			jPanelLogo.add(picLabel);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		fSEs.setText(ADDEVENT);
		fSEs.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {
				fSEsActionPerformed(evt);
			}
		});

		fEE.setText(EDITEVENT);
		fEE.addActionListener(new java.awt.event.ActionListener() {
			public void actionPerformed(java.awt.event.ActionEvent evt) {
				fEEActionPerformed(evt);
			}
		});

		javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
		getContentPane().setLayout(layout);

		layout.setAutoCreateGaps(true);
		layout.setAutoCreateContainerGaps(true);

		TitledBorder movementPatternTitle = BorderFactory.createTitledBorder("Movement Pattern");
		movementPatternTitle.setTitlePosition(TitledBorder.RIGHT);

		patternBoxSelector.setBorder(movementPatternTitle);

		JPanel locationPanel = new JPanel();
		locations = new JTextField(40);
		locations.setText("Insert the positions to be considered separated by a comma");

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

		layout.setHorizontalGroup(layout.createSequentialGroup().addGroup(layout.createParallelGroup()
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(jPanelLogo)
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(patternBoxSelector)
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(locationPanel)
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(ltlFormula)

				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(intentText)
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(variation)
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(examples)
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(relationships)
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(occurences)

		).addGroup(layout.createParallelGroup().addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
				.addComponent(this.loadMission).addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
				.addComponent(this.sendMission))

		);
		layout.setVerticalGroup(layout.createSequentialGroup()
				.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING)).addComponent(jPanelLogo).addGroup(

						layout.createParallelGroup().addGroup(layout.createSequentialGroup()

								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(patternBoxSelector)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(locationPanel)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(ltlFormula)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(intentText)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(variation)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(examples)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(relationships)
								.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
								.addComponent(occurences))

								.addGroup(layout.createSequentialGroup()
										.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
										.addComponent(this.loadMission)
										.addGroup(layout.createParallelGroup(GroupLayout.Alignment.LEADING))
										.addComponent(this.sendMission))));

		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
	      setBounds(0,0,screenSize.width/2, screenSize.height/2);
	      setVisible(true);
	      this.setResizable(true);
//		pack();
	}// </editor-fold>//GEN-END:initComponents

	private void fNEActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fNEActionPerformed
		// add new event

		if ((new NewEventDialog(this)).showDialog() != null) {
			EventSelectionValidator.startEditUpdate();
			EventSelectionValidator.stopEditUpdate();
		}

		this.requestFocus();
	}// GEN-LAST:event_fNEActionPerformed

	private void fMappingsActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fMappingsActionPerformed
		// TODO add your handling code here:
		updateSELandMapping();
	}// GEN-LAST:event_fMappingsActionPerformed

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

	private void fSEsActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fSEsActionPerformed
		// add sample events

		newEvent("P");
		newEvent("S");
		newEvent("T");
		newEvent("T1");
		newEvent("T2");
		newEvent("T3");
		newEvent("Q");
		newEvent("R");
		newEvent("Z");
		newEvent("ZS");
		newEvent("Z1");
		newEvent("Z2");
		newEvent("Z3");

		fSEs.setEnabled(false);

		this.requestFocus();
	}// GEN-LAST:event_fSEsActionPerformed

	private void fClearActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fClearActionPerformed
		// clear everything

		reset();

		updateScope();
		updatePattern();

		repaint();
		this.requestFocus();
	}// GEN-LAST:event_fClearActionPerformed

	private void fEEActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fEEActionPerformed
		// edit events

		if ((new EditEventDialog(this)).showDialog() != null) {
			EventSelectionValidator.startEditUpdate();
			EventSelectionValidator.stopEditUpdate();
		}

		this.requestFocus();
	}// GEN-LAST:event_fEEActionPerformed

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
	private javax.swing.JPanel scopeJPanel;
	private javax.swing.JPanel patternJPanel;
	private javax.swing.JPanel optionJPanel;
	private javax.swing.JPanel jPanelLogo;
	private javax.swing.JPanel jPanel5;
	private javax.swing.JPanel propertyPanel;
	private javax.swing.JPanel eventJPanel;
	private javax.swing.JScrollPane jScrollPane1;
	private javax.swing.JScrollPane jScrollPane2;
	// End of variables declaration//GEN-END:variables

	private String loadMission() {

		String selectedIdem = (String) patternBoxSelector.getSelectedItem();

		LTLFormula computedltlformula = LTLFormula.TRUE;

		String locationsText = locations.getText().replaceAll(" ", "");
		String[] selectedLocations = locationsText.split(",");
		switch (selectedIdem) {
		case "Visit":
			computedltlformula = Arrays.asList(selectedLocations).stream()
					.map(location -> (LTLFormula) new LTLEventually(new LTLIPropositionalAtom(location)))
					.reduce(LTLFormula.TRUE, conjunctionOperator);

			intentText.setText(
					"A robot must visit an area or a set of areas. This area can be a set of locations of a building, or a set of points of interest on a map that must be surveyed.");
			variation.setText(
					"If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l, while l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space. A variation can be obtained where not all the areas in the set must be visited. In this case, it is sufficient to replace the AND operator with an OR.");
			examples.setText(
					"A common usage example of the Visit pattern is a scenario in which a robot has to collect a set of items that are placed in different locations and bring them in a target destination. Visit and Avoidance patterns often go together. Avoidance patterns are used e.g. to require robots to avoid obstacles as they guard an area. Trigger patterns can also be used in combination with the Visit pattern to specify conditions upon which Visit should start or stop.");
			relationships.setText(
					"	The Visit pattern generalizes most of the core movement patterns that constrain how locations are visited.");
			occurences.setText(
					"Yoo et al. and Kress-Gazit et al. formulate an LTL mission specification to ensure that a set of areas are visited. In the first case, the visiting pattern is combined with the specification of past and future avoidance mission requirements. In the second case, an LTL mission specification is provided to describe the following mission requirement: the robot must go to rooms $P1$, $P2$, $P3$ and $P4$ in any order.");
			break;
		default:
			throw new IllegalArgumentException("No pattern with name " + selectedIdem);
		}
		ltlFormula.setText(computedltlformula.accept(new ToStringVisitor()));

		return computedltlformula.accept(new ToStringVisitor());
	}
}
