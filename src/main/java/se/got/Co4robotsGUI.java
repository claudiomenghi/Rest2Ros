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
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.Iterator;
import java.util.Set;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JTextPane;
import javax.swing.text.AttributeSet;
import javax.swing.text.SimpleAttributeSet;
import javax.swing.text.StyleConstants;
import javax.swing.text.StyleContext;

import se.got.engine.EventSelectionValidator;
import se.got.engine.EventStorage;
import se.got.engine.PSPController;
import se.got.gui.dialogs.EditEventDialog;
import se.got.gui.dialogs.NewEventDialog;
import se.got.mappings.LTLMapper;
import se.got.mappings.MTLMapper;
import se.got.mappings.PatternMapper;
import se.got.mappings.PrismMapper;
import se.got.mappings.QuantitativePrismMapper;
import se.got.mappings.SELMapper;
import se.got.sel.Event;
import se.got.sel.patterns.Pattern;
import se.got.sel.scopes.Scope;
import java.awt.Toolkit;

public class Co4robotsGUI extends javax.swing.JFrame implements PSPController {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private final static String TITLE = "co4robots: High Level Specification Panel";
	private final static String PATTERN = "Mission specification pattern";
	private final static String MAPPING = "Goal";
	private final static String EVENTNAMES = "Show service names";
	private final static String EVENTSPECIFICATION = "Show service specifications";
	private final static String NEWEVENT = "New service";
	private final static String EDITEVENT = "Edit service";
	private final static String ADDEVENT = "Add Service";
	private final static String EVENT = "Services";
	private final static String SCOPE = "Mission Scope";
	private final static String SEND_MISSION = "Send mission";

	public final static Color BACKGROUNDCOLOR = Color.WHITE;
	private EventStorage fEvents;

	private void initMappings() {
		fMappings.addItem(new SELMapper());
		fMappings.addItem(new LTLMapper());
		fMappings.addItem(new MTLMapper());
		fMappings.addItem(new PrismMapper());
		fMappings.addItem(new QuantitativePrismMapper());
	}

	public Co4robotsGUI() {
		initComponents();

		reset();

		initMappings();

		// connect scopes with controller
		fScopes.setController(this);
		fSelectedScope = fScopes.getSelectedScope();

		// connect patterns with controller
		fPatterns.setController(this);
		fSelectedPattern = fPatterns.getSelectedPattern();

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
		fScopes.updateEvents();
		fPatterns.updateEvents();
		fScopes.clearSelection();
		fPatterns.clearSelection();
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
		fSelectedScope = fScopes.getSelectedScope();

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
		fSelectedPattern = fPatterns.getSelectedPattern();

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

			PatternMapper lMapper = (PatternMapper) fMappings.getSelectedItem();

			if (lMapper != null) {
				String lMapping = lMapper.getMapping(fSelectedScope, fSelectedPattern);

				if (!lMapping.isEmpty()) {
					if (lMapper.hasMappingErrorOccurred())
						fMapping.setForeground(Color.red);
					else
						fMapping.setForeground(Color.black);
					fMapping.setText(lMapping);
				} else {
					fMapping.setForeground(Color.red);
					fMapping.setText(lMapper.getNotSupportedMessage());
				}
			}
		}
	}

	/**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        scopeJPanel = new javax.swing.JPanel();
        scopeJPanel.setBackground(BACKGROUNDCOLOR);
        
        fScopes = new se.got.gui.panels.scopes.ScopePanel();
        fScopes.setBackground(BACKGROUNDCOLOR);
        
        patternJPanel = new javax.swing.JPanel();
        fPatterns = new se.got.gui.panels.pattern.PatternPanel();
        optionJPanel = new javax.swing.JPanel();
        jPanelLogo = new javax.swing.JPanel();
        fEName = new javax.swing.JCheckBox();
        fESpec = new javax.swing.JCheckBox();
        fClear = new javax.swing.JButton();
        this.sendMission=new javax.swing.JButton();
        jPanel5 = new javax.swing.JPanel();
        jScrollPane1 = new javax.swing.JScrollPane();
        fSELP = new javax.swing.JTextPane();
        propertyPanel = new javax.swing.JPanel();
        fMappings = new javax.swing.JComboBox();
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
        fPatterns.setBackground(BACKGROUNDCOLOR);
        optionJPanel.setBackground(BACKGROUNDCOLOR);
        jPanelLogo.setBackground(BACKGROUNDCOLOR);
        fEName.setBackground(BACKGROUNDCOLOR);
        fESpec.setBackground(BACKGROUNDCOLOR);
        fClear.setBackground(BACKGROUNDCOLOR);
        jPanel5.setBackground(BACKGROUNDCOLOR);
        jScrollPane1.setBackground(BACKGROUNDCOLOR);
        fSELP.setBackground(BACKGROUNDCOLOR);
        fMappings.setBackground(BACKGROUNDCOLOR);
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

        scopeJPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(SCOPE));
        scopeJPanel.setToolTipText("");

        javax.swing.GroupLayout jPanel1Layout = new javax.swing.GroupLayout(scopeJPanel);
        scopeJPanel.setLayout(jPanel1Layout);
        jPanel1Layout.setHorizontalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel1Layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addComponent(fScopes, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
        );
        jPanel1Layout.setVerticalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel1Layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addComponent(fScopes, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
        );

        patternJPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(PATTERN));

        javax.swing.GroupLayout jPanel2Layout = new javax.swing.GroupLayout(patternJPanel);
        patternJPanel.setLayout(jPanel2Layout);
        jPanel2Layout.setHorizontalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel2Layout.createSequentialGroup()
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addComponent(fPatterns, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
        );
        jPanel2Layout.setVerticalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(fPatterns, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
        );

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

        fClear.setText("Reset Editor");
        fClear.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fClearActionPerformed(evt);
            }
        });

        this.sendMission.setText(SEND_MISSION);
        this.sendMission.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
               MissionSender sender=new MissionSender();
               try {
				sender.send("AAA");
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
            }
        });

        optionJPanel.setLayout(jPanel3Layout);

        jPanel3Layout.setHorizontalGroup(
            jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel3Layout.createSequentialGroup()
                .addGroup(jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(jPanel3Layout.createSequentialGroup()
                        .addContainerGap()
                        .addComponent(fESpec))
                    .addGroup(jPanel3Layout.createSequentialGroup()
                        .addGap(34, 34, 34)
                        .addComponent(fClear))
                    .addGroup(jPanel3Layout.createSequentialGroup()
                        .addContainerGap()
                        .addComponent(fEName)))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        
        jPanel3Layout.setVerticalGroup(
            jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel3Layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(fEName)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(fESpec)
                .addGap(18, 18, 18)
                .addComponent(fClear)
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        jPanelLogo.setLayout(new BorderLayout());
        
        BufferedImage myPicture;
		try {
			System.out.println(Co4robotsGUI.class.getClassLoader().getResourceAsStream("images/co4robotsLogo.png"));
			myPicture = ImageIO.read(Co4robotsGUI.class.getClassLoader().getResourceAsStream("images/co4robotsLogo.png"));
		
			ImageIcon icon=new ImageIcon(myPicture);
			Image image = icon.getImage();
			//Image newimg = icon.getScaledInstance(120, 120,  java.awt.Image.SCALE_SMOOTH); // scale it the smooth way  
			//icon = new ImageIcon(newimg);
			JLabel picLabel = new JLabel(icon);
	        jPanelLogo.add(picLabel);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
        

        jPanel5.setBorder(javax.swing.BorderFactory.createTitledBorder("High Level Mission"));

        fSELP.setEditable(true);
        
        //fSELP.setColumns(20);
        fSELP.setFont(new java.awt.Font("Lucida Grande", 1, 14)); // NOI18N
        //fSELP.setLineWrap(true);
        //fSELP.setRows(5);
        //fSELP.setWrapStyleWord(true);
        jScrollPane1.setViewportView(fSELP);

        javax.swing.GroupLayout jPanel5Layout = new javax.swing.GroupLayout(jPanel5);
        jPanel5.setLayout(jPanel5Layout);
        jPanel5Layout.setHorizontalGroup(
            jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel5Layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(jScrollPane1, javax.swing.GroupLayout.PREFERRED_SIZE, 334, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        jPanel5Layout.setVerticalGroup(
            jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel5Layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(jScrollPane1)
                .addContainerGap())
        );

        propertyPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(MAPPING));
        propertyPanel.setBackground(BACKGROUNDCOLOR);
        fMappings.setMaximumSize(new java.awt.Dimension(180, 27));
        fMappings.setMinimumSize(new java.awt.Dimension(180, 27));
        fMappings.setPreferredSize(new java.awt.Dimension(180, 27));
        fMappings.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fMappingsActionPerformed(evt);
            }
        });

        fMapping.setEditable(false);
        fMapping.setColumns(20);
        fMapping.setFont(new java.awt.Font("Lucida Grande", 1, 14)); // NOI18N
        fMapping.setLineWrap(true);
        fMapping.setRows(5);
        jScrollPane2.setViewportView(fMapping);

        jLabel1.setText("Target Logic:");

        jLabel2.setText("Formula:");

        javax.swing.GroupLayout jPanel6Layout = new javax.swing.GroupLayout(propertyPanel);
        propertyPanel.setLayout(jPanel6Layout);
        jPanel6Layout.setHorizontalGroup(
            jPanel6Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel6Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel6Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(jPanel6Layout.createSequentialGroup()
                        .addComponent(jLabel2)
                        .addGap(0, 0, Short.MAX_VALUE))
                    .addGroup(jPanel6Layout.createSequentialGroup()
                        .addGroup(jPanel6Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(jPanel6Layout.createSequentialGroup()
                                .addComponent(jLabel1)
                                .addGap(18, 18, 18)
                                .addComponent(fMappings, javax.swing.GroupLayout.PREFERRED_SIZE, 180, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addGap(0, 0, Short.MAX_VALUE)
                                .addComponent(this.sendMission)
                                .addGap(0, 0, Short.MAX_VALUE))
                            .addComponent(jScrollPane2))
                        .addContainerGap())))
        );
        jPanel6Layout.setVerticalGroup(
            jPanel6Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel6Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel6Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(jLabel1)
                    .addComponent(fMappings, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(this.sendMission)
                		)
                .addGap(18, 18, 18)
                .addComponent(jLabel2)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addComponent(jScrollPane2, javax.swing.GroupLayout.PREFERRED_SIZE, 170, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );

        eventJPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(EVENT));

        fNE.setText(NEWEVENT);
        fNE.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fNEActionPerformed(evt);
            }
        });

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

        javax.swing.GroupLayout jPanel7Layout = new javax.swing.GroupLayout(eventJPanel);
        eventJPanel.setLayout(jPanel7Layout);
        jPanel7Layout.setHorizontalGroup(
            jPanel7Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel7Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel7Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(fSEs)
                    .addComponent(fNE)
                    .addComponent(fEE))
                .addContainerGap(12, Short.MAX_VALUE))
        );

        jPanel7Layout.linkSize(javax.swing.SwingConstants.HORIZONTAL, new java.awt.Component[] {fEE, fNE, fSEs});

        jPanel7Layout.setVerticalGroup(
            jPanel7Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel7Layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(fNE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(fEE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(fSEs)
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        
        

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);

        
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                		 .addGroup(layout.createSequentialGroup().addComponent(jPanelLogo))
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(patternJPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                            .addComponent(jPanel5, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                            .addComponent(propertyPanel, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(scopeJPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(26, 26, 26)
                        .addComponent(optionJPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(eventJPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap(12, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGap(8, 8, 8)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING).addComponent(jPanelLogo))
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(scopeJPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING, false)
                        .addComponent(eventJPanel, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(optionJPanel, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                       ))
                .addGap(15, 15, 15)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                        .addComponent(jPanel5, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addGap(18, 18, 18)
                        .addComponent(propertyPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addComponent(patternJPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

	private void fNEActionPerformed(java.awt.event.ActionEvent evt) {// GEN-FIRST:event_fNEActionPerformed
		// add new event

		if ((new NewEventDialog(this)).showDialog() != null) {
			EventSelectionValidator.startEditUpdate();
			fScopes.updateEvents();
			fPatterns.updateEvents();
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

		fScopes.updateEvents();
		fPatterns.updateEvents();
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
			fScopes.updateEvents();
			fPatterns.updateEvents();
			EventSelectionValidator.stopEditUpdate();
		}

		this.requestFocus();
	}// GEN-LAST:event_fEEActionPerformed

	// Variables declaration - do not modify//GEN-BEGIN:variables
	private javax.swing.JButton fClear;
	private javax.swing.JButton sendMission;
	private javax.swing.JButton fEE;
	private javax.swing.JCheckBox fEName;
	private javax.swing.JCheckBox fESpec;
	private javax.swing.JTextArea fMapping;
	private javax.swing.JComboBox fMappings;
	private javax.swing.JButton fNE;
	private se.got.gui.panels.pattern.PatternPanel fPatterns;
	private javax.swing.JTextPane fSELP;
	private javax.swing.JButton fSEs;
	private se.got.gui.panels.scopes.ScopePanel fScopes;
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

}
